#include "utility.h"
#include "lio_sam/cloud_info.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16;   // 强制SSE填充以正确对齐内存

// 定义新类型里元素包括XYZI+ring+time, time主要用于去畸变,ring主要用于对点云进行归类
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

/**
 * 点云投影成深度图,类似lego loam中的做法
 * 列表示线束数量，行表示角度分辨率，比如16*1800,横向解析度则为360/1800=0.2
 */
class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;


public:
    ImageProjection():
    deskewFlag(0)
    {
        // subscriber, 订阅imu和odometry以及原始点云，这里的odom应该是gps+imu通过robot_localization包融合得到的.
        // imu和odom callback中主要是用来装数据
        // 点云处理的逻辑全部在cloudHandler中
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());// 允许指定hints到roscpp的传输层
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

    // publisher, 发布自定义的cloud_info和用于odometry的cloud
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

    // 初始化参数
        allocateMemory();
        resetParameters();

    // 设置控制台信息输出
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        // 将深度矩阵的所有默认初始值设置为 最大浮点数FLT_MAX
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection(){}

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
        // 将imu数据转到lidar坐标系下, 这里在params中配置过imu in lidar的外参
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        // 这里用双端队列来装数据，pop_front()方便做滑窗
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

  void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg) {
    // 锁存后装数据
    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
   }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    // 清除临时点云，并检查当前点云帧里面是否有ring和time通道
        if (!cachePointCloud(laserCloudMsg))
            return;

    // 配置好用于去畸变的imu和odom参数
    // 找到点云时间戳前后的GPS odom和imu 数据，并分别计算在这两帧时间内的位姿增量和旋转增量
    // 用于后续去畸变
        if (!deskewInfo())
            return;

    // 每一帧点云进来都这么处理

    // 点云去畸变后 投影成深度图
        projectPointCloud();

    // 从深度图中提取点云, 给lidar odometry用
        cloudExtraction();

    // 发布点云
        publishClouds();

    // 重置参数
        resetParameters();
    }

  bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    // 清除内存中的临时点云
        cloudQueue.push_back(*laserCloudMsg);// 当前帧点云压入
        if (cloudQueue.size() <= 2)// 同一时刻会有2帧点云，用于取得 当前帧点云的帧头和帧尾时间戳
            return false;

        // convert cloud
        // 点云队列先进先出,将最新的点云 转存到currentCloudMsg
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();// 转存完成后，弹出，可继续缓存下一帧点云
        // 激光雷达类型检测
        if (sensor == SensorType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
            // TODO:: remove nan
            // std::vector<int> indices;
            // pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            if(!laserCloudIn->is_dense)
            {
                static bool first = true;
                if(first){
                    ROS_ERROR("Point cloud is not in dense format, please remove NaN points best!");
                    ROS_ERROR("System will ignore it in this time.");
                    first = false;
                }

                laserCloudIn->is_dense = true;
            }
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        // velodyne驱动的默认配置是 timestamp_first_packet = false,取的组成当前帧的最后一个packet包来做时间戳
        // 按照lio_sam的去畸变逻辑，应该是把这个参数该成了 true，取的组成当前帧的第一个packet包来做时间戳
        cloudHeader = currentCloudMsg.header;// 当前帧点云
        timeScanCur = cloudHeader.stamp.toSec();// 代表当前帧点云帧头时间戳
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;// 下一帧点云帧头时间戳=当前帧点云帧尾时间戳

        // check dense flag
        // 像velodyne激光，它会在驱动层把无效的NaN值去除，然后再发出来；但不是所有激光都支持，已知速腾激光不行
        // echo点云话题后，若显示is_dense为true时 代表你的激光雷达已经做了这个工作
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        // 检查激光点云数据中是否包含 点云线束编号 信息
          // veloodyne和ouster都自带ring通道数据
          // 其他雷达需要注释掉这一部分,自行计算
          // 最好是选择雷达驱动自带ring通道的激光雷达；
          // 如果是自己通过点云x，y，z来计算垂直角度，再根据激光的处置角度分辨率来确定ring编号的话，在激光线束多，比如64线时
          // 此时由于垂直角度分辨率很小了，加上激光本身的测距波动，此时很容易算错角度而分错ring号
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        // 检查激光点云数据中是否包含 时间戳 信息
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                // 这里需要echo你的点云话题，观察确定时间戳通道的名字
                if (field.name == "time" || field.name == "timestamp" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // 保证当前帧点云数据时间戳头 或 戳尾中至少一个是在 当前imu队列覆盖的时间戳范围 中间
        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur
            || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        // 遍历imu队列, 计算点云帧对应的imu数据, 包括积分计算其转过的角度
        // 用于后续点云去畸变
        imuDeskewInfo();

        // odom去畸变参数配置
        odomDeskewInfo();

        return true;
    }

  void imuDeskewInfo() {
    //这个参数在地图优化程序中用到  首先为false 完成相关操作后置true
    cloudInfo.imuAvailable = false;

    // imu去畸变参数
    // timeScanCur指当前点云帧的时间戳
    while (!imuQueue.empty()) {
      // 以0.01为阈值 舍弃在激光帧采集时间timeScanCur之前的imu数据(所以你的 imu频率至少要大于100hz)
      // TODO: 将0.01定值改为 1/imu实际频率 以适应<100hz的imu
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)// 取到imu队列中时间戳最 左趋近timeScanCur 的那帧imu姿态作为当前帧点云的初始姿态
                // 用imu的欧拉角做scan的位姿估计,直接把值给了cloudInfo在地图优化程序中使用
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
            //  如果当前Imu时间比 当前帧点云时间戳尾还大0.01
            //  则表示已经完成针对 当前帧点云头尾时间内覆盖的所有imu数据的 姿态积分,可退出
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            // 第一次初始化时以下值都是0
            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            // 计算出相邻imu帧之间的时间差
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            // 当前imu帧对应转角 = 上一次帧imu转角 + 角速度*时间差
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        // 完成去畸变时imu的准备工作后, imuPointerCur 代表的是最后的那一帧imu数据的序号
        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        // 得到 当前帧点云覆盖的时间戳范围内 对应的所有imu数据
        cloudInfo.imuAvailable = true;
    }

  void odomDeskewInfo() {
        // 类似imu数据,用于标志当前点云帧的odom处理的信息是否有效
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
          /**********************************************************************/
            // 以0.01为阈值 舍弃在激光帧采集时间timeScanCur之前的odom数据
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }
        if (odomQueue.empty())
            return;
        // 第一个odom数据要出现在 当前帧点云之前
        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;
        /**让通常低频,不足100hz的odom数据也要 与当前帧点云的时间戳 左相差 不到1ms,是否合理?**/

        // get start odometry at the beinning of the scan
        // 遍历odom队列,将odom的位姿作为点云信息中预测位姿
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            // 之前已经将小于timeScanCur超过0.01的数据弹出
            // 所以startOdomMsg已经可代表起始激光扫描的起始时刻的里程计消息
            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        // 用当前odom队列的起始位姿作为当前帧点云的初始位姿
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        // 获得一帧扫描末尾的里程计消息,和初始位姿无关, 主要用于去畸变、运动补偿
        odomDeskewFlag = false;

        // 计算当前帧点云时间戳尾时刻的里程计消息
          // 如果最后一帧odom数据时间戳都在 timeScanEnd 之前,
          // 那很有可能取不到最接近timeScanEnd的odom数据,放弃寻找,提前退出
        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        // 扫描结束时的odom
        nav_msgs::Odometry endOdomMsg;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            // 取时间戳上 最右贴近的odom数据作为 帧尾时刻对应的odom
            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        // 对帧头和帧尾odom的位姿协方差矩阵 进行一致性判断 否，则退出
        // 不要求绝对一致 所以用round()函数保证 四舍五入到整数时能一致即可
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        // 获得帧头odom变换
        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x,
                                                            startOdomMsg.pose.pose.position.y,
                                                            startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        // 获得帧尾odom变换
        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x,
                                                          endOdomMsg.pose.pose.position.y,
                                                          endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        // 获得一帧扫描起始与结束时刻间的变换, 参考loam
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // 计算这个头、尾odom的增量, 用于后续去畸变
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        // 完成odom去畸变准备工作标志位
        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        // 最终要保证点云时间在两帧imu数据中间。
        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            //找到第一个大于point_time的imu数据就退出,这样的话
            //        back           point_time        front
            //  imuPointerFront-1    point_time   imuPointerFront
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        // pointTime对应imu队列的第一个或是最后一个，直接获得取其对应的旋转增量(在imuDeskewInfo函数中已经计算好了)
        // pointTime在imu队列中间的话，按比率获取
        if (imuPointerFront == 0 || pointTime > imuTime[imuPointerFront]) {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            // 到这里说明，该点的时间戳位于两个imu数据对应时间戳的中部位置
            int imuPointerBack = imuPointerFront - 1;
            // 计算point_time在imu队列的前后占比
            // 公式： x = a*[(b-x)/(b-a)] + b*[(x-a)/(b-a)]
            // a为头，b为尾  注意这里的关系是反的，头点a要乘x到尾点的比率
            double ratioFront =
                  (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack =
                  (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        // 这里没有启用该功能
        // 对于点云去畸变，主要是去的rpy姿态(尤其是yaw朝向角)上的畸变
        // 在低速运动下，相邻帧之间存在的位置上的畸变很小，很可能3cm不到(况且你激光雷达本身的测距精度上都还存在+/-1-3cm的波动呢)
        // 如果是高速移动、低帧率的激光雷达可能有用
        // 用到了里程计增量值
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

   PointType deskewPoint(PointType *point, double relTime) {
        // 根据时间戳,对每个点去畸变
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        // relTime是点在当前帧内的实际时间
        double pointTime = timeScanCur + relTime;

        // 用于补偿旋转和平移，根据点云的时间戳去计算其旋转
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        // 如果是第一帧数据
        if (firstPointFlag == true) {
          // 起始矩阵赋值再取逆
          transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                                      rotXCur, rotYCur,rotZCur)).inverse();
          firstPointFlag = false;
        }

        // transform points to start
        // 计算每个点相对于当前帧起始点之间的 相对偏移量,参考Loam
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        // 去完畸变的点
        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

      void projectPointCloud() {
        // 将点云投影成深度图(对于vlp16来说就是把原始xyz点云变成 16个2d scan数据)
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i) {
            // 遍历每个点, 按线束进行分类
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            // 点云有效距离判断
            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;
            // 记录当前点云所属ring号=深度图中的行号rowIdn。没有ring 的话需要依据垂直角度计算
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // 以ring号对点云降采样
            // 比如128线点云对你的CPU来说太多,你只要16线 16=128/8 downsampleRate此时设为8
            if (rowIdn % downsampleRate != 0)
                continue;
            // 激光点的水平角度, 计算当前点转过的角度，进而计算每个点投影到深度图中的列id和时间戳
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            // 角分辨率
            static float ang_res_x = 360.0/float(Horizon_SCAN);

            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 若该编号下的值已经不是初始值，说明已经被赋值过了
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            //  依据每个点field中的time时间戳，开始做点云去畸变--运动补偿方式
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            // 把计算好的当前点的 深度、行号、列号 归类到深度矩阵中
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            // 计算当前点在 无序排列中对应的下标索引值
            int index = columnIdn + rowIdn * Horizon_SCAN;

            // 将去完畸变的点 用 fullCloud 存好
            fullCloud->points[index] = thisPoint;
        }
    }

      void cloudExtraction() {
        int count = 0;
        // 提取点云给odometry
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)// 先按行遍历行号(对应着ring)，这里是16线所以是16行
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                // 深度图矩阵中不一定每个位置都有点，只取有深度的点
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    // 这里原来是std::thread, 现在改成了ros MultiThreadedSpinner
    // 阻塞微调, 类似于ros::spin(), 你可以在它的构造函数中指定线程数量,
    // 但如果不指定或者设为0, 它会根据你的CPU内核数创建线程.
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
