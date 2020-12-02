#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <reflector_ekf_slam/reflector_ekf_slam.h>

std::unique_ptr<ReflectorEKFSLAM> g_slam;
void LaserCallback(const sensor_msgs::LaserScanConstPtr& laser_ptr );
void EncoderCallback(const nav_msgs::OdometryConstPtr& en_ptr);

ros::Publisher g_landmark_pub;
ros::Publisher g_robot_pose_pub;
ros::Publisher g_path_pub;
double x0_ = 0.0;
double y0_ = 0.0;
double yaw0_ = 0.0;

std::vector<std::string> SplitString(const std::string &input,
                                     const char delimiter)
{
    std::istringstream stream(input);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(stream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<std::vector<double>> ReadFromTxtFile(const std::string& file)
{
    std::vector<std::vector<double>> result;
    std::ifstream in(file.c_str());  
    std::string line;  
  
    if(in) // 有该文件  
    {  
        while (getline (in, line)) // line中不包括每行的换行符  
        {   
            std::cout << line << std::endl;  
            if(!line.empty())
            {
                std::vector<double> vec;
                auto vstr_vec = SplitString(line, ',');
                for(auto &p : vstr_vec)
                {
                    vec.push_back(std::stod(p));
                }
                result.push_back(vec);
            }
        }  
    }  
    else // 没有该文件  
    {  
        std::cout <<"no such file" << std::endl;
        return result;  
    }  
    if(result.size() != 2)
    {
        result.clear();
        std::cout <<"format is not right, must be 2 line" << std::endl;
        return result;
    }      
    if(result.back().size() != 2 * result.front().size())
    {
        result.clear();
        std::cout <<"format is not right, must be 2 line" << std::endl;
    }  
    return result;
}

std::vector<std::vector<double>> map_;
std::string map_path_;

int main(int argc, char **argv)
{
    /***** 初始化ROS *****/
    ros::init(argc, argv, "slam_node"); 
    ros::NodeHandle nh;
    
    /***** 获取参数 *****/
    /* TODO 错误处理 */
    std::string laser_topic_name, encoder_topic_name;
            
    nh.getParam ( "laser", laser_topic_name);
    nh.getParam ( "odom", encoder_topic_name);
    ROS_INFO("Laser topic is: %s", laser_topic_name.c_str());
    ROS_INFO("Odom topic is: %s", encoder_topic_name.c_str());
    // Read initial pose from launch file, we need initial pose for relocalization
    std::string pose_str;
    if(nh.getParam("start_pose", pose_str) && !pose_str.empty())
    {
        auto v = SplitString(pose_str,',');
        if(v.size() != 3)
        {
            std::cout << "only support x y yaw" << std::endl;
            exit(0);
        }
        x0_ = std::stod(v[0]);
        y0_ = std::stod(v[1]);
        yaw0_ = std::stod(v[2]);
    }
    ROS_INFO("Start pose is: %f, %f, %f", x0_, y0_, yaw0_);
    std::string map_path;
    if(!nh.getParam("map_path", map_path))
    {
        ROS_ERROR("Can not get path, you must set path for mapping and localization!!");
        exit(-1);
    }
    ROS_INFO("Map path: %s", map_path.c_str());
    map_ = ReadFromTxtFile(map_path);
    
    const int length = map_path.length();
    std::string sub_str1 = map_path.substr(0, length - 4);
    map_path_ = sub_str1 + "_new.txt";
    
    
    /***** 初始化消息发布 *****/
    g_landmark_pub = nh.advertise<visualization_msgs::MarkerArray>( "ekf_slam/landmark", 1);
    g_robot_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_slam/pose", 1);
    g_path_pub = nh.advertise<nav_msgs::Path>("ekf_slam/path",1);
        
    /***** 初始化消息订阅 *****/
    ros::Subscriber laser_sub = nh.subscribe(laser_topic_name, 1, LaserCallback);
    ros::Subscriber encoder_sub = nh.subscribe(encoder_topic_name, 1, EncoderCallback);
    
    std::cout << "\n\nSYSTEM START \n\n";
    ros::spin();
    
    return 0;
}

void LaserCallback(const sensor_msgs::LaserScanConstPtr& laser_ptr )
{
    const double time = laser_ptr->header.stamp.toSec();
    if(!g_slam)
    {
        g_slam = common::make_unique<ReflectorEKFSLAM>(time, x0_, y0_, yaw0_);
        // load old map
        g_slam->loadFromVector(map_);
        // set save path
        g_slam->setSaveMapPath(map_path_);
    }else
    {
        g_slam->addLaser(laser_ptr);
        /* publish  landmarks */
        visualization_msgs::MarkerArray markers = g_slam->toRosMarkers(3.5); // 为了方便显示协方差做了放大
        g_landmark_pub.publish(markers);
        
        /* publish  robot pose */
        geometry_msgs::PoseWithCovarianceStamped pose = g_slam->toRosPose(); // pose的协方差在rviz也做了放大
        pose.header.stamp = laser_ptr->header.stamp;
        g_robot_pose_pub.publish(pose);

        /* publish path */
        g_path_pub.publish(g_slam->path());
    }
}


void EncoderCallback ( const nav_msgs::OdometryConstPtr& en_ptr )
{
    if(g_slam)
    {
        /* 加入ＥＫＦ-SLAM */
        g_slam->addEncoder(en_ptr);
        
        /* publish  landmarks */
        // visualization_msgs::MarkerArray markers = g_slam->toRosMarkers(3.5); // 为了方便显示协方差做了放大
        // g_landmark_pub.publish(markers);
        
        /* publish  robot pose */
        geometry_msgs::PoseWithCovarianceStamped pose = g_slam->toRosPose(); // pose的协方差在rviz也做了放大
        pose.header.stamp = en_ptr->header.stamp;
        g_robot_pose_pub.publish(pose);
    }
    
}
