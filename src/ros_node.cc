#include "ros_node.h"
#include "sensor/sensor_data.h"

Node::Node()
{
    LoadNodeOptions();
    if (!options_.use_laser && !options_.use_point_cloud)
    {
        LOG(ERROR) << "Only support laser and point cloud to detect reflector";
        LOG(ERROR) << "Please Set use_laser or use_point_cloud : true";
        return;
    }

    /***** 初始化消息发布 *****/
    landmark_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("ekf_slam/landmark", 1);
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_slam/pose", 1);
    path_publisher_ = node_handle_.advertise<nav_msgs::Path>("ekf_slam/path", 1);
    global_reflector_publisher_ =
        node_handle_.advertise<visualization_msgs::MarkerArray>("ekf_slam/global_landmark", 1);

    /***** 初始化消息订阅 *****/
    odometry_subscriber_ = node_handle_.subscribe(options_.odom_topic_name, 1, &Node::OdometryCallback, this);
    if (options_.use_laser)
        laser_subscriber_ = node_handle_.subscribe(options_.scan_topic_name, 1, &Node::ScanCallback, this);
    if (options_.use_point_cloud)
        point_cloud_subscriber_ =
            node_handle_.subscribe(options_.points_topic_name, 1, &Node::PointCloudCallback, this);

    if (options_.use_laser)
    {
        reflector_detect::ReflectorDetectOptions laser_reflector_options;
        laser_reflector_options.intensity_min = options_.intensity_min;
        laser_reflector_options.reflector_min_length = options_.reflector_min_length;
        laser_reflector_options.reflector_length_error = options_.reflector_length_error;
        laser_reflector_options.range_min = options_.range_min;
        laser_reflector_options.range_max = options_.range_max;
        laser_reflector_detector_ =
            common::make_unique<reflector_detect::LaserReflectorDetect>(laser_reflector_options);
        laser_reflector_detector_->SetSensorToBaseLinkTransform(options_.sensor_to_base_link);
    }

    if (options_.use_point_cloud)
    {
        reflector_detect::PointCloudOptions point_cloud_options;
        point_cloud_options.intensity_min = options_.intensity_min;
        point_cloud_reflector_detector_ =
            common::make_unique<reflector_detect::PointCloudReflectorDetect>(point_cloud_options);
        point_cloud_reflector_detector_->SetSensorToBaseLinkTransform(options_.sensor_to_base_link);
    }

    LOG(INFO) << "Reflector SLAM is start !!!!";
    ros::spin();
}

Node::~Node()
{
    if (slam_)
        SaveResult();
}

void Node::SaveResult()
{
    if (!slam_)
        return;
    
    std::string map_path = options_.result_path;
    if (map_path.empty())
        map_path = "/home/wsh/test.txt";
    const ekf::State state = slam_->GetState();
    const sensor::Map map = slam_->GetGlobalMap();
    LOG(INFO) << "Start to save result in " << map_path;
    // write data
    std::ofstream out(map_path.c_str(), std::ios::out);
    if (!map.reflector_map_.empty())
    {
        LOG(INFO) << "Write global map";
        for (int i = 0; i < map.reflector_map_.size(); ++i)
        {
            if (i != map.reflector_map_.size() - 1)
                out << map.reflector_map_[i].x() << "," << map.reflector_map_[i].y() << ",";
            else
                out << map.reflector_map_[i].x() << "," << map.reflector_map_[i].y();
        }
    }
    if (state.mu.rows() > 3)
    {
        LOG(INFO) << "Write new map";
        out << ",";
        const int N = (state.mu.rows() - 3) / 2;
        for (int i = 0; i < N; ++i)
        {
            if (i != N - 1)
                out << state.mu(3 + 2 * i) << "," << state.mu(3 + 2 * i + 1) << ",";
            else
                out << state.mu(3 + 2 * i) << "," << state.mu(3 + 2 * i + 1);
        }
    }
    out << std::endl;
    if (!map.reflector_map_.empty())
    {
        for (int i = 0; i < map.reflector_map_.size(); ++i)
        {
            if (i != map.reflector_map_.size() - 1)
                out << map.reflector_map_coviarance_[i](0, 0) << "," << map.reflector_map_coviarance_[i](0, 1) << ","
                    << map.reflector_map_coviarance_[i](1, 0) << "," << map.reflector_map_coviarance_[i](1, 1) << ",";
            else
                out << map.reflector_map_coviarance_[i](0, 0) << "," << map.reflector_map_coviarance_[i](0, 1) << ","
                    << map.reflector_map_coviarance_[i](1, 0) << "," << map.reflector_map_coviarance_[i](1, 1);
        }
    }
    if (state.mu.rows() > 3)
    {
        out << ",";
        const int N = (state.mu.rows() - 3) / 2;
        for (int i = 0; i < N; ++i)
        {
            if (i != N - 1)
                out << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0, 0) << "," << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0, 1) << ","
                    << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1, 0) << "," << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1, 1) << ",";
            else
                out << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0, 0) << "," << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0, 1) << ","
                    << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1, 0) << "," << state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1, 1);
        }
    }
    out << std::endl;
    out.close();
    LOG(INFO) << "Finish to save result";
}

void Node::LoadNodeOptions()
{
    /***** 获取参数 *****/
    node_handle_.getParam("scan", options_.scan_topic_name);
    node_handle_.getParam("odom", options_.odom_topic_name);
    node_handle_.getParam("points", options_.points_topic_name);
    LOG(INFO) << "Odometry topic is: " << options_.odom_topic_name;
    LOG(INFO) << "Scan topic is: " << options_.scan_topic_name;
    LOG(INFO) << "Point cloud topic is: " << options_.points_topic_name;

    // Read initial pose from launch file, we need initial pose for relocalization
    std::string pose_str;
    if (node_handle_.getParam("start_pose", pose_str) && !pose_str.empty())
    {
        const auto v = SplitString(pose_str, ',');
        if (v.size() != 3)
        {
            LOG(ERROR) << "Only support x y yaw";
            exit(-1);
        }
        options_.initial_pose =
            Eigen::Vector3d(std::stod(v[0]), std::stod(v[1]), std::stod(v[2]));
    }
    else
    {
        options_.initial_pose = Eigen::Vector3d(0., 0., 0.);
    }

    LOG(INFO) << "Start pose: " << options_.initial_pose;

    if (!node_handle_.getParam("map_path", options_.map_path))
    {
        LOG(ERROR) << "Can not get path, you must set path for mapping and localization!!";
        exit(-1);
    }
    LOG(INFO) << "Map path: " << options_.map_path;

    node_handle_.getParam("result_path", options_.result_path);
    LOG(INFO) << "Result path: " << options_.result_path;

    if (!node_handle_.getParam("use_imu", options_.use_imu))
    {
        options_.use_imu = false;
    }
    options_.use_imu = false;
    LOG(INFO) << "IMU will be supported in the future ...";

    if (!node_handle_.getParam("use_laser", options_.use_laser))
    {
        options_.use_laser = true;
    }
    LOG(INFO) << "Use laser to detect reflector: " << options_.use_laser;

    if (!node_handle_.getParam("use_point_cloud", options_.use_point_cloud))
    {
        options_.use_point_cloud = false;
    }
    LOG(INFO) << "Use point cloud to detect reflector: " << options_.use_point_cloud;

    double odometry_linear_coviance;
    if (node_handle_.getParam("linear_velocity_cov", odometry_linear_coviance))
    {
        options_.linear_velocity_cov = odometry_linear_coviance * odometry_linear_coviance;
    }
    else
    {
        options_.linear_velocity_cov = 0.05 * 0.05;
    }
    LOG(INFO) << "Linear velocity covariance is : " << std::sqrt(options_.linear_velocity_cov);

    double augular_coviance;
    if (node_handle_.getParam("angular_velocity_cov", augular_coviance))
    {
        options_.angular_velocity_cov = augular_coviance * augular_coviance;
    }
    else
    {
        options_.angular_velocity_cov = 0.08 * 0.08;
    }
    LOG(INFO) << "Angular velocity covariance is : " << std::sqrt(options_.angular_velocity_cov);

    double obervation_coviance;
    if (node_handle_.getParam("obervation_cov", obervation_coviance))
    {
        options_.observation_cov = obervation_coviance * obervation_coviance;
    }
    else
    {
        options_.observation_cov = 0.08 * 0.08;
    }
    LOG(INFO) << "Observation covariance is : " << std::sqrt(options_.observation_cov);

    double intensity_min;
    if (node_handle_.getParam("intensity_min", intensity_min))
    {
        options_.intensity_min = intensity_min;
    }
    else
    {
        options_.intensity_min = 160.;
    }
    LOG(INFO) << "Reflector detect intensity min value is : " << options_.intensity_min;

    double reflector_min_length;
    if (node_handle_.getParam("reflector_min_length", reflector_min_length))
    {
        options_.reflector_min_length = reflector_min_length;
    }
    else
    {
        options_.reflector_min_length = 0.18;
    }
    LOG(INFO) << "Reflector detect length min value is : " << options_.reflector_min_length;

    double reflector_length_error;
    if (node_handle_.getParam("reflector_length_error", reflector_length_error))
    {
        options_.reflector_length_error = reflector_length_error;
    }
    else
    {
        options_.reflector_length_error = 0.06;
    }
    LOG(INFO) << "Reflector detect length error value is : " << options_.reflector_length_error;

    float range_min;
    if (node_handle_.getParam("range_min", range_min))
    {
        options_.range_min = range_min;
    }
    else
    {
        options_.range_min = 0.3;
    }

    float range_max;
    if (node_handle_.getParam("range_max", range_max))
    {
        options_.range_max = range_max;
    }
    else
    {
        options_.range_max = 10.;
    }
    LOG(INFO) << "Laser Reflector detect used range: [ " << options_.range_min << "," << options_.range_max << "]";

    std::string extra_pose_str;
    if (node_handle_.getParam("sensor_to_base_link", extra_pose_str) && !extra_pose_str.empty())
    {
        const auto v = SplitString(extra_pose_str, ',');
        if (v.size() != 3)
        {
            LOG(ERROR) << "Only support x y yaw";
            exit(-1);
        }
        options_.sensor_to_base_link =
            transform::Rigid3d({stod(v[0]), stod(v[1]), 0.}, transform::RollPitchYaw(0., 0., stod(v[2])));
    }
    else
    {
        options_.sensor_to_base_link = transform::Rigid3d({0.13686, 0., 0.}, transform::RollPitchYaw(0., 0., 0.));
    }

    std::string odom_model;
    node_handle_.getParam("odom_model", odom_model);
    if(odom_model == "omni")
    {
        options_.odom_model = sensor::OdometryModel::OMNI;
        LOG(INFO) << "Use OMNI odometry model";
    }else
    {
        options_.odom_model = sensor::OdometryModel::DIFF;
        LOG(INFO) << "Use DIFF odometry model";
    }

}

void Node::ScanCallback(const sensor_msgs::LaserScanConstPtr &scan_ptr)
{
    const double time = scan_ptr->header.stamp.toSec();
    if (!slam_)
    {
        ekf::EKFOptions options;
        options.use_imu = options_.use_imu;
        options.init_time = time;
        options.init_pose = options_.initial_pose;
        options.map_path = options_.map_path;
        options.odom_model = options_.odom_model;
        options.linear_velocity_cov = options_.linear_velocity_cov;
        options.angular_velocity_cov = options_.angular_velocity_cov;
        options.observation_cov = options_.observation_cov;
        slam_ = common::make_unique<ekf::ReflectorEKFSLAM>(options);
        global_reflector_markers_ = ReflectorToRosMarkers(slam_->GetGlobalMap());
    }
    else
    {
        if (!laser_reflector_detector_)
        {
            LOG(ERROR) << "Laser reflector detector should be init first";
            exit(-1);
        }
        const auto observation = laser_reflector_detector_->HandleLaserScan(scan_ptr);
        slam_->HandleObservationMessage(observation);
        if (!observation.cloud_.empty())
        {
            const ekf::State state = slam_->GetState();

            /* publish  landmarks */
            visualization_msgs::MarkerArray markers = ReflectorToRosMarkers(state);
            landmark_publisher_.publish(markers);
            // publish global marker
            global_reflector_publisher_.publish(global_reflector_markers_);

            /* publish  robot pose */
            geometry_msgs::PoseWithCovarianceStamped robot_pose = StatePosetoRosPose(state);
            robot_pose.header.stamp = scan_ptr->header.stamp;
            pose_publisher_.publish(robot_pose);

            /* publish path */
            ekf_path_.header.stamp = scan_ptr->header.stamp;
            ekf_path_.header.frame_id = "world";
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = scan_ptr->header.stamp;
            pose.pose.position.x = state.mu(0);
            pose.pose.position.y = state.mu(1);
            const double theta = state.mu(2);
            pose.pose.orientation.x = 0.;
            pose.pose.orientation.y = 0.;
            pose.pose.orientation.z = std::sin(theta / 2);
            pose.pose.orientation.w = std::cos(theta / 2);
            ekf_path_.poses.push_back(pose);
            path_publisher_.publish(ekf_path_);
        }
    }
}

void Node::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &points_ptr)
{
    const double time = points_ptr->header.stamp.toSec();
    if (!slam_)
    {
        ekf::EKFOptions options;
        options.use_imu = options_.use_imu;
        options.init_time = time;
        options.init_pose = options_.initial_pose;
        options.map_path = options_.map_path;
        options.odom_model = options_.odom_model;
        options.linear_velocity_cov = options_.linear_velocity_cov;
        options.angular_velocity_cov = options_.angular_velocity_cov;
        options.observation_cov = options_.observation_cov;
        slam_ = common::make_unique<ekf::ReflectorEKFSLAM>(options);
        global_reflector_markers_ = ReflectorToRosMarkers(slam_->GetGlobalMap());
    }
    else
    {
        if (!point_cloud_reflector_detector_)
        {
            LOG(ERROR) << "Point cloud reflector detector should be init first";
            exit(-1);
        }
        const auto observation = point_cloud_reflector_detector_->HandlePointCloud(points_ptr);
        slam_->HandleObservationMessage(observation);
        if (!observation.cloud_.empty())
        {
            const ekf::State state = slam_->GetState();

            /* publish  landmarks */
            visualization_msgs::MarkerArray markers = ReflectorToRosMarkers(state);
            landmark_publisher_.publish(markers);
            // publish global marker
            global_reflector_publisher_.publish(global_reflector_markers_);

            /* publish  robot pose */
            geometry_msgs::PoseWithCovarianceStamped robot_pose = StatePosetoRosPose(state);
            robot_pose.header.stamp = points_ptr->header.stamp;
            pose_publisher_.publish(robot_pose);

            /* publish path */
            ekf_path_.header.stamp = points_ptr->header.stamp;
            ekf_path_.header.frame_id = "world";
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = points_ptr->header.stamp;
            pose.pose.position.x = state.mu(0);
            pose.pose.position.y = state.mu(1);
            const double theta = state.mu(2);
            pose.pose.orientation.x = 0.;
            pose.pose.orientation.y = 0.;
            pose.pose.orientation.z = std::sin(theta / 2);
            pose.pose.orientation.w = std::cos(theta / 2);
            ekf_path_.poses.push_back(pose);
            path_publisher_.publish(ekf_path_);
        }
    }
}

void Node::OdometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    const auto odom = ToOdometryData(*msg);
    if (options_.use_laser && laser_reflector_detector_)
    {
        laser_reflector_detector_->HandleOdometryData(odom);
    }

    if (slam_)
    {
        slam_->HandleOdometryMessage(odom);
        const ekf::State state = slam_->GetState();
        /* publish  robot pose */
        geometry_msgs::PoseWithCovarianceStamped robot_pose = StatePosetoRosPose(state);
        robot_pose.header.stamp = msg->header.stamp;
        pose_publisher_.publish(robot_pose);

        /* publish path */
        ekf_path_.header.stamp = msg->header.stamp;
        ekf_path_.header.frame_id = "world";
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = msg->header.stamp;
        pose.pose.position.x = state.mu(0);
        pose.pose.position.y = state.mu(1);
        const double theta = state.mu(2);
        pose.pose.orientation.x = 0.;
        pose.pose.orientation.y = 0.;
        pose.pose.orientation.z = std::sin(theta / 2);
        pose.pose.orientation.w = std::cos(theta / 2);
        ekf_path_.poses.push_back(pose);
        path_publisher_.publish(ekf_path_);
    }
}

sensor::OdometryData Node::ToOdometryData(const nav_msgs::Odometry &msg)
{
    sensor::OdometryData result;
    result.time = msg.header.stamp.toSec();
    result.position = Eigen::Vector3d(msg.pose.pose.position.x,
                                      msg.pose.pose.position.y, msg.pose.pose.position.z);
    result.orientation = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    result.linear_velocity = Eigen::Vector3d(msg.twist.twist.linear.x,
                                             msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    result.angular_velocity = Eigen::Vector3d(msg.twist.twist.angular.x,
                                              msg.twist.twist.angular.y, msg.twist.twist.angular.z);
    return result;
}

visualization_msgs::MarkerArray Node::ReflectorToRosMarkers(const sensor::Map &map, const double &scale)
{
    visualization_msgs::MarkerArray markers;
    const sensor::PointCloud &reflector_map = map.reflector_map_;
    const sensor::PointCloudCoviarance &reflector_map_coviarance = map.reflector_map_coviarance_;
    const int global_reflector_number = reflector_map.size();
    if (global_reflector_number == 0)
    {
        LOG(INFO) << "No global reflector";
        return markers;
    }
    LOG(INFO) << "Global reflector size is : " << global_reflector_number;
    for (int i = 0; i < global_reflector_number; i++)
    {
        const double mx = reflector_map[i].x();
        const double my = reflector_map[i].y();

        /* 计算地图点的协方差椭圆角度以及轴长 */
        const Eigen::Matrix2d sigma_m = reflector_map_coviarance[i]; //协方差
        // Calculate Eigen Value(D) and Vectors(V), simga_m = V * D * V^-1
        // D = | D1 0  |  V = |cos  -sin|
        //     | 0  D2 |      |sin  cos |
        Eigen::EigenSolver<Eigen::Matrix2d> eigen_solver(sigma_m);
        const auto eigen_value = eigen_solver.pseudoEigenvalueMatrix();
        const auto eigen_vector = eigen_solver.pseudoEigenvectors();
        // Calculate angle and x y
        const double angle = std::atan2(eigen_vector(1, 0), eigen_vector(0, 0));
        const double x_len = 2 * std::sqrt(eigen_value(0, 0) * 5.991);
        const double y_len = 2 * std::sqrt(eigen_value(1, 1) * 5.991);

        /* 构造marker */
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "ekf_slam";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0.;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = std::sin(angle / 2);
        marker.pose.orientation.w = std::cos(angle / 2);
        marker.scale.x = scale * x_len;
        marker.scale.y = scale * y_len;
        marker.scale.z = 0.1 * scale * (x_len + y_len);
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);
    } // for all mpts

    return markers;
}

visualization_msgs::MarkerArray Node::ReflectorToRosMarkers(const ekf::State &state, const double &scale)
{
    visualization_msgs::MarkerArray markers;
    const int N = state.mu.rows();
    if (N == 3)
    {
        LOG(INFO) << "No reflector detected";
        return markers;
    }

    const int M = (N - 3) / 2;
    LOG(INFO) << "Now reflector size is : " << M;
    for (int i = 0; i < M; i++)
    {
        const int id = 3 + 2 * i;
        const double mx = state.mu(id);
        const double my = state.mu(id + 1);

        /* 计算地图点的协方差椭圆角度以及轴长 */
        const Eigen::Matrix2d sigma_m = state.sigma.block(id, id, 2, 2); //协方差
        // Calculate Eigen Value(D) and Vectors(V), simga_m = V * D * V^-1
        // D = | D1 0  |  V = |cos  -sin|
        //     | 0  D2 |      |sin  cos |
        Eigen::EigenSolver<Eigen::Matrix2d> eigen_solver(sigma_m);
        const auto eigen_value = eigen_solver.pseudoEigenvalueMatrix();
        const auto eigen_vector = eigen_solver.pseudoEigenvectors();
        // Calculate angle and x y
        const double angle = std::atan2(eigen_vector(1, 0), eigen_vector(0, 0));
        const double x_len = 2 * std::sqrt(eigen_value(0, 0) * 5.991);
        const double y_len = 2 * std::sqrt(eigen_value(1, 1) * 5.991);

        /* 构造marker */
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time(state.time);
        marker.ns = "ekf_slam";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0.;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = std::sin(angle / 2);
        marker.pose.orientation.w = std::cos(angle / 2);
        marker.scale.x = scale * x_len;
        marker.scale.y = scale * y_len;
        marker.scale.z = 0.1 * scale * (x_len + y_len);
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);
    } // for all mpts

    return markers;
}

geometry_msgs::PoseWithCovarianceStamped Node::StatePosetoRosPose(const ekf::State &state)
{
    /* 转换带协方差的机器人位姿 */
    geometry_msgs::PoseWithCovarianceStamped rpose;
    rpose.header.frame_id = "world";

    rpose.pose.pose.position.x = state.mu(0);
    rpose.pose.pose.position.y = state.mu(1);
    rpose.pose.pose.orientation.x = 0.0;
    rpose.pose.pose.orientation.y = 0.0;
    rpose.pose.pose.orientation.z = std::sin(state.mu(2) / 2);
    rpose.pose.pose.orientation.w = std::cos(state.mu(2) / 2);

    rpose.pose.covariance.at(0) = state.sigma(0, 0);
    rpose.pose.covariance.at(1) = state.sigma(0, 1);
    rpose.pose.covariance.at(5) = state.sigma(0, 2);
    rpose.pose.covariance.at(6) = state.sigma(1, 0);
    rpose.pose.covariance.at(7) = state.sigma(1, 1);
    rpose.pose.covariance.at(11) = state.sigma(1, 2);
    rpose.pose.covariance.at(30) = state.sigma(2, 0);
    rpose.pose.covariance.at(31) = state.sigma(2, 1);
    rpose.pose.covariance.at(35) = state.sigma(2, 2);
    return rpose;
}

