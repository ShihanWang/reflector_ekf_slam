#include "ros_node.h"
#include "sensor/sensor_data.h"
#include <geometry_msgs/Point32.h>

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
    occupancy_grid_publisher_ =
        node_handle_.advertise<nav_msgs::OccupancyGrid>("ekf_slam/map", 1);
    matched_point_cloud_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud>("ekf_slam/matched_points", 1);

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

    wall_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(options_.map_publish_period_sec),
        &Node::PublishMap, this);

    save_map_service_ =
        node_handle_.advertiseService(
            "reflector_ekf_slam/save_map", &Node::HandleSaveMap, this);

    map_builder_ =
        common::make_unique<mapping::MapBuilder>(options_.map_builder_options);

    LOG(INFO) << "Reflector SLAM is start !!!!";
    ros::spin();
}

Node::~Node()
{
}

void Node::SaveReflectorResult(const std::string &filebase)
{
    if (!slam_)
        return;

    std::string map_path = filebase + ".txt";
    const ekf::State state = slam_->GetState();
    const sensor::Map map = slam_->GetGlobalMap();
    LOG(INFO) << "Start to save reflector map in " << map_path;
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
    LOG(INFO) << "Finish to save relfector map";
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

    if (!node_handle_.getParam("map_publish_period_sec", options_.map_publish_period_sec))
    {
        options_.map_publish_period_sec = 1.0;
    }
    LOG(INFO) << "Map publish period seconds: " << options_.map_publish_period_sec;

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

    if (!node_handle_.getParam("intensity_min", options_.intensity_min))
    {
        options_.intensity_min = 160.;
    }
    LOG(INFO) << "Reflector detect intensity min value is : " << options_.intensity_min;

    if (!node_handle_.getParam("reflector_min_length", options_.reflector_min_length))
    {
        options_.reflector_min_length = 0.18;
    }
    LOG(INFO) << "Reflector detect length min value is : " << options_.reflector_min_length;

    if (!node_handle_.getParam("reflector_length_error", options_.reflector_length_error))
    {
        options_.reflector_length_error = 0.06;
    }
    LOG(INFO) << "Reflector detect length error value is : " << options_.reflector_length_error;

    if (!node_handle_.getParam("range_min", options_.range_min))
    {
        options_.range_min = 0.3;
    }
    if (!node_handle_.getParam("range_max", options_.range_max))
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
    if (odom_model == "omni")
    {
        options_.odom_model = sensor::OdometryModel::OMNI;
        LOG(INFO) << "Use OMNI odometry model";
    }
    else
    {
        options_.odom_model = sensor::OdometryModel::DIFF;
        LOG(INFO) << "Use DIFF odometry model";
    }

    // Load map builder options
    if (!node_handle_.getParam("resolution", options_.map_builder_options.resolution))
    {
        options_.map_builder_options.resolution = 0.05;
    }
    LOG(INFO) << "Map resolution: " << options_.map_builder_options.resolution;

    if (!node_handle_.getParam("voxel_filter_size", options_.map_builder_options.voxel_filter_size))
    {
        options_.map_builder_options.voxel_filter_size = 0.025;
    }
    LOG(INFO) << "Voxel filter size: " << options_.map_builder_options.voxel_filter_size;

    sensor::AdaptiveVoxelFilterOptions adaptive_voxel_filter_options;
    if (!node_handle_.getParam("adaptive_voxel_filter_max_length", adaptive_voxel_filter_options.max_length))
    {
        adaptive_voxel_filter_options.max_length = 0.9;
    }
    if (!node_handle_.getParam("adaptive_voxel_filter_min_num_points", adaptive_voxel_filter_options.min_num_points))
    {
        adaptive_voxel_filter_options.min_num_points = 500;
    }
    if (!node_handle_.getParam("adaptive_voxel_filter_max_range", adaptive_voxel_filter_options.max_range))
    {
        adaptive_voxel_filter_options.max_range = 100.;
    }
    options_.map_builder_options.adaptive_voxel_options = adaptive_voxel_filter_options;
    LOG(INFO) << "Adaptive voxel filter size: { \n  max_length = " << adaptive_voxel_filter_options.max_length
              << ",\n  min_num_points = " << adaptive_voxel_filter_options.min_num_points << ",\n max_range = " << adaptive_voxel_filter_options.max_range << "\n}";

    scan_matching::RealTimeCorrelativeScanMatcherOptions real_time_scan_matcher_options;
    if (!node_handle_.getParam("real_time_csm_linear_search_window", real_time_scan_matcher_options.linear_search_window))
    {
        real_time_scan_matcher_options.linear_search_window = 0.2;
    }
    if (!node_handle_.getParam("real_time_csm_angular_search_window", real_time_scan_matcher_options.angular_search_window))
    {
        real_time_scan_matcher_options.angular_search_window = 15.;
    }
    real_time_scan_matcher_options.angular_search_window *= M_PI / 180.;
    if (!node_handle_.getParam("real_time_csm_translation_delta_cost_weight", real_time_scan_matcher_options.translation_delta_cost_weight))
    {
        real_time_scan_matcher_options.translation_delta_cost_weight = 1e-1;
    }
    if (!node_handle_.getParam("real_time_csm_rotation_delta_cost_weight", real_time_scan_matcher_options.rotation_delta_cost_weight))
    {
        real_time_scan_matcher_options.rotation_delta_cost_weight = 1e-1;
    }
    options_.map_builder_options.real_time_scan_matcher_options = real_time_scan_matcher_options;
    LOG(INFO) << "Real time scan matcher options: { \n  linear_search_window = " << real_time_scan_matcher_options.linear_search_window
              << ",\n  angular_search_window = " << real_time_scan_matcher_options.angular_search_window << ",\n translation_delta_cost_weight = " << real_time_scan_matcher_options.translation_delta_cost_weight << ",\n  rotation_delta_cost_weight = " << real_time_scan_matcher_options.rotation_delta_cost_weight << "\n}";

    scan_matching::CeresScanMatcherOptions2D ceres_scan_matcher_options;
    if (!node_handle_.getParam("ceres_scan_matcher_occupied_space_weight", ceres_scan_matcher_options.occupied_space_weight))
    {
        ceres_scan_matcher_options.occupied_space_weight = 1.;
    }
    if (!node_handle_.getParam("ceres_scan_matcher_translation_weight", ceres_scan_matcher_options.translation_weight))
    {
        ceres_scan_matcher_options.translation_weight = 0.1;
    }
    if (!node_handle_.getParam("ceres_scan_matcher_rotation_weight", ceres_scan_matcher_options.rotation_weight))
    {
        ceres_scan_matcher_options.rotation_weight = 0.4;
    }

    ceres::Solver::Options ceres_solver_options;
    if (!node_handle_.getParam("use_nonmonotonic_steps", ceres_solver_options.use_nonmonotonic_steps))
    {
        ceres_solver_options.use_nonmonotonic_steps = true;
    }
    if (!node_handle_.getParam("max_num_iterations", ceres_solver_options.max_num_iterations))
    {
        ceres_solver_options.max_num_iterations = 100;
    }
    if (!node_handle_.getParam("num_threads", ceres_solver_options.num_threads))
    {
        ceres_solver_options.num_threads = 2;
    }
    ceres_solver_options.linear_solver_type = ceres::DENSE_QR;

    ceres_scan_matcher_options.ceres_solver_options = ceres_solver_options;
    options_.map_builder_options.ceres_scan_matcher_options = ceres_scan_matcher_options;
    LOG(INFO) << "Ceres scan matcher options: { \n  occupied_space_weight = " << ceres_scan_matcher_options.occupied_space_weight
              << ",\n  translation_weight = " << ceres_scan_matcher_options.translation_weight << ",\n rotation_weight = " << ceres_scan_matcher_options.rotation_weight << "\n}";
    LOG(INFO) << "Ceres solver options: { \n  use_nonmonotonic_steps = " << ceres_solver_options.use_nonmonotonic_steps << ",\n  max_num_iterations = " << ceres_solver_options.max_num_iterations << ",\n  num_threads = " << ceres_solver_options.num_threads << ",\n  linear_solver_type = ceres::DENSE_QR \n}";

    mapping::ProbabilityGridRangeDataInserterOptions2D grid_data_inserter_options;
    if (!node_handle_.getParam("grid_data_inserter_insert_free_space", grid_data_inserter_options.insert_free_space))
    {
        grid_data_inserter_options.insert_free_space = true;
    }
    if (!node_handle_.getParam("grid_data_inserter_hit_probability", grid_data_inserter_options.hit_probability))
    {
        grid_data_inserter_options.hit_probability = 0.55;
    }
    if (!node_handle_.getParam("grid_data_inserter_miss_probability", grid_data_inserter_options.miss_probability))
    {
        grid_data_inserter_options.miss_probability = 0.49;
    }
    options_.map_builder_options.range_data_inserter_options = grid_data_inserter_options;
    LOG(INFO) << "Range data inserter options: { \n  insert_free_space = " << grid_data_inserter_options.insert_free_space
              << ",\n  hit_probability = " << grid_data_inserter_options.hit_probability << ",\n miss_probability = " << grid_data_inserter_options.miss_probability << "\n}";
}

sensor_msgs::PointCloud Node::ToPointCloud(const sensor::RangeData &range_data)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
    if (range_data.returns.empty())
        return cloud;
    for (const auto &point : range_data.returns)
    {
        geometry_msgs::Point32 p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.;
        cloud.points.push_back(p);
    }
    return cloud;
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
#ifndef USE_GPS
        slam_ = common::make_unique<ekf::ReflectorEKFSLAM>(options);
#else
        slam_ = common::make_unique<ekf::ReflectorEKFSLAMGPS>(options);
#endif
        global_reflector_markers_ = ReflectorToRosMarkers(slam_->GetGlobalMap());
    }
    else
    {
        if (!laser_reflector_detector_)
        {
            LOG(ERROR) << "Laser reflector detector should be init first";
            exit(-1);
        }
        auto observation = laser_reflector_detector_->HandleLaserScan(scan_ptr);
#ifdef USE_GPS
        ekf::State state;
        const sensor::RangeData range_data = laser_reflector_detector_->GetRangeData();
        {
            std::lock_guard<std::mutex> lock_slam(slam_mutex_);
            state = slam_->PredictState(scan_ptr->header.stamp.toSec());
        }
        const Eigen::Vector3d translation(state.mu(0), state.mu(1), 0.);
        const Eigen::Quaterniond rotation(std::cos(state.mu(2) / 2), 0., 0., std::sin(state.mu(2) / 2));
        transform::Rigid3d ekf_pose(translation, rotation);
        common::Time now_time = FromRos(scan_ptr->header.stamp);
        std::unique_ptr<mapping::MatchingResult> match_result;
        {
            std::lock_guard<std::mutex> lock(map_builder_mutex_);
            match_result = map_builder_->AddRangeData(now_time, range_data, ekf_pose);
        }
        if (match_result)
        {
            LOG(INFO) << "Match pose: " << transform::Project2D(match_result->local_pose).DebugString();
            sensor_msgs::PointCloud cloud = ToPointCloud(match_result->range_data_in_local);
            matched_point_cloud_publisher_.publish(cloud);
            observation.gps_pose_ =
                common::make_unique<transform::Rigid2d>(transform::Project2D(match_result->local_pose));
        }
        ekf::State latest_state;
        {
            std::lock_guard<std::mutex> lock_slam(slam_mutex_);
            slam_->HandleObservationMessage(observation);
            latest_state = slam_->GetState();
        }

        if (!observation.cloud_.empty())
        {
            /* publish  landmarks */
            visualization_msgs::MarkerArray markers = ReflectorToRosMarkers(latest_state);
            landmark_publisher_.publish(markers);
            // publish global marker
            global_reflector_publisher_.publish(global_reflector_markers_);

            /* publish  robot pose */
            geometry_msgs::PoseWithCovarianceStamped robot_pose = StatePosetoRosPose(latest_state);
            robot_pose.header.stamp = scan_ptr->header.stamp;
            pose_publisher_.publish(robot_pose);

            /* publish path */
            ekf_path_.header.stamp = scan_ptr->header.stamp;
            ekf_path_.header.frame_id = "world";
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = scan_ptr->header.stamp;
            pose.pose.position.x = latest_state.mu(0);
            pose.pose.position.y = latest_state.mu(1);
            const double theta = latest_state.mu(2);
            pose.pose.orientation.x = 0.;
            pose.pose.orientation.y = 0.;
            pose.pose.orientation.z = std::sin(theta / 2);
            pose.pose.orientation.w = std::cos(theta / 2);
            ekf_path_.poses.push_back(pose);
            path_publisher_.publish(ekf_path_);
        }
#else
        ekf::State state;
        {
            std::lock_guard<std::mutex> lock_slam(slam_mutex_);
            slam_->HandleObservationMessage(observation);
            state = slam_->GetState();
        }
        if (!observation.cloud_.empty())
        {
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
        const sensor::RangeData range_data = laser_reflector_detector_->GetRangeData();
        const Eigen::Vector3d translation(state.mu(0), state.mu(1), 0.);
        const Eigen::Quaterniond rotation(std::cos(state.mu(2) / 2), 0., 0., std::sin(state.mu(2) / 2));
        transform::Rigid3d ekf_pose(translation, rotation);
        common::Time now_time = FromRos(scan_ptr->header.stamp);
        std::lock_guard<std::mutex> lock(map_builder_mutex_);
        const auto match_result = map_builder_->AddRangeData(now_time, range_data, ekf_pose);
        if (match_result)
        {
            LOG(INFO) << "Match pose: " << transform::Project2D(match_result->local_pose).DebugString();
            sensor_msgs::PointCloud cloud = ToPointCloud(match_result->range_data_in_local);
            matched_point_cloud_publisher_.publish(cloud);
        }
#endif
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
        ekf::State state;
        {
            std::lock_guard<std::mutex> lock_slam(slam_mutex_);
            slam_->HandleObservationMessage(observation);
            state = slam_->GetState();
        }

        if (!observation.cloud_.empty())
        {
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

ros::Time Node::ToRos(const common::Time time)
{
    int64_t uts_timestamp = common::ToUniversal(time);
    int64_t ns_since_unix_epoch =
        (uts_timestamp -
         common::kUtsEpochOffsetFromUnixEpochInSeconds *
             10000000ll) *
        100ll;
    ros::Time ros_time;
    ros_time.fromNSec(ns_since_unix_epoch);
    return ros_time;
}

common::Time Node::FromRos(const ::ros::Time &time)
{
    // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
    // exactly 719162 days before the Unix epoch.
    return common::FromUniversal(
        (time.sec +
         common::kUtsEpochOffsetFromUnixEpochInSeconds) *
            10000000ll +
        (time.nsec + 50) / 100); // + 50 to get the rounding correct.
}

void Node::PublishMap(const ros::WallTimerEvent &timer_event)
{
    std::lock_guard<std::mutex> lock(map_builder_mutex_);
    if (!map_builder_)
        return;
    mapping::SubmapTexture response;
    if (!map_builder_->ToSubmapTexture(&response))
    {
        // LOG(WARNING) << "Wait for map data";
        return;
    }
    io::SubmapSlice submap_slice;
    mapping::ValueConversionTables value_tables;
    io::FillSubmapSlice(response.global_pose, response, &submap_slice, &value_tables);
    const auto result = io::PaintSubmapSlices(submap_slice, response.resolution);
    std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
        result, response.resolution, "world", ros::Time::now());
    occupancy_grid_publisher_.publish(*msg_ptr);
}

void Node::WritePgm(const io::Image &image, const double resolution,
                    io::FileWriter *file_writer)
{
    const std::string header = "P5\n# Cartographer map; " +
                               std::to_string(resolution) + " m/pixel\n" +
                               std::to_string(image.width()) + " " +
                               std::to_string(image.height()) + "\n255\n";
    file_writer->Write(header.data(), header.size());
    for (int y = 0; y < image.height(); ++y)
    {
        for (int x = 0; x < image.width(); ++x)
        {
            const char color = image.GetPixel(x, y)[0];
            file_writer->Write(&color, 1);
        }
    }
}

void Node::WriteYaml(const double resolution, const Eigen::Vector2d &origin,
                     const std::string &pgm_filename,
                     io::FileWriter *file_writer)
{
    // Magic constants taken directly from ros map_saver code:
    // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
    const std::string output =
        "image: " + pgm_filename + "\n" +
        "resolution: " + std::to_string(resolution) + "\n" + "origin: [" +
        std::to_string(origin.x()) + ", " + std::to_string(origin.y()) +
        ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    file_writer->Write(output.data(), output.size());
}

std::unique_ptr<nav_msgs::OccupancyGrid> Node::CreateOccupancyGridMsg(
    const io::PaintSubmapSlicesResult &painted_slices,
    const double resolution, const std::string &frame_id,
    const ros::Time &time)
{
    auto occupancy_grid = common::make_unique<nav_msgs::OccupancyGrid>();

    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height =
        cairo_image_surface_get_height(painted_slices.surface.get());

    occupancy_grid->header.stamp = time;
    occupancy_grid->header.frame_id = frame_id;
    occupancy_grid->info.map_load_time = time;
    occupancy_grid->info.resolution = resolution;
    occupancy_grid->info.width = width;
    occupancy_grid->info.height = height;
    occupancy_grid->info.origin.position.x =
        -painted_slices.origin.x() * resolution;
    occupancy_grid->info.origin.position.y =
        (-height + painted_slices.origin.y()) * resolution;
    occupancy_grid->info.origin.position.z = 0.;
    occupancy_grid->info.origin.orientation.w = 1.;
    occupancy_grid->info.origin.orientation.x = 0.;
    occupancy_grid->info.origin.orientation.y = 0.;
    occupancy_grid->info.origin.orientation.z = 0.;

    const uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
        cairo_image_surface_get_data(painted_slices.surface.get()));
    occupancy_grid->data.reserve(width * height);
    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value =
                observed == 0
                    ? -1
                    : common::RoundToInt((1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(100, value);
            occupancy_grid->data.push_back(value);
        }
    }

    return occupancy_grid;
}

bool Node::HandleSaveMap(
    reflector_ekf_slam::save_map::Request &request,
    reflector_ekf_slam::save_map::Response &response)
{
    if (!slam_ || !map_builder_)
    {
        response.flag = false;
        response.path = "SLAM is not received any data !!!!";
        return false;
    }
    std::string filebase = request.path;
    if (filebase.empty())
    {
        LOG(WARNING) << "Empty path for save map: " << filebase;
        filebase = options_.result_path;
        LOG(WARNING) << "Result will be saved at: " << filebase;
    }

    {
        std::lock_guard<std::mutex> lock_slam(slam_mutex_);
        SaveReflectorResult(filebase);
    }
    LOG(INFO) << "Start to write grid map";
    std::lock_guard<std::mutex> lock(map_builder_mutex_);
    mapping::SubmapTexture submap_response;
    if (!map_builder_->ToSubmapTexture(&submap_response))
    {
        LOG(WARNING) << "Map builder do not receive any data";
        response.flag = false;
        response.path = "Map builder do not receive any data !!!";
        return false;
    }
    io::SubmapSlice submap_slice;
    mapping::ValueConversionTables value_tables;
    io::FillSubmapSlice(submap_response.global_pose, submap_response, &submap_slice, &value_tables);
    auto result = io::PaintSubmapSlices(submap_slice, submap_response.resolution);

    io::StreamFileWriter pgm_writer(filebase + ".pgm");

    io::Image image(std::move(result.surface));
    WritePgm(image, submap_response.resolution, &pgm_writer);

    const Eigen::Vector2d origin(
        -result.origin.x() * submap_response.resolution,
        (result.origin.y() - image.height()) * submap_response.resolution);

    io::StreamFileWriter yaml_writer(filebase + ".yaml");
    WriteYaml(submap_response.resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
    LOG(INFO) << "Finish to write grid map";

    response.flag = true;
    response.path = filebase;

    return true;
}