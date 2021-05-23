#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>
#include <deque>
#include <mutex>
#include <memory>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <glog/logging.h>

#include "io/file_writer.h"
#include "io/submap_painter.h"
#include "io/image.h"
#include "mapping/map_limits.h"

#include "reflector_ekf_slam/ekf_slam_interface.h"
#include "reflector_ekf_slam/reflector_ekf_slam.h"
#include "reflector_detect/reflector_detect_interface.h"
#include "reflector_detect/laser/laser_reflector_detect.h"
#include "reflector_detect/point_cloud/point_cloud_reflector_detect.h"
#include "common/common.h"
#include "common/time.h"
#include "sensor/sensor_data.h"
#include "transform/rigid_transform.h"
#include "mapping/map_builder.h"

#include "reflector_ekf_slam/save_map.h"

class Node
{
public:
  Node();
  ~Node();

private:
  void ScanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr &msg);
  visualization_msgs::MarkerArray ReflectorToRosMarkers(const ekf::State &state, const double &scale = 3.5);
  visualization_msgs::MarkerArray ReflectorToRosMarkers(const sensor::Map &map, const double &scale = 3.5);
  geometry_msgs::PoseWithCovarianceStamped StatePosetoRosPose(const ekf::State &state);
  sensor::OdometryData ToOdometryData(const nav_msgs::Odometry &msg);
  sensor_msgs::PointCloud ToPointCloud(const sensor::RangeData &range_data);
  void PublishMap(const ros::WallTimerEvent &timer_event);
  bool HandleSaveMap(
      reflector_ekf_slam::save_map::Request &request,
      reflector_ekf_slam::save_map::Response &response);

  void LoadNodeOptions();
  void SaveReflectorResult(const std::string &filebase);
  ros::Time ToRos(const common::Time time);
  common::Time FromRos(const ros::Time &time);
  void WritePgm(const io::Image &image, const double resolution, io::FileWriter *file_writer);

  // Write the corresponding yaml into 'file_writer'.
  void WriteYaml(const double resolution, const Eigen::Vector2d &origin,
                 const std::string &pgm_filename,
                 io::FileWriter *file_writer);
  std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
      const io::PaintSubmapSlicesResult &painted_slices,
      const double resolution, const std::string &frame_id,
      const ros::Time &time);

  struct NodeOptions
  {
    bool use_laser;
    bool use_point_cloud;
    bool use_imu;
    std::string scan_topic_name;
    std::string points_topic_name;
    std::string odom_topic_name;
    Eigen::Vector3d initial_pose;
    std::string map_path;
    std::string result_path;
    sensor::OdometryModel odom_model;
    double linear_velocity_cov;
    double angular_velocity_cov;
    double observation_cov;
    double intensity_min;
    double reflector_min_length;
    double reflector_length_error;
    double map_publish_period_sec;
    float range_min;
    float range_max;
    transform::Rigid3d sensor_to_base_link;
    mapping::MapBuilderOptions map_builder_options;
  };

  ros::NodeHandle node_handle_;
  ros::Publisher landmark_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher global_reflector_publisher_;
  ros::Publisher occupancy_grid_publisher_;
  ros::Publisher matched_point_cloud_publisher_;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber laser_subscriber_;
  ros::Subscriber point_cloud_subscriber_;

  ros::ServiceServer save_map_service_;

  ros::WallTimer wall_timer_;

  nav_msgs::Path ekf_path_;
  visualization_msgs::MarkerArray global_reflector_markers_;

  std::mutex map_builder_mutex_;
  std::mutex slam_mutex_;
  std::mutex odometry_mutex_;
  std::deque<sensor::OdometryData> odometry_data_;

  NodeOptions options_;
  std::unique_ptr<ekf::ReflectorEKFSLAMInterface> slam_;
  std::unique_ptr<reflector_detect::ReflectorDetectInterface> laser_reflector_detector_;
  std::unique_ptr<reflector_detect::ReflectorDetectInterface> point_cloud_reflector_detector_;
  std::unique_ptr<mapping::MapBuilder> map_builder_;
};

#endif // NODE_H
