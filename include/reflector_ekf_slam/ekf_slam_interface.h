#ifndef EKF_SLAM_INTERFACE_H
#define EKF_SLAM_INTERFACE_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string>
#include <memory>
#include <cstddef>
#include <type_traits>
#include "common/common.h"
#include "sensor/sensor_data.h"

namespace ekf
{

struct ReflectorMatchResult
{
  // map - observation matched id
  std::vector<std::pair<int, int>> map_obs_match_ids;
  // mu_ - observation matched id
  std::vector<std::pair<int, int>> state_obs_match_ids;
  // new observed reflector id
  std::vector<int> new_ids;
};

struct EKFOptions
{
  bool use_imu;
  double init_time;
  Eigen::Vector3d init_pose;
  std::string map_path;
  sensor::OdometryModel odom_model;
  // 0.05 * 0.05
  double linear_velocity_cov;
  // 0.068 * 0.068
  double angular_velocity_cov;
  // 0.05 * 0.05
  double observation_cov;
};

struct State
{
  double time;
  Eigen::VectorXd mu;
  Eigen::MatrixXd sigma;
};

class ReflectorEKFSLAMInterface
{
public:
  ReflectorEKFSLAMInterface() {}
  virtual ~ReflectorEKFSLAMInterface() {}

  virtual void HandleOdometryMessage(const sensor::OdometryData &odometry) = 0;
  virtual void HandleImuMessage(const sensor::ImuData &imu) = 0;
  virtual void HandleObservationMessage(const sensor::Observation &observation) = 0;

  virtual Eigen::VectorXd &GetStateVector() = 0;
  virtual Eigen::MatrixXd &GetCoviarance() = 0;
  virtual double GetLatestTime() = 0;
  virtual State GetState() = 0;
  virtual sensor::Map GetGlobalMap() = 0;
};
} // namespace ekf

#endif // EKF_SLAM_INTERFACE_H
