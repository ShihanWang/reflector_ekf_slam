#ifndef REFLECTOR_EKF_SLAM_REFLECTOR_EKF_SLAM_GPS_H
#define REFLECTOR_EKF_SLAM_REFLECTOR_EKF_SLAM_GPS_H

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#include "reflector_ekf_slam/ekf_slam_interface.h"

namespace ekf
{
class ReflectorEKFSLAMGPS : public ReflectorEKFSLAMInterface
{
public:
  ReflectorEKFSLAMGPS(const EKFOptions &options);
  ReflectorEKFSLAMGPS() = delete;
  ~ReflectorEKFSLAMGPS() override;

  void HandleOdometryMessage(const sensor::OdometryData &odometry) override;
  void HandleImuMessage(const sensor::ImuData &imu) override;
  void HandleObservationMessage(const sensor::Observation &observation) override;
  State PredictState(const double &time) override;

  Eigen::VectorXd &GetStateVector() override
  {
    return state_.mu;
  }
  Eigen::MatrixXd &GetCoviarance() override
  {
    return state_.sigma;
  }
  double GetLatestTime() override
  {
    return state_.time;
  }
  State GetState() override
  {
    return state_;
  }
  sensor::Map GetGlobalMap() override
  {
    return map_;
  }

private:
  void LoadMapFromTxtFile(const std::string &file);

  ReflectorMatchResult ReflectorMatch(const sensor::Observation &obs);
  void Predict(const double &dt);

  EKFOptions options_;

  // include vx vy(maybe not exist) w
  Eigen::Vector3d vt_;

  Eigen::MatrixXd Qu_;
  Eigen::Matrix2d Qt_;

  /* 求解的扩展状态 均值 和 协方差 */
  State state_;

  sensor::Map map_;
};
} // namespace ekf
#endif // REFLECTOR_EKF_SLAM_REFLECTOR_EKF_SLAM_GPS_H
