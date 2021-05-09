#ifndef SENSOR_SENSOR_DATA_H
#define SENSOR_SENSOR_DATA_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "common/common.h"
#include "transform/transform.h"
#include "transform/rigid_transform.h"

namespace sensor
{
// xy
using PointCloud = std::vector<Eigen::Vector2f>;
using PointCloudCoviarance = std::vector<Eigen::Matrix2d>;
// xy time
using TimedPointCloud = std::vector<Eigen::Vector3f>;

class Observation
{
public:
  Observation() {}
  Observation(const double &time, const PointCloud &cloud) : time_(time), cloud_(cloud) {}
  double time_;
  PointCloud cloud_;
};

class Map
{
public:
  Map() {}
  Map(const PointCloud &cloud, const PointCloudCoviarance &cov) : reflector_map_(cloud), reflector_map_coviarance_(cov) {}
  PointCloud reflector_map_;
  PointCloudCoviarance reflector_map_coviarance_;
};

struct OdometryData
{
  double time;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
};

struct ImuData
{
  double time;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angulear_velocity;
};

enum OdometryModel
{
  DIFF,
  OMNI
};

inline PointCloud TransformPointCloud(const PointCloud &points, const transform::Rigid2f &pose)
{
  PointCloud result;
  for (const auto &p : points)
  {
    result.push_back(pose * p);
  }
  return result;
}

} // namespace sensor

#endif // SENSOR_SENSOR_DATA_H