#ifndef REFLECTOR_DETECT_REFLECTOR_DETECT_INTERFACE_H
#define REFLECTOR_DETECT_REFLECTOR_DETECT_INTERFACE_H

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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include "transform/rigid_transform.h"
#include "transform/transform.h"

namespace reflector_detect
{

class ReflectorDetectInterface
{
public:
  ReflectorDetectInterface() {}
  virtual ~ReflectorDetectInterface() {}
  virtual void SetSensorToBaseLinkTransform(const transform::Rigid3d &pose) { sensor_to_base_link_transform_ = pose; }
  virtual sensor::Observation HandleLaserScan(const sensor_msgs::LaserScanConstPtr &msg) { return sensor::Observation(); }
  virtual sensor::Observation HandlePointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) { return sensor::Observation(); }

  virtual void HandleOdometryData(const sensor::OdometryData &msg) {}
  virtual void HandleImuData(const sensor::ImuData &msg) {}

protected:
  transform::Rigid3d sensor_to_base_link_transform_;
};
} // namespace reflector_detect

#endif // REFLECTOR_DETECT_REFLECTOR_DETECT_INTERFACE_H
