#ifndef REFLECTOR_DETECT_LASER_LASER_REFLECTOR_DETECT_H
#define REFLECTOR_DETECT_LASER_LASER_REFLECTOR_DETECT_H
#include "reflector_detect/reflector_detect_interface.h"
#include "reflector_detect/pose_extrapolator_interface.h"

namespace reflector_detect
{
struct ReflectorDetectOptions
{
  double intensity_min;
  double reflector_min_length;
  double reflector_length_error;
  float range_min;
  float range_max;
};

class LaserReflectorDetect : public ReflectorDetectInterface
{
public:
  LaserReflectorDetect(const ReflectorDetectOptions &options);
  ~LaserReflectorDetect() override {}

  sensor::Observation HandleLaserScan(const sensor_msgs::LaserScanConstPtr &msg) override;
  void HandleOdometryData(const sensor::OdometryData &msg) override;

private:
  const ReflectorDetectOptions options_;
  std::unique_ptr<PoseExtrapolatorInterface> pose_extrapolator_;
};

} // namespace reflector_detect

#endif // REFLECTOR_DETECT_LASER_LASER_REFLECTOR_DETECT_H