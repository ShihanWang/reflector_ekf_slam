#ifndef REFLECTOR_DETECT_POSE_EXTRAPOLATOR_INTERFACE_H
#define REFLECTOR_DETECT_POSE_EXTRAPOLATOR_INTERFACE_H

#include "common/common.h"
#include "sensor/sensor_data.h"
#include "transform/transform.h"
#include "transform/rigid_transform.h"
#include "transform/timestamped_transform.h"

namespace reflector_detect
{

class PoseExtrapolatorInterface
{
  public:
    PoseExtrapolatorInterface() {}
    virtual ~PoseExtrapolatorInterface() {}
    virtual void TrimDataByTime(const double &time) = 0;
    virtual void HandleOdometryData(const sensor::OdometryData &msg) {}
    virtual void HandleImuData(const sensor::ImuData &msg) {}
    virtual transform::Rigid2d ExtrapolatorPose(const double &time) = 0;
};

} // namespace reflector_detect

#endif // REFLECTOR_DETECT_POSE_EXTRAPOLATOR_INTERFACE_H