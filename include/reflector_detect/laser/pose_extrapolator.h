#ifndef REFLECTOR_DETECT_POSE_EXTRAPOLATOR_H
#define REFLECTOR_DETECT_POSE_EXTRAPOLATOR_H
#include "reflector_detect/pose_extrapolator_interface.h"
#include <deque>
#include <mutex>

namespace reflector_detect
{
class PoseExtrapolator : public PoseExtrapolatorInterface
{
  public:
    PoseExtrapolator();
    ~PoseExtrapolator() override {}
    void TrimDataByTime(const double &time) override;
    void HandleOdometryData(const sensor::OdometryData &msg) override;
    transform::Rigid2d ExtrapolatorPose(const double &time) override;

  private:
    transform::Rigid2d Interpolator(const sensor::OdometryData &start, const sensor::OdometryData &end, const double &time);
    transform::Rigid2d Interpolator(const sensor::OdometryData &start, const double &time);

    std::deque<sensor::OdometryData> odometry_data_;
    std::mutex odometry_data_mutex_;
};
} // namespace reflector_detect

#endif // REFLECTOR_DETECT_POSE_EXTRAPOLATOR_H