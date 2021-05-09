#include "transform/timestamped_transform.h"

namespace transform
{

TimestampedTransform Interpolate(const TimestampedTransform &start,
                                 const TimestampedTransform &end,
                                 const double &time)
{
    const double duration = end.time - start.time;
    const double factor = (time - start.time) / duration;
    const Eigen::Vector3d origin =
        start.transform.translation() +
        (end.transform.translation() - start.transform.translation()) * factor;
    const Eigen::Quaterniond rotation =
        Eigen::Quaterniond(start.transform.rotation())
            .slerp(factor, Eigen::Quaterniond(end.transform.rotation()));
    return TimestampedTransform{time, transform::Rigid3d(origin, rotation)};
}

} // namespace transform
