#ifndef TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "transform/rigid_transform.h"

namespace transform
{

struct TimestampedTransform
{
  double time;
  transform::Rigid3d transform;
};

TimestampedTransform Interpolate(const TimestampedTransform &start,
                                 const TimestampedTransform &end,
                                 const double &time);

} // namespace transform

#endif // TRANSFORM_TIMESTAMPED_TRANSFORM_H_
