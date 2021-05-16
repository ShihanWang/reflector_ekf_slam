

#include "sensor/range_data.h"

namespace sensor
{

RangeData TransformRangeData(const RangeData &range_data,
                             const transform::Rigid2f &transform)
{
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
  };
}

RangeData TransformRangeData(const RangeData &range_data,
                             const transform::Rigid3f &transform)
{
  return TransformRangeData(range_data, transform::Project2D(transform));
}

} // namespace sensor
