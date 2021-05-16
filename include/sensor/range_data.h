#ifndef SENSOR_RANGE_DATA_H_
#define SENSOR_RANGE_DATA_H_

#include "sensor/sensor_data.h"
#include "transform/transform.h"
#include "transform/rigid_transform.h"

namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData {
  Eigen::Vector2f origin;
  PointCloud returns;
  PointCloud misses;
};

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid2f& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);


}  // namespace sensor


#endif  // RANGE_DATA_H_
