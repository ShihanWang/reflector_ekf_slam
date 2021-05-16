#ifndef MAPPING_MAP_BUILDER_H
#define MAPPING_MAP_BUILDER_H
#include "common/common.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"
#include "sensor/sensor_data.h"
#include "sensor/range_data.h"
#include "common/time.h"
#include <memory>
#include "sensor/voxel_filter.h"
#include "mapping/submap_2d.h"

namespace mapping
{
struct MapBuilderOptions
{
    float voxel_filter_size;
    sensor::AdaptiveVoxelFilterOptions adaptive_voxel_options;
};

struct MatchingResult
{
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
};

class MapBuilder
{
  public:
    MapBuilder(const MapBuilderOptions &options);
    ~MapBuilder();
    std::unique_ptr<MatchingResult> AddRangeData(
        common::Time time,
        const sensor::RangeData &range_data,
        const transform::Rigid3d &ekf_pose);

  private:
    sensor::RangeData TransformToGravityAlignedFrameAndFilter(
        const transform::Rigid3f &transform_to_gravity_aligned_frame,
        const sensor::RangeData &range_data);

    // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
    // observed pose, or nullptr on failure.
    std::unique_ptr<transform::Rigid2d> ScanMatch(
        common::Time time, const transform::Rigid2d &pose_prediction,
        const sensor::PointCloud &filtered_gravity_aligned_point_cloud);

    MapBuilderOptions options_;
};
} // namespace mapping

#endif // MAPPING_MAP_BUILDER_H