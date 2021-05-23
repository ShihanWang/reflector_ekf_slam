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
#include "mapping/grid_2d.h"
#include "mapping/value_conversion_tables.h"
#include "mapping/probability_grid_range_data_inserter_2d.h"
#include "scan_matching/ceres_scan_matcher_2d.h"
#include "scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "scan_matching/correlative_scan_matcher_2d.h"

namespace mapping
{
struct MapBuilderOptions
{
    float resolution;
    float voxel_filter_size;
    sensor::AdaptiveVoxelFilterOptions adaptive_voxel_options;
    scan_matching::RealTimeCorrelativeScanMatcherOptions real_time_scan_matcher_options;
    scan_matching::CeresScanMatcherOptions2D ceres_scan_matcher_options;
    ProbabilityGridRangeDataInserterOptions2D range_data_inserter_options;
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
    bool ToSubmapTexture(SubmapTexture *const response);

  private:
    sensor::RangeData TransformToGravityAlignedFrameAndFilter(
        const transform::Rigid3f &transform_to_gravity_aligned_frame,
        const sensor::RangeData &range_data);

    // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
    // observed pose, or nullptr on failure.
    std::unique_ptr<transform::Rigid2d> ScanMatch(
        common::Time time, const transform::Rigid2d &pose_prediction,
        const sensor::PointCloud &filtered_gravity_aligned_point_cloud);
    void InsertIntoSubmap(const sensor::RangeData &range_data_in_local);
    std::unique_ptr<Grid2D> CreateGrid(const Eigen::Vector2f &origin);

    MapBuilderOptions options_;
    std::unique_ptr<Submap2D> submap_;
    std::unique_ptr<scan_matching::RealTimeCorrelativeScanMatcher2D>
        real_time_correlative_scan_matcher_;
    std::unique_ptr<scan_matching::CeresScanMatcher2D> ceres_scan_matcher_;
    ValueConversionTables conversion_tables_;
    std::unique_ptr<ProbabilityGridRangeDataInserter2D> range_data_inserter_;
};
} // namespace mapping

#endif // MAPPING_MAP_BUILDER_H