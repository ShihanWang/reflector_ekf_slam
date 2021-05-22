#include "mapping/map_builder.h"
#include <glog/logging.h>

namespace mapping
{
MapBuilder::MapBuilder(const MapBuilderOptions &options) : options_(options)
{
  real_time_correlative_scan_matcher_ =
      common::make_unique<scan_matching::RealTimeCorrelativeScanMatcher2D>(options_.real_time_scan_matcher_options);
  ceres_scan_matcher_ =
      common::make_unique<scan_matching::CeresScanMatcher2D>(options_.ceres_scan_matcher_options);
  range_data_inserter_ =
      common::make_unique<ProbabilityGridRangeDataInserter2D>(options_.range_data_inserter_options);
}

MapBuilder::~MapBuilder()
{
}

sensor::RangeData
MapBuilder::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f &transform_to_gravity_aligned_frame,
    const sensor::RangeData &range_data)
{
  const sensor::RangeData cropped =
      sensor::TransformRangeData(range_data, transform_to_gravity_aligned_frame);

  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(options_.voxel_filter_size).Filter(cropped.returns),
      sensor::VoxelFilter(options_.voxel_filter_size).Filter(cropped.misses)};
}

std::unique_ptr<transform::Rigid2d> MapBuilder::ScanMatch(
    const common::Time time, const transform::Rigid2d &pose_prediction,
    const sensor::PointCloud &filtered_gravity_aligned_point_cloud)
{
  if (!submap_)
  {
    return common::make_unique<transform::Rigid2d>(pose_prediction);
  }
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  const double score = real_time_correlative_scan_matcher_->Match(
      pose_prediction, filtered_gravity_aligned_point_cloud,
      *submap_->grid(), &initial_ceres_pose);

  auto pose_observation = common::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_->Match(pose_prediction.translation(), initial_ceres_pose,
                             filtered_gravity_aligned_point_cloud,
                             *submap_->grid(), pose_observation.get(),
                             &summary);
  return pose_observation;
}

std::unique_ptr<MatchingResult> MapBuilder::AddRangeData(
    common::Time time,
    const sensor::RangeData &range_data,
    const transform::Rigid3d &ekf_pose)
{
  if (range_data.returns.empty())
  {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }
  const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
      ekf_pose.rotation());
  const sensor::RangeData gravity_aligned_range_data =
      TransformToGravityAlignedFrameAndFilter(gravity_alignment.cast<float>(), range_data);
  const transform::Rigid2d pose_prediction = transform::Project2D(
      ekf_pose * gravity_alignment.inverse());
  const sensor::PointCloud &filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(options_.adaptive_voxel_options).Filter(gravity_aligned_range_data.returns);
  if (filtered_gravity_aligned_point_cloud.empty())
  {
    return nullptr;
  }
  // local map frame <- gravity-aligned frame
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);
  if (pose_estimate_2d == nullptr)
  {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;

  sensor::RangeData range_data_in_local =
      TransformRangeData(range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));

  InsertIntoSubmap(range_data_in_local);

  return common::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local)});
}

void MapBuilder::InsertIntoSubmap(const sensor::RangeData &range_data_in_local)
{
  if (!submap_)
  {
    submap_ = common::make_unique<Submap2D>(
        range_data_in_local.origin,
        std::unique_ptr<Grid2D>(
            static_cast<Grid2D *>(CreateGrid(range_data_in_local.origin).release())),
        &conversion_tables_);
  }
  submap_->InsertRangeData(range_data_in_local, range_data_inserter_.get());
}

std::unique_ptr<Grid2D> MapBuilder::CreateGrid(
    const Eigen::Vector2f &origin)
{
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.resolution;
  return common::make_unique<ProbabilityGrid>(
      MapLimits(resolution,
                origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                            resolution *
                                            Eigen::Vector2d::Ones(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
      &conversion_tables_);
}

bool MapBuilder::ToSubmapTexture(SubmapTexture *const response) {
    if(!submap_) return false;
    submap_->GetMapTextureData(response);
    return true;
}

} // namespace mapping
