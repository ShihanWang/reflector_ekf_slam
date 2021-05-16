#include "mapping/map_builder.h"
#include <glog/logging.h>

namespace mapping
{
MapBuilder::MapBuilder(const MapBuilderOptions &options)
{
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
    // if (active_submaps_.submaps().empty())
    // {
    //     return absl::make_unique<transform::Rigid2d>(pose_prediction);
    // }
    // std::shared_ptr<const Submap2D> matching_submap =
    //     active_submaps_.submaps().front();
    // // The online correlative scan matcher will refine the initial estimate for
    // // the Ceres scan matcher.
    // transform::Rigid2d initial_ceres_pose = pose_prediction;

    // if (options_.use_online_correlative_scan_matching())
    // {
    //     const double score = real_time_correlative_scan_matcher_.Match(
    //         pose_prediction, filtered_gravity_aligned_point_cloud,
    //         *matching_submap->grid(), &initial_ceres_pose);
    //     kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
    // }

    auto pose_observation = common::make_unique<transform::Rigid2d>();
    // ceres::Solver::Summary summary;
    // ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
    //                           filtered_gravity_aligned_point_cloud,
    //                           *matching_submap->grid(), pose_observation.get(),
    //                           &summary);
    // if (pose_observation)
    // {
    //     kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    //     const double residual_distance =
    //         (pose_observation->translation() - pose_prediction.translation())
    //             .norm();
    //     kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    //     const double residual_angle =
    //         std::abs(pose_observation->rotation().angle() -
    //                  pose_prediction.rotation().angle());
    //     kScanMatcherResidualAngleMetric->Observe(residual_angle);
    // }
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
}

} // namespace mapping
