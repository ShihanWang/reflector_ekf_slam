
#include "scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "mapping/grid_2d.h"
#include "scan_matching/occupied_space_cost_function_2d.h"
#include "scan_matching/rotation_delta_cost_functor_2d.h"
#include "scan_matching/translation_delta_cost_functor_2d.h"
#include "transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace scan_matching
{

CeresScanMatcher2D::CeresScanMatcher2D(
    const CeresScanMatcherOptions2D &options)
    : options_(options)
{
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(const Eigen::Vector2d &target_translation,
                               const transform::Rigid2d &initial_pose_estimate,
                               const sensor::PointCloud &point_cloud,
                               const mapping::Grid2D &grid,
                               transform::Rigid2d *const pose_estimate,
                               ceres::Solver::Summary *const summary) const
{
    double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                     initial_pose_estimate.translation().y(),
                                     initial_pose_estimate.rotation().angle()};
    ceres::Problem problem;
    CHECK_GT(options_.occupied_space_weight, 0.);

    problem.AddResidualBlock(
        CreateOccupiedSpaceCostFunction2D(
            options_.occupied_space_weight /
                std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, grid),
        nullptr /* loss function */, ceres_pose_estimate);

    CHECK_GT(options_.translation_weight, 0.);
    problem.AddResidualBlock(
        TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.translation_weight, target_translation),
        nullptr /* loss function */, ceres_pose_estimate);
    CHECK_GT(options_.rotation_weight, 0.);
    problem.AddResidualBlock(
        RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.rotation_weight, ceres_pose_estimate[2]),
        nullptr /* loss function */, ceres_pose_estimate);

    ceres::Solve(options_.ceres_solver_options, &problem, summary);

    *pose_estimate = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

} // namespace scan_matching
