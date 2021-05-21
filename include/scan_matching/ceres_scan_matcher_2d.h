
#ifndef SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "mapping/grid_2d.h"
#include "sensor/sensor_data.h"
#include "ceres/ceres.h"

namespace scan_matching {

struct CeresScanMatcherOptions2D
{
    double occupied_space_weight;
    double translation_weight;
    double rotation_weight;
    
};

// Align scans with an existing map using Ceres.
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const CeresScanMatcherOptions2D& options);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector2d& target_translation,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, const mapping::Grid2D& grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
  const CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching

#endif  // SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
