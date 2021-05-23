
#ifndef MAPPING_MAP_LIMITS_H_
#define MAPPING_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "common/math.h"
#include "common/port.h"
#include "mapping/xy_index.h"
#include "sensor/range_data.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"
#include "glog/logging.h"

namespace mapping
{

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
class MapLimits
{
public:
  MapLimits(const double resolution, const Eigen::Vector2d &max,
            const CellLimits &cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits)
  {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d &max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  const CellLimits &cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the 'point' which may be outside
  // the map, i.e., negative or too large indices that will return false for
  // Contains().
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f &point) const
  {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  // Returns the center of the cell at 'cell_index'.
  Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const
  {
    return {max_.x() - resolution() * (cell_index[1] + 0.5),
            max_.y() - resolution() * (cell_index[0] + 0.5)};
  }

  // Returns true if the ProbabilityGrid contains 'cell_index'.
  bool Contains(const Eigen::Array2i &cell_index) const
  {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

private:
  double resolution_;
  Eigen::Vector2d max_;
  CellLimits cell_limits_;
};

} // namespace mapping

#endif // MAPPING_MAP_LIMITS_H_
