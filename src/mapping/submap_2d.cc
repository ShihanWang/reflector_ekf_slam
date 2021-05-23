
#include "mapping/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "common/port.h"
#include "common/common.h"
#include "mapping/probability_grid_range_data_inserter_2d.h"
#include "glog/logging.h"

namespace mapping
{

Submap2D::Submap2D(const Eigen::Vector2f &origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables *conversion_tables)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables)
{
  grid_ = std::move(grid);
}

void Submap2D::InsertRangeData(
    const sensor::RangeData &range_data,
    const ProbabilityGridRangeDataInserter2D *range_data_inserter)
{
  CHECK(grid_);
  CHECK(!insertion_finished());
  range_data_inserter->Insert(range_data, grid_.get());
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish()
{
  CHECK(grid_);
  CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();
  set_insertion_finished(true);
}

void Submap2D::GetMapTextureData(SubmapTexture *const response) const
{
  if (!grid_)
    return;
  grid()->DrawToSubmapTexture(response, local_pose());
}

} // namespace mapping
