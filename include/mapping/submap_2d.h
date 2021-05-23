#ifndef MAPPING_SUBMAP_2D_H_
#define MAPPING_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "mapping/grid_2d.h"
#include "mapping/map_limits.h"
#include "mapping/probability_grid_range_data_inserter_2d.h"
#include "mapping/value_conversion_tables.h"
#include "sensor/range_data.h"
#include "transform/rigid_transform.h"
#include "mapping/submaps.h"

namespace mapping
{

class Submap2D : public Submap
{
public:
  Submap2D(const Eigen::Vector2f &origin, std::unique_ptr<Grid2D> grid,
           ValueConversionTables *conversion_tables);

  const Grid2D *grid() const { return grid_.get(); }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData &range_data,
                       const ProbabilityGridRangeDataInserter2D *range_data_inserter);
  void Finish();
  void GetMapTextureData(SubmapTexture *const response) const;

private:
  std::unique_ptr<Grid2D> grid_;
  ValueConversionTables *conversion_tables_;
};

} // namespace mapping

#endif // MAPPING_SUBMAP_2D_H_
