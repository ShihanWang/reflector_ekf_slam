#ifndef MAPPING_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define MAPPING_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>

#include "common/port.h"
#include "mapping/probability_grid.h"
#include "mapping/xy_index.h"
#include "sensor/range_data.h"
#include "mapping/grid_2d.h"

namespace mapping
{

struct ProbabilityGridRangeDataInserterOptions2D
{
    bool insert_free_space;
    float hit_probability;
    float miss_probability;
};

class ProbabilityGridRangeDataInserter2D
{
  public:
    explicit ProbabilityGridRangeDataInserter2D(
        const ProbabilityGridRangeDataInserterOptions2D &options);

    ProbabilityGridRangeDataInserter2D(
        const ProbabilityGridRangeDataInserter2D &) = delete;
    ProbabilityGridRangeDataInserter2D &operator=(
        const ProbabilityGridRangeDataInserter2D &) = delete;

    // Inserts 'range_data' into 'probability_grid'.
    void Insert(const sensor::RangeData &range_data,
                Grid2D *grid) const;

  private:
    const ProbabilityGridRangeDataInserterOptions2D options_;
    const std::vector<uint16> hit_table_;
    const std::vector<uint16> miss_table_;
};

} // namespace mapping

#endif // MAPPING_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
