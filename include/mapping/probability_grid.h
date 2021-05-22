
#ifndef MAPPINGD_PROBABILITY_GRID_H_
#define MAPPINGD_PROBABILITY_GRID_H_

#include <vector>

#include "common/port.h"
#include "mapping/grid_2d.h"
#include "mapping/map_limits.h"
#include "mapping/xy_index.h"

namespace mapping {

// Represents a 2D grid of probabilities.
class ProbabilityGrid : public Grid2D{
 public:
  explicit ProbabilityGrid(const MapLimits& limits,
                           ValueConversionTables* conversion_tables);

  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability);

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table);

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i& cell_index) const;

  std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
  bool DrawToSubmapTexture(
      SubmapTexture* const texture,
      transform::Rigid3d local_pose) const override;

 private:
  ValueConversionTables* conversion_tables_;
};

}  // namespace mapping

#endif  // MAPPINGD_PROBABILITY_GRID_H_
