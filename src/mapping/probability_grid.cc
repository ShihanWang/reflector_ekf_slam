
#include "mapping/probability_grid.h"

#include <limits>

#include "mapping/probability_values.h"
#include "common/common.h"

namespace mapping {

ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}


// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  if (*cell >= kUpdateMarker) {
    return false;
  }
  mutable_update_indices()->push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

// Returns the probability of the cell with 'cell_index'.
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}


std::unique_ptr<Grid2D> ProbabilityGrid::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      common::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), conversion_tables_);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }

  return std::unique_ptr<Grid2D>(cropped_grid.release());
}


}  // namespace mapping
