#ifndef SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_

#include "mapping/grid_2d.h"
#include "sensor/sensor_data.h"
#include "ceres/ceres.h"

namespace scan_matching
{

// Creates a cost function for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
ceres::CostFunction *CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud &point_cloud,
    const mapping::Grid2D &grid);

} // namespace scan_matching

#endif // SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H_
