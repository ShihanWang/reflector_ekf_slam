#ifndef SENSOR_VOXEL_FILTER_H_
#define SENSOR_VOXEL_FILTER_H_

#include <bitset>
#include <unordered_set>

#include "sensor/sensor_data.h"

namespace sensor {

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  explicit VoxelFilter(float size) : resolution_(size) {}

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Returns a voxel filtered copy of 'point_cloud'.
  PointCloud Filter(const PointCloud& point_cloud);

 private:
  using KeyType = std::bitset<3 * 32>;

  static KeyType IndexToKey(const Eigen::Array3i& index);

  Eigen::Array3i GetCellIndex(const Eigen::Vector3f& point) const;

  float resolution_;
  std::unordered_set<KeyType> voxel_set_;
};

struct AdaptiveVoxelFilterOptions
{
    double max_length;
    double min_num_points;
    double max_range;
};

class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor

#endif  // SENSOR_VOXEL_FILTER_H_
