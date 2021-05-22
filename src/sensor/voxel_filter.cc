
#include "sensor/voxel_filter.h"

#include <cmath>

#include "common/math.h"

namespace sensor
{

namespace
{

PointCloud FilterByMaxRange(const PointCloud &point_cloud,
                            const float max_range)
{
  PointCloud result;
  for (const auto &point : point_cloud)
  {
    if (point.norm() <= max_range)
    {
      result.push_back(point);
    }
  }
  return result;
}

PointCloud AdaptivelyVoxelFiltered(
    const AdaptiveVoxelFilterOptions &options,
    const PointCloud &point_cloud)
{
  if (point_cloud.size() <= options.min_num_points) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  PointCloud result = VoxelFilter(options.max_length).Filter(point_cloud);
  if (result.size() >= options.min_num_points) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = options.max_length;
       high_length > 1e-2f * options.max_length; high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(low_length).Filter(point_cloud);
    if (result.size() >= options.min_num_points)
    {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      while ((high_length - low_length) / low_length > 1e-1f)
      {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate =
            VoxelFilter(mid_length).Filter(point_cloud);
        if (candidate.size() >= options.min_num_points)
        {
          low_length = mid_length;
          result = candidate;
        }
        else
        {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

} // namespace

PointCloud VoxelFilter::Filter(const PointCloud &point_cloud)
{
  PointCloud results;
  for (const Eigen::Vector2f &point : point_cloud)
  {
    const Eigen::Vector3f point3f(point.x(), point.y(), 0.);

    auto it_inserted = voxel_set_.insert(IndexToKey(GetCellIndex(point3f)));
    if (it_inserted.second)
    {
      results.push_back(point);
    }
  }
  return results;
}

VoxelFilter::KeyType VoxelFilter::IndexToKey(const Eigen::Array3i &index)
{
  KeyType k_0(static_cast<uint32>(index[0]));
  KeyType k_1(static_cast<uint32>(index[1]));
  KeyType k_2(static_cast<uint32>(index[2]));
  return (k_0 << 2 * 32) | (k_1 << 1 * 32) | k_2;
}

Eigen::Array3i VoxelFilter::GetCellIndex(const Eigen::Vector3f &point) const
{
  Eigen::Array3f index = point.array() / resolution_;
  return Eigen::Array3i(common::RoundToInt(index.x()),
                        common::RoundToInt(index.y()),
                        common::RoundToInt(index.z()));
}

AdaptiveVoxelFilter::AdaptiveVoxelFilter(
    const AdaptiveVoxelFilterOptions &options)
    : options_(options) {}

PointCloud AdaptiveVoxelFilter::Filter(const PointCloud &point_cloud) const
{
  return AdaptivelyVoxelFiltered(
      options_, FilterByMaxRange(point_cloud, options_.max_range));
}

} // namespace sensor