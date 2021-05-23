#include "io/points_batch.h"

namespace io
{

void RemovePoints(std::unordered_set<int> to_remove, PointsBatch *batch)
{
  const int new_num_points = batch->points.size() - to_remove.size();
  std::vector<Eigen::Vector3f> points;
  points.reserve(new_num_points);
  std::vector<float> intensities;
  if (!batch->intensities.empty())
  {
    intensities.reserve(new_num_points);
  }
  std::vector<FloatColor> colors;
  if (!batch->colors.empty())
  {
    colors.reserve(new_num_points);
  }

  for (size_t i = 0; i < batch->points.size(); ++i)
  {
    if (to_remove.count(i) == 1)
    {
      continue;
    }
    points.push_back(batch->points[i]);
    if (!batch->colors.empty())
    {
      colors.push_back(batch->colors[i]);
    }
    if (!batch->intensities.empty())
    {
      intensities.push_back(batch->intensities[i]);
    }
  }
  batch->points = std::move(points);
  batch->intensities = std::move(intensities);
  batch->colors = std::move(colors);
}

} // namespace io
