#ifndef REFLECTOR_DETECT_POINT_CLOUD_POINT_CLOUD_REFLECTOR_DETECT_H
#define REFLECTOR_DETECT_POINT_CLOUD_POINT_CLOUD_REFLECTOR_DETECT_H
#include "reflector_detect/reflector_detect_interface.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <cstddef>
#include <type_traits>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace reflector_detect
{
// using PointTI = pcl::PointXYZI;
// using Cloud = pcl::PointCloud<PointTI>;
// using CloudPtr = Cloud::Ptr;

struct PointCloudOptions
{
  double intensity_min;
};

class PointCloudReflectorDetect : public ReflectorDetectInterface
{
public:
  PointCloudReflectorDetect(const PointCloudOptions &options) : options_(options) {}
  ~PointCloudReflectorDetect() override {}

  sensor::Observation HandlePointCloud(const sensor_msgs::PointCloud2ConstPtr &msg) override;

private:
  const PointCloudOptions options_;
};

} // namespace reflector_detect

#endif // REFLECTOR_DETECT_POINT_CLOUD_POINT_CLOUD_REFLECTOR_DETECT_H