#include "reflector_detect/point_cloud/point_cloud_reflector_detect.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"
#include <glog/logging.h>

namespace reflector_detect
{

sensor::Observation PointCloudReflectorDetect::HandlePointCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    using PointTI = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<PointTI>;
    using CloudPtr = Cloud::Ptr;

    sensor::Observation observation;
    observation.time_ = msg->header.stamp.toSec();
    CloudPtr points_raw(new Cloud);
    CloudPtr reflector_points_i(new Cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reflector_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reflector_points_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sensor::PointCloud centers;

    // auto point_transformed_to_base_link = [&](const Eigen::Vector2f& p) -> Eigen::Vector2f{
    //     const float x = p.x() * std::cos(lidar_to_base_link_.z()) - p.y() * std::sin(lidar_to_base_link_.z()) + lidar_to_base_link_.x();
    //     const float y = p.x() * std::sin(lidar_to_base_link_.z()) + p.y() * std::cos(lidar_to_base_link_.z()) + lidar_to_base_link_.y();
    //     return Eigen::Vector2f(x,y);
    // };
    const auto sensor_to_base_link = sensor_to_base_link_transform_;

    pcl::fromROSMsg(*msg, *points_raw);
    for (int i = 0; i < points_raw->points.size(); i++)
    {
        if (points_raw->at(i).intensity > options_.intensity_min)
        {
            reflector_points_i->points.push_back(points_raw->at(i));
        }
    }

    pcl::copyPointCloud(*reflector_points_i, *reflector_points);

    // 离群点剔除
    // 1)统计滤波法
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(reflector_points);
    sor.setMeanK(30);            //临近点
    sor.setStddevMulThresh(0.5); //距离大于1倍标准方差，值越大，丢掉的点越少
    sor.filter(*reflector_points_filtered);

    // pcl::toROSMsg(*reflector_points_filtered,msg_reflector);
    // msg_reflector.header.frame_id = "world";

    // 2)半径滤波法
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // outrem.setInputCloud(reflector_points);
    // outrem.setRadiusSearch(0.3);
    // outrem.setMinNeighborsInRadius(4);
    // outrem.filter(*reflector_points_filtered);

    // 取反，获取被剔除的离群点
    // sor.setNegative(true);
    // sor.filter(*cloud_filtered);

    // 通过聚类，提取出反光板点云块
    // 创建一个Kd树对象作为提取点云时所用的方法，
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(reflector_points_filtered); //创建点云索引向量，用于存储实际的点云信息
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //欧式聚类对象
    ec.setClusterTolerance(0.2);                       //设置近邻搜索的搜索半径(m)
    ec.setMinClusterSize(4);                           //设置一个聚类需要的最少点数目
    ec.setMaxClusterSize(160);                         //设置一个聚类需要的最大点数目
    ec.setSearchMethod(tree);                          //设置点云的搜索机制
    ec.setInputCloud(reflector_points_filtered);
    ec.extract(cluster_indices); //从点云中提取聚类，并将 聚类后的点云块索引 保存在cluster_indices中

    //迭代访问点云索引cluster_indices，直到分割出所有聚类出的反光板点云
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(reflector_points_filtered->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header = reflector_points_filtered->header;
        cloud_cluster->sensor_orientation_ = reflector_points_filtered->sensor_orientation_;
        cloud_cluster->sensor_origin_ = reflector_points_filtered->sensor_origin_;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        LOG(INFO) << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points.";
        // PCL函数计算质心
        Eigen::Vector4f centroid;                         // 质量m默认为1 (x,y,z,1)
        pcl::compute3DCentroid(*cloud_cluster, centroid); // 计算当前质心
        Eigen::Vector2f now_point{centroid(0), centroid(1)};
        centers.push_back(transform::Project2D(sensor_to_base_link).cast<float>() * now_point);
    }

    if (centers.empty())
        return observation;
    observation.cloud_ = centers;
    // 输出检测到的反光板个数
    // std::cout << "\n detected " << obs.cloud_.size() << " reflectors" << std::endl;
    LOG(INFO) << "Detected " << observation.cloud_.size() << " reflectors";
    return observation;
}

} // namespace reflector_detect