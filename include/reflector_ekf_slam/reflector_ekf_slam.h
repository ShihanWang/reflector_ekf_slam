#ifndef REFLECTOR_EKF_SLAM_H
#define REFLECTOR_EKF_SLAM_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <memory>
#include <cstddef>
#include <type_traits>

// Cartographer code ...
namespace common
{
  // Implementation of c++14's std::make_unique, taken from
  // https://isocpp.org/files/papers/N3656.txt
  template <class T>
  struct _Unique_if
  {
    typedef std::unique_ptr<T> _Single_object;
  };

  template <class T>
  struct _Unique_if<T[]>
  {
    typedef std::unique_ptr<T[]> _Unknown_bound;
  };

  template <class T, size_t N>
  struct _Unique_if<T[N]>
  {
    typedef void _Known_bound;
  };

  template <class T, class... Args>
  typename _Unique_if<T>::_Single_object make_unique(Args &&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }

  template <class T>
  typename _Unique_if<T>::_Unknown_bound make_unique(size_t n)
  {
    typedef typename std::remove_extent<T>::type U;
    return std::unique_ptr<T>(new U[n]());
  }

  template <class T, class... Args>
  typename _Unique_if<T>::_Known_bound make_unique(Args &&...) = delete;

} // namespace common


typedef std::vector<Eigen::Vector2f> PointCloud;
typedef std::vector<Eigen::Matrix2d> PointCloudCoviarance;

class Observation
{
  public:
    Observation() {}
    Observation(const double &time, const PointCloud &cloud) : time_(time), cloud_(cloud) {}
    double time_;
    PointCloud cloud_;
}; // class Observation

class Map
{
  public:
  Map(){}
  Map(const PointCloud& cloud, const PointCloudCoviarance& cov): reflector_map_(cloud), reflector_map_coviarance_(cov){}
  PointCloud reflector_map_;
  PointCloudCoviarance reflector_map_coviarance_;
};
struct matched_ids{
    // map - observation matched id
    std::vector<std::pair<int,int>> map_obs_match_ids;
    // mu_ - observation matched id
    std::vector<std::pair<int,int>> state_obs_match_ids;
    // new observed reflector id
    std::vector<int> new_ids;
};

class ReflectorEKFSLAM
{
  public:
    ReflectorEKFSLAM(const double &time, const double &x0 = 0.0, const double &y0 = 0.0, const double &yaw0 = 0.0);
    ReflectorEKFSLAM() = delete;
    ~ReflectorEKFSLAM();
    void addEncoder(const nav_msgs::Odometry::ConstPtr &odometry); //加入编码器数据进行运动更新
    void addLaser(const sensor_msgs::LaserScan::ConstPtr& scan);

    void loadFromVector(const std::vector<std::vector<double>>& vecs);
    void setSaveMapPath(const std::string& path)
    {
      map_path_ = path;
    }

    visualization_msgs::MarkerArray toRosMarkers(double scale); //将路标点转换成ROS的marker格式，用于发布显示
    geometry_msgs::PoseWithCovarianceStamped toRosPose(); //将机器人位姿转化成ROS的pose格式，用于发布显示

    nav_msgs::Path& path(){
        return ekf_path_;}
    Eigen::VectorXd& mu(){
        return mu_;}
    Eigen::MatrixXd& sigma(){
        return sigma_;}

private:

    bool getObservations(const sensor_msgs::LaserScan& msg, Observation& obs);
    void normAngle(double& angle);
    matched_ids detectMatchedIds(const Observation& obs);
    void predict(const double& dt);

    double reflector_length_error_ = 0.06;
    double reflector_min_length_ = 0.18;//0.3
    double intensity_min_ = 200.0;//700
    // double reflector_min_length_ = 0.3;//0.3
    // double intensity_min_ = 700.0;//700

    nav_msgs::Path ekf_path_;

    /* 系统配置参数 */
    double wt_ = 0.0, vt_ = 0.0; // 里程计参数
    Eigen::Vector3d lidar_to_base_link_; // 机器人外参数
    Eigen::Matrix2d Qu_; // 里程计协方差参数
    Eigen::Matrix2d Qt_; // 观测协方差参数

    /* 求解的扩展状态 均值 和 协方差 */
    Eigen::VectorXd mu_; //均值
    Eigen::MatrixXd sigma_; //方差

    Map map_;
    double time_;
    Eigen::Vector3d init_pose_;
    std::string map_path_;

}; //class ReflectorEKFSLAM

#endif
