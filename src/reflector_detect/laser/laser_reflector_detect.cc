#include "reflector_detect/laser/laser_reflector_detect.h"
#include "reflector_detect/laser/pose_extrapolator.h"
#include "common/common.h"
#include "sensor/sensor_data.h"

#include <deque>
#include <vector>
#include <glog/logging.h>

namespace reflector_detect
{

LaserReflectorDetect::LaserReflectorDetect(const ReflectorDetectOptions &options) : options_(options),
                                                                                    pose_extrapolator_(common::make_unique<PoseExtrapolator>())
{
}

/*  Add 2 new feature:
 *  1.Correct Motion Distortion by extrapolator, and correct ekf time by using USE_CORRECT_TIME(option)
 *  2.Union first and last reflector for 360 laser scan
 */

sensor::Observation LaserReflectorDetect::HandleLaserScan(const sensor_msgs::LaserScanConstPtr &msg)
{
    sensor::Observation observation;
    observation.time_ = msg->header.stamp.toSec();
    if (msg->range_min < 0 || msg->range_max <= msg->range_min)
    {
        // std::cout << "Scan message range min and max is wrong" << std::endl;
        LOG(ERROR) << "Scan message range min and max is wrong";
        exit(-1);
    }
    if (msg->angle_increment < 0.f && msg->angle_max <= msg->angle_min)
    {
        // std::cout << "Scan message angle min and max and angle increment is wrong" << std::endl;
        LOG(ERROR) << "Scan message angle min and max and angle increment is wrong";
        exit(-1);
    }
    // All reflector points
    std::deque<sensor::TimedPointCloud> reflector_points;
    std::deque<std::vector<int>> reflector_ids;
    // Now reflector points, ids and center
    sensor::TimedPointCloud reflector;
    std::vector<int> reflector_id;
    // All laser scan points in base_link frame
    sensor::TimedPointCloud point_cloud;

    const double last_point_time = msg->header.stamp.toSec();
    const double point_delta_t = msg->scan_time / msg->ranges.size();
    const double first_point_time = last_point_time - msg->scan_time;
    float angle = msg->angle_min;
    if (pose_extrapolator_)
        pose_extrapolator_->TrimDataByTime(first_point_time);
    const transform::Rigid2f sensor_to_base_link = transform::Project2D(sensor_to_base_link_transform_).cast<float>();
    const bool is_circle_scan = (msg->angle_max - msg->angle_min - 2 * M_PI) < 1e-6;

    // Detect reflectors and motion distortion correction hear
    // 反光板点云提取部分
    const int points_number = msg->ranges.size();
    for (int i = 0; i < points_number; ++i)
    {
        // Get range data
        const float range = msg->ranges[i];

        if (range >= msg->range_min && range <= msg->range_max)
        {
            // Get now point xy value in sensor frame
            const Eigen::Vector2f now_point(range * std::cos(angle), range * std::sin(angle));
            // Transform sensor point to base link
            const auto postion_in_base_link = sensor_to_base_link * now_point;
            point_cloud.push_back({postion_in_base_link.x(), postion_in_base_link.y(),
                                   first_point_time + i * point_delta_t});
        }

        // 只处理距离在[range_min_,range_max_]范围用内的点云
        // 因为当反光板距离激光较远时,激光能扫到的反光板点云数量很少,极不稳定。所以只考虑近距离内的反光板点云
        if (options_.range_min <= range && range <= options_.range_max)
        {
            // Detect reflector
            const double intensity = msg->intensities[i];
            // 通过强度阈值来提取来自反光板的点云
            if (intensity > options_.intensity_min)
            {
                // Add the first point
                if (reflector.empty())
                {
                    reflector.push_back(point_cloud.back());
                    reflector_id.push_back(i);
                    // now_center += point_cloud.back();
                }
                else
                {
                    const int last_id = reflector_id.back(); // 取得上一个点云的id
                    // Add connected points
                    // 若点云强度是连续很高,则认为是来自同一块反光板的点云(这里假定反光板反射回的点云总是大于强度阈值且连续成片)
                    if (i - last_id == 1)
                    {
                        reflector.push_back(point_cloud.back());
                        reflector_id.push_back(i);
                        // now_center += point_cloud.back();// 累加点云,用于求和取平均
                    }
                    // 反光板间隙点云检测部分
                    // 因为点云的强度值影响因素很多,有可能一条反光板点云的中间会有某几个点强度较低
                    // 这样会造成1块反光板检测为多块 或者 1块反光板检测出来缺失一部分,从而影响同一反光板的中心点计算
                    else
                    {
                        // 间隙点云是否存在的标志位
                        bool detected_gap = false;
                        // 若与上一个反光板点云相差最多3个点 且 当前点与之前的点在同一平面 且 当前点的下一个点也是高强度点云
                        // 则认为是反光板中间部分的弱强度点云,即间隙点云
                        if (i - last_id < 4 && fabs(msg->ranges[i] - msg->ranges[last_id]) < 0.3 && msg->intensities[i + 1 < points_number ? i + 1 : i] > options_.intensity_min)
                        {
                            detected_gap = true;
                            // 存储反光板中的间隙点云
                            int j = last_id + 1;
                            for (; j < i; ++j)
                            {
                                const float range_gap = msg->ranges[j];
                                const float angle_gap = angle - msg->angle_increment * (i - j);
                                if (std::isinf(range_gap)) // scan中很可能存在inf值
                                    continue;
                                // std::cout << "\n!!!!!!!!!!!!\n"<< "points_number=" << points_number << " i=" << i << " j=" << j << "\n";
                                // std::cout << range_gap << " -- " << angle_gap;
                                const Eigen::Vector2f gap_point = sensor_to_base_link *
                                                                  Eigen::Vector2f(range_gap * std::cos(angle_gap), range_gap * std::sin(angle_gap));
                                reflector.push_back({gap_point.x(), gap_point.y(),
                                                     first_point_time + j * point_delta_t});
                                reflector_id.push_back(j);
                                // now_center += gap_point;
                            }
                            // 存储当前反光板中正常的高强度点云

                            // std::cout << "\n>>>gap_points of reflector detected! now add it back.";
                            LOG(INFO) << "Detect gap points of reflector! now add it back.";
                            reflector.push_back(point_cloud.back());
                            reflector_id.push_back(i);
                            // now_center += point_cloud.back();
                        }

                        if (!detected_gap)
                        {

                            // Calculate reflector length
                            // 当下一帧高强度点云不是连续的,表明一块反光板上的点云已经检索完毕
                            // 此时,front和back的点云代表这块反光板的第一个点和最后一个点
                            // 求取位移差=检测到的反光板宽度
                            const float reflector_length = std::hypotf(reflector.front().x() - reflector.back().x(),
                                                                       reflector.front().y() - reflector.back().y());
                            // Add good reflector and its center
                            // 允许检测出的反光板宽度误差在 reflector_length_error 设定的误差范围内
                            if ((is_circle_scan && reflector_id.front() == 0) || fabs(reflector_length - options_.reflector_min_length) < options_.reflector_length_error)
                            {
                                reflector_points.push_back(reflector); // 存入反光板点云
                                reflector_ids.push_back(reflector_id);
                                // centers.push_back(now_center / reflector.size());// 求平均 获得反光板点云的中心点位置
                            }
                            // Update now reflector
                            // 清除缓存,准备下一个反光板点云的存储
                            reflector.clear();
                            reflector_id.clear();
                            // now_center.setZero(2);
                            // 当前反光板点云就是 下一个反光板的第一个点
                            reflector_id.push_back(i);
                            reflector.push_back(point_cloud.back());
                            // now_center += point_cloud.back();
                            // point_cloud.clear();// 防止内存占用一直增加
                            // std::cout << "\n reflector +1";
                            LOG(INFO) << "Add a new reflector";
                        }
                    }
                }
            }
        }
        // 一个点云判断完毕,下一个点云角度按照角度分辨率增加
        angle += msg->angle_increment;
    }
    // Process last reflector and first reflector
    if (!reflector.empty())
    {
        if (reflector_points.size() > 0)
        {
            const int first_reflector_first_point_id = reflector_ids.front().front();
            const int last_reflector_last_point_id = reflector_id.back();
            const auto first_point = reflector_points.front().front().head<2>();
            const auto first_reflector_last_point = reflector_points.front().back().head<2>();
            const auto last_point = reflector.back().head<2>();
            const auto last_reflector_first_point = reflector.front().head<2>();
            if (is_circle_scan && first_reflector_first_point_id == 0 &&
                last_reflector_last_point_id == points_number - 1 &&
                (last_point - first_point).norm() < 0.1)
            {
                // Union last and first reflector
                auto &first_points = reflector_points.front();
                first_points.insert(first_points.end(), reflector.begin(), reflector.end());
            }
            else
            {
                const float reflector_length = std::hypotf(reflector.front().x() - reflector.back().x(),
                                                           reflector.front().y() - reflector.back().y());
                if (fabs(reflector_length - options_.reflector_min_length) < options_.reflector_length_error)
                {
                    reflector_points.push_back(reflector);
                }
            }
            if (is_circle_scan && last_reflector_last_point_id == 0)
            {
                // Process first reflector
                const float first_reflector_length = (first_reflector_last_point - last_reflector_first_point).norm();
                // Delete first reflector
                if (fabs(first_reflector_length - options_.reflector_min_length) >= options_.reflector_length_error)
                {
                    reflector_points.pop_front();
                }
            }
        }
        else
        {
            const float reflector_length = std::hypotf(reflector.front().x() - reflector.back().x(),
                                                       reflector.front().y() - reflector.back().y());
            if (fabs(reflector_length - options_.reflector_min_length) < options_.reflector_length_error)
            {
                reflector_points.push_back(reflector);
            }
        }
    }
    else if (is_circle_scan && reflector_ids.front().front() == 0)
    {
        // Only process first reflector
        const float first_reflector_length =
            (reflector_points.front().front().head<2>() - reflector_points.back().front().head<2>()).norm();
        // Delete first reflector
        if (fabs(first_reflector_length - options_.reflector_min_length) >= options_.reflector_length_error)
        {
            reflector_points.pop_front();
        }
    }

    // Correct motion distortion by pose extrapolator
    double max_time_stamp = point_cloud.back().z();
    transform::Rigid2d max_time_pose;
    if (pose_extrapolator_)
    {
        range_data_.origin = sensor_to_base_link_transform_.translation().head<2>().cast<float>();
        range_data_.returns.clear();
        range_data_.misses.clear();
        std::vector<transform::Rigid2d> poses;
        for (const auto &p : point_cloud)
        {
            poses.push_back(pose_extrapolator_->ExtrapolatorPose(p.z()));
        }
        CHECK(poses.size() == point_cloud.size() && point_cloud.size() > 0);
        max_time_pose = poses.back();
        const auto last_pose_inverse = poses.back().inverse();
        for (size_t i = 0; i < poses.size(); ++i)
        {
            range_data_.returns.push_back(
                (last_pose_inverse * poses[i]).cast<float>() * point_cloud[i].head<2>());
        }
    }
    else
    {
        range_data_.origin = sensor_to_base_link_transform_.translation().head<2>().cast<float>();
        range_data_.returns.clear();
        range_data_.misses.clear();
        max_time_pose = transform::Rigid2d();
        for (auto &point : point_cloud)
        {
            range_data_.returns.push_back(point.head<2>());
        }
    }

    if (reflector_points.empty())
    {
        return observation;
    }

    std::vector<sensor::PointCloud> all_points_in_odom;
    for (auto &points : reflector_points)
    {
        sensor::PointCloud pts_in_odom;
        for (auto &p : points)
        {
            const double point_time = p.z();
            transform::Rigid2d pose;
            if (pose_extrapolator_)
            {
                pose = pose_extrapolator_->ExtrapolatorPose(point_time);
            }
            else
            {
                pose = transform::Rigid2d();
            }
            pts_in_odom.push_back(pose.cast<float>() * p.head<2>());
        }
        all_points_in_odom.push_back(pts_in_odom);
    }
    for (const auto &points : all_points_in_odom)
    {
        const auto pts_in_base_link = sensor::TransformPointCloud(points, max_time_pose.inverse().cast<float>());
        Eigen::Vector2f center(0., 0.);
        for (const auto &p : pts_in_base_link)
        {
            center += p;
        }
        observation.cloud_.push_back(center / pts_in_base_link.size());
    }

#ifdef USE_CORRECT_TIME
    observation.time_ = max_time_stamp;
#endif
    // 输出检测到的反光板个数
    // std::cout << "\n detected " << observation.cloud_.size() << " reflectors" << std::endl;
    LOG(INFO) << "Detect " << observation.cloud_.size() << " reflectors";

    return observation;
}

void LaserReflectorDetect::HandleOdometryData(const sensor::OdometryData &msg)
{
    if (pose_extrapolator_)
        pose_extrapolator_->HandleOdometryData(msg);
}

} // namespace reflector_detect