#include "reflector_detect/laser/pose_extrapolator.h"

#include <cmath>

namespace reflector_detect
{

PoseExtrapolator::PoseExtrapolator()
{
}

void PoseExtrapolator::TrimDataByTime(const double &time)
{
    std::lock_guard<std::mutex> lock(odometry_data_mutex_);
#ifdef USE_UNIFORM_VELOCITY
    while (odometry_data_.size() > 2 && odometry_data_[1].time <= time)
    {
        odometry_data_.pop_front();
    }
#else
    while (odometry_data_.size() > 1 && odometry_data_.front().time < time)
    {
        odometry_data_.pop_front();
    }
#endif
}

void PoseExtrapolator::HandleOdometryData(const sensor::OdometryData &msg)
{
    std::lock_guard<std::mutex> lock(odometry_data_mutex_);
    odometry_data_.push_back(msg);
}

transform::Rigid2d PoseExtrapolator::ExtrapolatorPose(const double &time)
{
    std::lock_guard<std::mutex> lock(odometry_data_mutex_);
    if (odometry_data_.empty())
        return transform::Rigid2d();
#ifdef USE_UNIFORM_VELOCITY
    if (odometry_data_.size() < 2)
        return transform::Rigid2d();
    const double start_time = odometry_data_.front().time;
    const double end_time = odometry_data_.back().time;
    if (time <= start_time)
        return Interpolator(odometry_data_[0], odometry_data_[1], time);
    if (time >= end_time)
    {
        const auto end_data = odometry_data_.back();
        odometry_data_.pop_front();
        const auto start_data = odometry_data_.back();
        odometry_data_.push_back(end_data);
        return Interpolator(start_data, end_data, time);
    }
    std::deque<sensor::OdometryData> temp_data;
    sensor::OdometryData start_data;
    while (!odometry_data_.empty() && odometry_data_.front().time <= time)
    {
        start_data = odometry_data_.front();
        odometry_data_.pop_front();
        temp_data.push_back(start_data);
    }
    const auto end_data = odometry_data_.front();
    while (!temp_data.empty())
    {
        odometry_data_.push_front(temp_data.back());
        temp_data.pop_back();
    }
    return Interpolator(start_data, end_data, time);
#else
    const double start_time = odometry_data_.front().time;
    const double end_time = odometry_data_.back().time;
    if (time <= start_time)
        return Interpolator(odometry_data_.front(), time);
    if (time >= end_time)
        return Interpolator(odometry_data_.back(), time);
    sensor::OdometryData odom_data;
    for (const auto &data : odometry_data_)
    {
        if (data.time >= time)
            odom_data = data;
    }
    return Interpolator(odom_data, time);
#endif
}

transform::Rigid2d PoseExtrapolator::Interpolator(const sensor::OdometryData &start,
                                                  const sensor::OdometryData &end, const double &time)
{
    assert(end.time > start.time);
    const double start_time = start.time;
    const double end_time = end.time;
    const double ratio = (time - start_time) / (end_time - start_time);
    const auto translation = start.position + ratio * (end.position - start.position);
    const auto relative_rotation = end.orientation.inverse() * start.orientation;
    const auto angular_velocity =
        transform::RotationQuaternionToAngleAxisVector(relative_rotation) / (end_time - start_time);
    const double start_yaw = 2 * std::atan2(start.orientation.z(), start.orientation.w());
    const double now_yaw = start_yaw + (time - start_time) * angular_velocity.z();
    return transform::Rigid2d(translation.head<2>(), {now_yaw});
}

transform::Rigid2d PoseExtrapolator::Interpolator(const sensor::OdometryData &start, const double &time)
{
    if (start.time <= time)
    {
        const double delta_t = start.time - time;
        const double angular_velocity = start.angular_velocity.z();
        const double odom_yaw = 2 * std::atan2(start.orientation.z(), start.orientation.w());
        const double now_yaw = odom_yaw - angular_velocity * delta_t;
        const double vx = start.linear_velocity.x();
        const double vy = start.linear_velocity.y();
        const double odom_x = start.position.x();
        const double odom_y = start.position.y();
        const double x = odom_x - vx * delta_t * std::cos(now_yaw) + vy * delta_t * std::sin(now_yaw);
        const double y = odom_y - vx * delta_t * std::sin(now_yaw) - vy * delta_t * std::cos(now_yaw);
        return transform::Rigid2d({x, y}, {now_yaw});
    }
    const double delta_t = time - start.time;
    const double angular_velocity = start.angular_velocity.z();
    const double odom_yaw = 2 * std::atan2(start.orientation.z(), start.orientation.w());
    const double now_yaw = odom_yaw - angular_velocity * delta_t;
    const double vx = start.linear_velocity.x();
    const double vy = start.linear_velocity.y();
    const double odom_x = start.position.x();
    const double odom_y = start.position.y();
    const double x = odom_x + vx * delta_t * std::cos(now_yaw) - vy * delta_t * std::sin(now_yaw);
    const double y = odom_y + vx * delta_t * std::sin(now_yaw) + vy * delta_t * std::cos(now_yaw);
    return transform::Rigid2d({x, y}, {now_yaw});
}

} // namespace reflector_detect