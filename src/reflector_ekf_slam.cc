#include <reflector_ekf_slam/reflector_ekf_slam.h>

ReflectorEKFSLAM::ReflectorEKFSLAM (const double &time, const double &x0, const double &y0, const double &yaw0): time_(time)
{
    init_pose_.x() = x0;
    init_pose_.y() = y0;
    init_pose_.z() = yaw0;
    vt_ = 0.0;
    wt_ = 0.0;
    /* 初始时刻机器人位姿为0，绝对准确, 协方差为0 */
    mu_ = Eigen::Vector3d::Zero();
    mu_.topRows(3) = init_pose_;
    sigma_.resize(3, 3);
    sigma_.setZero();   
    Qu_ << 0.02 * 0.02, 0.f, 0.f, 0.034 * 0.034;
    Qt_ << 0.03 * 0.03, 0.f, 0.f, 0.03 * 0.03; 
    lidar_to_base_link_ << 0.41589, 0.25639, 0.0;
}

ReflectorEKFSLAM::~ReflectorEKFSLAM()
{
    if(!map_path_.empty() && !map_.reflector_map_.empty())
    {
        // write data
        std::ofstream out(map_path_.c_str(), std::ios::in);
        if(!map_.reflector_map_.empty())  
        {
            for(int i = 0; i < map_.reflector_map_.size(); ++i)
            {
                if( i != map_.reflector_map_.size()- 1)
                    out << map_.reflector_map_[i].x() << "," << map_.reflector_map_[i].y() << ",";
                else
                    out << map_.reflector_map_[i].x() << "," << map_.reflector_map_[i].y();                
            }
        } 
        if(mu_.rows() > 3)
        {
            out << ",";
            const int N = (mu_.rows() - 3) / 2;
            for(int i = 0; i < N; ++i)
            {
                if(i != N - 1)
                    out << mu_(3 + 2 * i) << "," << mu_(3 + 2 * i + 1) << ",";
                else
                    out << mu_(3 + 2 * i) << "," << mu_(3 + 2 * i + 1);  
            }
        }
        out << std::endl;
        if(!map_.reflector_map_.empty())  
        {
            for(int i = 0; i < map_.reflector_map_.size(); ++i)
            {
                if( i != map_.reflector_map_.size()- 1)
                    out << map_.reflector_map_coviarance_[i](0,0) << "," << map_.reflector_map_coviarance_[i](0,1) << ","
                        << map_.reflector_map_coviarance_[i](1,0) << "," << map_.reflector_map_coviarance_[i](1,1) << ",";
                else
                    out << map_.reflector_map_coviarance_[i](0,0) << "," << map_.reflector_map_coviarance_[i](0,1) << ","
                        << map_.reflector_map_coviarance_[i](1,0) << "," << map_.reflector_map_coviarance_[i](1,1);                
            }
        } 
        if(mu_.rows() > 3)
        {
            out << ",";
            const int N = (mu_.rows() - 3) / 2;
            for(int i = 0; i < N; ++i)
            {
                if(i != N - 1)
                    out << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0,0) << "," << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0,1) << ","
                        << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1,0) << "," << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1,1) << ",";
                else
                    out << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0,0) << "," << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(0,1) << ","
                        << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1,0) << "," << sigma_.block(3 + 2 * i, 3 + 2 * i, 2, 2)(1,1); 
            }
        }
        out << std::endl;
        out.close();
    }
}

void ReflectorEKFSLAM::addEncoder (const nav_msgs::Odometry::ConstPtr &odometry)
{
    if(odometry->header.stamp.toSec() <= time_)
        return;
    const double now_time = odometry->header.stamp.toSec();
    /***** 保存上一帧编码器数据 *****/
    wt_ = odometry->twist.twist.angular.z;
    vt_ = odometry->twist.twist.linear.x;
    const double dt = now_time - time_;    
    const double delta_theta = wt_ * dt;
    const double delta_x = vt_ * dt * std::cos(mu_(2) + delta_theta / 2);
    const double delta_y = vt_ * dt * std::sin(mu_(2) + delta_theta / 2);

    const int N = mu_.rows();
    /***** 更新协方差 *****/
    /* 构造 Gt */
    const double angular_half_delta =  mu_(2) + delta_theta / 2;
    Eigen::MatrixXd G_xi = Eigen::MatrixXd::Zero(N, 3);

    Eigen::Matrix3d G_xi_2 = Eigen::Matrix3d::Identity();
    G_xi_2(0, 2) = -vt_ * dt * std::sin(angular_half_delta);
    G_xi_2(1, 2) = vt_ * dt * std::cos(angular_half_delta); 
    G_xi.block(0,0,3,3) = G_xi_2;

    /* 构造 Gu' */
    Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 2);
    Eigen::MatrixXd G_u_2(3, 2);
    G_u_2 << dt * std::cos(angular_half_delta), -vt_ * dt * dt * std::sin(angular_half_delta) / 2,
        dt * std::sin(angular_half_delta), vt_ * dt * dt * std::cos(angular_half_delta) / 2,
        0, dt;
    G_u.block(0,0,3,2) = G_u_2;
    /* 更新协方差 */
    sigma_ = G_xi * sigma_ * G_xi.transpose() + G_u * Qu_ * G_u.transpose();
    /***** 更新均值 *****/
    mu_.topRows(3) += Eigen::Vector3d(delta_x, delta_y, delta_theta);
    mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2))); //norm
    time_ = now_time;
}

void ReflectorEKFSLAM::addLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    const double now_time = scan->header.stamp.toSec();
    const double dt = now_time - time_;

    // Predict now pose
    const double delta_theta = wt_ * dt;
    const double delta_x = vt_ * dt * std::cos(mu_(2) + delta_theta / 2);
    const double delta_y = vt_ * dt * std::sin(mu_(2) + delta_theta / 2);
    const int N = mu_.rows();
    /***** 更新协方差 *****/
    /* 构造 Gt */
    const double angular_half_delta =  mu_(2) + delta_theta / 2;
    Eigen::MatrixXd G_xi = Eigen::MatrixXd::Zero(N, 3);

    Eigen::Matrix3d G_xi_2 = Eigen::Matrix3d::Identity();
    G_xi_2(0, 2) = -vt_ * dt * std::sin(angular_half_delta);
    G_xi_2(1, 2) = vt_ * dt * std::cos(angular_half_delta); 
    G_xi.block(0,0,3,3) = G_xi_2;

    /* 构造 Gu' */
    Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 2);
    Eigen::MatrixXd G_u_2(3, 2);
    G_u_2 << dt * std::cos(angular_half_delta), -vt_ * dt * dt * std::sin(angular_half_delta) / 2,
        dt * std::sin(angular_half_delta), vt_ * dt * dt * std::cos(angular_half_delta) / 2,
        0, dt;
    G_u.block(0,0,3,2) = G_u_2;
    /* 更新协方差 */
    sigma_ = G_xi * sigma_ * G_xi.transpose() + G_u * Qu_ * G_u.transpose();
    /***** 更新均值 *****/
    mu_.topRows(3) += Eigen::Vector3d(delta_x, delta_y, delta_theta);
    mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2))); //norm
    time_ = now_time;

    Observation observation;
    if(!getObservations(*scan, observation))
    {
        std::cout << "do not have detect any reflector" << std::endl;
        return;
    }
    matched_ids result = detectMatchedIds(observation);
    const int M_ = result.map_obs_match_ids.size();
    const int M = result.state_obs_match_ids.size();
    const int MM = M + M_;
    if(MM > 0)
    {
        Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(2 * MM, N);
        Eigen::VectorXd zt = Eigen::VectorXd::Zero(2 * MM);
        Eigen::VectorXd zt_hat = Eigen::VectorXd::Zero(2 * MM);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2 * MM, 2 * MM);
        const double cos_theta = std::cos(mu_(2));
        const double sin_theta = std::sin(mu_(2));
        Eigen::Matrix2d B;
        B << cos_theta, sin_theta, -sin_theta, cos_theta;
        if(M > 0)
        {
            const auto xy = [=](const int &id) -> Eigen::Vector2d {
                return Eigen::Vector2d(mu_(3 + 2 * id), mu_(3 + 2 * id + 1));
            };
            for(int i = 0; i < M; ++i)
            {
                const int local_id = result.state_obs_match_ids[i].first;
                const int global_id = result.state_obs_match_ids[i].second;
                zt(2 * i) = observation.cloud_[local_id].x();
                zt(2 * i + 1) = observation.cloud_[local_id].y();
                const double delta_x = xy(global_id).x() - mu_(0);
                const double delta_y = xy(global_id).y() - mu_(1);
                zt_hat(2 * i) = delta_x * cos_theta + delta_y * sin_theta;
                zt_hat(2 * i + 1) = -delta_x * sin_theta + delta_y * cos_theta;
                Eigen::MatrixXd A_i(2, 3);
                A_i << -cos_theta, -sin_theta, -delta_x * sin_theta + delta_y * cos_theta,
                    sin_theta, -cos_theta, -delta_x * cos_theta - delta_y * sin_theta;
                H_t.block(2 * i, 0, 2, 3) = A_i;
                H_t.block(2 * i, 3 + 2 * global_id, 2, 2) = B;
                Q.block(2 * i, 2 * i, 2, 2) = Qt_;
            }
        }
        if(M_ > 0)
        {
            const auto xy = [&](const int &id) -> Eigen::Vector2f {
                return map_.reflector_map_[id];
            };

            for(int i = 0; i < M_; ++i)
            {
                const int local_id = result.map_obs_match_ids[i].first;
                const int global_id =result.map_obs_match_ids[i].second;
                zt(2 * (M + i)) = observation.cloud_[local_id].x();
                zt(2 * (M + i + 1)) = observation.cloud_[local_id].y();
                const double delta_x = xy(global_id).x() - mu_(0);
                const double delta_y = xy(global_id).y() - mu_(1);

                zt_hat(2 * (M + i)) = delta_x * cos_theta + delta_y * sin_theta;
                zt_hat(2 * (M + i + 1)) = -delta_x * sin_theta + delta_y * cos_theta;

                Eigen::MatrixXd A_i(2, 3);
                A_i << -cos_theta, -sin_theta, -delta_x * sin_theta + delta_y * cos_theta,
                    sin_theta, -cos_theta, -delta_x * cos_theta - delta_y * sin_theta;
                H_t.block(2 * (M + i), 0, 2, 3) = A_i;

                Q.block(2 * (M + i), 2 * (M + i), 2, 2) = Qt_;
            }
        }
        const auto K_t = sigma_ * H_t.transpose() * (H_t * sigma_ * H_t.transpose() + Q).inverse();
        mu_ += K_t * (zt - zt_hat);
        mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2)));
        sigma_ = sigma_ - K_t * H_t * sigma_;
    }
    
    const int N2 = result.new_ids.size();
    if(N2 > 0)
    {
        // increase X_estimate and coviarance size
        const int M_e = N + 2 * N2;
        Eigen::VectorXd tmp_xe = Eigen::VectorXd::Zero(M_e);
        tmp_xe.topRows(N) = mu_.topRows(N);

        Eigen::MatrixXd tmp_sigma = Eigen::MatrixXd::Zero(M_e, M_e);
        tmp_sigma.block(0, 0, N, N) = sigma_;
        const Eigen::Matrix3d sigma_xi = sigma_.block(0, 0, 3, 3);
        const double sin_theta = std::sin(mu_(2));
        const double cos_theta = std::cos(mu_(2));
        Eigen::Matrix2d G_z;
        G_z << cos_theta, -sin_theta, sin_theta, cos_theta;
        auto point_transformed = [&](const Eigen::Vector2f& p) -> Eigen::Vector2f{
            const float x = p.x() * std::cos(mu_(2)) + p.y() * std::sin(mu_(2)) + mu_(0);
            const float y = -p.x() * std::sin(mu_(2)) + p.y() * std::cos(mu_(2)) + mu_(1);
            return Eigen::Vector2f(x,y);
        };
        for (int i = 0; i < N2; i++)
        {
            const int local_id = result.new_ids[i];
            const auto point = point_transformed(observation.cloud_[local_id]);
            tmp_xe(N + 2 * i) =  point.x();
            tmp_xe(N + 2 * i + 1) = point.y();
            const double rx = observation.cloud_[local_id].x();
            const double ry = observation.cloud_[local_id].y();
            Eigen::MatrixXd Gp_i(2, 3);
            Gp_i << 1., 0., -rx * sin_theta - ry * cos_theta, 0., 1., rx * cos_theta - ry * sin_theta;
            Eigen::Matrix2d sigma_mm = Gp_i * sigma_xi * Gp_i.transpose() + G_z * Qt_ * G_z.transpose();
            Eigen::MatrixXd G_fx = Eigen::MatrixXd::Zero(2, N);
            G_fx.topLeftCorner(2, 3) = Gp_i;
            Eigen::MatrixXd sigma_mx = G_fx * sigma_;
            tmp_sigma.block(N, 0, 2, N) = sigma_mx;
            tmp_sigma.block(0, N, N, 2) = sigma_mx.transpose();
            tmp_sigma.block(N, N, 2, 2) = sigma_mm;
        }
        sigma_.resize(M_e, M_e);
        sigma_ = tmp_sigma;
        mu_.resize(M_e);
        mu_ = tmp_xe;
    }

    ekf_path_.header.stamp = scan->header.stamp;
    ekf_path_.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp =  scan->header.stamp;
    pose.pose.position.x = mu_(0);
    pose.pose.position.y = mu_(1);
    const double theta = mu_(2);
    pose.pose.orientation.x = 0.;
    pose.pose.orientation.y = 0.;
    pose.pose.orientation.z = std::sin(theta / 2);
    pose.pose.orientation.w = std::cos(theta / 2);
    ekf_path_.poses.push_back(pose);
}

bool ReflectorEKFSLAM::getObservations(const sensor_msgs::LaserScan& msg, Observation& obs)
{
    if (msg.range_min < 0 || msg.range_max <= msg.range_min)
    {
        std::cout << "Scan message range min and max is wrong" << std::endl;
        exit(-1);
    }
    if (msg.angle_increment < 0.f && msg.angle_max <= msg.angle_min)
    {
        std::cout << "Scan message angle min and max and angle increment is wrong" << std::endl;
        exit(-1);
    }
    // All reflector points
    std::vector<PointCloud> reflector_points;
    // All reflector centers
    PointCloud centers;
    // Now reflector points, ids and center
    PointCloud reflector;
    std::vector<int> reflector_id;
    Eigen::Vector2f now_center(0.f, 0.f);
    // All laser scan points in global frame
    PointCloud point_cloud;
   
    float angle = msg.angle_min;

    auto point_transformed = [&](const Eigen::Vector2f& p) -> Eigen::Vector2f{
        const float x = p.x() * std::cos(lidar_to_base_link_.z()) + p.y() * std::sin(lidar_to_base_link_.z()) + lidar_to_base_link_.x();
        const float y = -p.x() * std::sin(lidar_to_base_link_.z()) + p.y() * std::cos(lidar_to_base_link_.z()) + lidar_to_base_link_.y();
        return Eigen::Vector2f(x,y);
    };

    // Detect reflectors and motion distortion correction hear
    for (int i = 0; i < msg.ranges.size(); ++i)
    {
        // Get range data
        const float range = msg.ranges[i];
        if (msg.range_min <= range && range <= msg.range_max)
        {
            // Get now point xy value in sensor frame
            Eigen::Vector2f now_point(range * std::cos(angle), range * std::sin(angle));
            // Transform sensor point to odom frame and then transform to global frame
            point_cloud.push_back(point_transformed(now_point));

            // Detect reflector
            const double intensity = msg.intensities[i];
            if (intensity > intensity_min_)
            {
                // Add the first point
                if (reflector.empty())
                {
                    reflector.push_back(point_cloud.back());
                    reflector_id.push_back(i);
                    now_center += point_cloud.back();
                }
                else
                {
                    const int last_id = reflector_id.back();
                    // Add connected points
                    if (i - last_id == 1)
                    {
                        reflector.push_back(point_cloud.back());
                        reflector_id.push_back(i);
                        now_center += point_cloud.back();
                    }
                    else
                    {
                        // Calculate reflector length
                        const float reflector_length = std::hypotf(reflector.front().x() - reflector.back().x(),
                                                                reflector.front().y() - reflector.back().y());
                        // Add good reflector and its center
                        if (fabs(reflector_length - reflector_min_length_) < reflector_length_error_)
                        {
                            reflector_points.push_back(reflector);
                            centers.push_back(now_center / reflector.size());
                        }
                        // Update now reflector
                        reflector.clear();
                        reflector_id.clear();
                        now_center.setZero(2);
                        reflector_id.push_back(i);
                        reflector.push_back(point_cloud.back());
                        now_center += point_cloud.back();
                    }
                }
            }
        }
        angle += msg.angle_increment;
    }
    if (!reflector.empty())
    {
        const float reflector_length = std::hypotf(reflector.front().x() - reflector.back().x(),
                                                   reflector.front().y() - reflector.back().y());
        if (fabs(reflector_length - reflector_min_length_) < reflector_length_error_)
        {
            reflector_points.push_back(reflector);
            centers.push_back(now_center / reflector.size());
        }
    }
    if(centers.empty())
        return false;
    obs.time_ = msg.header.stamp.toSec();
    obs.cloud_ = centers;
    return true;
}

visualization_msgs::MarkerArray ReflectorEKFSLAM::toRosMarkers(double scale)
{

    visualization_msgs::MarkerArray markers;
    int N = 0;
    for(int i = 4; i < mu_.rows(); i+=2)
    {
        double& mx = mu_(i-1);
        double& my = mu_(i);


        /* 计算地图点的协方差椭圆角度以及轴长 */
        Eigen::Matrix2d sigma_m = sigma_.block(i-1, i-1, 2, 2); //协方差
        // Calculate Eigen Value(D) and Vectors(V), simga_m = V * D * V^-1
        // D = | D1 0  |  V = |cos  -sin|
        //     | 0  D2 |      |sin  cos |
        Eigen::EigenSolver<Eigen::Matrix2d> eigen_solver(sigma_m);
        const auto eigen_value = eigen_solver.pseudoEigenvalueMatrix();
        const auto eigen_vector = eigen_solver.pseudoEigenvectors();
        // Calculate angle and x y 
        const double angle = std::atan2(eigen_vector(1, 0), eigen_vector(0, 0));
        const double x_len = 2 * std::sqrt(eigen_value(0, 0) * 5.991);
        const double y_len = 2 * std::sqrt(eigen_value(1, 1) * 5.991);
        
        /* 构造marker */
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "ekf_slam";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = std::sin(angle / 2);
        marker.pose.orientation.w = std::cos(angle / 2);
        marker.scale.x = scale * x_len;
        marker.scale.y = scale * y_len;
        marker.scale.z = 0.1 * scale * (x_len + y_len);
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        
        markers.markers.push_back(marker);
    }// for all mpts
    
    return markers;
}

geometry_msgs::PoseWithCovarianceStamped ReflectorEKFSLAM::toRosPose()
{
    /* 转换带协方差的机器人位姿 */
    geometry_msgs::PoseWithCovarianceStamped rpose;
    rpose.header.frame_id = "world";
    rpose.pose.pose.position.x = mu_(0);
    rpose.pose.pose.position.y = mu_(1);
    rpose.pose.pose.orientation.x = 0.0;
    rpose.pose.pose.orientation.y = 0.;
    rpose.pose.pose.orientation.z = std::sin(mu_(2) / 2);
    rpose.pose.pose.orientation.w = std::cos(mu_(2) / 2);
    
    rpose.pose.covariance.at(0) = sigma_(0,0);
    rpose.pose.covariance.at(1) = sigma_(0,1);
    rpose.pose.covariance.at(6) = sigma_(1,0);
    rpose.pose.covariance.at(7) = sigma_(1,1);
    rpose.pose.covariance.at(5) = sigma_(0,2);
    rpose.pose.covariance.at(30) = sigma_(2,0);
    rpose.pose.covariance.at(35) = sigma_(2,2);
    
    return rpose;
}

void ReflectorEKFSLAM::normAngle ( double& angle )
{
    const static double PI = 3.1415926;
    static double Two_PI = 2.0 * PI;
    if( angle >= PI)
        angle -= Two_PI;
    if( angle < -PI)
        angle += Two_PI;
}

matched_ids ReflectorEKFSLAM::detectMatchedIds(const Observation& obs)
{
    matched_ids ids;
    if(obs.cloud_.empty())
    {
        std::cout << "should never reach here" << std::endl;
        exit(-1);
    }
    if (mu_.rows() == 3 && map_.reflector_map_.empty())
    {
        for (int i = 0; i < obs.cloud_.size(); ++i)
            ids.new_ids.push_back(i);
        return ids;
    }

    auto point_transformed = [&](const Eigen::Vector2f& p) -> Eigen::Vector2f{
        const float x = p.x() * std::cos(mu_(2)) + p.y() * std::sin(mu_(2)) + mu_(0);
        const float y = -p.x() * std::sin(mu_(2)) + p.y() * std::cos(mu_(2)) + mu_(1);
        return Eigen::Vector2f(x,y);
    };
    const int M = (mu_.rows() - 3) / 2;
    const int M_ = map_.reflector_map_.size();
    for(int i = 0; i < obs.cloud_.size(); ++i)
    {
        const auto reflector = point_transformed(obs.cloud_[i]);
        // Match with global map
        if (M_ > 0)
        {
            std::vector<std::pair<double, int>> distance_id;
            for (int j = 0; j < M_; ++j)
            {
                // Get global reflector covariance
                const Eigen::Matrix2d sigma = map_.reflector_map_coviarance_[j];
                const Eigen::Vector2f delta_state = map_.reflector_map_[j] - reflector;
                const auto delta_double_state = delta_state.cast<double>().transpose();
                // Calculate Ma distance
                const double dist = std::sqrt(delta_double_state * sigma * delta_double_state.transpose());
                distance_id.push_back({dist, j});
            }
            std::sort(distance_id.begin(), distance_id.end(),
                    [](const std::pair<double, int> &left,
                        const std::pair<double, int> &right) {
                        return left.first <= right.first;
                    });
            const auto best_match = distance_id.front();
            if (best_match.first < 0.05)
            {
              ids.map_obs_match_ids.push_back({i, best_match.second});
              continue;
            }
        }
        if (M > 0)
        {
            std::vector<std::pair<double, int>> distance_id;
            for (int j = 0; j < M; ++j)
            {
                Eigen::Vector2f global_reflector(mu_(3 + 2 * j),mu_(3 + 2 * j + 1));
                const Eigen::Matrix2d sigma = sigma_.block(3 + 2 * j, 3 + 2 * j, 2, 2);
                const Eigen::Vector2f delta_state = reflector - global_reflector;
                const auto delta_double_state = delta_state.cast<double>().transpose();
                // Calculate Ma distance
                const double dist = std::sqrt(delta_double_state * sigma * delta_double_state.transpose());
                distance_id.push_back({dist, j});
            }
            std::sort(distance_id.begin(), distance_id.end(),
                    [](const std::pair<double, int> &left,
                        const std::pair<double, int> &right) {
                        return left.second <= right.second;
                    });
            const auto best_match = distance_id.front();
            if (best_match.first < 0.05)
            {
                ids.state_obs_match_ids.push_back({i, best_match.second});
                continue;
            }
        }
        ids.new_ids.push_back(i);
    }
    return ids;
}

void ReflectorEKFSLAM::loadFromVector(const std::vector<std::vector<double>>& vecs)
{
    PointCloud reflector_map;
    PointCloudCoviarance reflector_map_coviarance;
    if(vecs.empty())
        return;
    if(vecs[0].size() % 2 == 1)
        return;
    for(int i = 0; i < vecs[0].size() / 2; ++i)
    {
        reflector_map.push_back(Eigen::Vector2f(vecs[0][2 * i], vecs[0][2 * i + 1]));
    }
    for(int i = 0; i < vecs[1].size() / 4; ++i)
    {
        Eigen::Matrix2d p;
        p << vecs[0][4 * i], vecs[0][4 * i + 1], vecs[0][4 * i + 2], vecs[0][4 * i + 3];
        reflector_map_coviarance.push_back(p);
    }
    map_.reflector_map_ = reflector_map;
    map_.reflector_map_coviarance_ = reflector_map_coviarance;
}


