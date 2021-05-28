#include <reflector_ekf_slam/reflector_ekf_slam_gps.h>
#include <glog/logging.h>

namespace ekf
{
ReflectorEKFSLAMGPS::ReflectorEKFSLAMGPS(const EKFOptions &options) : options_(options), vt_(Eigen::Vector3d::Zero())
{
    state_.time = options_.init_time;
    state_.mu = options_.init_pose;
    state_.sigma.resize(3, 3);
    state_.sigma.setZero();

    switch (options_.odom_model)
    {
    case sensor::OdometryModel::DIFF:
        Qu_ = Eigen::MatrixXd::Zero(2, 2);
        Qu_ << options_.linear_velocity_cov, 0.f,
            0.f, options_.angular_velocity_cov;
        break;
    case sensor::OdometryModel::OMNI:
        Qu_ = Eigen::MatrixXd::Zero(3, 3);
        Qu_ << options_.linear_velocity_cov, 0.f, 0.f,
            0.f, options_.linear_velocity_cov, 0.f,
            0.f, 0.f, options_.angular_velocity_cov;
        break;
    default:
        Qu_ = Eigen::MatrixXd::Zero(3, 3);
        Qu_ << options_.linear_velocity_cov, 0.f, 0.f,
            0.f, options_.linear_velocity_cov, 0.f,
            0.f, 0.f, options_.angular_velocity_cov;
        break;
    }
    Qt_ << options_.observation_cov, 0.f,
        0.f, options_.observation_cov;
    // Load map
    LoadMapFromTxtFile(options_.map_path);
}

ReflectorEKFSLAMGPS::~ReflectorEKFSLAMGPS()
{
}

void ReflectorEKFSLAMGPS::LoadMapFromTxtFile(const std::string &file)
{
    if (file.empty() || !IsFileExist(file))
        return;
    std::ifstream in(file.c_str());
    std::string line;
    std::vector<std::vector<double>> result;
    if (in) // 有该文件
    {
        while (getline(in, line)) // line中不包括每行的换行符
        {
            LOG(INFO) << line;
            if (!line.empty())
            {
                std::vector<double> vec;
                auto vstr_vec = SplitString(line, ',');
                for (auto &p : vstr_vec)
                {
                    vec.push_back(std::stod(p));
                }
                result.push_back(vec);
            }
        }
    }
    else // 没有该文件
    {
        // std::cout << "No map file in the map path!" << std::endl;
        LOG(INFO) << "No map file in the map path!";
        return;
    }

    if (result.size() != 2 || result.back().size() != 2 * result.front().size())
    {
        // std::cout <<"format is not right, must be 2 line" << std::endl;
        LOG(INFO) << "Format is not right, must be 2 line";
        return;
    }
    sensor::PointCloud reflector_map;
    sensor::PointCloudCoviarance reflector_map_coviarance;

    for (int i = 0; i < result[0].size() / 2; ++i)
    {
        reflector_map.push_back(Eigen::Vector2f(result[0][2 * i], result[0][2 * i + 1]));
    }
    for (int i = 0; i < result[1].size() / 4; ++i)
    {
        Eigen::Matrix2d p;
        p << result[0][4 * i], result[0][4 * i + 1], result[0][4 * i + 2], result[0][4 * i + 3];
        reflector_map_coviarance.push_back(p);
    }
    map_.reflector_map_ = reflector_map;
    map_.reflector_map_coviarance_ = reflector_map_coviarance;
}

State ReflectorEKFSLAMGPS::PredictState(const double &time)
{
    State result = state_;
    const double dt = time - state_.time;
    if (options_.odom_model == sensor::OdometryModel::DIFF)
    {
        const double delta_theta = vt_.z() * dt;
        const double delta_x = vt_.x() * dt * std::cos(state_.mu(2) + delta_theta / 2);
        const double delta_y = vt_.x() * dt * std::sin(state_.mu(2) + delta_theta / 2);

        const int N = state_.mu.rows();
        /***** 更新协方差 *****/
        /* 构造 Gt */
        const double angular_half_delta = state_.mu(2) + delta_theta / 2;
        Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
        G_xi(0, 2) = -vt_.x() * dt * std::sin(angular_half_delta);
        G_xi(1, 2) = vt_.x() * dt * std::cos(angular_half_delta);

        /* 构造 Gu' */
        Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 2);
        Eigen::MatrixXd G_u_2(3, 2);
        G_u_2 << dt * std::cos(angular_half_delta), -vt_.x() * dt * dt * std::sin(angular_half_delta) / 2,
            dt * std::sin(angular_half_delta), vt_.x() * dt * dt * std::cos(angular_half_delta) / 2,
            0, dt;
        G_u.block(0, 0, 3, 2) = G_u_2;
        /* 更新协方差 */
        result.sigma = G_xi * state_.sigma * G_xi.transpose() + G_u * Qu_ * G_u.transpose();
        /***** 更新均值 *****/
        result.mu.topRows(3) = state_.mu.topRows(3) + Eigen::Vector3d(delta_x, delta_y, delta_theta);
        result.mu(2) = std::atan2(std::sin(result.mu(2)), std::cos(result.mu(2))); //norm
        return result;
    }
    const double delta_theta = vt_.z() * dt;
    const double delta_x = vt_.x() * dt * std::cos(state_.mu(2)) - vt_.y() * dt * std::sin(state_.mu(2));
    const double delta_y = vt_.x() * dt * std::sin(state_.mu(2)) + vt_.y() * dt * std::cos(state_.mu(2));
    const int N = state_.mu.rows();
    /***** 更新协方差 *****/
    /* 构造 Gt */
    Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
    G_xi(0, 2) = -vt_.x() * dt * std::sin(state_.mu(2)) - vt_.y() * dt * std::cos(state_.mu(2));
    G_xi(1, 2) = vt_.x() * dt * std::cos(state_.mu(2)) - vt_.y() * dt * std::sin(state_.mu(2));

    /* 构造 Gu' */
    Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 3);
    Eigen::Matrix3d G_u_2;
    G_u_2 << dt * std::cos(state_.mu(2)), -dt * std::sin(state_.mu(2)), 0.,
        dt * std::sin(state_.mu(2)), dt * std::cos(state_.mu(2)), 0.,
        0., 0., dt;
    G_u.block(0, 0, 3, 3) = G_u_2;
    /* 更新协方差 */
    result.sigma = G_xi * state_.sigma * G_xi.transpose() + G_u * Qu_ * G_u.transpose();
    /***** 更新均值 *****/
    result.mu.topRows(3) = state_.mu.topRows(3) + Eigen::Vector3d(delta_x, delta_y, delta_theta);
    result.mu(2) = std::atan2(std::sin(result.mu(2)), std::cos(result.mu(2))); //norm
    return result;
}

void ReflectorEKFSLAMGPS::Predict(const double &dt)
{
    if (options_.odom_model == sensor::OdometryModel::DIFF)
    {
        const double delta_theta = vt_.z() * dt;
        const double delta_x = vt_.x() * dt * std::cos(state_.mu(2) + delta_theta / 2);
        const double delta_y = vt_.x() * dt * std::sin(state_.mu(2) + delta_theta / 2);

        const int N = state_.mu.rows();
        /***** 更新协方差 *****/
        /* 构造 Gt */
        const double angular_half_delta = state_.mu(2) + delta_theta / 2;
        Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
        G_xi(0, 2) = -vt_.x() * dt * std::sin(angular_half_delta);
        G_xi(1, 2) = vt_.x() * dt * std::cos(angular_half_delta);

        /* 构造 Gu' */
        Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 2);
        Eigen::MatrixXd G_u_2(3, 2);
        G_u_2 << dt * std::cos(angular_half_delta), -vt_.x() * dt * dt * std::sin(angular_half_delta) / 2,
            dt * std::sin(angular_half_delta), vt_.x() * dt * dt * std::cos(angular_half_delta) / 2,
            0, dt;
        G_u.block(0, 0, 3, 2) = G_u_2;
        /* 更新协方差 */
        state_.sigma = G_xi * state_.sigma * G_xi.transpose() + G_u * Qu_ * G_u.transpose();
        /***** 更新均值 *****/
        state_.mu.topRows(3) += Eigen::Vector3d(delta_x, delta_y, delta_theta);
        state_.mu(2) = std::atan2(std::sin(state_.mu(2)), std::cos(state_.mu(2))); //norm
        return;
    }
    const double delta_theta = vt_.z() * dt;
    const double delta_x = vt_.x() * dt * std::cos(state_.mu(2)) - vt_.y() * dt * std::sin(state_.mu(2));
    const double delta_y = vt_.x() * dt * std::sin(state_.mu(2)) + vt_.y() * dt * std::cos(state_.mu(2));
    const int N = state_.mu.rows();
    /***** 更新协方差 *****/
    /* 构造 Gt */
    Eigen::MatrixXd G_xi = Eigen::MatrixXd::Identity(N, N);
    G_xi(0, 2) = -vt_.x() * dt * std::sin(state_.mu(2)) - vt_.y() * dt * std::cos(state_.mu(2));
    G_xi(1, 2) = vt_.x() * dt * std::cos(state_.mu(2)) - vt_.y() * dt * std::sin(state_.mu(2));

    /* 构造 Gu' */
    Eigen::MatrixXd G_u = Eigen::MatrixXd::Zero(N, 3);
    Eigen::Matrix3d G_u_2;
    G_u_2 << dt * std::cos(state_.mu(2)), -dt * std::sin(state_.mu(2)), 0.,
        dt * std::sin(state_.mu(2)), dt * std::cos(state_.mu(2)), 0.,
        0., 0., dt;
    G_u.block(0, 0, 3, 3) = G_u_2;
    /* 更新协方差 */
    state_.sigma = G_xi * state_.sigma * G_xi.transpose() + G_u * Qu_ * G_u.transpose();
    /***** 更新均值 *****/
    state_.mu.topRows(3) += Eigen::Vector3d(delta_x, delta_y, delta_theta);
    state_.mu(2) = std::atan2(std::sin(state_.mu(2)), std::cos(state_.mu(2))); //norm
}

void ReflectorEKFSLAMGPS::HandleOdometryMessage(const sensor::OdometryData &odometry)
{
    // drop old data
    if (odometry.time < state_.time)
        return;
    if (!options_.use_imu)
    {
        /***** 保存上一帧编码器数据 *****/
        vt_ = Eigen::Vector3d(odometry.linear_velocity.x(), odometry.linear_velocity.y(), odometry.angular_velocity.z());
        const double dt = odometry.time - state_.time;
        Predict(dt);
        state_.time = odometry.time;
        return;
    }
    // use imu
}
void ReflectorEKFSLAMGPS::HandleImuMessage(const sensor::ImuData &imu)
{
    // use imu
}

void ReflectorEKFSLAMGPS::HandleObservationMessage(const sensor::Observation &observation)
{
    // Predict now pose
    const double dt = observation.time_ - state_.time;
    Predict(dt);
    state_.time = observation.time_;
    if (observation.cloud_.empty())
        return;
    ReflectorMatchResult result = ReflectorMatch(observation);
    const int M_ = result.map_obs_match_ids.size();
    const int M = result.state_obs_match_ids.size();
    // std::cout << "Match with old map size is: " << M_ << std::endl;
    // std::cout << "Match with state vector size is: " << M << std::endl;
    LOG(INFO) << "Match with old map size is: " << M_;
    LOG(INFO) << "Match with state vector size is: " << M;
    const int MM = M + M_;
    const int N = state_.mu.rows();
    if (MM > 0)
    {
        Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(2 * MM, N);
        Eigen::VectorXd zt = Eigen::VectorXd::Zero(2 * MM);
        Eigen::VectorXd zt_hat = Eigen::VectorXd::Zero(2 * MM);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2 * MM, 2 * MM);
        const double cos_theta = std::cos(state_.mu(2));
        const double sin_theta = std::sin(state_.mu(2));
        Eigen::Matrix2d B;
        B << cos_theta, sin_theta, -sin_theta, cos_theta;
        if (M > 0)
        {
            const auto xy = [&](const int &id) -> Eigen::Vector2d {
                return Eigen::Vector2d(state_.mu(3 + 2 * id), state_.mu(3 + 2 * id + 1));
            };
            for (int i = 0; i < M; ++i)
            {
                const int local_id = result.state_obs_match_ids[i].first;
                const int global_id = result.state_obs_match_ids[i].second;
                zt(2 * i) = observation.cloud_[local_id].x();
                zt(2 * i + 1) = observation.cloud_[local_id].y();
                const double delta_x = xy(global_id).x() - state_.mu(0);
                const double delta_y = xy(global_id).y() - state_.mu(1);
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
        if (M_ > 0)
        {
            const auto xy = [&](const int &id) -> Eigen::Vector2f {
                return map_.reflector_map_[id];
            };

            for (int i = 0; i < M_; ++i)
            {
                const int local_id = result.map_obs_match_ids[i].first;
                const int global_id = result.map_obs_match_ids[i].second;
                zt(2 * (M + i)) = observation.cloud_[local_id].x();
                zt(2 * (M + i) + 1) = observation.cloud_[local_id].y();
                const double delta_x = xy(global_id).x() - state_.mu(0);
                const double delta_y = xy(global_id).y() - state_.mu(1);

                zt_hat(2 * (M + i)) = delta_x * cos_theta + delta_y * sin_theta;
                zt_hat(2 * (M + i) + 1) = -delta_x * sin_theta + delta_y * cos_theta;

                Eigen::MatrixXd A_i(2, 3);
                A_i << -cos_theta, -sin_theta, -delta_x * sin_theta + delta_y * cos_theta,
                    sin_theta, -cos_theta, -delta_x * cos_theta - delta_y * sin_theta;
                H_t.block(2 * (M + i), 0, 2, 3) = A_i;

                Q.block(2 * (M + i), 2 * (M + i), 2, 2) = Qt_;
            }
        }
        if (!observation.gps_pose_)
        {
            const auto K_t = state_.sigma * H_t.transpose() * (H_t * state_.sigma * H_t.transpose() + Q).inverse();
            state_.mu += K_t * (zt - zt_hat);
            state_.mu(2) = std::atan2(std::sin(state_.mu(2)), std::cos(state_.mu(2)));
            state_.sigma = state_.sigma - K_t * H_t * state_.sigma;
        }
        else
        {
            Eigen::MatrixXd H_t_2 = Eigen::MatrixXd::Zero(2 * MM + 3, N);
            H_t_2.topRows(2 * MM) = H_t;
            H_t_2.block(2 * MM, 0, 3, 3) = Eigen::Matrix3d::Identity();
            Eigen::VectorXd zt_2 = Eigen::VectorXd::Zero(2 * MM + 3);
            zt_2.topRows(2 * MM) = zt;
            zt_2.bottomRows(3) = Eigen::Vector3d(observation.gps_pose_->translation().x(),
                                                 observation.gps_pose_->translation().y(),
                                                 observation.gps_pose_->rotation().angle());
            Eigen::VectorXd zt_hat_2 = Eigen::VectorXd::Zero(2 * MM + 3);
            zt_hat_2.topRows(2 * MM) = zt_hat;
            zt_hat_2.bottomRows(3) = state_.mu.topRows(3);
            Eigen::VectorXd delta_zt = zt_2 - zt_hat_2;
            const double delta_theta = delta_zt(2 * MM + 2);
            Eigen::Quaterniond dq(std::cos(delta_theta / 2), 0., 0., std::sin(delta_theta / 2));
            delta_zt(2 * MM + 2) = transform::RotationQuaternionToAngleAxisVector(dq)(2);
            Eigen::MatrixXd Q_2 = Eigen::MatrixXd::Zero(2 * MM + 3, 2 * MM + 3);
            Q_2.block(0, 0, 2 * MM, 2 * MM) = Q;
            Eigen::Matrix3d pose_coviarance;
            pose_coviarance << 0.05 * 0.05, 0., 0.,
                0., 0.05 * 0.05, 0.,
                0., 0., 0.017 * 0.017;
            Q_2.block(2 * MM, 2 * MM, 3, 3) = pose_coviarance;
            const auto K_t = state_.sigma * H_t_2.transpose() * (H_t_2 * state_.sigma * H_t_2.transpose() + Q_2).inverse();
            state_.mu += K_t * delta_zt;
            state_.mu(2) = std::atan2(std::sin(state_.mu(2)), std::cos(state_.mu(2)));
            state_.sigma = state_.sigma - K_t * H_t_2 * state_.sigma;
        }
    }

    const int N2 = result.new_ids.size();
    if (N2 > 0)
    {
        LOG(INFO) << "Add " << N2 << " reflectors";
        // increase X_estimate and coviarance size
        const int M_e = N + 2 * N2;
        Eigen::VectorXd tmp_xe = Eigen::VectorXd::Zero(M_e);
        tmp_xe.topRows(N) = state_.mu.topRows(N);

        Eigen::MatrixXd tmp_sigma = Eigen::MatrixXd::Zero(M_e, M_e);
        tmp_sigma.block(0, 0, N, N) = state_.sigma;
        const Eigen::Matrix3d sigma_xi = state_.sigma.block(0, 0, 3, 3);
        const double sin_theta = std::sin(state_.mu(2));
        const double cos_theta = std::cos(state_.mu(2));
        Eigen::Matrix2d G_zi;
        G_zi << cos_theta, -sin_theta, sin_theta, cos_theta;
        auto point_transformed_to_global_frame = [&](const Eigen::Vector2f &p) -> Eigen::Vector2f {
            const float x = p.x() * std::cos(state_.mu(2)) - p.y() * std::sin(state_.mu(2)) + state_.mu(0);
            const float y = p.x() * std::sin(state_.mu(2)) + p.y() * std::cos(state_.mu(2)) + state_.mu(1);
            return Eigen::Vector2f(x, y);
        };
        Eigen::MatrixXd G_p(2 * N2, 3);
        Eigen::MatrixXd G_z(2 * N2, 2);
        Eigen::MatrixXd G_fx = Eigen::MatrixXd::Zero(2 * N2, N);

        for (int i = 0; i < N2; i++)
        {
            const int local_id = result.new_ids[i];
            const auto point = point_transformed_to_global_frame(observation.cloud_[local_id]);
            // update state vector
            tmp_xe(N + 2 * i) = point.x();
            tmp_xe(N + 2 * i + 1) = point.y();
            // for update cov
            const double rx = observation.cloud_[local_id].x();
            const double ry = observation.cloud_[local_id].y();
            Eigen::MatrixXd Gp_i(2, 3);
            Gp_i << 1., 0., -rx * sin_theta - ry * cos_theta, 0., 1., rx * cos_theta - ry * sin_theta;
            G_p.block(2 * i, 0, 2, 3) = Gp_i;
            G_z.block(2 * i, 0, 2, 2) = G_zi;
            Eigen::MatrixXd G_fx_i = Eigen::MatrixXd::Zero(2, N);
            G_fx_i.topLeftCorner(2, 3) = Gp_i;
            G_fx.block(2 * i, 0, 2, N) = G_fx_i;
        }
        const auto sigma_mm = G_p * sigma_xi * G_p.transpose() + G_z * Qt_ * G_z.transpose();
        const auto sigma_mx = G_fx * state_.sigma;
        tmp_sigma.block(N, 0, 2 * N2, N) = sigma_mx;
        tmp_sigma.block(0, N, N, 2 * N2) = sigma_mx.transpose();
        tmp_sigma.block(N, N, 2 * N2, 2 * N2) = sigma_mm;

        state_.sigma.resize(M_e, M_e);
        state_.sigma = tmp_sigma;
        state_.mu.resize(M_e);
        state_.mu = tmp_xe;
    }
    LOG(INFO) << "Update now pose is: " << state_.mu(0) << "," << state_.mu(1) << "," << state_.mu(2);
    LOG(INFO) << "state vector:  \n"
              << state_.mu;
}

ReflectorMatchResult ReflectorEKFSLAMGPS::ReflectorMatch(const sensor::Observation &obs)
{
    ReflectorMatchResult ids;
    if (obs.cloud_.empty())
    {
        // std::cout << "should never reach here" << std::endl;
        LOG(ERROR) << "Should never reach here";
        exit(-1);
    }
    if (state_.mu.rows() == 3 && map_.reflector_map_.empty())
    {
        for (int i = 0; i < obs.cloud_.size(); ++i)
            ids.new_ids.push_back(i);

        // std::cout << "\n >>>Reflector map is empty.\n";
        LOG(ERROR) << "Reflector map is empty";
        return ids;
    }

    auto point_transformed_to_global_frame = [&](const Eigen::Vector2f &p) -> Eigen::Vector2f {
        const float x = p.x() * std::cos(state_.mu(2)) - p.y() * std::sin(state_.mu(2)) + state_.mu(0);
        const float y = p.x() * std::sin(state_.mu(2)) + p.y() * std::cos(state_.mu(2)) + state_.mu(1);
        return Eigen::Vector2f(x, y);
    };

    const int M = (state_.mu.rows() - 3) / 2;
    const int M_ = map_.reflector_map_.size();
    for (int i = 0; i < obs.cloud_.size(); ++i)
    {
        const auto reflector = point_transformed_to_global_frame(obs.cloud_[i]);
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
                Eigen::Vector2f global_reflector(state_.mu(3 + 2 * j), state_.mu(3 + 2 * j + 1));
                const Eigen::Matrix2d sigma = state_.sigma.block(3 + 2 * j, 3 + 2 * j, 2, 2);
                const Eigen::Vector2f delta_state = reflector - global_reflector;
                const auto delta_double_state = delta_state.cast<double>().transpose();
                // Calculate Ma distance
                // const double dist = std::sqrt(delta_double_state * sigma * delta_double_state.transpose());
                const double dist = std::sqrt(delta_double_state * delta_double_state.transpose());
                distance_id.push_back({dist, j});
            }
            std::sort(distance_id.begin(), distance_id.end(),
                      [](const std::pair<double, int> &left,
                         const std::pair<double, int> &right) {
                          return left.first <= right.first;
                      });
            const auto best_match = distance_id.front();
            if (best_match.first < 0.6)
            {
                ids.state_obs_match_ids.push_back({i, best_match.second});
                continue;
            }
        }
        ids.new_ids.push_back(i);
    }
    return ids;
}

} // namespace ekf
