#include "my_mpc_controller/mpc_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

extern "C" {
#include "osqp/osqp.h"
}

using nav2_util::declare_parameter_if_not_declared;

namespace my_mpc_controller {

    void MyMPCController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

        node_ = parent;
        auto node = node_.lock();
        tf_ = tf;
        plugin_name_ = name;
        costmap_ros_ = costmap_ros;
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".q_11", rclcpp::ParameterValue(30.0));
        node->get_parameter(plugin_name_ + ".q_11", q_11);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".q_22", rclcpp::ParameterValue(30.0));
        node->get_parameter(plugin_name_ + ".q_22", q_22);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".q_33", rclcpp::ParameterValue(30.0));
        node->get_parameter(plugin_name_ + ".q_33", q_33);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".r_11", rclcpp::ParameterValue(0.1));
        node->get_parameter(plugin_name_ + ".r_11", r_11);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".r_22", rclcpp::ParameterValue(0.1));
        node->get_parameter(plugin_name_ + ".r_22", r_22);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".r_33", rclcpp::ParameterValue(0.5));
        node->get_parameter(plugin_name_ + ".r_33", r_33);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".f_11", rclcpp::ParameterValue(60.0));
        node->get_parameter(plugin_name_ + ".f_11", f_11);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".f_22", rclcpp::ParameterValue(60.0));
        node->get_parameter(plugin_name_ + ".f_22", f_22);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".f_33", rclcpp::ParameterValue(60.0));
        node->get_parameter(plugin_name_ + ".f_33", f_33);

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".v_max", rclcpp::ParameterValue(2.0));
        node->get_parameter(plugin_name_ + ".v_max", max_v_);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".w_max", rclcpp::ParameterValue(1.0));
        node->get_parameter(plugin_name_ + ".w_max", max_w_);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".a_v_max", rclcpp::ParameterValue(1.5));
        node->get_parameter(plugin_name_ + ".a_v_max", max_a_v_);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".a_w_max", rclcpp::ParameterValue(1.0));
        node->get_parameter(plugin_name_ + ".a_w_max", max_a_w_);

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".terminal_control_weight", rclcpp::ParameterValue(20.0));
        node->get_parameter(
            plugin_name_ + ".terminal_control_weight", terminal_control_weight_);


        Q_.setIdentity();
        Q_(0, 0) = q_11; // x 权重
        Q_(1, 1) = q_22; // y 权重
        Q_(2, 2) = q_33;  // theta 权重

        R_.setIdentity();
        R_(0, 0) = r_11; // v_x 权重
        R_(1, 1) = r_22; // v_y 权重
        R_(2, 2) = r_33; // w 权重

        F_.setIdentity();
        F_(0, 0) = f_11;
        F_(1, 1) = f_22;
        F_(2, 2) = f_33;

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".n", rclcpp::ParameterValue(10));
        node->get_parameter(plugin_name_ + ".n", N_);

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".track_path_yaw", rclcpp::ParameterValue(true));
        node->get_parameter(plugin_name_ + ".track_path_yaw", track_path_yaw_);

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
        double transform_tolerance = 0.1;
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

        // 调试用
        mpc_debug_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("/mpc_debug/u", 1);
        ref_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(plugin_name_ + "/target_path", 10);

        RCLCPP_INFO(logger_, "自定义MPC控制器配置完成");
    }

    void MyMPCController::setPlan(const nav_msgs::msg::Path &path) {

        global_plan_ = path;
        processed_path_ = preprocessPath(global_plan_);
        // 新路径的索引空间和旧路径无关，必须重置，否则 sampleReferencePath()
        // 会从旧索引开始搜索，轻则参考点错误，重则越界访问。
        last_closest_index_ = 0;
    }

    std::vector<PathPoint> MyMPCController::preprocessPath(const nav_msgs::msg::Path &path) {

        std::vector<PathPoint> processed_path;
        if (path.poses.empty()) return processed_path;
        const std::size_t pose_count = path.poses.size();
        double cumulative_s = 0.0;
        processed_path.reserve(pose_count);

        for (std::size_t i = 0; i < pose_count; ++ i) {

            PathPoint pt;
            pt.x = path.poses[i].pose.position.x;
            pt.y = path.poses[i].pose.position.y;

            // s
            if (i > 0) {

                double
                    dx = pt.x - processed_path.back().x,
                    dy = pt.y - processed_path.back().y;
                cumulative_s += std::hypot(dx, dy);
            }
            pt.s = cumulative_s;

            // yaw
            if (i + 1 < pose_count) {

                double
                    dx_next = path.poses[i + 1].pose.position.x - pt.x,
                    dy_next = path.poses[i + 1].pose.position.y - pt.y;
                pt.theta = std::atan2(dy_next, dx_next);
            } else if (i > 0) {

                pt.theta = processed_path.back().theta;
            } else {

                pt.theta = tf2::getYaw(path.poses[i].pose.orientation);
            }

            processed_path.push_back(pt);
        }
        return processed_path;
    }

    /**
     * @brief 线性插值辅助函数
     * @param path 原始路径
     * @param s_target 目标 s 值
     * @param start_search_idx 起始搜索索引
     * @return Eigen::Vector3d 目标点坐标
     */
    Eigen::Vector3d interpolatePathByS(
        const std::vector<PathPoint>& path,
        double s_target,
        int start_search_idx
    ) {

        const int point_count = static_cast<int>(path.size());
        if (point_count == 1 || start_search_idx >= point_count - 1) {

            return Eigen::Vector3d(path.back().x, path.back().y, path.back().theta);
        }

        const int first_idx = std::max(0, start_search_idx);
        for (int i = first_idx; i < point_count - 1; ++ i) {

            const double segment_length = path[i + 1].s - path[i].s;
            if (segment_length <= 1e-12) {

                continue;
            }

            if (s_target >= path[i].s && s_target <= path[i + 1].s) {

                const double rate = (s_target - path[i].s) / segment_length;

                const double x = path[i].x + rate * (path[i + 1].x - path[i].x);
                const double y = path[i].y + rate * (path[i + 1].y - path[i].y);

                // 角度插值
                double diff = path[i + 1].theta - path[i].theta;
                while (diff > M_PI) diff -= 2 * M_PI;
                while (diff < -M_PI) diff += 2 * M_PI;
                const double theta = path[i].theta + rate * diff;

                return Eigen::Vector3d(x, y, theta);
            }
        }
        return Eigen::Vector3d(path.back().x, path.back().y, path.back().theta);
    }

    /**
     * @brief MPC 路径采样函数
     * @param global_path 原始路径 (nav_msgs::msg::Path 转换后的结构)
     * @param cur_pose 当前机器人位姿 [x, y, theta]
     * @param v_ref 当前参考速度 (可以是动态计算的限速)
     * @param N 预测步长
     * @param dt 采样周期
     * @return 采样后的参考序列 x_ref [x0, y0, th0, x1, y1, th1, ...]
     */
    Eigen::VectorXd MyMPCController::sampleReferencePath(
        const std::vector<PathPoint> &global_path,
        const Eigen::Vector3d &cur_pose,
        double v_ref,
        int N,
        double dt
    ) {

        Eigen::VectorXd x_ref_vec = Eigen::VectorXd::Zero(3 * (N + 1));
        const int point_count = static_cast<int>(global_path.size());
        if (point_count == 0) {

            return x_ref_vec;
        }

        if (point_count == 1) {

            const double ref_yaw =
                track_path_yaw_ ? global_path[0].theta : cur_pose(2);
            x_ref_vec.segment<3>(0) <<
                global_path[0].x, global_path[0].y, ref_yaw;
            for (int i = 1; i <= N; ++ i) {

                x_ref_vec.segment<3>(i * 3) = x_ref_vec.segment<3>(0);
            }
            return x_ref_vec;
        }

        // 把机器人位置投影到最近路径线段上，得到连续的参考弧长 s_start。
        // 比“找最近离散点”更平滑，横向偏差大时也不会有明显滞后。
        int best_segment_idx = 0;
        double best_s = global_path.front().s;
        double best_dist_sq = std::numeric_limits<double>::max();
        for (int i = 0; i + 1 < point_count; ++ i) {

            const double segment_len = global_path[i + 1].s - global_path[i].s;
            if (segment_len <= 1e-12) {

                continue;
            }

            const double
                dx = global_path[i + 1].x - global_path[i].x,
                dy = global_path[i + 1].y - global_path[i].y;
            const double segment_len_sq = dx * dx + dy * dy;
            if (segment_len_sq <= 1e-12) {

                continue;
            }

            const double rel_x = cur_pose(0) - global_path[i].x;
            const double rel_y = cur_pose(1) - global_path[i].y;
            const double t = std::clamp(
                (rel_x * dx + rel_y * dy) / segment_len_sq, 0.0, 1.0);
            const double
                proj_x = global_path[i].x + t * dx,
                proj_y = global_path[i].y + t * dy,
                dist_sq = (cur_pose(0) - proj_x) * (cur_pose(0) - proj_x) +
                          (cur_pose(1) - proj_y) * (cur_pose(1) - proj_y);
            if (dist_sq < best_dist_sq) {

                best_dist_sq = dist_sq;
                best_segment_idx = i;
                best_s = global_path[i].s + t * segment_len;
            }
        }
        last_closest_index_ = best_segment_idx;

        const double s_start = best_s;
        const double total_path_len = global_path.back().s;

        // k=0 使用当前最近路径点，k=1..N 向前采样。
        // 旧实现从 k=1 开始填充，给初始误差人为加了一个 v_ref*dt 的偏置。
        for (int i = 0; i <= N; ++ i) {

            const double s_target = s_start + i * v_ref * dt;

            Eigen::Vector3d ref_point;
            if (s_target >= total_path_len) {

                ref_point << global_path.back().x, global_path.back().y,
                    global_path.back().theta;
            } else {

                ref_point = interpolatePathByS(global_path, s_target, best_segment_idx);
            }

            // 默认跟踪路径航向；如果只想纯平移跟踪，可设置 track_path_yaw:=false。
            if (!track_path_yaw_) {

                ref_point(2) = cur_pose(2);
            }

            x_ref_vec.segment<3>(i * 3) = ref_point;
        }

        return x_ref_vec;
    }

    /**
     * @brief 发布轨迹消息
     * @param X_ref 参考轨迹序列
     * @return 无
     */
    void MyMPCController::publish_trajectory_msg(const Eigen::VectorXd &X_ref) {

        auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.stamp = clock_->now();
        path_msg->header.frame_id = "odom";

        for (int i = 0; i <= N_; ++ i) {

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg->header;

            // 提取 x, y
            pose.pose.position.x = X_ref(3 * i);
            pose.pose.position.y = X_ref(3 * i + 1);
            pose.pose.position.z = 0.01;

            // 提取 theta 并转换为四元数
            double theta = X_ref(3 * i + 2);
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            pose.pose.orientation = tf2::toMsg(q);

            path_msg->poses.push_back(pose);
        }
        ref_path_pub_->publish(*path_msg);
    }

    void MyMPCController::updateDiscreteModel(
        Eigen::Matrix3d &A,
        Eigen::Matrix3d &B,
        const double theta,
        const double dt
    ) {

        /*
            x_{k+1} = x_k + ( v_xk * \cos\theta_k - v_yk * \sin\theta_k ) \cdot \Delta t
            y_{k+1} = y_k + ( v_xk \sin\theta_k + v_yk * \cos\theta_k ) \cdot \Delta t
            \theta_{k+1} = \theta_k + \omega_k \cdot \Delta t
            A = [1 0 0]
                [0 1 0]
                [0 0 1]
            B = [\cos\theta * \Delta t -\sin\theta * \Delta t 0       ]
                [\sin\theta * \Delta t  \cos\theta * \Delta t 0       ]
                [0                     0                      \Delta t]
        */
        A.setIdentity();
        // A << 1.0, 0.0, -v * sin(theta) * dt,
        //      0.0, 1.0,  v * cos(theta) * dt,
        //      0.0, 0.0,                  1.0;
        B << cos(theta) * dt, -sin(theta) * dt, 0.0,
             sin(theta) * dt,  cos(theta) * dt, 0.0,
             0.0,                          0.0,  dt;
    }

    /**
     * @brief 构建 MPC 求解所需的 H f G
     * @param A, B 当前时刻线性化后的系统矩阵
     * @param x_k 当前机器人位姿 (3x1)
     * @param X_ref:采样得到的参考轨迹序列 (3*(N+1) x 1)
     * @param Q, R, F 权重矩阵 (状态, 控制, 终端状态)
     * @param N 预测步长
     */
    MPCmatrices replaceLargeMatrix(
        const Eigen::Matrix3d &A,
        const Eigen::Matrix3d &B,
        const Eigen::Vector3d &x_k,
        const Eigen::VectorXd &X_ref,
        const Eigen::Matrix3d &Q,
        const Eigen::Matrix3d &R,
        const Eigen::Matrix3d &F,
        int N,
        double terminal_control_weight
    ) {

        const int
            state_dim = 3,
            control_dim = 3;

        // 构建 M
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero((N + 1) * state_dim, state_dim);
        Eigen::Matrix3d A_pow = Eigen::Matrix3d::Identity();
        for (int i = 0; i <= N; ++ i) {

            M.block<3, 3>(i * state_dim, 0) = A_pow;
            A_pow *= A;
        }

        // 构建 C
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero((N + 1) * state_dim, N * control_dim);
        for (int i = 1; i <= N; ++ i) {

            for (int j = 0; j <= i - 1; ++ j) {

                A_pow = Eigen::Matrix3d::Identity();
                for (int k = 0; k < i - 1 - j; ++ k) {

                    A_pow *= A;
                }
                C.block<3, 3>(i * state_dim, j * control_dim) = A_pow * B;
            }
        }

        // 构建 Q_bar
        Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero((N + 1) * state_dim, (N + 1) * state_dim);
        for (int i = 0; i <= N - 1; ++ i) {

            Q_bar.block<3, 3>(i * state_dim, i * state_dim) = Q;
        }
        Q_bar.block<3, 3>(N * state_dim, N * state_dim) = F; // 终端权重

        // 构建 R_bar
        Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(N * control_dim, N * control_dim);
        for (int i = 0; i <= N - 1; ++ i) {

            R_bar.block<3, 3>(i * control_dim, i * control_dim) = R;
        }

        // 对最后一步控制量额外惩罚，让预测轨迹终点速度趋近于 0。
        // 没有这一项时，MPC 允许“到点时仍带速度”，真实执行层一滞后就会冲过目标。
        R_bar.block<3, 3>(
            (N - 1) * control_dim,
            (N - 1) * control_dim) +=
            terminal_control_weight * Eigen::Matrix3d::Identity();

        // 输出
        MPCmatrices mpcm_out;
        Eigen::VectorXd E = M * x_k - X_ref;
        mpcm_out.H = C.transpose() * Q_bar * C + R_bar;
        mpcm_out.f = 2.0 * C.transpose() * Q_bar * E;
        mpcm_out.G = E.transpose() * Q_bar * E;

        return mpcm_out;
    }

    geometry_msgs::msg::TwistStamped MyMPCController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker */*goal_checker*/) {

        auto node = node_.lock();

        auto make_zero_twist = [&]() {

            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = clock_->now();
            cmd_vel.header.frame_id = "base_link";
            return cmd_vel;
        };

        if (!node || processed_path_.empty()) {

            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "MPC has no valid global path, returning zero velocity");
            return make_zero_twist();
        }

        // x_k
        const double
            x = pose.pose.position.x,
            y = pose.pose.position.y,
            theta = tf2::getYaw(pose.pose.orientation);
        const double
            v_ref_x_raw = velocity.linear.x,
            v_ref_y_raw = velocity.linear.y,
            w_ref_raw = velocity.angular.z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta) ||
            !std::isfinite(v_ref_x_raw) || !std::isfinite(v_ref_y_raw) ||
            !std::isfinite(w_ref_raw)
        ) {

            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "MPC received non-finite pose or velocity, returning zero velocity");
            return make_zero_twist();
        }

        const double v_ref_x = v_ref_x_raw;
        const double v_ref_y = v_ref_y_raw;
        const double w_ref = w_ref_raw;
        Eigen::Vector3d x_k(x, y, theta);

        // dt：优先使用实际控制周期，异常时回退到标称周期。
        rclcpp::Time current_time = clock_->now();
        double dt = dt_;
        if (has_last_cmd_time_) {

            dt = (current_time - last_cmd_time_).seconds();
        }
        if (dt <= 0.0 || dt > 0.5) {

            dt = dt_;
        }
        last_cmd_time_ = current_time;
        has_last_cmd_time_ = true;

        // X_ref
        // 参考点间距由“期望速度”决定，不要用当前速度生成参考轨迹：
        // 若参考间距随当前速度变化，低速时 MPC 会停在低速平衡点，越走越慢。
        // 这里按到终点的距离做一个简单的梯形速度曲线，保证低速起步并提前减速。
        const double dist_to_goal = std::hypot(
            processed_path_.back().x - x,
            processed_path_.back().y - y);
        double v_ref = std::min(
            max_v_, std::sqrt(2.0 * max_a_v_ * dist_to_goal));
        v_ref = std::max(0.25, v_ref);

        Eigen::VectorXd X_ref = sampleReferencePath(processed_path_, x_k, v_ref, N_, dt);

        // 线性化AB
        Eigen::Matrix3d A;
        Eigen::Matrix3d B;
        updateDiscreteModel(A, B, theta, dt); // 暂时先这样

        // 构建HfG
        MPCmatrices mpcm = replaceLargeMatrix(
            A, B, x_k, X_ref, Q_, R_, F_, N_, terminal_control_weight_);

        // 求解U
        Eigen::SparseMatrix<double> H_sparse = 2.0 * mpcm.H.sparseView();
        int n = mpcm.f.rows();
        Eigen::VectorXd l(2 * n), u(2 * n);
        Eigen::VectorXd U_sol = Eigen::VectorXd::Zero(n);

        // 构建完整约束矩阵A
        std::vector<Eigen::Triplet<double>> A_tri;
        for (int i = 0; i < n / 3; ++ i) {

            int col = 3 * i, row = 3 * i;
            int row_acc = row + n;

            // A 上半部分填充
            A_tri.emplace_back(row, col, 1.0);
            A_tri.emplace_back(row + 1, col + 1, 1.0);
            A_tri.emplace_back(row + 2, col + 2, 1.0);

            l(row) = -max_v_;
            u(row) = max_v_;
            l(row + 1) = -max_v_;
            u(row + 1) = max_v_;
            l(row + 2) = -max_w_;
            u(row + 2) = max_w_;

            // A 下半部分填充
            A_tri.emplace_back(row_acc, col, 1.0);
            A_tri.emplace_back(row_acc + 1, col + 1, 1.0);
            A_tri.emplace_back(row_acc + 2, col + 2, 1.0);

            if (i == 0) {

                // 第一步基于初始速度
                l(row_acc) = v_ref_x - max_a_v_ * dt;
                u(row_acc) = v_ref_x + max_a_v_ * dt;
                l(row_acc + 1) = v_ref_y - max_a_v_ * dt;
                u(row_acc + 1) = v_ref_y + max_a_v_ * dt;
                l(row_acc + 2) = w_ref - max_a_w_ * dt;
                u(row_acc + 2) = w_ref + max_a_w_ * dt;
            } else {

                // 后续步计算速度差值
                l(row_acc) = -max_a_v_ * dt;
                u(row_acc) = max_a_v_ * dt;
                l(row_acc + 1) = -max_a_v_ * dt;
                u(row_acc + 1) = max_a_v_ * dt;
                l(row_acc + 2) = -max_a_w_ * dt;
                u(row_acc + 2) = max_a_w_ * dt;

                A_tri.emplace_back(row_acc, col - 3, -1.0);
                A_tri.emplace_back(row_acc + 1, col - 2, -1.0);
                A_tri.emplace_back(row_acc + 2, col - 1, -1.0);
            }
        }

        // OSQP 求约束解
        // ====================================
        OSQPSettings *settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
        OSQPData *data = (OSQPData*)c_malloc(sizeof(OSQPData));
        Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int> H_csc = H_sparse.triangularView<Eigen::Upper>(); // csc形式
        Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int> A_csc(2 * n, n);
        A_csc.setFromTriplets(A_tri.begin(), A_tri.end());
        A_csc.makeCompressed();
        H_csc.makeCompressed();

        data->n = n; // 变量
        data->m = 2 * n; // 约束
        data->P = csc_matrix( // csc矩阵信息
            n, n,
            H_csc.nonZeros(),
            H_csc.valuePtr(),
            (c_int*)H_csc.innerIndexPtr(),
            (c_int*)H_csc.outerIndexPtr()
        );
        data->q = mpcm.f.data(); // 一次项
        data->A = csc_matrix( // 约束矩阵
            2 * n, n,
            A_csc.nonZeros(),
            A_csc.valuePtr(),
            (c_int*)A_csc.innerIndexPtr(),
            (c_int*)A_csc.outerIndexPtr()
        );

        // 约束向量
        data->l = l.data();
        data->u = u.data();

        // 开始求解
        osqp_set_default_settings(settings);
        settings->warm_start = 1;
        settings->verbose = 0; // 生产环境关闭日志

        OSQPWorkspace *work = nullptr;
        c_int status = osqp_setup(&work, data, settings);
        if (status == 0 && work) {

            osqp_solve(work);
            const c_int solver_status = work->info ? work->info->status_val : OSQP_NON_CVX;
            if ((solver_status == OSQP_SOLVED ||
                 solver_status == OSQP_SOLVED_INACCURATE) &&
                work->solution
            ) {

                U_sol.resize(n);
                for (int i = 0; i < n / 3; ++ i) {

                    U_sol[i * 3] = work->solution->x[i * 3];
                    U_sol[i * 3 + 1] = work->solution->x[i * 3 + 1];
                    U_sol[i * 3 + 2] = work->solution->x[i * 3 + 2];
                }
            } else {

                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 2000,
                    "MPC: OSQP solve failed, status=%d. Returning zero velocity.",
                    static_cast<int>(solver_status));
            }
        } else {

            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "MPC: OSQP setup failed, status=%d. Returning zero velocity.",
                static_cast<int>(status));
        }

        // 释放
        if (work) osqp_cleanup(work);
        if (data) {

            if (data->P) c_free(data->P);
            if (data->A) c_free(data->A);
            c_free(data);
        }
        if (settings) c_free(settings);
        // =============================================

        double
            v_xcmd = U_sol(0),
            v_ycmd = U_sol(1),
            w_cmd = U_sol(2);

        // 限速
        v_xcmd = std::clamp(v_xcmd, -max_v_, max_v_);
        v_ycmd = std::clamp(v_ycmd, -max_v_, max_v_);
        w_cmd = std::clamp(w_cmd, -max_w_, max_w_);

        // 终点保护：如果已经很近且几乎停住，直接输出零速度，
        // 避免因模型/执行器滞后在目标点附近来回蹭。
        const double current_linear_speed =
            std::hypot(v_ref_x, v_ref_y);
        if (dist_to_goal < 0.03 && current_linear_speed < 0.05) {

            return make_zero_twist();
        }

        // 终点保护：命令速度不能超过“剩余距离内能刹停的速度”。
        // 这只是最后一道安全限幅，正常减速应由 terminal_control_weight 完成。
        const double stop_speed_limit = std::sqrt(
            2.0 * max_a_v_ * std::max(0.0, dist_to_goal));
        const double cmd_linear_speed = std::hypot(v_xcmd, v_ycmd);
        if (cmd_linear_speed > stop_speed_limit &&
            cmd_linear_speed > 1e-3)
        {
            const double scale = stop_speed_limit / cmd_linear_speed;
            v_xcmd *= scale;
            v_ycmd *= scale;
        }

        // 封装
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = clock_->now();
        cmd_vel.header.frame_id = "base_link";
        cmd_vel.twist.linear.x = v_xcmd;
        cmd_vel.twist.linear.y = v_ycmd;
        cmd_vel.twist.angular.z = w_cmd;

        // 发布调试信息至foxglove
        mpc_debug_pub_->publish(cmd_vel);
        publish_trajectory_msg(X_ref);

        return cmd_vel;
    }
    void MyMPCController::activate() { RCLCPP_INFO(logger_, "插件已激活"); }
    void MyMPCController::deactivate() { RCLCPP_INFO(logger_, "插件已停用"); }
    void MyMPCController::cleanup() { RCLCPP_INFO(logger_, "插件已清理"); }
    void MyMPCController::setSpeedLimit(const double & speed_limit, const bool & percentage) {

        (void)speed_limit;
        (void)percentage;
        RCLCPP_INFO(logger_, "收到限速指令，当前插件尚未实现具体的限速逻辑");
    }
} // my_mpc_controller

// 注册算法插件
PLUGINLIB_EXPORT_CLASS(my_mpc_controller::MyMPCController, nav2_core::Controller)
