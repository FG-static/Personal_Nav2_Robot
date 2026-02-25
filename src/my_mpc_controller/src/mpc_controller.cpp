#include "my_mpc_controller/mpc_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

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
            node, plugin_name_ + ".r_22", rclcpp::ParameterValue(0.5));
        node->get_parameter(plugin_name_ + ".r_22", r_22);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".f_11", rclcpp::ParameterValue(60.0));
        node->get_parameter(plugin_name_ + ".f_11", f_11);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".f_22", rclcpp::ParameterValue(60.0));
        node->get_parameter(plugin_name_ + ".f_22", f_22);
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".f_33", rclcpp::ParameterValue(60.0));
        node->get_parameter(plugin_name_ + ".f_33", f_33);

        Q_.setIdentity(); 
        Q_(0, 0) = q_11; // x 权重
        Q_(1, 1) = q_22; // y 权重
        Q_(2, 2) = q_33;  // theta 权重

        R_.setIdentity();
        R_(0, 0) = r_11; // v 权重
        R_(1, 1) = r_22; // w 权重

        F_.setIdentity(); 
        F_(0, 0) = f_11;
        F_(1, 1) = f_22;
        F_(2, 2) = f_33;

        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".n", rclcpp::ParameterValue(10));
        node->get_parameter(plugin_name_ + ".n", N_);

        double transform_tolerance;
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

        RCLCPP_INFO(logger_, "自定义MPC控制器配置完成");
    }
     
    void MyMPCController::setPlan(const nav_msgs::msg::Path &path) {

        global_plan_ = path;
        processed_path_ = preprocessPath(global_plan_); 
    }
    
    std::vector<PathPoint> MyMPCController::preprocessPath(const nav_msgs::msg::Path &path) {

        std::vector<PathPoint> processed_path;
        if (path.poses.empty()) return processed_path;
        double cumulative_s = 0.0;
        processed_path.reserve(path.poses.size());

        for (int i = 0; i < path.poses.size(); ++ i) {

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
            if (i < path.poses.size() - 1) {

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
     */
    Eigen::Vector3d interpolatePathByS(const std::vector<PathPoint>& path, double s_target, int start_search_idx) {
        
        for (int i = start_search_idx; i < path.size() - 1; ++ i) {
            
            if (s_target >= path[i].s && s_target <= path[i + 1].s) {
                
                double rate = (s_target - path[i].s) / (path[i + 1].s - path[i].s);
                
                double 
                    x = path[i].x + rate * (path[i + 1].x - path[i].x),
                    y = path[i].y + rate * (path[i + 1].y - path[i].y);
                
                // 角度插值
                double diff = path[i + 1].theta - path[i].theta;
                while (diff > M_PI) diff -= 2 * M_PI;
                while (diff < -M_PI) diff += 2 * M_PI;
                double theta = path[i].theta + rate * diff;
                
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
     * @return 采样后的参考序列 x_ref [x1, y1, th1, x2, y2, th2, ...]
     */
    Eigen::VectorXd MyMPCController::sampleReferencePath(
        const std::vector<PathPoint> &global_path,
        const Eigen::Vector3d &cur_pose,
        double v_ref,
        int N,
        double dt
    ) {

        Eigen::VectorXd x_ref_vec(3 * (N + 1));

        // 寻点
        int min_idx = last_closest_index_;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = last_closest_index_; i < global_path.size(); ++ i) {

            double d = hypot(global_path[i].x - cur_pose(0), global_path[i].y - cur_pose(1));
            if (d < min_dist) {

                min_dist = d;
                min_idx = i;
                last_closest_index_ = i;
            }
        }

        // 累积距离
        double
            s_start = global_path[min_idx].s,
            total_path_len = global_path.back().s;
        
        // 采样
        for (int i = 1; i <= N + 1; ++ i) {

            // 暂时先不考虑各段速度不同
            double s_target = s_start + i * v_ref * dt;

            Eigen::Vector3d ref_point;

            if (s_target >= total_path_len) {

                ref_point << global_path.back().x, global_path.back().y, global_path.back().theta;
            } else {

                // 防突变线性插值
                ref_point = interpolatePathByS(global_path, s_target, min_idx);
            }

            // 填充到Xk
            x_ref_vec.segment<3>((i - 1) * 3) = ref_point;
        }

        return x_ref_vec;
    }

    void MyMPCController::updateDiscreteModel(
        Eigen::Matrix3d &A, 
        Eigen::Matrix<double, 3, 2> &B,
        const double theta,
        const double v,
        const double dt
    ) {

        /*
            x_{k+1} = x_k + v_k \cos\theta_k \cdot \Delta t
            y_{k+1} = y_k + v_k \sin\theta_k \cdot \Delta t
            \theta_{k+1} = \theta_k + \omega_k \cdot \Delta t
            A = [1 0 -v \sin\theta \Delta t]
                [0 1 v \cos\theta \Delta t ]
                [0 0 1                     ]
            B = [\cos\theta \Delta t 0       ]
                [\sin\theta \Delta t 0       ]
                [0                   \Delta t]
        */
        A << 1.0, 0.0, -v * sin(theta) * dt,
             0.0, 1.0,  v * cos(theta) * dt,
             0.0, 0.0,                  1.0;
        B << cos(theta) * dt, 0.0,
             sin(theta) * dt, 0.0,
             0.0,              dt;
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
        const Eigen::Matrix<double, 3, 2> &B,
        const Eigen::Vector3d &x_k,
        const Eigen::VectorXd &X_ref,
        const Eigen::Matrix3d &Q,
        const Eigen::Matrix2d &R,
        const Eigen::Matrix3d &F,
        int N
    ) {

        const int
            state_dim = 3,
            control_dim = 2;

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
                C.block<3, 2>(i * state_dim, j * control_dim) = A_pow * B;
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

            R_bar.block<2, 2>(i * control_dim, i * control_dim) = R;
        }

        // 输出
        MPCmatrices mpcm_out;
        Eigen::VectorXd E = M * x_k - X_ref;
        mpcm_out.H = C.transpose() * Q_bar * C + R_bar;
        mpcm_out.f = C.transpose() * Q_bar * E;
        mpcm_out.G = E.transpose() * Q_bar * E;

        return mpcm_out;
    }

    geometry_msgs::msg::TwistStamped MyMPCController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker */*goal_checker*/) {

        auto node = node_.lock();
        // x_k
        double 
            x = pose.pose.position.x,
            y = pose.pose.position.y,
            theta = tf2::getYaw(pose.pose.orientation);
        Eigen::Vector3d x_k(x, y, theta);
        
        // dt
        double dt;
        static rclcpp::Time last_time = clock_->now();
        rclcpp::Time current_time = clock_->now();
        dt = (current_time - last_time).seconds();
        if (dt <= 0.0 || dt > 0.5) dt = dt_;
        last_time = current_time;

        // X_ref
        Eigen::VectorXd X_ref = sampleReferencePath(processed_path_, x_k, velocity.linear.x, N_, dt);

        // 线性化AB
        Eigen::Matrix3d A;
        Eigen::Matrix<double, 3, 2> B;
        updateDiscreteModel(A, B, theta, std::max(std::abs(velocity.linear.x), 0.1), dt); // 暂时先这样
        
        // 构建HfG
        MPCmatrices mpcm = replaceLargeMatrix(A, B, x_k, X_ref, Q_, R_, F_, N_);

        // 求解U
        Eigen::VectorXd U_sol;
        U_sol = (mpcm.H + 1e-6 * Eigen::MatrixXd::Identity(mpcm.H.rows(), mpcm.H.cols())).ldlt().solve(-mpcm.f); // LM方法
        // TODO：有机会可用osqp求约束解

        double
            v_cmd = U_sol(0),
            w_cmd = U_sol(1);

        // 封装
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = clock_->now();
        cmd_vel.header.frame_id = "base_link";
        cmd_vel.twist.linear.x = v_cmd;
        cmd_vel.twist.angular.z = w_cmd;
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