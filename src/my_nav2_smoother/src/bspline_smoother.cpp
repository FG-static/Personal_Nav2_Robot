#include "my_nav2_smoother/bspline_smoother.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

extern "C" {
#include "osqp/osqp.h"
}

namespace my_bspline_smoother {

    void MyBSplineSmoother::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, 
        std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/
    ) {
            
        costmap_sub_ = costmap_sub;
        node_ = parent.lock();
        name_ = name;

        nav2_util::declare_parameter_if_not_declared(node_, name + ".w_smooth", rclcpp::ParameterValue(10.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".w_guide", rclcpp::ParameterValue(1.0));

        node_->get_parameter(name + ".w_smooth", w_smooth_);
        node_->get_parameter(name + ".w_guide", w_guide_);
    }
    void MyBSplineSmoother::activate() { RCLCPP_INFO(node_->get_logger(), "插件已激活"); }
    void MyBSplineSmoother::deactivate() { RCLCPP_INFO(node_->get_logger(), "插件已停用"); }
    void MyBSplineSmoother::cleanup() { RCLCPP_INFO(node_->get_logger(), "插件已清理"); }
    bool MyBSplineSmoother::smooth(
        nav_msgs::msg::Path &path,
        const rclcpp::Duration &/*max_time*/
    ) {

        if (path.poses.size() < 4) return true;
        nav_msgs::msg::Path raw_path = path;

        // FIXME：其他参数处理
        applyBSplineAlgorithm(path, raw_path);

        return true;
    }
    void MyBSplineSmoother::applyBSplineAlgorithm(
        nav_msgs::msg::Path &path, 
        const nav_msgs::msg::Path &raw_path
    ) {

        if (raw_path.poses.size() < 4) {
            
            path = raw_path;
            return;
        }
        std::vector<double> 
            ref_path_x,
            ref_path_y,
            smooth_path_x,
            smooth_path_y;
        for (int i = 0; i < raw_path.poses.size(); ++ i) {

            ref_path_x.push_back(raw_path.poses[i].pose.position.x);
            ref_path_y.push_back(raw_path.poses[i].pose.position.y);
        }
        solveBSplineQP(ref_path_x, w_smooth_, w_guide_, smooth_path_x);
        solveBSplineQP(ref_path_y, w_smooth_, w_guide_, smooth_path_y);
        int n = smooth_path_x.size();
        for (size_t i = 0; i < n; ++ i) {

            double yaw;
            
            if (i < n - 1) {

                yaw = std::atan2(smooth_path_y[i + 1] - smooth_path_y[i], 
                                smooth_path_x[i + 1] - smooth_path_x[i]);
            } else {

                yaw = std::atan2(smooth_path_y[i] - smooth_path_y[i - 1], 
                                smooth_path_x[i] - smooth_path_x[i - 1]);
            }

            // 更新位姿
            path.poses[i].pose.position.x = smooth_path_x[i];
            path.poses[i].pose.position.y = smooth_path_y[i];
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw); // Roll=0, Pitch=0, Yaw=yaw
            path.poses[i].pose.orientation = tf2::toMsg(q);
        }
    }
    /**
     * @brief 使用 B-Spline 二次规划平滑路径
     * @param p_ref 原始参考路径的坐标序列 (x 或 y)
     * @param w_s 平滑权重
     * @param w_g 引导项权重
     * @param p_smooth 输出的平滑后的坐标序列
     * @return 成功返回true
     */
    bool MyBSplineSmoother::solveBSplineQP(
        const std::vector<double> &p_ref,
        double w_s,
        double w_g,
        std::vector<double> &p_smooth
    ) {

        RCLCPP_INFO(node_->get_logger(), "BSpline算法启动");
        int n = p_ref.size();
        if (n < 4) return false;

        Eigen::SparseMatrix<double> H(n, n);
        std::vector<Eigen::Triplet<double>> triplets;
        for (int i = 0; i < n - 3; ++ i) {

            for (int r = 0; r < 4; ++ r) {

                for(int c = 0; c < 4; ++ c) {

                    triplets.push_back(Eigen::Triplet<double>(i + r, i + c, w_s * Q_data[r][c]));
                }
            }
        }

        // 引导项
        for (int i = 0; i < n; ++ i) {

            triplets.push_back(Eigen::Triplet<double>(i, i, w_g));
        }
        H.setFromTriplets(triplets.begin(), triplets.end());

        Eigen::VectorXd f(n);
        for (int i = 0; i < n; ++ i) f(i) = -w_g * p_ref[i];

        // 边界约束
        Eigen::VectorXd lower_bound(n), upper_bound(n);
        for (int i = 0; i < n; ++i) {

            lower_bound(i) = p_ref[i] - 1.0; 
            upper_bound(i) = p_ref[i] + 1.0;
            if (i < 2 || i > n - 3) {

                lower_bound(i) = p_ref[i];
                upper_bound(i) = p_ref[i];
            }
        }
        
        // OSQP求解
        RCLCPP_INFO(node_->get_logger(), "OSQP开始初始化");
        OSQPSettings *settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
        OSQPData *data = (OSQPData*)c_malloc(sizeof(OSQPData));
        Eigen::SparseMatrix<double, Eigen::ColMajor, int> H_csc = H.triangularView<Eigen::Upper>(); // csc形式
        Eigen::SparseMatrix<double, Eigen::ColMajor, int> A_csc(n, n);
        A_csc.setIdentity();
        A_csc.makeCompressed();

        data->n = n; // 变量
        data->m = n; // 约束
        data->P = csc_matrix( // csc矩阵信息
            n, n,
            H_csc.nonZeros(),
            H_csc.valuePtr(),
            (c_int*)H_csc.innerIndexPtr(),
            (c_int*)H_csc.outerIndexPtr()
        );
        data->q = f.data(); // 一次项
        data->A = csc_matrix( // 约束矩阵
            n, n, 
            A_csc.nonZeros(),
            A_csc.valuePtr(),
            (c_int*)A_csc.innerIndexPtr(),
            (c_int*)A_csc.outerIndexPtr()
        );

        // 约束向量
        data->l = lower_bound.data();
        data->u = upper_bound.data();

        // 开始求解
        osqp_set_default_settings(settings);
        settings->warm_start = 1;
        settings->verbose = 0; // 生产环境关闭日志
        RCLCPP_INFO(node_->get_logger(), "OSQP正在初始化");

        OSQPWorkspace *work = nullptr;
        c_int status = osqp_setup(&work, data, settings);
        bool success = false;
        if (status == 0 && work) {

            RCLCPP_INFO(node_->get_logger(), "OSQP开始求解");
            osqp_solve(work);
            if (work->info->status_val >= 0 && work->solution) {

                p_smooth.resize(n);
                for (int i = 0; i < n; ++ i) {

                    p_smooth[i] = work->solution->x[i];
                }
                success = true;
            }
        } else {

            RCLCPP_INFO(node_->get_logger(), "OSQP初始化失败");
        }

        // 释放
        if (work) osqp_cleanup(work);
        if (data) {

            if (data->P) c_free(data->P);
            if (data->A) c_free(data->A);
            c_free(data);
        }
        if (settings) c_free(settings);

        return success;
    }
} // my_bspline_smoother

PLUGINLIB_EXPORT_CLASS(my_bspline_smoother::MyBSplineSmoother, nav2_core::Smoother)