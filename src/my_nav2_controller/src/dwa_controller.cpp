#include "my_nav2_controller/dwa_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace my_nav2_controller {

    struct Path {
        double v, w,      // 速度组合
            score,        // 总评分
            cost,         // 障碍物代价
            dist_to_path; // 距离全局路径的距离
    };

    void MyDWAController::configure(
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
        
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_v", rclcpp::ParameterValue(0.5)
        );
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_w", rclcpp::ParameterValue(1.0)
        );
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".lim_a", rclcpp::ParameterValue(2.5)
        );
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".lim_aw", rclcpp::ParameterValue(3.2)
        );
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.5)
        );
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1)
        );

        // 获取参数
        node->get_parameter(plugin_name_ + ".max_v", max_v);
        node->get_parameter(plugin_name_ + ".max_w", max_w);
        node->get_parameter(plugin_name_ + ".lim_a", lim_a);
        node->get_parameter(plugin_name_ + ".lim_aw", lim_aw);
        node->get_parameter(plugin_name_ + ".sim_time", sim_time_);

        double transform_tolerance;
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

        RCLCPP_INFO(logger_, "自定义DWA控制器配置完成");
    }
     
    void MyDWAController::setPlan(const nav_msgs::msg::Path &path) {

        global_plan_ = path;
    }

    geometry_msgs::msg::TwistStamped MyDWAController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker *goal_checker) {

        auto node = node_.lock();
        auto costmap = costmap_ros_->getCostmap();

        // 动态窗口
        double dt = 0.1,
            v_min = std::max(0.0, velocity.linear.x - lim_a * dt),
            v_max = std::min(max_v, velocity.linear.x + lim_a * dt),
            w_min = velocity.angular.z - lim_aw * dt,
            w_max = velocity.angular.z + lim_aw * dt;

        Path best_path = {0.0, 0.0, -1e9, 0.0, 0.0};

        // 采样循环
        for (double v = v_min; v <= v_max; v += 0.02) {

            for (double w = w_min; w <= w_max; w += 0.1) {

                double x = 0.0, y = 0.0, alp = 0.0;
                bool collided = false;
                double min_dist_to_path = 1e9;

                for (double t = 0; t < sim_time_; t += 0.2) {

                    x += v * cos(alp) * 0.2;
                    y += v * sin(alp) * 0.2;
                    alp += w * 0.2;
                    unsigned int mx, my;
                    // 地图内判断
                    if (costmap_ros_->getCostmap()->worldToMap(x + pose.pose.position.x, y + pose.pose.position.y, mx, my)) {

                        // 获取代价
                        unsigned char cost = costmap->getCost(mx, my);
                        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) { // 撞墙了

                            collided = true;
                            break;
                        }
                    }
                }

                if (collided) continue;
                // 评分
                double score = v * 1.0 - std::abs(w) * 0.1; // 暂时先这样
                if (score > best_path.score) best_path = {v, w, score, 0.0, 0.0};
            }
        }
        
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = clock_->now();
        cmd_vel.header.frame_id = "base_link";
        cmd_vel.twist.linear.x = best_path.v;
        cmd_vel.twist.angular.z = best_path.w;

        return cmd_vel;
    }
    void MyDWAController::activate() { RCLCPP_INFO(logger_, "插件已激活"); }
    void MyDWAController::deactivate() { RCLCPP_INFO(logger_, "插件已停用"); }
    void MyDWAController::cleanup() { RCLCPP_INFO(logger_, "插件已清理"); }
    void MyDWAController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
        
        (void)speed_limit;
        (void)percentage;
        RCLCPP_INFO(logger_, "收到限速指令，当前插件尚未实现具体的限速逻辑");
    }
} // my_nav2_controller

// 注册算法插件
PLUGINLIB_EXPORT_CLASS(my_nav2_controller::MyDWAController, nav2_core::Controller)