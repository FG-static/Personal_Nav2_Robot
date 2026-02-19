#include "my_nav2_smoother/gradient_smoother.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_nav2_smoother {

    void MyGradientSmoother::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, 
        std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/
    ) {
            
        costmap_sub_ = costmap_sub;
        node_ = parent.lock();
        name_ = name;

        nav2_util::declare_parameter_if_not_declared(node_, name + ".alpha", rclcpp::ParameterValue(0.1));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".beta", rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".max_iterations", rclcpp::ParameterValue(500));

        node_->get_parameter(name + ".alpha", alpha_);
        node_->get_parameter(name + ".beta", beta_);
        node_->get_parameter(name + ".max_iterations", max_iterations_);
    }
    void MyGradientSmoother::activate() { RCLCPP_INFO(node_->get_logger(), "插件已激活"); }
    void MyGradientSmoother::deactivate() { RCLCPP_INFO(node_->get_logger(), "插件已停用"); }
    void MyGradientSmoother::cleanup() { RCLCPP_INFO(node_->get_logger(), "插件已清理"); }
    bool MyGradientSmoother::smooth(
        nav_msgs::msg::Path &path,
        const rclcpp::Duration &/*max_time*/
    ) {

        if (path.poses.size() < 3) return true;
        nav_msgs::msg::Path raw_path = path;

        applyGradientDescent(path, raw_path);

        return true;
    }
    void MyGradientSmoother::applyGradientDescent(
        nav_msgs::msg::Path &path, 
        const nav_msgs::msg::Path &raw_path
    ) {

        auto costmap = costmap_sub_->getCostmap();
        int n = path.poses.size();

        for (int iter = 0; iter < max_iterations_; ++ iter) {

            for (int i = 1; i < n - 1; ++ i) {

                double 
                    &x1 = path.poses[i].pose.position.x,
                    &y1 = path.poses[i].pose.position.y,
                    &x2 = path.poses[i + 1].pose.position.x,
                    &y2 = path.poses[i + 1].pose.position.y,
                    &x0 = path.poses[i - 1].pose.position.x,
                    &y0 = path.poses[i - 1].pose.position.y;
                double 
                    old_x = x1,
                    old_y = y1;
                
                x1 += 
                    alpha_ * (x0 + x2 - 2.0 * x1) + 
                    beta_ * (raw_path.poses[i].pose.position.x - x1);
                y1 += 
                    alpha_ * (y0 + y2 - 2.0 * y1) + 
                    beta_ * (raw_path.poses[i].pose.position.y - y1);
                
                // 碰撞检查
                unsigned int mx, my;
                if (costmap->worldToMap(x1, y1, mx, my)) {

                    if (costmap->getCost(mx, my) >= 250) {

                        x1 = old_x;
                        y1 = old_y;
                    }
                }
            }
            for (int i = 0; i < n - 1; ++ i) {

                double 
                    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x,
                    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
                double yaw = std::atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                path.poses[i].pose.orientation = tf2::toMsg(q);
            }
            if (n > 1) 
                path.poses.back().pose.orientation = path.poses[n - 2].pose.orientation;
        }
    }
} // my_nav2_smoother

PLUGINLIB_EXPORT_CLASS(my_nav2_smoother::MyGradientSmoother, nav2_core::Smoother);