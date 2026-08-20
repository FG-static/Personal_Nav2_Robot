#include "my_nav2_robot/wall_wait.hpp"

#include <chrono>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace my_nav2_robot
{

void WallWait::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> /*collision_checker*/)
{
    node_ = parent;
    name_ = name;
    auto node = node_.lock();
    action_server_ = std::make_shared<ActionServer>(
        node, name_, std::bind(&WallWait::execute, this));
}

void WallWait::cleanup()
{
    action_server_.reset();
}

void WallWait::activate()
{
    if (action_server_) {
        action_server_->activate();
    }
}

void WallWait::deactivate()
{
    if (action_server_) {
        action_server_->deactivate();
    }
}

void WallWait::execute()
{
    auto node = node_.lock();
    if (!node || !action_server_ || !action_server_->is_server_active()) {
        return;
    }

    const auto goal = action_server_->get_current_goal();
    if (!goal) {
        return;
    }

    const auto wait_for = std::chrono::duration<double>(
        rclcpp::Duration(goal->time).seconds());
    const auto start = std::chrono::steady_clock::now();
    auto feedback = std::make_shared<WaitAction::Feedback>();
    auto result = std::make_shared<WaitAction::Result>();
    rclcpp::WallRate loop_rate(20.0);

    while (rclcpp::ok() && action_server_->is_server_active()) {
        if (action_server_->is_cancel_requested()) {
            result->total_elapsed_time = rclcpp::Duration::from_seconds(
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count());
            action_server_->terminate_all(result);
            return;
        }

        const auto elapsed = std::chrono::steady_clock::now() - start;
        const double remaining =
            wait_for.count() - std::chrono::duration<double>(elapsed).count();
        if (remaining <= 0.0) {
            result->total_elapsed_time = rclcpp::Duration::from_seconds(wait_for.count());
            action_server_->succeeded_current(result);
            return;
        }

        feedback->time_left = rclcpp::Duration::from_seconds(remaining);
        action_server_->publish_feedback(feedback);
        loop_rate.sleep();
    }
}

}  // namespace my_nav2_robot

PLUGINLIB_EXPORT_CLASS(my_nav2_robot::WallWait, nav2_core::Behavior)
