#include "my_nav2_bt_nodes/manage_trajectory_action.hpp"

#include <cmath>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"
#include "nav2_behavior_tree/json_utils.hpp"

namespace my_nav2_bt_nodes
{

ManageTrajectoryAction::ManageTrajectoryAction(
    const std::string &xml_tag_name,
    const BT::NodeConfiguration &conf)
: BT::SyncActionNode(xml_tag_name, conf) {
}

BT::PortsList ManageTrajectoryAction::providedPorts() {

    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return {
        BT::InputPort<nav_msgs::msg::Path>("input_path", "Candidate path from smoother"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal"),
        BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path submitted to controller"),
    };
}

bool ManageTrajectoryAction::isPathValid(
    const nav_msgs::msg::Path &path
) const {

    if (path.poses.empty())
        return false;

    for (const auto &pose : path.poses) {

        if (!std::isfinite(pose.pose.position.x) ||
            !std::isfinite(pose.pose.position.y) ||
            !std::isfinite(pose.pose.position.z))
            return false;
    }

    return true;
}

bool ManageTrajectoryAction::isGoalChanged(
    const geometry_msgs::msg::PoseStamped &goal
) const {

    if (!has_active_path_)
        return true;

    const double dx = goal.pose.position.x - active_goal_.pose.position.x;
    const double dy = goal.pose.position.y - active_goal_.pose.position.y;
    if (std::hypot(dx, dy) > goal_tolerance_xy_)
        return true;

    const double yaw = getYaw(goal.pose.orientation);
    const double active_yaw = getYaw(active_goal_.pose.orientation);
    return std::abs(normalizeAngle(yaw - active_yaw)) > goal_tolerance_yaw_;
}

double ManageTrajectoryAction::getYaw(
    const geometry_msgs::msg::Quaternion &q
) const {

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double ManageTrajectoryAction::normalizeAngle(double angle) const {

    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

BT::NodeStatus ManageTrajectoryAction::tick() {

    nav_msgs::msg::Path input_path;
    if (!getInput("input_path", input_path))
        return BT::NodeStatus::FAILURE;

    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal))
        return BT::NodeStatus::FAILURE;

    if (!isPathValid(input_path)) {

        if (!has_active_path_)
            return BT::NodeStatus::FAILURE;

        setOutput("output_path", active_path_);
        return BT::NodeStatus::SUCCESS;
    }

    if (!has_active_path_ || isGoalChanged(goal)) {

        active_path_ = input_path;
        active_goal_ = goal;
        has_active_path_ = true;
    }

    setOutput("output_path", active_path_);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace my_nav2_bt_nodes

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<my_nav2_bt_nodes::ManageTrajectoryAction>("ManageTrajectory");
}
