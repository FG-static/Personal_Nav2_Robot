#include "my_nav2_bt_nodes/manage_trajectory_action.hpp"

#include <cmath>
#include <functional>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"
#include "nav2_behavior_tree/json_utils.hpp"

namespace my_nav2_bt_nodes
{

ManageTrajectoryAction::ManageTrajectoryAction(
    const std::string &xml_tag_name,
    const BT::NodeConfiguration &conf
) : BT::SyncActionNode(xml_tag_name, conf) {

    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    std::string replan_event_topic = "trajectory_generation/replan_event";
    (void)getInput("replan_event_topic", replan_event_topic);

    event_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = event_callback_group_;

    replan_event_sub_ = node_->create_subscription<rm_interfaces::msg::ReplanEvent>(
        replan_event_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&ManageTrajectoryAction::onReplanEvent, this, std::placeholders::_1),
        subscription_options);

    event_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    event_executor_->add_callback_group(event_callback_group_, node_->get_node_base_interface());
    event_executor_thread_ = std::thread([this]() {
        event_executor_->spin();
    });
}

ManageTrajectoryAction::~ManageTrajectoryAction() {

    if (event_executor_)
        event_executor_->cancel();

    if (event_executor_thread_.joinable())
        event_executor_thread_.join();
}

BT::PortsList ManageTrajectoryAction::providedPorts() {

    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return {
        BT::InputPort<nav_msgs::msg::Path>("input_path", "Candidate path from smoother"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal"),
        BT::InputPort<std::string>(
            "replan_event_topic",
            "trajectory_generation/replan_event",
            "Topic carrying planner replan events"),
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

    return !isSameGoal(goal, active_goal_);
}

bool ManageTrajectoryAction::isSameGoal(
    const geometry_msgs::msg::PoseStamped &lhs,
    const geometry_msgs::msg::PoseStamped &rhs
) const {

    const double dx = lhs.pose.position.x - rhs.pose.position.x;
    const double dy = lhs.pose.position.y - rhs.pose.position.y;
    if (std::hypot(dx, dy) > goal_tolerance_xy_)
        return false;

    const double lhs_yaw = getYaw(lhs.pose.orientation);
    const double rhs_yaw = getYaw(rhs.pose.orientation);
    return std::abs(normalizeAngle(lhs_yaw - rhs_yaw)) <= goal_tolerance_yaw_;
}

bool ManageTrajectoryAction::shouldConsumeReplanEvent(
    const nav_msgs::msg::Path &candidate_path,
    const geometry_msgs::msg::PoseStamped &goal,
    uint64_t &event_id
) {

    std::lock_guard<std::mutex> lock(event_mutex_);
    if (!pending_replan_ || last_seen_event_id_ == last_consumed_event_id_)
        return false;

    if (!isSameGoal(event_goal_, goal))
        return false;

    const rclcpp::Time candidate_stamp(candidate_path.header.stamp);
    const rclcpp::Time required_candidate_stamp(last_replan_candidate_stamp_);
    if (candidate_stamp < required_candidate_stamp)
        return false;

    event_id = last_seen_event_id_;
    last_consumed_event_id_ = last_seen_event_id_;
    pending_replan_ = false;
    return true;
}

void ManageTrajectoryAction::onReplanEvent(
    const rm_interfaces::msg::ReplanEvent::SharedPtr msg
) {

    if (!msg->need_replan)
        return;

    std::lock_guard<std::mutex> lock(event_mutex_);
    if (msg->event_id <= last_seen_event_id_)
        return;

    pending_replan_ = true;
    last_seen_event_id_ = msg->event_id;
    last_replan_candidate_stamp_ = msg->candidate_path_stamp;
    event_goal_ = msg->goal;
    event_reason_ = msg->reason;

    RCLCPP_INFO(
        node_->get_logger(),
        "ManageTrajectory received replan event id=%lu reason=%u",
        static_cast<unsigned long>(last_seen_event_id_),
        event_reason_);
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

    uint64_t consumed_event_id = 0;
    const bool consume_replan_event =
        shouldConsumeReplanEvent(input_path, goal, consumed_event_id);

    if (!has_active_path_ || isGoalChanged(goal) || consume_replan_event) {

        active_path_ = input_path;
        active_goal_ = goal;
        has_active_path_ = true;

        if (consume_replan_event) {
            RCLCPP_INFO(
                node_->get_logger(),
                "ManageTrajectory consumed replan event id=%lu",
                static_cast<unsigned long>(consumed_event_id));
        }
    }

    setOutput("output_path", active_path_);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace my_nav2_bt_nodes

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<my_nav2_bt_nodes::ManageTrajectoryAction>("ManageTrajectory");
}
