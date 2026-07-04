#ifndef MY_NAV2_BT_NODES__MANAGE_TRAJECTORY_ACTION_HPP_
#define MY_NAV2_BT_NODES__MANAGE_TRAJECTORY_ACTION_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/action_node.h"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rm_interfaces/msg/replan_event.hpp"

namespace my_nav2_bt_nodes {

class ManageTrajectoryAction : public BT::SyncActionNode {

public:

    ManageTrajectoryAction(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf);
    ~ManageTrajectoryAction() override;

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:

    bool isPathValid(const nav_msgs::msg::Path &path) const;
    bool isGoalChanged(const geometry_msgs::msg::PoseStamped &goal) const;
    bool isSameGoal(
        const geometry_msgs::msg::PoseStamped &lhs,
        const geometry_msgs::msg::PoseStamped &rhs) const;
    bool shouldConsumeReplanEvent(
        const nav_msgs::msg::Path &candidate_path,
        const geometry_msgs::msg::PoseStamped &goal,
        uint64_t &event_id);
    void onReplanEvent(const rm_interfaces::msg::ReplanEvent::SharedPtr msg);
    double getYaw(const geometry_msgs::msg::Quaternion &q) const;
    double normalizeAngle(double angle) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr event_callback_group_;
    rclcpp::Subscription<rm_interfaces::msg::ReplanEvent>::SharedPtr replan_event_sub_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> event_executor_;
    std::thread event_executor_thread_;

    nav_msgs::msg::Path active_path_;
    geometry_msgs::msg::PoseStamped active_goal_;
    bool has_active_path_ = false;

    mutable std::mutex event_mutex_;
    bool pending_replan_ = false;
    uint64_t last_seen_event_id_ = 0;
    uint64_t last_consumed_event_id_ = 0;
    builtin_interfaces::msg::Time last_replan_candidate_stamp_;
    geometry_msgs::msg::PoseStamped event_goal_;
    uint8_t event_reason_ = rm_interfaces::msg::ReplanEvent::UNKNOWN;

    double goal_tolerance_xy_ = 0.05;
    double goal_tolerance_yaw_ = 0.05;
};

}  // namespace my_nav2_bt_nodes

#endif  // MY_NAV2_BT_NODES__MANAGE_TRAJECTORY_ACTION_HPP_
