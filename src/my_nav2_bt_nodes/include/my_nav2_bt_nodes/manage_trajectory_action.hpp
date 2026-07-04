#ifndef MY_NAV2_BT_NODES__MANAGE_TRAJECTORY_ACTION_HPP_
#define MY_NAV2_BT_NODES__MANAGE_TRAJECTORY_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"

namespace my_nav2_bt_nodes {

class ManageTrajectoryAction : public BT::SyncActionNode {

public:

    ManageTrajectoryAction(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:

    bool isPathValid(const nav_msgs::msg::Path &path) const;
    bool isGoalChanged(const geometry_msgs::msg::PoseStamped &goal) const;
    double getYaw(const geometry_msgs::msg::Quaternion &q) const;
    double normalizeAngle(double angle) const;

    nav_msgs::msg::Path active_path_;
    geometry_msgs::msg::PoseStamped active_goal_;
    bool has_active_path_ = false;
    double goal_tolerance_xy_ = 0.05;
    double goal_tolerance_yaw_ = 0.05;
};

}  // namespace my_nav2_bt_nodes

#endif  // MY_NAV2_BT_NODES__MANAGE_TRAJECTORY_ACTION_HPP_
