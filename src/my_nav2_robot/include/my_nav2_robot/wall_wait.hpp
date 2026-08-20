#ifndef MY_NAV2_ROBOT__WALL_WAIT_HPP_
#define MY_NAV2_ROBOT__WALL_WAIT_HPP_

#include <memory>
#include <string>

#include "nav2_core/behavior.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace my_nav2_robot
{

// Humble nav2_behaviors/Wait waits on /clock. After Ctrl-C, Gazebo dies,
// sim time freezes, and Wait never returns, so BT/action callback errors spam.
// This replacement waits on wall time and exits when the action server deactivates.
class WallWait : public nav2_core::Behavior
{
public:
    using WaitAction = nav2_msgs::action::Wait;
    using ActionServer = nav2_util::SimpleActionServer<WaitAction>;

    WallWait() = default;
    ~WallWait() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        const std::string & name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker)
        override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;

private:
    void execute();

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string name_;
    std::shared_ptr<ActionServer> action_server_;
};

}  // namespace my_nav2_robot

#endif  // MY_NAV2_ROBOT__WALL_WAIT_HPP_
