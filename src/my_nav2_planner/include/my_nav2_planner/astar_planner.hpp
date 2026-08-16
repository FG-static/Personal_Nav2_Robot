#ifndef MY_NAV2_PLANNER__ASTAR_PLANNER
#define MY_NAV2_PLANNER__ASTAR_PLANNER

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <queue>

#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "rm_interfaces/msg/replan_event.hpp"

namespace my_nav2_planner {

    class MyAStarPlanner : public nav2_core::GlobalPlanner {

    public:

        MyAStarPlanner() = default;
        ~MyAStarPlanner() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
        void activate() override;
        void deactivate() override;
        void cleanup() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal,
            std::function<bool()> /*cancel_checker*/) override;
    private:

        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_util::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::string global_frame_, name_;
        double unknown_cost_, interpolation_resolution_;
        bool treat_unknown_as_free_ = false;
        rclcpp::Publisher<rm_interfaces::msg::ReplanEvent>::SharedPtr replan_event_pub_;
        std::uint64_t replan_event_id_ = 0;
    };
} // namespace my_nav2_planner

#endif // MY_NAV2_PLANNER__ASTAR_PLANNER