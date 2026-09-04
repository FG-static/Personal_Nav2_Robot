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

        /// 栅格搜索结果：回溯后的格子序列（含起终点）与搜索统计
        struct AStarSearchResult {
            std::vector<std::uint64_t> cells;
            bool found = false;
            std::size_t expanded_nodes = 0;
            std::size_t generated_nodes = 0;
            std::size_t open_peak = 0;
        };

        /**
         * @brief 在给定 costmap 上执行 8 邻域 A* 栅格搜索（可独立测试的核心）
         *
         * 对角移动要求两个正交邻格都不是硬障碍（防穿角）。
         * goal 格为硬障碍且与 start 不同格时抛 nav2_core::GoalOccupied。
         * @param cancel_checker 取消回调，每 1024 次扩展检查一次，触发时返回未找到
         */
        AStarSearchResult searchCells(
            const nav2_costmap_2d::Costmap2D & costmap,
            unsigned int start_x, unsigned int start_y,
            unsigned int goal_x, unsigned int goal_y,
            const std::function<bool()> & cancel_checker = {});

    private:

        /// 判断格子是否为硬障碍（与主搜索规则一致：阈值以上且非未知）
        bool isBlockedCell(
            const nav2_costmap_2d::Costmap2D & costmap, int cx, int cy) const;

        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_util::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::string global_frame_, name_;
        double unknown_cost_ = 5.0, interpolation_resolution_ = 0.1;
        double obstacle_cost_weight_ = 6.0;
        bool treat_unknown_as_free_ = false;
        // metrics 中整图距离变换的最小复评间隔（秒）：>0 节流，<=0 每次都评估
        double metrics_min_interval_ = 1.0;
        rclcpp::Time last_metrics_eval_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Publisher<rm_interfaces::msg::ReplanEvent>::SharedPtr replan_event_pub_;
        std::uint64_t replan_event_id_ = 0;
        int cost_threshold_ = 253; // 0 < x <= 253
    };
} // namespace my_nav2_planner

#endif // MY_NAV2_PLANNER__ASTAR_PLANNER
