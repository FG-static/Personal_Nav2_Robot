#include "my_nav2_planner/astar_planner.hpp"
#include "my_planning_metrics/path_metrics.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

namespace my_nav2_planner {

    void MyAStarPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        replan_event_pub_ = node_->create_publisher<rm_interfaces::msg::ReplanEvent>(
            "trajectory_generation/replan_event",
            rclcpp::QoS(1).reliable().transient_local());

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".unknown_cost", rclcpp::ParameterValue(5.0));
        node_->get_parameter(name_ + ".unknown_cost", unknown_cost_);
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".treat_unknown_as_free", rclcpp::ParameterValue(false));
        node_->get_parameter(
            name_ + ".treat_unknown_as_free", treat_unknown_as_free_);
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".cost_threshold", rclcpp::ParameterValue(250));
        node_->get_parameter(name_ + ".cost_threshold", cost_threshold_);
        cost_threshold_ = std::clamp(cost_threshold_, 0, 253);

        //RCLCPP_INFO(node_->get_logger(), "自定义A*规划器配置完成");
    }

    void MyAStarPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "插件已激活"); }
    void MyAStarPlanner::deactivate() { RCLCPP_INFO(node_->get_logger(), "插件已停用"); }
    void MyAStarPlanner::cleanup() { RCLCPP_INFO(node_->get_logger(), "插件已清理"); }

    nav_msgs::msg::Path MyAStarPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> /*cancel_checker*/) {

        const auto total_start_time = std::chrono::steady_clock::now();
        auto logMetrics =
            [&](
                bool success,
                double front_end_ms,
                std::size_t expanded_nodes,
                std::size_t generated_nodes,
                std::size_t open_peak,
                const nav_msgs::msg::Path &path
            ) {
                const auto planning_end_time = std::chrono::steady_clock::now();
                const double planner_total_ms =
                    std::chrono::duration<double, std::milli>(
                        planning_end_time - total_start_time).count();

                const auto metrics_start_time = std::chrono::steady_clock::now();
                my_planning_metrics::ObstacleDistanceField distance_field;
                const bool distance_ready =
                    !path.poses.empty() && distance_field.build(costmap_);
                const my_planning_metrics::PathMetrics path_metrics =
                    my_planning_metrics::evaluatePath(
                        path,
                        distance_ready ? &distance_field : nullptr);
                const double metrics_eval_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - metrics_start_time).count();

                RCLCPP_INFO(
                    node_->get_logger(),
                    "FRONTEND_METRICS algorithm=AStar success=%s reused=false "
                    "expanded_nodes=%zu generated_nodes=%zu open_peak=%zu "
                    "front_end_ms=%.3f planner_total_ms=%.3f metrics_eval_ms=%.3f "
                    "path_points=%zu path_length_m=%.3f max_curvature_1pm=%.3f "
                    "min_lethal_obstacle_distance_m=%.3f",
                    success ? "true" : "false",
                    expanded_nodes,
                    generated_nodes,
                    open_peak,
                    front_end_ms,
                    planner_total_ms,
                    metrics_eval_ms,
                    path_metrics.point_count,
                    path_metrics.length_m,
                    path_metrics.max_curvature_inv_m,
                    path_metrics.min_lethal_obstacle_distance_m);
            };

        nav_msgs::msg::Path global_path;
        global_path.header.frame_id = global_frame_;
        global_path.header.stamp = node_->now();

        // 坐标转换
        unsigned int mx_start, my_start, mx_goal, my_goal;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
            !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {

            RCLCPP_ERROR(node_->get_logger(), "Start or Goal is outside of costmap bounds");
            logMetrics(false, 0.0, 0, 0, 0, global_path);
            return global_path;
        }

        // 路径规划
        int width = costmap_->getSizeInCellsX(),
            height = costmap_->getSizeInCellsY();
        int map_size = width * height;
        std::vector<double> g_values(map_size, std::numeric_limits<double>::max()); // 从起点到每个节点的实际代价
        std::vector<int> parent_map(map_size, -1); // 记录每个节点的父节点索引，便于回溯路径

        typedef std::pair<double, int> Node; // A*算法中的节点，包含f值（g+h）和节点索引
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list; // A*算法的优先队列，按照f值（g+h）排序

        int start_idx = my_start * width + mx_start,
            goal_idx = my_goal * width + mx_goal;

        g_values[start_idx] = 0.0;
        open_list.push({0.0, start_idx});

        bool found_path = false;
        std::vector<bool> discovered(static_cast<std::size_t>(map_size), false);
        discovered[start_idx] = true;
        std::size_t expanded_nodes = 0;
        std::size_t generated_nodes = 1;
        std::size_t open_peak = 1;

        const auto search_start_time = std::chrono::steady_clock::now();
        while (!open_list.empty()) {

            int cur_idx = open_list.top().second;
            open_list.pop();
            ++expanded_nodes;

            if (cur_idx == goal_idx) {

                found_path = true;
                break;
            }

            int cx = cur_idx % width,
                cy = cur_idx / width;

            for (int dx = -1; dx <= 1; ++ dx) {

                for (int dy = -1; dy <= 1; ++ dy) {

                    if (dx == 0 && dy == 0) continue;
                    int nx = cx + dx,
                        ny = cy + dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                    int next_idx = ny * width + nx;
                    unsigned char cost = costmap_->getCost(nx, ny);

                    // 障碍物仍然不可通行；未知区域根据开关决定是否与自由区域完全等价。
                    // 超过阈值的成本视为障碍物，未知区域根据开关决定是否与自由区域完全等价。
                    if (cost >= cost_threshold_ &&
                        cost != nav2_costmap_2d::NO_INFORMATION)
                        continue;

                    double extra_cost = 0.0;
                    if (cost == nav2_costmap_2d::NO_INFORMATION &&
                        !treat_unknown_as_free_)
                        extra_cost = unknown_cost_;

                    double step_cost = std::sqrt(dx * dx + dy * dy);
                    double tentative_g = g_values[cur_idx] + step_cost + extra_cost;

                    if (tentative_g < g_values[next_idx]) {

                        if (!discovered[next_idx]) {
                            discovered[next_idx] = true;
                            ++generated_nodes;
                        }
                        g_values[next_idx] = tentative_g;
                        parent_map[next_idx] = cur_idx;

                        double h_cost = std::sqrt(std::pow(nx - (int)mx_goal, 2) +
                                        std::pow(ny - (int)my_goal, 2));
                        open_list.push({tentative_g + h_cost, next_idx});
                        open_peak = std::max(open_peak, open_list.size());
                    }
                }
            }
        }
        const auto search_end_time = std::chrono::steady_clock::now();
        const double search_ms =
            std::chrono::duration<double, std::milli>(
                search_end_time - search_start_time).count();

        if (found_path) {

            std::vector<int> path_;
            int curr_idx = goal_idx;
            while (curr_idx != -1) {

                path_.push_back(curr_idx);
                curr_idx = parent_map[curr_idx];
            }
            std::reverse(path_.begin(), path_.end());

            for (int idx : path_) {

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = global_frame_;
                pose.header.stamp = node_->now();

                unsigned int mx = idx % width,
                    my = idx / width;
                double wx, wy;
                costmap_->mapToWorld(mx, my, wx, wy);

                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.orientation = goal.pose.orientation;
                global_path.poses.push_back(pose);
            }
        } else {

            global_path.poses.clear();
            RCLCPP_WARN(node_->get_logger(), "A* failed to find a path from start to goal");
        }

        if (found_path && !global_path.poses.empty() && replan_event_pub_) {
            rm_interfaces::msg::ReplanEvent event;
            event.header.stamp = node_->now();
            event.header.frame_id = global_frame_;
            event.need_replan = true;
            event.event_id = ++replan_event_id_;
            event.reason = rm_interfaces::msg::ReplanEvent::FORCED;
            event.candidate_path_stamp = global_path.header.stamp;
            event.goal = goal;
            replan_event_pub_->publish(event);

            RCLCPP_INFO(
                node_->get_logger(),
                "Published A* replan event id=%lu",
                static_cast<unsigned long>(event.event_id));
        }

        logMetrics(
            found_path,
            search_ms,
            expanded_nodes,
            generated_nodes,
            open_peak,
            global_path);
        return global_path;
    }
}// namespace my_nav2_planner

PLUGINLIB_EXPORT_CLASS(my_nav2_planner::MyAStarPlanner, nav2_core::GlobalPlanner)
