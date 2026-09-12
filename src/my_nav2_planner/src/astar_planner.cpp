#include "my_nav2_planner/astar_planner.hpp"
#include "my_planning_metrics/path_metrics.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
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
            node_, name_ + ".cost_threshold", rclcpp::ParameterValue(253));
        node_->get_parameter(name_ + ".cost_threshold", cost_threshold_);
        cost_threshold_ = std::clamp(cost_threshold_, 0, 253);
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".obstacle_cost_weight", rclcpp::ParameterValue(6.0));
        node_->get_parameter(name_ + ".obstacle_cost_weight", obstacle_cost_weight_);
        if (obstacle_cost_weight_ < 0.0) {
            obstacle_cost_weight_ = 0.0;
        }
        // >0 时对 metrics 的整图距离变换按该间隔（秒）节流，<=0 每次都评估
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".metrics_min_interval", rclcpp::ParameterValue(1.0));
        node_->get_parameter(name_ + ".metrics_min_interval", metrics_min_interval_);

        //RCLCPP_INFO(node_->get_logger(), "自定义A*规划器配置完成");
    }

    void MyAStarPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "插件已激活"); }
    void MyAStarPlanner::deactivate() { RCLCPP_INFO(node_->get_logger(), "插件已停用"); }
    void MyAStarPlanner::cleanup() { RCLCPP_INFO(node_->get_logger(), "插件已清理"); }

    nav_msgs::msg::Path MyAStarPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) {

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

                // 整图距离变换按间隔节流：间隔内复用上次评估跳过重建，
                // 用 min_lethal=-1 标记本次未评估
                bool evaluate_distance = metrics_min_interval_ <= 0.0;
                if (!evaluate_distance) {
                    
                    const rclcpp::Time now = node_->now();
                    evaluate_distance =
                        last_metrics_eval_time_.nanoseconds() == 0 ||
                        (now - last_metrics_eval_time_).seconds() >=
                            metrics_min_interval_;
                }

                my_planning_metrics::ObstacleDistanceField distance_field;
                bool distance_ready = false;
                double metrics_eval_ms = 0.0;
                if (evaluate_distance && !path.poses.empty()) {
                    
                    const auto metrics_start_time = std::chrono::steady_clock::now();
                    distance_ready = distance_field.build(costmap_);
                    metrics_eval_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - metrics_start_time).count();
                    if (distance_ready) {
                        
                        last_metrics_eval_time_ = node_->now();
                    }
                }
                const my_planning_metrics::PathMetrics path_metrics =
                    my_planning_metrics::evaluatePath(
                        path,
                        distance_ready ? &distance_field : nullptr);

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
                    distance_ready ?
                        path_metrics.min_lethal_obstacle_distance_m : -1.0);
            };

        nav_msgs::msg::Path global_path;
        global_path.header.frame_id = global_frame_;
        global_path.header.stamp = node_->now();

        // 坐标转换；出图按 nav2_core 契约抛异常
        unsigned int mx_start, my_start, mx_goal, my_goal;
        const bool start_in_map = costmap_->worldToMap(
            start.pose.position.x, start.pose.position.y, mx_start, my_start);
        const bool goal_in_map = costmap_->worldToMap(
            goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);
        if (!start_in_map || !goal_in_map) {

            RCLCPP_ERROR(node_->get_logger(), "Start or Goal is outside of costmap bounds");
            logMetrics(false, 0.0, 0, 0, 0, global_path);
            if (!start_in_map) {
                throw nav2_core::PlannerException("Start is outside of costmap bounds");
            }
            throw nav2_core::PlannerException("Goal is outside of costmap bounds");
        }

        // start 落在硬障碍内保持宽容：警告后继续规划（常见于贴墙起步）
        if (isBlockedCell(*costmap_, static_cast<int>(mx_start), static_cast<int>(my_start))) {

            RCLCPP_WARN(
                node_->get_logger(),
                "Start cell is inside a lethal/inscribed obstacle, planning anyway");
        }

        // 栅格搜索；goal 占据等域错误由 searchCells 抛出
        // Humble planner_server 的 createPlan 没有 cancel_checker，单元测试可直接测 searchCells
        const auto search_start_time = std::chrono::steady_clock::now();
        AStarSearchResult search;
        try {
            search = searchCells(
                *costmap_, mx_start, my_start, mx_goal, my_goal, {});
        } catch (const nav2_core::PlannerException & ex) {
            RCLCPP_WARN(node_->get_logger(), "A* planning failed: %s", ex.what());
            logMetrics(false, 0.0, 0, 0, 0, global_path);
            throw;
        }
        const auto search_end_time = std::chrono::steady_clock::now();
        const double search_ms =
            std::chrono::duration<double, std::milli>(
                search_end_time - search_start_time).count();

        if (!search.found) {

            RCLCPP_WARN(node_->get_logger(), "A* failed to find a path from start to goal");
            logMetrics(false, search_ms, search.expanded_nodes,
                search.generated_nodes, search.open_peak, global_path);
            throw nav2_core::PlannerException(
                "A* failed to find a path from start to goal");
        }

        // 遍历搜索结果，将路径点转换为 PoseStamped 并添加到 global_path 中
        const int width = static_cast<int>(costmap_->getSizeInCellsX());
        for (const std::uint64_t idx : search.cells) {

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.header.stamp = node_->now();

            const unsigned int mx = static_cast<unsigned int>(idx % width);
            const unsigned int my = static_cast<unsigned int>(idx / width);
            double wx, wy;
            costmap_->mapToWorld(mx, my, wx, wy);

            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation = goal.pose.orientation;
            global_path.poses.push_back(pose);
        }

        if (replan_event_pub_) {
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
            true,
            search_ms,
            search.expanded_nodes,
            search.generated_nodes,
            search.open_peak,
            global_path);
        return global_path;
    }

    bool MyAStarPlanner::isBlockedCell(
        const nav2_costmap_2d::Costmap2D & costmap, int cx, int cy) const {

        if (cx < 0 || cy < 0 ||
            cx >= static_cast<int>(costmap.getSizeInCellsX()) ||
            cy >= static_cast<int>(costmap.getSizeInCellsY())) {
            return true;
        }
        const unsigned char cost = costmap.getCost(
            static_cast<unsigned int>(cx), static_cast<unsigned int>(cy));
        return cost >= cost_threshold_ &&
               cost != nav2_costmap_2d::NO_INFORMATION;
    }

    MyAStarPlanner::AStarSearchResult MyAStarPlanner::searchCells(
        const nav2_costmap_2d::Costmap2D & costmap,
        unsigned int start_x, unsigned int start_y,
        unsigned int goal_x, unsigned int goal_y,
        const std::function<bool()> & cancel_checker) {

        AStarSearchResult result;
        const int width = static_cast<int>(costmap.getSizeInCellsX());
        const int height = static_cast<int>(costmap.getSizeInCellsY());
        if (width <= 0 || height <= 0 ||
            start_x >= static_cast<unsigned int>(width) ||
            start_y >= static_cast<unsigned int>(height) ||
            goal_x >= static_cast<unsigned int>(width) ||
            goal_y >= static_cast<unsigned int>(height)) {
            return result;
        }

        const int start_idx = static_cast<int>(start_y) * width + static_cast<int>(start_x);
        const int goal_idx = static_cast<int>(goal_y) * width + static_cast<int>(goal_x);

        // goal 落在硬障碍内时提前退出；goal==start 保留原单点成功行为
        if (start_idx != goal_idx &&
            isBlockedCell(costmap, static_cast<int>(goal_x), static_cast<int>(goal_y))) {
            throw nav2_core::PlannerException("Goal cell is inside a lethal/inscribed obstacle");
        }

        constexpr double kInfinityCost = std::numeric_limits<double>::max();
        std::vector<double> g_values(static_cast<std::size_t>(width) * height, kInfinityCost);
        std::vector<int> parent_map(static_cast<std::size_t>(width) * height, -1);

        typedef std::pair<double, int> Node; // A*算法中的节点，包含f值（g+h）和节点索引
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

        g_values[start_idx] = 0.0;
        open_list.push({0.0, start_idx});

        std::vector<bool> discovered(g_values.size(), false);
        discovered[start_idx] = true;

        constexpr std::size_t kCancelCheckInterval = 1024;
        while (!open_list.empty()) {

            const int cur_idx = open_list.top().second;
            open_list.pop();
            ++result.expanded_nodes;

            if (cancel_checker &&
                result.expanded_nodes % kCancelCheckInterval == 0U &&
                cancel_checker()) {
                if (node_) {
                    RCLCPP_WARN(node_->get_logger(), "A* planning cancelled");
                }
                return result;
            }

            if (cur_idx == goal_idx) {
                result.found = true;
                break;
            }

            const int cx = cur_idx % width,
                      cy = cur_idx / width;

            for (int dx = -1; dx <= 1; ++ dx) {

                for (int dy = -1; dy <= 1; ++ dy) {

                    if (dx == 0 && dy == 0) continue;
                    const int nx = cx + dx,
                              ny = cy + dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                    // 防对角穿角：斜向移动要求两个正交邻格都不是硬障碍
                    if (dx != 0 && dy != 0 &&
                        (isBlockedCell(costmap, cx + dx, cy) ||
                         isBlockedCell(costmap, cx, cy + dy))) {
                        continue;
                    }

                    const int next_idx = ny * width + nx;
                    const unsigned char cost = costmap.getCost(
                        static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));

                    // 超过阈值（默认 inscribed/lethal）视为硬障碍。
                    // 低于阈值的膨胀代价进入 g，未知区域按开关决定是否加罚。
                    double extra_cost = 0.0;
                    if (cost == nav2_costmap_2d::NO_INFORMATION) {
                        if (!treat_unknown_as_free_) {
                            extra_cost = unknown_cost_;
                        }
                    } else if (cost >= cost_threshold_) {
                        continue;
                    } else {
                        extra_cost = obstacle_cost_weight_ *
                            static_cast<double>(cost) /
                            static_cast<double>(
                                nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
                    }

                    const double step_cost = std::sqrt(dx * dx + dy * dy);
                    const double tentative_g =
                        g_values[cur_idx] + step_cost + extra_cost;

                    if (tentative_g < g_values[next_idx]) {

                        if (!discovered[next_idx]) {
                            discovered[next_idx] = true;
                            ++result.generated_nodes;
                        }
                        g_values[next_idx] = tentative_g;
                        parent_map[next_idx] = cur_idx;

                        const double h_cost = std::sqrt(std::pow(nx - static_cast<int>(goal_x), 2) +
                                        std::pow(ny - static_cast<int>(goal_y), 2));
                        open_list.push({tentative_g + h_cost, next_idx});
                        result.open_peak = std::max(result.open_peak, open_list.size());
                    }
                }
            }
        }

        if (!result.found) {
            return result;
        }

        // 回溯路径（含起终点）
        int curr_idx = goal_idx;
        while (curr_idx != -1) {
            result.cells.push_back(static_cast<std::uint64_t>(curr_idx));
            curr_idx = parent_map[curr_idx];
        }
        std::reverse(result.cells.begin(), result.cells.end());
        return result;
    }
}// namespace my_nav2_planner

PLUGINLIB_EXPORT_CLASS(my_nav2_planner::MyAStarPlanner, nav2_core::GlobalPlanner)
