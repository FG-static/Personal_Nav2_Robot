#include "my_rrtstar_planner/rrtstar_planner.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "my_planning_metrics/path_metrics.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

namespace my_rrtstar_planner {

    void MyRRTStarPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".step_size", rclcpp::ParameterValue(1.0));
        node_->get_parameter(name_ + ".step_size", step_size_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".max_iterations", rclcpp::ParameterValue(50000));
        node_->get_parameter(name_ + ".max_iterations", max_iterations_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".search_radius", rclcpp::ParameterValue(2.0));
        node_->get_parameter(name_ + ".search_radius", search_radius_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".goal_sample_rate", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".goal_sample_rate", goal_sample_rate_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.2));
        node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".collision_check_resolution", rclcpp::ParameterValue(0.02));
        node_->get_parameter(name_ + ".collision_check_resolution", collision_check_resolution_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".max_iterations_after_goal", rclcpp::ParameterValue(1000));
        node_->get_parameter(name_ + ".max_iterations_after_goal", max_iterations_after_goal_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
        node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".treat_unknown_as_free", rclcpp::ParameterValue(false));
        node_->get_parameter(
            name_ + ".treat_unknown_as_free", treat_unknown_as_free_);

        // 初始化随机数生成器
        rng.seed(std::random_device{}());
        double 
            origin_x = costmap_ros->getCostmap()->getOriginX(),
            origin_y = costmap_ros->getCostmap()->getOriginY(),
            size_x = costmap_ros->getCostmap()->getSizeInMetersX(),
            size_y = costmap_ros->getCostmap()->getSizeInMetersY();
        uni_x = std::uniform_real_distribution<double>(origin_x, origin_x + size_x);
        uni_y = std::uniform_real_distribution<double>(origin_y, origin_y + size_y);

        RCLCPP_INFO(node_->get_logger(), "自定义RRT*规划器配置完成");
    }

    void MyRRTStarPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "插件已激活"); }
    void MyRRTStarPlanner::deactivate() { RCLCPP_INFO(node_->get_logger(), "插件已停用"); }
    void MyRRTStarPlanner::cleanup() { RCLCPP_INFO(node_->get_logger(), "插件已清理"); }

    bool MyRRTStarPlanner::isCellTraversable(unsigned int mx, unsigned int my) const {

        const unsigned char cost = costmap_->getCost(mx, my);
        if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return true;
        if (cost == nav2_costmap_2d::NO_INFORMATION)
            return allow_unknown_ || treat_unknown_as_free_;
        return false;
    }

    bool MyRRTStarPlanner::isCollisionFreePath(int idx1, int idx2) {

        // 将地图索引转换为地图坐标，再转换为世界坐标进行检查
        unsigned int width = costmap_->getSizeInCellsX();
        unsigned int mx1 = idx1 % width, my1 = idx1 / width;
        unsigned int mx2 = idx2 % width, my2 = idx2 / width;
        
        double wx1, wy1, wx2, wy2;
        costmap_->mapToWorld(mx1, my1, wx1, wy1);
        costmap_->mapToWorld(mx2, my2, wx2, wy2);
        
        // 在世界坐标中进行线性插值检查
        double dist = std::sqrt((wx2 - wx1) * (wx2 - wx1) + (wy2 - wy1) * (wy2 - wy1));
        int steps = std::max(1, static_cast<int>(dist / collision_check_resolution_));
        
        for (int i = 0; i <= steps; ++ i) {

            double t = (steps > 0) ? static_cast<double>(i) / steps : 0.0;
            double 
                wx = (1 - t) * wx1 + t * wx2,
                wy = (1 - t) * wy1 + t * wy2;
            
            unsigned int mx, my;
            if (!costmap_->worldToMap(wx, wy, mx, my)) return false; // 超出地图范围
            
            if (!isCellTraversable(mx, my)) return false;
        }
        return true;
    }

    nav_msgs::msg::Path MyRRTStarPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) {

        const auto total_start_time = std::chrono::steady_clock::now();
        std::size_t goal_samples = 0;
        std::size_t random_samples = 0;
        std::size_t blocked_sample_rejections = 0;
        std::size_t no_motion_rejections = 0;
        std::size_t blocked_node_rejections = 0;
        std::size_t duplicate_rejections = 0;
        std::size_t collision_rejections = 0;
        std::size_t successful_extensions = 0;
        auto logMetrics =
            [&](
                bool success,
                int iterations,
                int first_solution_iteration,
                std::size_t tree_nodes,
                std::size_t rewire_count,
                double first_solution_ms,
                double refinement_ms,
                double front_end_ms,
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
                    "FRONTEND_METRICS algorithm=RRTStar success=%s reused=false "
                    "expanded_nodes=%zu tree_nodes=%zu iterations=%d "
                    "first_solution_iteration=%d rewires=%zu first_solution_ms=%.3f "
                    "front_end_refinement_ms=%.3f front_end_ms=%.3f "
                    "planner_total_ms=%.3f metrics_eval_ms=%.3f path_points=%zu "
                    "path_length_m=%.3f max_curvature_1pm=%.3f "
                    "min_lethal_obstacle_distance_m=%.3f goal_samples=%zu "
                    "random_samples=%zu successful_extensions=%zu "
                    "blocked_sample_rejections=%zu no_motion_rejections=%zu "
                    "blocked_node_rejections=%zu duplicate_rejections=%zu "
                    "collision_rejections=%zu",
                    success ? "true" : "false",
                    tree_nodes,
                    tree_nodes,
                    iterations,
                    first_solution_iteration,
                    rewire_count,
                    first_solution_ms,
                    refinement_ms,
                    front_end_ms,
                    planner_total_ms,
                    metrics_eval_ms,
                    path_metrics.point_count,
                    path_metrics.length_m,
                    path_metrics.max_curvature_inv_m,
                    path_metrics.min_lethal_obstacle_distance_m,
                    goal_samples,
                    random_samples,
                    successful_extensions,
                    blocked_sample_rejections,
                    no_motion_rejections,
                    blocked_node_rejections,
                    duplicate_rejections,
                    collision_rejections);
            };

        nav_msgs::msg::Path global_path;
        global_path.header.frame_id = global_frame_;
        global_path.header.stamp = node_->now();

        // 坐标转换
        unsigned int mx_start, my_start, mx_goal, my_goal;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
            !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {

            RCLCPP_ERROR(node_->get_logger(), "Start or Goal is outside of costmap bounds");
            logMetrics(false, 0, -1, 0, 0, -1.0, 0.0, 0.0, global_path);
            return global_path;
        }

        const unsigned char start_cost = costmap_->getCost(mx_start, my_start);
        const unsigned char goal_cost = costmap_->getCost(mx_goal, my_goal);
        if (!isCellTraversable(mx_start, my_start) ||
            !isCellTraversable(mx_goal, my_goal)) {

            RCLCPP_WARN(
                node_->get_logger(),
                "RRT* rejected blocked start/goal cell: start_cost=%u goal_cost=%u "
                "allow_unknown=%s",
                static_cast<unsigned int>(start_cost),
                static_cast<unsigned int>(goal_cost),
                allow_unknown_ ? "true" : "false");
            logMetrics(false, 0, -1, 0, 0, -1.0, 0.0, 0.0, global_path);
            return global_path;
        }

        // 路径规划
        int width = costmap_->getSizeInCellsX(), 
            height = costmap_->getSizeInCellsY();
        // 将米为单位的参数转换为地图格子数，保持单位一致
        double 
            resolution = costmap_->getResolution(),
            step_size_cells = step_size_ / resolution,
            search_radius_cells = search_radius_ / resolution,
            goal_tolerance_cells = goal_tolerance_ / resolution;

        int start_idx = my_start * width + mx_start,
            goal_idx = my_goal * width + mx_goal;

        if (start_idx == goal_idx) {

            global_path.poses.push_back(start);
            global_path.poses.push_back(goal);
            logMetrics(true, 0, 0, 1, 0, 0.0, 0.0, 0.0, global_path);
            return global_path;
        }

        tree.clear();
        tree.emplace_back(start_idx, -1, 0.0); // 将起点加入树中，父节点索引为-1，代价为0
        std::vector<int> node_index_by_cell(
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height), -1);
        node_index_by_cell[static_cast<std::size_t>(start_idx)] = 0;
        double best_cost = std::numeric_limits<double>::infinity(); // 记录找到的路径的最优代价
        int goal_node_idx = -1; // 记录找到的目标节点在树中的索引

        bool found_path = false;

        // 开始寻路
        int stop_iter = 0; // 优化迭代计数器
        // 记录算法耗时
        auto search_start_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point first_solution_time;
        bool has_first_solution_time = false;
        int iterations_executed = 0;
        int first_solution_iteration = -1;
        std::size_t rewire_count = 0;
        for (int iter = 0; iter < max_iterations_; ++ iter) {
            iterations_executed = iter + 1;

            double r = std::uniform_real_distribution<double>(0.0, 1.0)(rng);
            int sample_idx; // 采样点在地图中的索引
            if (r < goal_sample_rate_) {

                sample_idx = goal_idx;
                ++goal_samples;
            } else {

                double rx = uni_x(rng), ry = uni_y(rng);
                unsigned int mx, my;
                if (!costmap_->worldToMap(rx, ry, mx, my)) continue; // 采样点在地图外，丢弃
                ++random_samples;
                // 如果采样点落在障碍上也丢弃
                if (!isCellTraversable(mx, my)) {

                    ++blocked_sample_rejections;
                    continue;
                }
                sample_idx = my * width + mx;
            }

            const int sample_mx = sample_idx % width;
            const int sample_my = sample_idx / width;

            // RRT* 先寻找距离采样点最近的树节点，再向采样点扩展固定步长。
            int nearest_idx = -1;
            double nearest_distance_sq = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < tree.size(); ++ i) {

                const int node_mx = tree[i].pos_idx % width;
                const int node_my = tree[i].pos_idx / width;
                const double dx = static_cast<double>(sample_mx - node_mx);
                const double dy = static_cast<double>(sample_my - node_my);
                const double distance_sq = dx * dx + dy * dy;
                if (distance_sq < nearest_distance_sq) {

                    nearest_distance_sq = distance_sq;
                    nearest_idx = static_cast<int>(i);
                }
            }

            if (nearest_idx < 0)
                continue;

            const int nearest_mx = tree[nearest_idx].pos_idx % width;
            const int nearest_my = tree[nearest_idx].pos_idx / width;
            double dx = static_cast<double>(sample_mx - nearest_mx);
            double dy = static_cast<double>(sample_my - nearest_my);
            const double dist = std::hypot(dx, dy);
            if (dist > step_size_cells && dist > 0.0) {

                dx *= step_size_cells / dist;
                dy *= step_size_cells / dist;
            }

            const int new_mx_signed = nearest_mx + static_cast<int>(std::lround(dx));
            const int new_my_signed = nearest_my + static_cast<int>(std::lround(dy));
            if (new_mx_signed < 0 || new_my_signed < 0 ||
                new_mx_signed >= width || new_my_signed >= height) {

                continue;
            }

            const unsigned int new_mx = static_cast<unsigned int>(new_mx_signed);
            const unsigned int new_my = static_cast<unsigned int>(new_my_signed);
            const int new_idx = new_my_signed * width + new_mx_signed;

            if (new_idx == tree[nearest_idx].pos_idx) {

                ++no_motion_rejections;
                continue;
            }
            if (node_index_by_cell[static_cast<std::size_t>(new_idx)] >= 0) {

                ++duplicate_rejections;
                continue;
            }
            if (!isCellTraversable(new_mx, new_my)) {

                ++blocked_node_rejections;
                continue;
            }
            if (!isCollisionFreePath(tree[nearest_idx].pos_idx, new_idx)) {

                ++collision_rejections;
                continue;
            }

            // 在新节点邻域内选择总代价最低且无碰撞的父节点。
            int best_parent_idx = nearest_idx;
            const double nearest_edge_cost = std::hypot(
                static_cast<double>(new_mx_signed - nearest_mx),
                static_cast<double>(new_my_signed - nearest_my));
            double cost_to_new_node = tree[nearest_idx].cost + nearest_edge_cost;
            for (size_t i = 0; i < tree.size(); ++ i) {

                const int candidate_mx = tree[i].pos_idx % width;
                const int candidate_my = tree[i].pos_idx / width;
                const double distance_to_new = std::hypot(
                    static_cast<double>(new_mx_signed - candidate_mx),
                    static_cast<double>(new_my_signed - candidate_my));
                if (distance_to_new > search_radius_cells)
                    continue;

                const double candidate_cost = tree[i].cost + distance_to_new;
                if (candidate_cost >= cost_to_new_node)
                    continue;
                if (!isCollisionFreePath(tree[i].pos_idx, new_idx)) {

                    ++collision_rejections;
                    continue;
                }

                best_parent_idx = static_cast<int>(i);
                cost_to_new_node = candidate_cost;
            }

            tree.emplace_back(new_idx, best_parent_idx, cost_to_new_node);
            const int new_node_idx = static_cast<int>(tree.size() - 1);
            node_index_by_cell[static_cast<std::size_t>(new_idx)] = new_node_idx;
            ++successful_extensions;

            // 重连附近节点 Rewire
            for (int i = 1; i < new_node_idx; ++ i) {

                const int curr_mx = tree[i].pos_idx % width;
                const int curr_my = tree[i].pos_idx / width;
                const double dist_to_new_node = std::hypot(
                    static_cast<double>(curr_mx - new_mx_signed),
                    static_cast<double>(curr_my - new_my_signed));
                if (dist_to_new_node > search_radius_cells)
                    continue;

                const double cost_via_new_node = cost_to_new_node + dist_to_new_node;
                if (cost_via_new_node >= tree[i].cost)
                    continue;
                if (!isCollisionFreePath(tree[i].pos_idx, new_idx)) {

                    ++collision_rejections;
                    continue;
                }

                const double old_cost = tree[i].cost;
                tree[i].parent = new_node_idx;
                tree[i].cost = cost_via_new_node;
                updateChildrenCost(i, cost_via_new_node - old_cost);
                ++rewire_count;
            }

            // 检查是否到达目标点附近
            const double ddx_goal = static_cast<double>(
                static_cast<int>(mx_goal) - new_mx_signed);
            const double ddy_goal = static_cast<double>(
                static_cast<int>(my_goal) - new_my_signed);
            const double dist_to_goal_sqrd =
                ddx_goal * ddx_goal + ddy_goal * ddy_goal;

            if (dist_to_goal_sqrd <= goal_tolerance_cells * goal_tolerance_cells) {

                // 检查到目标的路径是否碰撞
                if (isCollisionFreePath(new_idx, goal_idx)) {

                    found_path = true;
                    if (stop_iter == 0) {

                        first_solution_time = std::chrono::steady_clock::now();
                        has_first_solution_time = true;
                        first_solution_iteration = iter + 1;
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "RRT* found a path to the goal in %d iterations, now optimizing...",
                            iter + 1);
                    }
                    double cost_to_goal = cost_to_new_node + std::sqrt(dist_to_goal_sqrd);
                    if (cost_to_goal < best_cost) {

                        best_cost = cost_to_goal;
                        goal_node_idx = new_node_idx;
                    }
                } else {

                    ++collision_rejections;
                }
            }
            if (found_path) {

                stop_iter ++;

                if (stop_iter >= max_iterations_after_goal_) {

                    RCLCPP_INFO(node_->get_logger(), "RRT* optimization finished after %d iterations", iter + 1);
                    break;
                }
            }
        }
        const auto search_end_time = std::chrono::steady_clock::now();
        const double first_solution_ms = has_first_solution_time ?
            std::chrono::duration<double, std::milli>(first_solution_time - search_start_time).count() :
            -1.0;
        const double refinement_ms = has_first_solution_time ?
            std::chrono::duration<double, std::milli>(search_end_time - first_solution_time).count() :
            0.0;
        const double front_end_ms =
            std::chrono::duration<double, std::milli>(
                search_end_time - search_start_time).count();

        if (found_path && goal_node_idx != -1) {

            std::vector<int> path_;
            int curr_idx = goal_node_idx;
            while (curr_idx != -1) {

                path_.push_back(tree[curr_idx].pos_idx);
                curr_idx = tree[curr_idx].parent;
            }
            std::reverse(path_.begin(), path_.end());
            
            // 添加目标点
            path_.push_back(goal_idx);

            // 线性插值生成全局路径
            for (size_t i = 0; i < path_.size() - 1; ++ i) {

                unsigned int mx1 = path_[i] % width, my1 = path_[i] / width;
                unsigned int mx2 = path_[i + 1] % width, my2 = path_[i + 1] / width;

                double wx1, wy1, wx2, wy2;
                costmap_->mapToWorld(mx1, my1, wx1, wy1);
                costmap_->mapToWorld(mx2, my2, wx2, wy2);

                double dist = std::hypot(wx2 - wx1, wy2 - wy1);
                // 根据距离和碰撞检查分辨率计算插值点的数量，确保路径平滑且足够密集进行碰撞检查
                int steps = std::max(1, static_cast<int>(dist / collision_check_resolution_));

                for (int j = 0; j < steps; ++ j) {

                    double t = static_cast<double>(j) / steps;
                    double 
                        wx = (1 - t) * wx1 + t * wx2,
                        wy = (1 - t) * wy1 + t * wy2;

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = global_frame_;
                    pose.header.stamp = node_->now();
                    pose.pose.position.x = wx;
                    pose.pose.position.y = wy;
                    pose.pose.orientation.w = goal.pose.orientation.w;
                    global_path.poses.push_back(pose);
                }
            }
        } else {

            global_path.poses.clear();
            RCLCPP_WARN(
                node_->get_logger(),
                "RRT* failed after %d iterations: tree=%zu extensions=%zu "
                "blocked_samples=%zu blocked_nodes=%zu duplicates=%zu "
                "no_motion=%zu collision_edges=%zu",
                iterations_executed,
                tree.size(),
                successful_extensions,
                blocked_sample_rejections,
                blocked_node_rejections,
                duplicate_rejections,
                no_motion_rejections,
                collision_rejections);
        }
        const bool success = found_path && goal_node_idx != -1 && !global_path.poses.empty();
        logMetrics(
            success,
            iterations_executed,
            first_solution_iteration,
            tree.size(),
            rewire_count,
            first_solution_ms,
            refinement_ms,
            front_end_ms,
            global_path);
        return global_path;
    }

    /**
     * @brief 递归更新子节点的代价
     * @param parent_idx 父节点在树中的索引
     * @param cost_delta 父节点代价的变化量（新代价 - 旧代价）
     * 这个函数会遍历树中所有以 parent_idx 为父节点的节点，更新它们的代价，并递归更新它们的子节点的代价
     * @return void
     */
    void MyRRTStarPlanner::updateChildrenCost(int parent_idx, double cost_delta) {
        
        // 递归遍历所有子节点，更新它们的代价
        for (size_t i = 0; i < tree.size(); ++ i) {

            if (tree[i].parent == parent_idx) {

                tree[i].cost += cost_delta;
                updateChildrenCost(i, cost_delta); // 递归更新其子节点
            }
        }
    }
}// namespace my_rrtstar_planner

PLUGINLIB_EXPORT_CLASS(my_rrtstar_planner::MyRRTStarPlanner, nav2_core::GlobalPlanner)
