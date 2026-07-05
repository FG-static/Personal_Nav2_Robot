#include "my_hybrid_astar_planner/hybrid_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;

namespace my_hybrid_astar_planner {

// 哈希函数 用于快速查找已扩展节点索引
std::size_t StateKeyHasher::operator()(const StateKey &key) const {

    std::size_t seed = static_cast<std::size_t>(key.mx);
    seed = seed * 73856093U ^ static_cast<std::size_t>(key.my);
    seed = seed * 19349663U ^ static_cast<std::size_t>(key.theta_id);
    return seed;
}

void MyHybridAStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

    node_ = parent.lock();
    if (!node_)
        throw nav2_core::PlannerException("Failed to lock lifecycle node in MyHybridAStarPlanner");

    tf_ = std::move(tf);
    name_ = std::move(name);
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    replan_event_pub_ = node_->create_publisher<rm_interfaces::msg::ReplanEvent>(
        "trajectory_generation/replan_event",
        rclcpp::QoS(1).reliable().transient_local());

    auto declare_double_param =
        [&](const std::string &param_name, double default_value, double &target) {
            declare_parameter_if_not_declared(
                node_, name_ + "." + param_name, rclcpp::ParameterValue(default_value));
            node_->get_parameter(name_ + "." + param_name, target);
        };
    auto declare_int_param =
        [&](const std::string &param_name, int default_value, int &target) {
            declare_parameter_if_not_declared(
                node_, name_ + "." + param_name, rclcpp::ParameterValue(default_value));
            node_->get_parameter(name_ + "." + param_name, target);
        };
    auto declare_bool_param =
        [&](const std::string &param_name, bool default_value, bool &target) {
            declare_parameter_if_not_declared(
                node_, name_ + "." + param_name, rclcpp::ParameterValue(default_value));
            node_->get_parameter(name_ + "." + param_name, target);
        };

    declare_double_param("unknown_cost", params_.unknown_cost, params_.unknown_cost);
    declare_double_param(
        "interpolation_resolution",
        params_.interpolation_resolution,
        params_.interpolation_resolution);
    declare_double_param("xy_resolution", params_.xy_resolution, params_.xy_resolution);
    declare_int_param("yaw_bin_count", params_.yaw_bin_count, params_.yaw_bin_count);
    declare_double_param("step_time", params_.step_time, params_.step_time);
    declare_double_param("primitive_duration", params_.primitive_duration, params_.primitive_duration);
    declare_double_param("max_vel_x", params_.max_vel_x, params_.max_vel_x);
    declare_double_param("max_vel_y", params_.max_vel_y, params_.max_vel_y);
    declare_double_param("max_vel_theta", params_.max_vel_theta, params_.max_vel_theta);
    declare_double_param("acc_lim_x", params_.acc_lim_x, params_.acc_lim_x);
    declare_double_param("acc_lim_y", params_.acc_lim_y, params_.acc_lim_y);
    declare_double_param("acc_lim_theta", params_.acc_lim_theta, params_.acc_lim_theta);
    declare_bool_param("allow_reverse", params_.allow_reverse, params_.allow_reverse);
    declare_double_param(
        "obstacle_cost_weight",
        params_.obstacle_cost_weight,
        params_.obstacle_cost_weight);
    declare_double_param(
        "goal_tolerance_xy",
        params_.goal_tolerance_xy,
        params_.goal_tolerance_xy);
    declare_double_param(
        "goal_tolerance_yaw",
        params_.goal_tolerance_yaw,
        params_.goal_tolerance_yaw);
    declare_double_param(
        "heuristic_grid_weight",
        params_.heuristic_grid_weight,
        params_.heuristic_grid_weight);
    declare_double_param(
        "heuristic_yaw_weight",
        params_.heuristic_yaw_weight,
        params_.heuristic_yaw_weight);
    declare_double_param(
        "path_tangent_change_weight",
        params_.path_tangent_change_weight,
        params_.path_tangent_change_weight);
    declare_double_param("rs_weight", params_.rs_weight, params_.rs_weight);
    declare_double_param(
        "rs_reverse_penalty",
        params_.rs_reverse_penalty,
        params_.rs_reverse_penalty);
    declare_double_param(
        "rs_gear_switch_penalty",
        params_.rs_gear_switch_penalty,
        params_.rs_gear_switch_penalty);
    declare_double_param(
        "analytic_expansion_distance",
        params_.analytic_expansion_distance,
        params_.analytic_expansion_distance);
    declare_double_param(
        "replan_time_threshold",
        params_.replan_time_threshold,
        params_.replan_time_threshold);
    declare_double_param(
        "path_prune_distance",
        params_.path_prune_distance,
        params_.path_prune_distance);
    declare_int_param(
        "max_iterations",
        params_.max_iterations,
        params_.max_iterations);
    declare_double_param(
        "max_planning_time",
        params_.max_planning_time,
        params_.max_planning_time);
    declare_bool_param(
        "immediate_replan_if_blocked",
        params_.immediate_replan_if_blocked,
        params_.immediate_replan_if_blocked);
    declare_bool_param(
        "reuse_path_if_valid",
        params_.reuse_path_if_valid,
        params_.reuse_path_if_valid);

    buildMotionPrimitives();

    RCLCPP_INFO(
        node_->get_logger(),
        "Mecanum Hybrid A* planner configured: yaw_bins=%d primitives=%zu max_vel=(%.2f, %.2f, %.2f) max_iter=%d max_time=%.2fs",
        params_.yaw_bin_count,
        motion_primitives_.size(),
        params_.max_vel_x,
        params_.max_vel_y,
        params_.max_vel_theta,
        params_.max_iterations,
        params_.max_planning_time);
}

void MyHybridAStarPlanner::activate() {

    RCLCPP_INFO(node_->get_logger(), "Mecanum Hybrid A* planner activated");
}

void MyHybridAStarPlanner::deactivate() {

    RCLCPP_INFO(node_->get_logger(), "Mecanum Hybrid A* planner deactivated");
}

void MyHybridAStarPlanner::cleanup() {

    motion_primitives_.clear();
    costmap_ = nullptr;
    RCLCPP_INFO(node_->get_logger(), "Mecanum Hybrid A* planner cleaned up");
}

nav_msgs::msg::Path MyHybridAStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    std::function<bool()> cancel_checker) {

    if (!node_ || !costmap_)
        throw nav2_core::PlannerException("Planner is not configured");

    nav_msgs::msg::Path global_path;
    global_path.header.frame_id = global_frame_;
    global_path.header.stamp = node_->now();

    if (cancel_checker && cancel_checker()) {

        RCLCPP_WARN(node_->get_logger(), "Hybrid A* planning cancelled before search start");
        return global_path;
    }

    unsigned int mx_start = 0;
    unsigned int my_start = 0;
    unsigned int mx_goal = 0;
    unsigned int my_goal = 0;
    if (!validatePose(start, mx_start, my_start, "start") ||
        !validatePose(goal, mx_goal, my_goal, "goal"))
        return global_path;

    const PlannerPose start_pose{start.pose.position.x, start.pose.position.y, poseToYaw(start)};
    const PlannerPose goal_pose{goal.pose.position.x, goal.pose.position.y, poseToYaw(goal)};

    if (isGoalReached(start_pose, goal_pose)) {

        global_path.poses.push_back(start);
        global_path.poses.push_back(goal);
        return global_path;
    }

    const rclcpp::Time now = node_->now();

    uint8_t replan_reason = rm_interfaces::msg::ReplanEvent::FORCED;

    if (params_.reuse_path_if_valid && has_last_path_ && !last_path_.poses.empty()) {

        const double cached_goal_error = pointDistance2D(last_path_.poses.back(), goal);
        const bool same_goal = cached_goal_error <= params_.goal_tolerance_xy;
        const bool blocked = params_.immediate_replan_if_blocked && isCachedPathBlocked(last_path_);
        const bool timeout = (now - last_plan_time_).seconds() >= params_.replan_time_threshold;
        if (!same_goal)
            replan_reason = rm_interfaces::msg::ReplanEvent::GOAL_CHANGED;
        else if (blocked)
            replan_reason = rm_interfaces::msg::ReplanEvent::PATH_BLOCKED;
        else if (timeout)
            replan_reason = rm_interfaces::msg::ReplanEvent::TIME_EXPIRED;

        if (same_goal && !blocked && !timeout) {

            nav_msgs::msg::Path reused = pruneCachedPath(last_path_, start);
            if (!reused.poses.empty()) {

                reused.header.frame_id = global_frame_;
                reused.header.stamp = now;
                return reused;
            }
        }
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Hybrid A* skeleton ready. Search phase will use %zu mecanum motion primitives.",
        motion_primitives_.size());

    if (!computeGridHeuristic(mx_goal, my_goal)) {

        RCLCPP_ERROR(node_->get_logger(), "Failed to build grid heuristic");
        return global_path;
    }

    std::vector<HybridNode> nodes;
    const int max_iterations = std::max(1, params_.max_iterations);
    const std::size_t max_node_count = static_cast<std::size_t>(max_iterations);
    nodes.reserve(std::min<std::size_t>(max_node_count, 4096));

    std::unordered_map<StateKey, int, StateKeyHasher> node_lookup;
    node_lookup.reserve(std::min<std::size_t>(max_node_count, 4096));

    struct OpenEntry {

        double f;
        double h;
        double g;
        int node_idx;
        bool operator>(const OpenEntry &other) const {

            if (f != other.f)
                return f > other.f;
            if (h != other.h)
                return h > other.h;
            if (g != other.g)
                return g > other.g;
            return node_idx > other.node_idx;
        }
    };

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open_list;

    // 起始节点
    HybridNode start_node;
    start_node.pose = start_pose;
    start_node.key = discretizeState(start_pose);
    start_node.g = 0.0;
    start_node.h_grid = computeNodeHeuristic(start_pose, goal_pose);
    start_node.h_yaw = 0.0;
    start_node.h_rs = 0.0;
    start_node.f = start_node.g + start_node.h_grid;
    if (!std::isfinite(start_node.f)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Hybrid A* start heuristic is not finite. Search may fail quickly.");
    }

    nodes.push_back(start_node);
    node_lookup[start_node.key] = 0;
    open_list.push({start_node.f, start_node.h_grid, start_node.g, 0});

    const rclcpp::Time search_start_time = node_->now();
    int expanded_iterations = 0;

    // 搜索
    while (!open_list.empty()) {

        if (cancel_checker && cancel_checker()) {

            RCLCPP_WARN(node_->get_logger(), "Planning cancelled");
            return nav_msgs::msg::Path{};
        }

        if (expanded_iterations >= max_iterations) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Hybrid A* aborted after reaching max_iterations=%d, nodes=%zu, open=%zu",
                max_iterations,
                nodes.size(),
                open_list.size());
            return global_path;
        }

        if (params_.max_planning_time > 0.0 &&
            (node_->now() - search_start_time).seconds() > params_.max_planning_time) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Hybrid A* aborted after %.3fs, expanded=%d, nodes=%zu, open=%zu",
                (node_->now() - search_start_time).seconds(),
                expanded_iterations,
                nodes.size(),
                open_list.size());
            return global_path;
        }

        const int cur_idx = open_list.top().node_idx;
        open_list.pop();
        if (cur_idx < 0 || cur_idx >= static_cast<int>(nodes.size())) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Hybrid A* open list produced invalid node index %d, nodes=%zu",
                cur_idx,
                nodes.size());
            return global_path;
        }

        HybridNode &current_node = nodes[cur_idx];
        if (current_node.closed) continue;
        current_node.closed = true;
        ++expanded_iterations;

        if (isGoalReached(current_node.pose, goal_pose)) {

            nav_msgs::msg::Path planned_path = reconstructPath(nodes, cur_idx, start, goal);
            last_path_ = planned_path;
            has_last_path_ = true;
            last_plan_time_ = now;
            publishReplanEvent(replan_reason, goal, planned_path.header.stamp);
            RCLCPP_INFO(
                node_->get_logger(),
                "Hybrid A* succeeded in %.3fs, expanded=%d, nodes=%zu, path_points=%zu",
                (node_->now() - search_start_time).seconds(),
                expanded_iterations,
                nodes.size(),
                planned_path.poses.size());
            return planned_path;
        }

        for (const auto &primitive : motion_primitives_) {

            PlannerPose next_pose;
            std::vector<PlannerPose> sampled_poses;
            double transition_cost = 0.0;
            if (!simulatePrimitive(current_node, primitive, next_pose, sampled_poses, transition_cost))
                continue;

            StateKey next_key = discretizeState(next_pose);

            unsigned int end_mx = 0;
            unsigned int end_my = 0;
            if (!costmap_->worldToMap(next_pose.x, next_pose.y, end_mx, end_my))
                continue;

            // Penalize abrupt changes between consecutive path tangents.
            double new_g = current_node.g + transition_cost;
            if (current_node.parent_index >= 0) {

                const auto &parrent_node = nodes[current_node.parent_index];

                const double
                    v1x = current_node.pose.x - parrent_node.pose.x,
                    v1y = current_node.pose.y - parrent_node.pose.y,
                    v2x = next_pose.x - current_node.pose.x,
                    v2y = next_pose.y - current_node.pose.y;

                if (std::hypot(v1x, v1y) > 1e-6 &&
                    std::hypot(v2x, v2y) > 1e-6) {

                    const double yaw1 = std::atan2(v1y, v1x);
                    const double yaw2 = std::atan2(v2y, v2x);
                    const double dtheta =
                        std::abs(normalizeAngle(yaw2 - yaw1));
                    new_g += params_.path_tangent_change_weight * dtheta;
                }
            }

            // push node
            auto it = node_lookup.find(next_key);
            if (it == node_lookup.end()) { // 未拓展过的

                if (nodes.size() >= max_node_count) {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Hybrid A* aborted after reaching max node count=%zu, expanded=%d, open=%zu",
                        max_node_count,
                        expanded_iterations,
                        open_list.size());
                    return global_path;
                }

                HybridNode next_node;
                next_node.key = next_key;
                next_node.pose = next_pose;
                next_node.g = new_g;
                next_node.parent_index = cur_idx;
                next_node.parent_primitive_id = primitive.id;
                next_node.h_grid = computeNodeHeuristic(next_pose, goal_pose);
                next_node.h_yaw = 0.0;
                next_node.h_rs = 0.0;
                next_node.f = next_node.g + next_node.h_grid;

                const int new_index = static_cast<int>(nodes.size());
                nodes.push_back(next_node);
                node_lookup[next_key] = new_index;
                open_list.push({next_node.f, next_node.h_grid, next_node.g, new_index});
            } else { // 非同一路径拓展过的

                HybridNode &old_node = nodes[it->second];
                if (!old_node.closed && new_g < old_node.g) {

                    old_node.pose = next_pose;
                    old_node.g = new_g;
                    old_node.parent_index = cur_idx;
                    old_node.parent_primitive_id = primitive.id;
                    old_node.h_grid = computeNodeHeuristic(next_pose, goal_pose);
                    old_node.h_yaw = 0.0;
                    old_node.h_rs = 0.0;
                    old_node.f = old_node.g + old_node.h_grid;

                    open_list.push({old_node.f, old_node.h_grid, old_node.g, it->second});
                }
            }
        }
    }

    RCLCPP_WARN(
        node_->get_logger(),
        "Hybrid A* failed to find a valid path after %.3fs, expanded=%d, nodes=%zu",
        (node_->now() - search_start_time).seconds(),
        expanded_iterations,
        nodes.size());
    return global_path;
}

// 查障碍物是否挡住路径
bool MyHybridAStarPlanner::isCachedPathBlocked(const nav_msgs::msg::Path &path) const {

    for (const auto &pose : path.poses) {

        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
            return true;

        const unsigned char cost = costmap_->getCost(mx, my);
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
            cost != nav2_costmap_2d::NO_INFORMATION)
            return true;
    }

    return false;
}

nav_msgs::msg::Path MyHybridAStarPlanner::pruneCachedPath(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::PoseStamped &cur_pose
) const {

    nav_msgs::msg::Path pruned;
    pruned.header = path.header;

    if (path.poses.empty())
        return pruned;

    geometry_msgs::msg::PoseStamped current = cur_pose;
    current.header = path.header;
    pruned.poses.push_back(current);

    const int prune_index = findBestPruneIndex(path, cur_pose);
    for (int i = prune_index; i < static_cast<int>(path.poses.size()); ++ i)
        pruned.poses.push_back(path.poses[i]);

    if (pruned.poses.size() == 1)
        pruned.poses.push_back(path.poses.back());

    return pruned;
}

int MyHybridAStarPlanner::findBestPruneIndex(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::PoseStamped &cur_pose
) const {

    if (path.poses.size() < 2)
        return 0;

    const double ox = cur_pose.pose.position.x;
    const double oy = cur_pose.pose.position.y;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    int best_index = 0;

    for (std::size_t i = 0; i + 1 < path.poses.size(); ++ i) {

        const auto &p1 = path.poses[i].pose.position;
        const auto &p2 = path.poses[i + 1].pose.position;
        const double vx = p2.x - p1.x;
        const double vy = p2.y - p1.y;
        const double segment_len_sq = vx * vx + vy * vy;
        if (segment_len_sq <= 1e-9)
            continue;

        const double t = ((ox - p1.x) * vx + (oy - p1.y) * vy) / segment_len_sq;
        if (t < 0.0 || t > 1.0)
            continue;

        const double proj_x = p1.x + t * vx;
        const double proj_y = p1.y + t * vy;
        const double dx = ox - proj_x;
        const double dy = oy - proj_y;
        const double distance_sq = dx * dx + dy * dy;

        if (distance_sq < best_distance_sq) {

            best_distance_sq = distance_sq;
            best_index = static_cast<int>(i + 1);
        }
    }

    if (std::isfinite(best_distance_sq))
        return std::min(best_index, static_cast<int>(path.poses.size()) - 1);

    double nearest_distance_sq = std::numeric_limits<double>::infinity();
    int nearest_index = 0;
    for (std::size_t i = 0; i < path.poses.size(); ++ i) {

        const double dx = ox - path.poses[i].pose.position.x;
        const double dy = oy - path.poses[i].pose.position.y;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < nearest_distance_sq) {

            nearest_distance_sq = distance_sq;
            nearest_index = static_cast<int>(i);
        }
    }

    if (std::sqrt(nearest_distance_sq) <= params_.path_prune_distance)
        return std::min(nearest_index + 1, static_cast<int>(path.poses.size()) - 1);

    return nearest_index;
}

double MyHybridAStarPlanner::pointDistance2D(
    const geometry_msgs::msg::PoseStamped &a,
    const geometry_msgs::msg::PoseStamped &b
) const {

    return std::hypot(
        a.pose.position.x - b.pose.position.x,
        a.pose.position.y - b.pose.position.y);
}

void MyHybridAStarPlanner::publishReplanEvent(
    uint8_t reason,
    const geometry_msgs::msg::PoseStamped &goal,
    const builtin_interfaces::msg::Time &candidate_path_stamp
) {

    if (!replan_event_pub_ || !node_)
        return;

    rm_interfaces::msg::ReplanEvent event;
    event.header.stamp = node_->now();
    event.header.frame_id = global_frame_;
    event.need_replan = true;
    event.event_id = ++replan_event_id_;
    event.reason = reason;
    event.candidate_path_stamp = candidate_path_stamp;
    event.goal = goal;
    replan_event_pub_->publish(event);

    RCLCPP_INFO(
        node_->get_logger(),
        "Published replan event id=%lu reason=%u",
        static_cast<unsigned long>(event.event_id),
        event.reason);
}

/**
 * @brief 路径回溯重建
 * @param nodes 路径点
 * @param goal_index 目标点索引，用于使用其父节点回溯重建路径
 * @param start 起始点
 * @param goal 目标点
 * @return nav_msgs::msg::Path 一条路径
 */
nav_msgs::msg::Path MyHybridAStarPlanner::reconstructPath(
    const std::vector<HybridNode> &nodes,
    int goal_index,
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal
) const {

    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();

    std::vector<int> chain_indices;
    for (int index = goal_index; index >= 0; index = nodes[index].parent_index)
        chain_indices.push_back(index);
    std::reverse(chain_indices.begin(), chain_indices.end());

    geometry_msgs::msg::PoseStamped start_pose = start;
    start_pose.header = path.header;
    path.poses.push_back(start_pose);

    for (std::size_t i = 1; i < chain_indices.size(); ++ i) {

        const HybridNode &parent_node = nodes[chain_indices[i - 1]];
        const HybridNode &child_node = nodes[chain_indices[i]];
        const MotionPrimitive *primitive = findMotionPrimitiveById(child_node.parent_primitive_id);
        if (primitive == nullptr)
            continue;

        PlannerPose pose = parent_node.pose;
        for (std::size_t step = 0; step < primitive->samples.size(); ++ step) {

            pose = integrateMecanumMotion(
                pose,
                primitive->v_x,
                primitive->v_y,
                primitive->omega,
                params_.step_time
            );

            geometry_msgs::msg::PoseStamped sampled_pose;
            sampled_pose.header = path.header;
            sampled_pose.pose.position.x = pose.x;
            sampled_pose.pose.position.y = pose.y;
            sampled_pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, pose.yaw);
            sampled_pose.pose.orientation = tf2::toMsg(q);
            path.poses.push_back(sampled_pose);
        }
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header = path.header;
    path.poses.push_back(goal_pose);

    return path;
}


void MyHybridAStarPlanner::buildMotionPrimitives() {

    motion_primitives_.clear();
    int primitive_id = 0;
    const double vx = params_.max_vel_x;
    const double vy = params_.max_vel_y;
    const double omega = params_.max_vel_theta;

    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, vx, 0.0, 0.0, MotionDirection::FORWARD));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, vx, vy, 0.0, MotionDirection::FORWARD));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, vx, -vy, 0.0, MotionDirection::FORWARD));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, vy, 0.0, MotionDirection::LATERAL_LEFT));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, -vy, 0.0, MotionDirection::LATERAL_RIGHT));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, vx, 0.0, omega, MotionDirection::FORWARD));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, vx, 0.0, -omega, MotionDirection::FORWARD));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, vy, omega, MotionDirection::LATERAL_LEFT));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, vy, -omega, MotionDirection::LATERAL_LEFT));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, -vy, omega, MotionDirection::LATERAL_RIGHT));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, -vy, -omega, MotionDirection::LATERAL_RIGHT));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, 0.0, omega, MotionDirection::ROTATE));
    motion_primitives_.push_back(makePrimitive(
        primitive_id ++, 0.0, 0.0, -omega, MotionDirection::ROTATE));

    if (params_.allow_reverse) {

        motion_primitives_.push_back(makePrimitive(
            primitive_id ++, -vx, 0.0, 0.0, MotionDirection::REVERSE));
        motion_primitives_.push_back(makePrimitive(
            primitive_id ++, -vx, vy, 0.0, MotionDirection::REVERSE));
        motion_primitives_.push_back(makePrimitive(
            primitive_id ++, -vx, -vy, 0.0, MotionDirection::REVERSE));
        motion_primitives_.push_back(makePrimitive(
            primitive_id ++, -vx, 0.0, omega, MotionDirection::REVERSE));
        motion_primitives_.push_back(makePrimitive(
            primitive_id ++, -vx, 0.0, -omega, MotionDirection::REVERSE));
    }
}

MotionPrimitive MyHybridAStarPlanner::makePrimitive(
    int id,
    double v_x,
    double v_y,
    double omega,
    MotionDirection direction
) const {

    MotionPrimitive primitive;
    primitive.id = id;
    primitive.v_x = v_x;
    primitive.v_y = v_y;
    primitive.omega = omega;
    primitive.duration = params_.primitive_duration;
    primitive.direction = direction;
    primitive.travel_cost =
        params_.primitive_duration * std::hypot(v_x, v_y) +
        0.1 * params_.primitive_duration * std::abs(omega); // 执行代价

    PlannerPose pose;
    const int steps = std::max(
        1,
        static_cast<int>(std::ceil(params_.primitive_duration / params_.step_time)));
    for (int step = 1; step <= steps; ++ step) {

        pose = integrateMecanumMotion(pose, v_x, v_y, omega, params_.step_time);
        MotionSample sample;
        sample.pose = pose;
        sample.time_from_start = step * params_.step_time;
        primitive.samples.push_back(sample);
    }

    return primitive;
}

PlannerPose MyHybridAStarPlanner::integrateMecanumMotion(
    const PlannerPose &start_pose,
    double v_x,
    double v_y,
    double omega,
    double dt
) const {

    const double cos_yaw = std::cos(start_pose.yaw);
    const double sin_yaw = std::sin(start_pose.yaw);

    PlannerPose next_pose;
    next_pose.x = start_pose.x + (v_x * cos_yaw - v_y * sin_yaw) * dt;
    next_pose.y = start_pose.y + (v_x * sin_yaw + v_y * cos_yaw) * dt;
    next_pose.yaw = normalizeAngle(start_pose.yaw + omega * dt);
    return next_pose;
}

/**
 * @brief 离散化连续位姿为格子位姿
 */
StateKey MyHybridAStarPlanner::discretizeState(const PlannerPose &pose) const {

    StateKey key;
    const double resolution = costmap_ ? costmap_->getResolution() : params_.xy_resolution;
    const double origin_x = costmap_ ? costmap_->getOriginX() : 0.0;
    const double origin_y = costmap_ ? costmap_->getOriginY() : 0.0;

    // Align the search key with the underlying costmap grid instead of assuming
    // the global map origin is always at (0, 0).
    key.mx = static_cast<int>(std::floor((pose.x - origin_x) / resolution));
    key.my = static_cast<int>(std::floor((pose.y - origin_y) / resolution));

    const double normalized_yaw = normalizeAngle(pose.yaw);
    const double ratio = (normalized_yaw + M_PI) / (2.0 * M_PI);
    key.theta_id = static_cast<int>(std::floor(ratio * params_.yaw_bin_count)) % params_.yaw_bin_count;
    return key;
}

// 初始化 h_2D 做一次Dijkstra算法
bool MyHybridAStarPlanner::computeGridHeuristic(
    unsigned int goal_mx, unsigned int goal_my
) {

    const unsigned int width = costmap_->getSizeInCellsX();
    const unsigned int height = costmap_->getSizeInCellsY();
    const unsigned int map_size = width * height;

    heuristic_grid_.assign(map_size, std::numeric_limits<double>::infinity());
    heuristic_ready_ = false;

    using GridQueueNode = std::pair<double, unsigned int>;
    std::priority_queue<GridQueueNode,
        std::vector<GridQueueNode>,
        std::greater<GridQueueNode>> open_list;

    auto toIndex = [width](unsigned int mx, unsigned int my) {
        return my * width + mx;
    };

    const unsigned int goal_idx = toIndex(goal_mx, goal_my);
    heuristic_grid_[goal_idx] = 0.0;
    open_list.push({0.0, goal_idx});

    static const int kNeighbors[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    while (!open_list.empty()) {

        const double cur_cost = open_list.top().first;
        const unsigned int cur_idx = open_list.top().second;
        open_list.pop();

        if (cur_cost > heuristic_grid_[cur_idx]) continue;

        const unsigned int
            cx = cur_idx % width,
            cy = cur_idx / width;

        for (const auto &offset : kNeighbors) {

            const int
                nx = static_cast<int>(cx) + offset[0],
                ny = static_cast<int>(cy) + offset[1];

            if (nx < 0 || ny < 0
                || nx >= static_cast<int>(width)
                || ny >= static_cast<int>(height)
            ) continue;

            if (!isCellTraversable(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) continue;

            const unsigned char cost =
                costmap_->getCost(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
            const double step_cost = (offset[0] == 0 || offset[1] == 0) ? 1.0 : std::sqrt(2.0);

            double obstacle_cost = 0.0;
            if (cost == nav2_costmap_2d::NO_INFORMATION)
                obstacle_cost = params_.unknown_cost;
            else {

                obstacle_cost = params_.obstacle_cost_weight *
                    static_cast<double>(cost) /
                    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
            }

            const double tentative_cost = cur_cost + step_cost + obstacle_cost;
            const unsigned int next_idx =
                toIndex(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));

            if (tentative_cost < heuristic_grid_[next_idx]) {

                heuristic_grid_[next_idx] = tentative_cost;
                open_list.push({tentative_cost, next_idx});
            }
        }
    }
    heuristic_ready_ = true;
    return true;
}

// 构造总启发式代价
double MyHybridAStarPlanner::computeNodeHeuristic(
    const PlannerPose &pose,
    const PlannerPose &goal
) const {

    unsigned int mx = 0;
    unsigned int my = 0;
    double h_grid = std::numeric_limits<double>::infinity();
    if (costmap_ && costmap_->worldToMap(pose.x, pose.y, mx, my))
        h_grid = params_.heuristic_grid_weight * getGridHeuristic(mx, my);

    const double h_yaw = params_.heuristic_yaw_weight *
        std::abs(normalizeAngle(goal.yaw - pose.yaw));
    const double h_rs = params_.rs_weight *
        std::hypot(goal.x - pose.x, goal.y - pose.y);

    return h_grid + h_yaw + h_rs;
}

const MotionPrimitive * MyHybridAStarPlanner::findMotionPrimitiveById(int primitive_id) const {

    for (const auto &primitive : motion_primitives_) {

        if (primitive.id == primitive_id)
            return &primitive;
    }

    return nullptr;
}

bool MyHybridAStarPlanner::simulatePrimitive(
    const HybridNode &current,
    const MotionPrimitive &primitive,
    PlannerPose &end_pose,
    std::vector<PlannerPose> &samples,
    double &transition_cost
) const {

    samples.clear();
    transition_cost = primitive.travel_cost;

    PlannerPose pose = current.pose;
    for (std::size_t step = 0; step < primitive.samples.size(); ++step) {

        pose = integrateMecanumMotion(
            pose,
            primitive.v_x,
            primitive.v_y,
            primitive.omega,
            params_.step_time);

        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap_->worldToMap(pose.x, pose.y, mx, my) || !isCellTraversable(mx, my))
            return false;

        const unsigned char cost = costmap_->getCost(mx, my);
        if (cost == nav2_costmap_2d::NO_INFORMATION) {
            transition_cost += params_.unknown_cost;
        } else {
            transition_cost += params_.obstacle_cost_weight *
                static_cast<double>(cost) /
                static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
        }

        samples.push_back(pose);
    }

    if (samples.empty())
        return false;

    end_pose = samples.back();
    return true;
}

bool MyHybridAStarPlanner::isCellTraversable(
    unsigned int mx,
    unsigned int my
) const {

    const unsigned char cost = costmap_->getCost(mx, my);
    return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
           cost == nav2_costmap_2d::NO_INFORMATION;
}

/**
 * @brief 利用A*跑得先验地图获取某点代价
 */
double MyHybridAStarPlanner::getGridHeuristic(
    unsigned int mx,
    unsigned int my
) const {

    if (!heuristic_ready_)
        return std::numeric_limits<double>::infinity();

    const unsigned int width = costmap_->getSizeInCellsX();
    const unsigned int height = costmap_->getSizeInCellsY();
    if (mx >= width || my >= height)
        return std::numeric_limits<double>::infinity();

    return heuristic_grid_[my * width + mx];
}

bool MyHybridAStarPlanner::validatePose(
    const geometry_msgs::msg::PoseStamped &pose,
    unsigned int &mx,
    unsigned int &my,
    const char *label) const {

    if (!pose.header.frame_id.empty() && pose.header.frame_id != global_frame_) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "%s pose frame '%s' does not match planner global frame '%s'",
            label,
            pose.header.frame_id.c_str(),
            global_frame_.c_str());
        return false;
    }

    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
        RCLCPP_ERROR(node_->get_logger(), "%s pose is outside costmap bounds", label);
        return false;
    }

    return true;
}

double MyHybridAStarPlanner::poseToYaw(const geometry_msgs::msg::PoseStamped &pose) const {

    return tf2::getYaw(pose.pose.orientation);
}

double MyHybridAStarPlanner::normalizeAngle(double angle) const {

    return std::atan2(std::sin(angle), std::cos(angle));
}

bool MyHybridAStarPlanner::isGoalReached(
    const PlannerPose &current,
    const PlannerPose &goal) const {

    const double position_error = std::hypot(goal.x - current.x, goal.y - current.y);
    const double yaw_error = std::abs(normalizeAngle(goal.yaw - current.yaw));
    return position_error <= params_.goal_tolerance_xy &&
           yaw_error <= params_.goal_tolerance_yaw;
}

} // namespace my_hybrid_astar_planner

PLUGINLIB_EXPORT_CLASS(my_hybrid_astar_planner::MyHybridAStarPlanner, nav2_core::GlobalPlanner)
