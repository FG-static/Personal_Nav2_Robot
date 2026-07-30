#include "my_hybrid_astar_planner/hybrid_astar_planner.hpp"
#include "my_planning_metrics/path_metrics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;

namespace my_hybrid_astar_planner {
namespace {

std_msgs::msg::ColorRGBA makeCostColor(double normalized_cost, double alpha) {

    const double value = std::clamp(normalized_cost, 0.0, 1.0);
    std_msgs::msg::ColorRGBA color;
    color.a = static_cast<float>(alpha);

    if (value < 0.25) {

        color.r = 0.0F;
        color.g = static_cast<float>(4.0 * value);
        color.b = 1.0F;
    } else if (value < 0.5) {

        color.r = 0.0F;
        color.g = 1.0F;
        color.b = static_cast<float>(2.0 - 4.0 * value);
    } else if (value < 0.75) {

        color.r = static_cast<float>(4.0 * value - 2.0);
        color.g = 1.0F;
        color.b = 0.0F;
    } else {

        color.r = 1.0F;
        color.g = static_cast<float>(4.0 - 4.0 * value);
        color.b = 0.0F;
    }

    return color;
}

} // namespace

// 哈希函数 用于快速查找已扩展节点索引
std::size_t StateKeyHasher::operator()(const StateKey &key) const {

    std::size_t seed = static_cast<std::size_t>(key.mx);
    seed = seed * 73856093U ^ static_cast<std::size_t>(key.my);
    seed = seed * 19349663U ^ static_cast<std::size_t>(key.theta_id);
    seed = seed * 83492791U ^ static_cast<std::size_t>(key.direction_id);
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
    auto declare_string_param =
        [&](const std::string &param_name, const std::string &default_value, std::string &target) {
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
        "heuristic_goal_dist_weight",
        params_.heuristic_goal_dist_weight,
        params_.heuristic_goal_dist_weight);
    declare_double_param(
        "path_tangent_change_weight",
        params_.path_tangent_change_weight,
        params_.path_tangent_change_weight);
    declare_double_param(
        "primitive_switch_weight",
        params_.primitive_switch_weight,
        params_.primitive_switch_weight);
    declare_double_param(
        "direction_switch_weight",
        params_.direction_switch_weight,
        params_.direction_switch_weight);
    declare_double_param(
        "velocity_direction_change_weight",
        params_.velocity_direction_change_weight,
        params_.velocity_direction_change_weight);
    declare_double_param(
        "primitive_omega_change_weight",
        params_.primitive_omega_change_weight,
        params_.primitive_omega_change_weight);
    declare_double_param(
        "goal_progress_weight",
        params_.goal_progress_weight,
        params_.goal_progress_weight);
    declare_double_param(
        "goal_direction_weight",
        params_.goal_direction_weight,
        params_.goal_direction_weight);
    declare_double_param(
        "angular_motion_weight",
        params_.angular_motion_weight,
        params_.angular_motion_weight);
    declare_int_param(
        "primitive_translation_angle_count",
        params_.primitive_translation_angle_count,
        params_.primitive_translation_angle_count);
    declare_int_param(
        "primitive_omega_sample_count",
        params_.primitive_omega_sample_count,
        params_.primitive_omega_sample_count);
    declare_double_param(
        "primitive_speed_ratio",
        params_.primitive_speed_ratio,
        params_.primitive_speed_ratio);
    declare_bool_param(
        "include_pure_rotation_primitives",
        params_.include_pure_rotation_primitives,
        params_.include_pure_rotation_primitives);
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
    declare_bool_param(
        "visualize_primitive_costs",
        params_.visualize_primitive_costs,
        params_.visualize_primitive_costs);
    declare_string_param(
        "primitive_cost_frame",
        params_.primitive_cost_frame,
        params_.primitive_cost_frame);
    declare_bool_param(
        "primitive_cost_show_text",
        params_.primitive_cost_show_text,
        params_.primitive_cost_show_text);
    declare_double_param(
        "primitive_cost_marker_z",
        params_.primitive_cost_marker_z,
        params_.primitive_cost_marker_z);
    declare_double_param(
        "primitive_cost_publish_rate",
        params_.primitive_cost_publish_rate,
        params_.primitive_cost_publish_rate);

    primitive_cost_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "trajectory_generation/primitive_costs",
        rclcpp::QoS(1).reliable().transient_local());
    if (params_.visualize_primitive_costs && params_.primitive_cost_publish_rate > 0.0) {

        const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / params_.primitive_cost_publish_rate));
        primitive_cost_timer_ = node_->create_wall_timer(
            timer_period,
            std::bind(&MyHybridAStarPlanner::updatePrimitiveCostVisualization, this));
        primitive_cost_timer_->cancel();
    }

    buildMotionPrimitives();

    RCLCPP_INFO(
        node_->get_logger(),
        "Mecanum Hybrid A* planner configured: yaw_bins=%d primitives=%zu "
        "max_vel=(%.2f, %.2f, %.2f) max_iter=%d max_time=%.2fs",
        params_.yaw_bin_count,
        motion_primitives_.size(),
        params_.max_vel_x,
        params_.max_vel_y,
        params_.max_vel_theta,
        params_.max_iterations,
        params_.max_planning_time);
    if (params_.visualize_primitive_costs) {

        RCLCPP_INFO(
            node_->get_logger(),
            "Primitive cost visualization enabled on /trajectory_generation/primitive_costs "
            "in frame '%s' at %.1f Hz",
            params_.primitive_cost_frame.c_str(),
            params_.primitive_cost_publish_rate);
    }
}

void MyHybridAStarPlanner::activate() {

    if (primitive_cost_timer_)
        primitive_cost_timer_->reset();
    RCLCPP_INFO(node_->get_logger(), "Mecanum Hybrid A* planner activated");
}

void MyHybridAStarPlanner::deactivate() {

    if (primitive_cost_timer_)
        primitive_cost_timer_->cancel();
    RCLCPP_INFO(node_->get_logger(), "Mecanum Hybrid A* planner deactivated");
}

void MyHybridAStarPlanner::cleanup() {

    if (primitive_cost_timer_)
        primitive_cost_timer_->cancel();
    std::lock_guard<std::mutex> lock(planning_mutex_);
    primitive_cost_timer_.reset();
    motion_primitives_.clear();
    primitive_cost_pub_.reset();
    has_primitive_cost_goal_ = false;
    costmap_ = nullptr;
    RCLCPP_INFO(node_->get_logger(), "Mecanum Hybrid A* planner cleaned up");
}

nav_msgs::msg::Path MyHybridAStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    std::function<bool()> cancel_checker) {

    std::lock_guard<std::mutex> lock(planning_mutex_);
    if (!node_ || !costmap_)
        throw nav2_core::PlannerException("Planner is not configured");
    has_primitive_cost_goal_ = false;

    const auto total_start_time = std::chrono::steady_clock::now();
    int expanded_iterations = 0;
    std::size_t generated_nodes = 0;
    std::size_t open_peak = 0;
    last_grid_expanded_cells_ = 0;
    auto logMetrics =
        [&](
            bool success,
            bool reused,
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
            const std::size_t hybrid_expanded_nodes =
                static_cast<std::size_t>(std::max(0, expanded_iterations));

            RCLCPP_INFO(
                node_->get_logger(),
                "FRONTEND_METRICS algorithm=HybridAStar success=%s reused=%s "
                "grid_expanded_cells=%zu hybrid_expanded_nodes=%zu "
                "expanded_nodes=%zu generated_nodes=%zu open_peak=%zu "
                "front_end_ms=%.3f planner_total_ms=%.3f metrics_eval_ms=%.3f "
                "path_points=%zu path_length_m=%.3f max_curvature_1pm=%.3f "
                "min_lethal_obstacle_distance_m=%.3f",
                success ? "true" : "false",
                reused ? "true" : "false",
                last_grid_expanded_cells_,
                hybrid_expanded_nodes,
                last_grid_expanded_cells_ + hybrid_expanded_nodes,
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

    if (cancel_checker && cancel_checker()) {

        RCLCPP_WARN(node_->get_logger(), "Hybrid A* planning cancelled before search start");
        logMetrics(false, false, 0.0, global_path);
        return global_path;
    }

    unsigned int mx_start = 0;
    unsigned int my_start = 0;
    unsigned int mx_goal = 0;
    unsigned int my_goal = 0;
    if (!validatePose(start, mx_start, my_start, "start") ||
        !validatePose(goal, mx_goal, my_goal, "goal")) {
        logMetrics(false, false, 0.0, global_path);
        return global_path;
    }

    const PlannerPose start_pose{start.pose.position.x, start.pose.position.y, poseToYaw(start)};
    const PlannerPose goal_pose{goal.pose.position.x, goal.pose.position.y, poseToYaw(goal)};

    if (isGoalReached(start_pose, goal_pose)) {

        global_path.poses.push_back(start);
        global_path.poses.push_back(goal);
        logMetrics(true, false, 0.0, global_path);
        return global_path;
    }
    primitive_cost_goal_ = goal_pose;
    has_primitive_cost_goal_ = true;

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
                logMetrics(true, true, 0.0, reused);
                return reused;
            }
        }
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Hybrid A* skeleton ready. Search phase will use %zu mecanum motion primitives.",
        motion_primitives_.size());

    const auto front_end_start_time = std::chrono::steady_clock::now();
    if (!computeGridHeuristic(mx_goal, my_goal)) {

        RCLCPP_ERROR(node_->get_logger(), "Failed to build grid heuristic");
        const double front_end_search_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - front_end_start_time).count();
        logMetrics(false, false, front_end_search_ms, global_path);
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
    start_node.key.direction_id = static_cast<int>(MotionDirection::TRANSLATE);
    start_node.g = 0.0;
    updateHeuristicTerms(start_node, start_pose, goal_pose);
    if (!std::isfinite(start_node.f)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Hybrid A* start heuristic is not finite. Search may fail quickly.");
    }

    nodes.push_back(start_node);
    node_lookup[start_node.key] = 0;
    generated_nodes = 1;
    open_list.push({
        start_node.f,
        start_node.h_grid + start_node.h_yaw + start_node.h_goal_dist,
        start_node.g,
        0});
    open_peak = 1;

    const rclcpp::Time search_start_time = node_->now();
    std::vector<PrimitiveCostDebug> root_primitive_costs;

    // 搜索
    while (!open_list.empty()) {

        if (cancel_checker && cancel_checker()) {

            RCLCPP_WARN(node_->get_logger(), "Planning cancelled");
            const double front_end_search_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - front_end_start_time).count();
            logMetrics(false, false, front_end_search_ms, global_path);
            return nav_msgs::msg::Path{};
        }

        if (expanded_iterations >= max_iterations) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Hybrid A* aborted after reaching max_iterations=%d, nodes=%zu, open=%zu",
                max_iterations,
                nodes.size(),
                open_list.size());
            const double front_end_search_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - front_end_start_time).count();
            logMetrics(false, false, front_end_search_ms, global_path);
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
            const double front_end_search_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - front_end_start_time).count();
            logMetrics(false, false, front_end_search_ms, global_path);
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
            const double front_end_search_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - front_end_start_time).count();
            logMetrics(false, false, front_end_search_ms, global_path);
            return global_path;
        }

        if (nodes[cur_idx].closed)
            continue;

        nodes[cur_idx].closed = true;
        ++expanded_iterations;

        // Copy the expanded node locally so later nodes.push_back() reallocations
        // cannot invalidate references used during primitive expansion.
        const HybridNode current_node = nodes[cur_idx];

        if (isGoalReached(current_node.pose, goal_pose)) {

            nav_msgs::msg::Path planned_path = reconstructPath(nodes, cur_idx, start, goal);
            if (!root_primitive_costs.empty()) {

                publishPrimitiveCostVisualization(
                    root_primitive_costs,
                    findFirstPrimitiveId(nodes, cur_idx));
            }
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
            const double front_end_search_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - front_end_start_time).count();
            logMetrics(true, false, front_end_search_ms, planned_path);
            return planned_path;
        }

        for (const auto &primitive : motion_primitives_) {

            const bool capture_root_cost =
                params_.visualize_primitive_costs && cur_idx == 0;
            PrimitiveCostDebug debug_cost;
            debug_cost.primitive_id = primitive.id;
            debug_cost.travel_cost = primitive.travel_cost;
            std::size_t debug_cost_index = std::numeric_limits<std::size_t>::max();

            PlannerPose next_pose;
            std::vector<PlannerPose> sampled_poses;
            double transition_cost = 0.0;
            if (!simulatePrimitive(current_node, primitive, next_pose, sampled_poses, transition_cost)) {

                if (capture_root_cost)
                    root_primitive_costs.push_back(debug_cost);
                continue;
            }

            StateKey next_key = discretizeState(next_pose);
            next_key.direction_id = static_cast<int>(primitive.direction);

            unsigned int end_mx = 0;
            unsigned int end_my = 0;
            if (!costmap_->worldToMap(next_pose.x, next_pose.y, end_mx, end_my)) {

                if (capture_root_cost)
                    root_primitive_costs.push_back(debug_cost);
                continue;
            }

            const double switch_cost =
                computePrimitiveSwitchCost(current_node, primitive);
            const double goal_directed_cost =
                computeGoalDirectedCost(current_node.pose, next_pose, goal_pose);
            double tangent_change_cost = 0.0;
            if (current_node.parent_index >= 0) {

                tangent_change_cost = computePathTangentChangeCost(
                    nodes[current_node.parent_index].pose,
                    current_node.pose,
                    next_pose);
            }

            const double delta_g =
                transition_cost + switch_cost + goal_directed_cost + tangent_change_cost;
            const double new_g = current_node.g + delta_g;

            if (capture_root_cost) {

                HybridNode scored_node;
                scored_node.g = new_g;
                updateHeuristicTerms(scored_node, next_pose, goal_pose);

                debug_cost.feasible = true;
                debug_cost.obstacle_cost =
                    std::max(0.0, transition_cost - primitive.travel_cost);
                debug_cost.switch_cost = switch_cost;
                debug_cost.goal_directed_cost = goal_directed_cost;
                debug_cost.tangent_change_cost = tangent_change_cost;
                debug_cost.delta_g = delta_g;
                debug_cost.h_grid = scored_node.h_grid;
                debug_cost.h_yaw = scored_node.h_yaw;
                debug_cost.h_goal_dist = scored_node.h_goal_dist;
                debug_cost.score = scored_node.f;
                debug_cost_index = root_primitive_costs.size();
                root_primitive_costs.push_back(debug_cost);
            }

            // push node
            auto it = node_lookup.find(next_key);
            if (it == node_lookup.end()) { // 未拓展过的

                if (nodes.size() >= max_node_count) {
                    if (capture_root_cost)
                        publishPrimitiveCostVisualization(root_primitive_costs, -1);
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Hybrid A* aborted after reaching max node count=%zu, expanded=%d, open=%zu",
                        max_node_count,
                        expanded_iterations,
                        open_list.size());
                    const double front_end_search_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - front_end_start_time).count();
                    logMetrics(false, false, front_end_search_ms, global_path);
                    return global_path;
                }

                HybridNode next_node;
                next_node.key = next_key;
                next_node.pose = next_pose;
                next_node.g = new_g;
                next_node.parent_index = cur_idx;
                next_node.parent_primitive_id = primitive.id;
                updateHeuristicTerms(next_node, next_pose, goal_pose);

                const int new_index = static_cast<int>(nodes.size());
                nodes.push_back(next_node);
                ++generated_nodes;
                node_lookup[next_key] = new_index;
                if (debug_cost_index < root_primitive_costs.size())
                    root_primitive_costs[debug_cost_index].accepted = true;
                open_list.push({
                    next_node.f,
                    next_node.h_grid + next_node.h_yaw + next_node.h_goal_dist,
                    next_node.g,
                    new_index});
                open_peak = std::max(open_peak, open_list.size());
            } else { // 非同一路径拓展过的

                HybridNode &old_node = nodes[it->second];
                if (!old_node.closed && new_g < old_node.g) {

                    old_node.pose = next_pose;
                    old_node.g = new_g;
                    old_node.parent_index = cur_idx;
                    old_node.parent_primitive_id = primitive.id;
                    updateHeuristicTerms(old_node, next_pose, goal_pose);
                    if (debug_cost_index < root_primitive_costs.size())
                        root_primitive_costs[debug_cost_index].accepted = true;

                    open_list.push({
                        old_node.f,
                        old_node.h_grid + old_node.h_yaw + old_node.h_goal_dist,
                        old_node.g,
                        it->second});
                    open_peak = std::max(open_peak, open_list.size());
                }
            }
        }

        if (cur_idx == 0 && !root_primitive_costs.empty())
            publishPrimitiveCostVisualization(root_primitive_costs, -1);
    }

    RCLCPP_WARN(
        node_->get_logger(),
        "Hybrid A* failed to find a valid path after %.3fs, expanded=%d, nodes=%zu",
        (node_->now() - search_start_time).seconds(),
        expanded_iterations,
        nodes.size());
    const double front_end_search_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - front_end_start_time).count();
    logMetrics(false, false, front_end_search_ms, global_path);
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

int MyHybridAStarPlanner::findFirstPrimitiveId(
    const std::vector<HybridNode> &nodes,
    int goal_index
) const {

    if (goal_index < 0 || goal_index >= static_cast<int>(nodes.size()))
        return -1;

    int node_index = goal_index;
    for (std::size_t depth = 0; depth < nodes.size(); ++depth) {

        const int parent_index = nodes[node_index].parent_index;
        if (parent_index == 0)
            return nodes[node_index].parent_primitive_id;
        if (parent_index < 0 || parent_index >= static_cast<int>(nodes.size()))
            return -1;
        node_index = parent_index;
    }

    return -1;
}

std::vector<PrimitiveCostDebug> MyHybridAStarPlanner::evaluatePrimitiveCostsAtPose(
    const PlannerPose &current_pose,
    const PlannerPose &goal_pose
) const {

    std::vector<PrimitiveCostDebug> costs;
    costs.reserve(motion_primitives_.size());

    HybridNode current_node;
    current_node.pose = current_pose;
    current_node.key = discretizeState(current_pose);
    current_node.key.direction_id = static_cast<int>(MotionDirection::TRANSLATE);
    current_node.g = 0.0;

    std::unordered_map<StateKey, std::size_t, StateKeyHasher> best_by_state;
    best_by_state.reserve(motion_primitives_.size());

    for (const MotionPrimitive &primitive : motion_primitives_) {

        PrimitiveCostDebug debug_cost;
        debug_cost.primitive_id = primitive.id;
        debug_cost.travel_cost = primitive.travel_cost;

        PlannerPose next_pose;
        std::vector<PlannerPose> sampled_poses;
        double transition_cost = 0.0;
        if (!simulatePrimitive(
                current_node,
                primitive,
                next_pose,
                sampled_poses,
                transition_cost)) {

            costs.push_back(debug_cost);
            continue;
        }

        debug_cost.feasible = true;
        debug_cost.obstacle_cost =
            std::max(0.0, transition_cost - primitive.travel_cost);
        debug_cost.switch_cost = computePrimitiveSwitchCost(current_node, primitive);
        debug_cost.goal_directed_cost =
            computeGoalDirectedCost(current_pose, next_pose, goal_pose);
        debug_cost.delta_g =
            transition_cost + debug_cost.switch_cost + debug_cost.goal_directed_cost;

        HybridNode scored_node;
        scored_node.g = debug_cost.delta_g;
        updateHeuristicTerms(scored_node, next_pose, goal_pose);
        debug_cost.h_grid = scored_node.h_grid;
        debug_cost.h_yaw = scored_node.h_yaw;
        debug_cost.h_goal_dist = scored_node.h_goal_dist;
        debug_cost.score = scored_node.f;

        StateKey next_key = discretizeState(next_pose);
        next_key.direction_id = static_cast<int>(primitive.direction);
        const std::size_t current_cost_index = costs.size();
        if (!(next_key == current_node.key)) {

            auto best_it = best_by_state.find(next_key);
            if (best_it == best_by_state.end()) {

                debug_cost.accepted = true;
                best_by_state.emplace(next_key, current_cost_index);
            } else if (debug_cost.delta_g < costs[best_it->second].delta_g) {

                costs[best_it->second].accepted = false;
                debug_cost.accepted = true;
                best_it->second = current_cost_index;
            }
        }

        costs.push_back(debug_cost);
    }

    return costs;
}

void MyHybridAStarPlanner::updatePrimitiveCostVisualization() {

    std::unique_lock<std::mutex> lock(planning_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !node_ || !tf_ || !costmap_ ||
        !heuristic_ready_ || !has_primitive_cost_goal_) {

        return;
    }

    try {

        const geometry_msgs::msg::TransformStamped transform =
            tf_->lookupTransform(
                global_frame_,
                params_.primitive_cost_frame,
                tf2::TimePointZero);
        PlannerPose current_pose;
        current_pose.x = transform.transform.translation.x;
        current_pose.y = transform.transform.translation.y;
        current_pose.yaw = tf2::getYaw(transform.transform.rotation);

        publishPrimitiveCostVisualization(
            evaluatePrimitiveCostsAtPose(current_pose, primitive_cost_goal_),
            -1,
            true);
    } catch (const tf2::TransformException &ex) {

        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Cannot update primitive cost visualization: %s",
            ex.what());
    }
}

void MyHybridAStarPlanner::publishPrimitiveCostVisualization(
    const std::vector<PrimitiveCostDebug> &costs,
    int selected_primitive_id,
    bool realtime
) const {

    if (!params_.visualize_primitive_costs || !primitive_cost_pub_ || !node_ || costs.empty())
        return;

    visualization_msgs::msg::MarkerArray marker_array;
    const builtin_interfaces::msg::Time stamp;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = params_.primitive_cost_frame;
    delete_marker.header.stamp = stamp;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    double min_score = std::numeric_limits<double>::infinity();
    double max_score = -std::numeric_limits<double>::infinity();
    int local_best_primitive_id = -1;
    double local_best_score = std::numeric_limits<double>::infinity();
    for (const PrimitiveCostDebug &cost : costs) {

        if (!cost.feasible || !std::isfinite(cost.score))
            continue;

        min_score = std::min(min_score, cost.score);
        max_score = std::max(max_score, cost.score);
        if (cost.accepted && cost.score < local_best_score) {

            local_best_primitive_id = cost.primitive_id;
            local_best_score = cost.score;
        }
    }

    auto findCost = [&](int primitive_id) -> const PrimitiveCostDebug * {

        for (const PrimitiveCostDebug &cost : costs) {

            if (cost.primitive_id == primitive_id)
                return &cost;
        }
        return nullptr;
    };

    if (local_best_primitive_id >= 0) {

        const PrimitiveCostDebug *best_cost = findCost(local_best_primitive_id);
        if (best_cost == nullptr)
            local_best_primitive_id = -1;
    }

    const double score_range =
        std::isfinite(min_score) && std::isfinite(max_score) ? max_score - min_score : 0.0;
    auto costColor = [&](const PrimitiveCostDebug &cost, double alpha) {

        if (!cost.feasible) {

            std_msgs::msg::ColorRGBA color;
            color.r = 0.12F;
            color.g = 0.12F;
            color.b = 0.12F;
            color.a = static_cast<float>(alpha);
            return color;
        }
        if (!std::isfinite(cost.score)) {

            std_msgs::msg::ColorRGBA color;
            color.r = 0.45F;
            color.g = 0.0F;
            color.b = 0.55F;
            color.a = static_cast<float>(alpha);
            return color;
        }

        const double normalized =
            score_range > 1e-9 ? (cost.score - min_score) / score_range : 0.0;
        std_msgs::msg::ColorRGBA color = makeCostColor(normalized, alpha);
        if (!cost.accepted)
            color.a *= 0.35F;
        return color;
    };

    struct CostSector {

        double angle = 0.0;
        double radius = 0.0;
        const PrimitiveCostDebug *cost = nullptr;
    };

    std::vector<CostSector> sectors;
    for (const PrimitiveCostDebug &cost : costs) {

        const MotionPrimitive *primitive = findMotionPrimitiveById(cost.primitive_id);
        if (primitive == nullptr || primitive->samples.empty() ||
            std::hypot(primitive->v_x, primitive->v_y) <= 1e-6 ||
            std::abs(primitive->omega) > 1e-6) {

            continue;
        }

        const PlannerPose &end_pose = primitive->samples.back().pose;
        sectors.push_back({
            std::atan2(end_pose.y, end_pose.x),
            std::hypot(end_pose.x, end_pose.y),
            &cost});
    }
    std::sort(
        sectors.begin(),
        sectors.end(),
        [](const CostSector &lhs, const CostSector &rhs) {
            return lhs.angle < rhs.angle;
        });

    const double marker_z = params_.primitive_cost_marker_z;
    if (sectors.size() >= 2) {

        visualization_msgs::msg::Marker heatmap;
        heatmap.header.frame_id = params_.primitive_cost_frame;
        heatmap.header.stamp = stamp;
        heatmap.ns = "primitive_cost_heatmap";
        heatmap.id = 0;
        heatmap.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        heatmap.action = visualization_msgs::msg::Marker::ADD;
        heatmap.pose.orientation.w = 1.0;
        heatmap.scale.x = 1.0;
        heatmap.scale.y = 1.0;
        heatmap.scale.z = 1.0;
        heatmap.color.a = 0.45F;
        heatmap.frame_locked = true;

        const double two_pi = 2.0 * std::acos(-1.0);
        for (std::size_t i = 0; i < sectors.size(); ++i) {

            const CostSector &previous =
                sectors[(i + sectors.size() - 1) % sectors.size()];
            const CostSector &current = sectors[i];
            const CostSector &next = sectors[(i + 1) % sectors.size()];
            double previous_gap = std::fmod(current.angle - previous.angle + two_pi, two_pi);
            double next_gap = std::fmod(next.angle - current.angle + two_pi, two_pi);
            if (previous_gap <= 1e-9)
                previous_gap = two_pi;
            if (next_gap <= 1e-9)
                next_gap = two_pi;

            const double left_angle = current.angle - 0.5 * previous_gap;
            const double right_angle = current.angle + 0.5 * next_gap;
            const double left_radius = 0.5 * (current.radius + previous.radius);
            const double right_radius = 0.5 * (current.radius + next.radius);

            geometry_msgs::msg::Point origin;
            origin.z = marker_z;
            geometry_msgs::msg::Point left;
            left.x = left_radius * std::cos(left_angle);
            left.y = left_radius * std::sin(left_angle);
            left.z = marker_z;
            geometry_msgs::msg::Point right;
            right.x = right_radius * std::cos(right_angle);
            right.y = right_radius * std::sin(right_angle);
            right.z = marker_z;

            heatmap.points.push_back(origin);
            heatmap.points.push_back(left);
            heatmap.points.push_back(right);
            const std_msgs::msg::ColorRGBA color = costColor(*current.cost, 0.45);
            heatmap.colors.push_back(color);
            heatmap.colors.push_back(color);
            heatmap.colors.push_back(color);
        }
        marker_array.markers.push_back(heatmap);
    }

    const double costmap_resolution = costmap_ ? costmap_->getResolution() : 0.05;
    const double line_width = std::max(0.012, 0.3 * costmap_resolution);
    const double endpoint_scale = std::max(0.035, 0.75 * costmap_resolution);
    const double rotation_radius = std::max(
        0.08,
        0.35 * params_.primitive_duration *
            std::max(params_.max_vel_x, params_.max_vel_y));
    double max_reach = 0.0;

    for (const PrimitiveCostDebug &cost : costs) {

        const MotionPrimitive *primitive = findMotionPrimitiveById(cost.primitive_id);
        if (primitive == nullptr || primitive->samples.empty())
            continue;

        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = params_.primitive_cost_frame;
        path_marker.header.stamp = stamp;
        path_marker.ns = "primitive_cost_paths";
        path_marker.id = primitive->id;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x =
            primitive->id == selected_primitive_id ? 2.5 * line_width : line_width;
        path_marker.color = costColor(cost, 0.95);
        path_marker.frame_locked = true;

        const bool pure_rotation =
            std::hypot(primitive->v_x, primitive->v_y) <= 1e-6 &&
            std::abs(primitive->omega) > 1e-6;
        if (pure_rotation) {

            geometry_msgs::msg::Point point;
            point.x = rotation_radius;
            point.z = marker_z + 0.01;
            path_marker.points.push_back(point);
            for (const MotionSample &sample : primitive->samples) {

                point.x = rotation_radius * std::cos(primitive->omega * sample.time_from_start);
                point.y = rotation_radius * std::sin(primitive->omega * sample.time_from_start);
                path_marker.points.push_back(point);
            }
        } else {

            geometry_msgs::msg::Point point;
            point.z = marker_z + 0.01;
            path_marker.points.push_back(point);
            for (const MotionSample &sample : primitive->samples) {

                point.x = sample.pose.x;
                point.y = sample.pose.y;
                path_marker.points.push_back(point);
            }
        }

        const geometry_msgs::msg::Point endpoint = path_marker.points.back();
        max_reach = std::max(max_reach, std::hypot(endpoint.x, endpoint.y));
        marker_array.markers.push_back(path_marker);

        if (primitive->id == selected_primitive_id) {

            visualization_msgs::msg::Marker selected_marker = path_marker;
            selected_marker.ns = "selected_primitive";
            selected_marker.id = 0;
            selected_marker.scale.x = line_width;
            selected_marker.color.r = 1.0F;
            selected_marker.color.g = 1.0F;
            selected_marker.color.b = 1.0F;
            selected_marker.color.a = 1.0F;
            marker_array.markers.push_back(selected_marker);
        }

        visualization_msgs::msg::Marker endpoint_marker;
        endpoint_marker.header = path_marker.header;
        endpoint_marker.ns = "primitive_cost_endpoints";
        endpoint_marker.id = primitive->id;
        endpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
        endpoint_marker.action = visualization_msgs::msg::Marker::ADD;
        endpoint_marker.pose.position = endpoint;
        endpoint_marker.pose.orientation.w = 1.0;
        endpoint_marker.scale.x = endpoint_scale;
        endpoint_marker.scale.y = endpoint_scale;
        endpoint_marker.scale.z = endpoint_scale;
        endpoint_marker.color = costColor(cost, 1.0);
        endpoint_marker.frame_locked = true;
        marker_array.markers.push_back(endpoint_marker);

        if (params_.primitive_cost_show_text) {

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = path_marker.header;
            text_marker.ns = "primitive_cost_labels";
            text_marker.id = primitive->id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position = endpoint;
            const double endpoint_length = std::hypot(endpoint.x, endpoint.y);
            if (endpoint_length > 1e-6) {

                text_marker.pose.position.x *= 1.18;
                text_marker.pose.position.y *= 1.18;
            }
            text_marker.pose.position.z = marker_z + std::max(0.055, costmap_resolution);
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = std::max(0.025, 0.5 * costmap_resolution);
            text_marker.color.r = 1.0F;
            text_marker.color.g = 1.0F;
            text_marker.color.b = 1.0F;
            text_marker.color.a = 1.0F;
            text_marker.frame_locked = true;

            std::ostringstream label;
            if (primitive->id == selected_primitive_id)
                label << "PATH ";
            else if (primitive->id == local_best_primitive_id)
                label << "MIN ";
            label << "#" << primitive->id << " ";
            if (!cost.feasible)
                label << "blocked";
            else if (!std::isfinite(cost.score))
                label << "f=inf";
            else
                label << "f=" << std::fixed << std::setprecision(2) << cost.score;
            if (cost.feasible && !cost.accepted)
                label << " dup";
            text_marker.text = label.str();
            marker_array.markers.push_back(text_marker);
        }

        RCLCPP_DEBUG(
            node_->get_logger(),
            "Primitive #%d feasible=%s accepted=%s u=(%.3f, %.3f, %.3f) "
            "travel=%.3f obstacle=%.3f switch=%.3f goal=%.3f tangent=%.3f "
            "delta_g=%.3f h=(%.3f, %.3f, %.3f) f=%.3f",
            primitive->id,
            cost.feasible ? "true" : "false",
            cost.accepted ? "true" : "false",
            primitive->v_x,
            primitive->v_y,
            primitive->omega,
            cost.travel_cost,
            cost.obstacle_cost,
            cost.switch_cost,
            cost.goal_directed_cost,
            cost.tangent_change_cost,
            cost.delta_g,
            cost.h_grid,
            cost.h_yaw,
            cost.h_goal_dist,
            cost.score);
    }

    visualization_msgs::msg::Marker summary_marker;
    summary_marker.header.frame_id = params_.primitive_cost_frame;
    summary_marker.header.stamp = stamp;
    summary_marker.ns = "primitive_cost_summary";
    summary_marker.id = 0;
    summary_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    summary_marker.action = visualization_msgs::msg::Marker::ADD;
    summary_marker.pose.position.y = -max_reach - 0.08;
    summary_marker.pose.position.z = marker_z + 0.12;
    summary_marker.pose.orientation.w = 1.0;
    summary_marker.scale.z = std::max(0.035, 0.6 * costmap_resolution);
    summary_marker.color.r = 1.0F;
    summary_marker.color.g = 1.0F;
    summary_marker.color.b = 1.0F;
    summary_marker.color.a = 1.0F;
    summary_marker.frame_locked = true;

    std::ostringstream summary;
    if (realtime)
        summary << "LIVE ";
    summary << "f = delta_g + h | blue: low, red: high, gray: blocked";
    if (std::isfinite(min_score) && std::isfinite(max_score)) {

        summary << "\nrange=[" << std::fixed << std::setprecision(2)
                << min_score << ", " << max_score << "]";
    }
    if (selected_primitive_id >= 0)
        summary << " PATH=#" << selected_primitive_id;
    summary_marker.text = summary.str();
    marker_array.markers.push_back(summary_marker);

    primitive_cost_pub_->publish(marker_array);
}


void MyHybridAStarPlanner::buildMotionPrimitives() {

    motion_primitives_.clear();
    int primitive_id = 0;

    const int angle_count = std::max(1, params_.primitive_translation_angle_count);
    const int omega_sample_count = params_.primitive_omega_sample_count;
    const double speed_ratio = std::clamp(params_.primitive_speed_ratio, 0.0, 1.0);
    const double max_omega = params_.max_vel_theta;

    std::vector<double> omega_samples;
    if (omega_sample_count <= 1) {

        omega_samples.push_back(0.0);
    } else {

        if (omega_sample_count != 3) {

            RCLCPP_WARN(
                node_->get_logger(),
                "primitive_omega_sample_count=%d is not supported in stage 1, using {-max, 0, +max}",
                omega_sample_count);
        }
        omega_samples.push_back(-max_omega);
        omega_samples.push_back(0.0);
        omega_samples.push_back(max_omega);
    }

    const double two_pi = 2.0 * std::acos(-1.0);
    for (int angle_id = 0; angle_id < angle_count; ++ angle_id) {

        const double angle = two_pi * static_cast<double>(angle_id) /
            static_cast<double>(angle_count);
        const double v_x = speed_ratio * params_.max_vel_x * std::cos(angle);
        const double v_y = speed_ratio * params_.max_vel_y * std::sin(angle);

        if (std::hypot(v_x, v_y) <= 1e-6)
            continue;

        for (const double omega : omega_samples) {

            motion_primitives_.push_back(makePrimitive(
                primitive_id ++, v_x, v_y, omega, MotionDirection::TRANSLATE));
        }
    }

    if (params_.include_pure_rotation_primitives && std::abs(max_omega) > 1e-6) {

        motion_primitives_.push_back(makePrimitive(
            primitive_id ++, 0.0, 0.0, max_omega, MotionDirection::ROTATE));
        motion_primitives_.push_back(makePrimitive(
            primitive_id ++, 0.0, 0.0, -max_omega, MotionDirection::ROTATE));
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
        params_.angular_motion_weight * params_.primitive_duration * std::abs(omega); // 执行代价

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
    last_grid_expanded_cells_ = 0;

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
        ++last_grid_expanded_cells_;

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

    HybridNode node;
    node.g = 0.0;
    updateHeuristicTerms(node, pose, goal);
    return node.h_grid + node.h_yaw + node.h_goal_dist;
}

void MyHybridAStarPlanner::updateHeuristicTerms(
    HybridNode &node,
    const PlannerPose &pose,
    const PlannerPose &goal
) const {

    unsigned int mx = 0;
    unsigned int my = 0;
    node.h_grid = std::numeric_limits<double>::infinity();
    if (costmap_ && costmap_->worldToMap(pose.x, pose.y, mx, my))
        node.h_grid = params_.heuristic_grid_weight * getGridHeuristic(mx, my);

    node.h_yaw = params_.heuristic_yaw_weight *
        std::abs(normalizeAngle(goal.yaw - pose.yaw));
    node.h_goal_dist = params_.heuristic_goal_dist_weight *
        std::hypot(goal.x - pose.x, goal.y - pose.y);
    node.f = node.g + node.h_grid + node.h_yaw + node.h_goal_dist;
}

const MotionPrimitive * MyHybridAStarPlanner::findMotionPrimitiveById(int primitive_id) const {

    for (const auto &primitive : motion_primitives_) {

        if (primitive.id == primitive_id)
            return &primitive;
    }

    return nullptr;
}

double MyHybridAStarPlanner::computePrimitiveSwitchCost(
    const HybridNode &current,
    const MotionPrimitive &next_primitive
) const {

    const MotionPrimitive *previous_primitive =
        findMotionPrimitiveById(current.parent_primitive_id);
    if (previous_primitive == nullptr)
        return 0.0;

    double switch_cost = 0.0;
    if (previous_primitive->id != next_primitive.id)
        switch_cost += params_.primitive_switch_weight;

    const double prev_speed =
        std::hypot(previous_primitive->v_x, previous_primitive->v_y);
    const double next_speed =
        std::hypot(next_primitive.v_x, next_primitive.v_y);
    const bool prev_translate = prev_speed > 1e-6;
    const bool next_translate = next_speed > 1e-6;

    if (prev_translate != next_translate)
        switch_cost += params_.direction_switch_weight;

    if (prev_translate && next_translate) {

        const double prev_angle =
            std::atan2(previous_primitive->v_y, previous_primitive->v_x);
        const double next_angle =
            std::atan2(next_primitive.v_y, next_primitive.v_x);
        switch_cost += params_.velocity_direction_change_weight *
            std::abs(normalizeAngle(next_angle - prev_angle));
    }

    switch_cost += params_.primitive_omega_change_weight * params_.primitive_duration *
        std::abs(next_primitive.omega - previous_primitive->omega);

    return switch_cost;
}

double MyHybridAStarPlanner::computeGoalDirectedCost(
    const PlannerPose &current,
    const PlannerPose &next,
    const PlannerPose &goal
) const {

    double cost = 0.0;
    const double current_goal_dist = std::hypot(goal.x - current.x, goal.y - current.y);
    const double next_goal_dist = std::hypot(goal.x - next.x, goal.y - next.y);
    if (next_goal_dist > current_goal_dist) {

        cost += params_.goal_progress_weight * (next_goal_dist - current_goal_dist);
    }

    const double step_x = next.x - current.x;
    const double step_y = next.y - current.y;
    const double step_dist = std::hypot(step_x, step_y);
    if (step_dist > 1e-6 && current_goal_dist > 1e-6) {

        const double move_yaw = std::atan2(step_y, step_x);
        const double goal_yaw = std::atan2(goal.y - current.y, goal.x - current.x);
        const double direction_error = std::abs(normalizeAngle(move_yaw - goal_yaw));
        cost += params_.goal_direction_weight * direction_error * step_dist;
    }

    return cost;
}

double MyHybridAStarPlanner::computePathTangentChangeCost(
    const PlannerPose &previous,
    const PlannerPose &current,
    const PlannerPose &next
) const {

    const double previous_dx = current.x - previous.x;
    const double previous_dy = current.y - previous.y;
    const double next_dx = next.x - current.x;
    const double next_dy = next.y - current.y;
    if (std::hypot(previous_dx, previous_dy) <= 1e-6 ||
        std::hypot(next_dx, next_dy) <= 1e-6) {

        return 0.0;
    }

    const double previous_yaw = std::atan2(previous_dy, previous_dx);
    const double next_yaw = std::atan2(next_dy, next_dx);
    return params_.path_tangent_change_weight *
        std::abs(normalizeAngle(next_yaw - previous_yaw));
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
