#include "my_teb_controller/teb_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "angles/angles.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

#include "my_teb_controller/teb_graph_optimizer.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace my_teb_controller {

PoseSE2::PoseSE2(double x_in, double y_in, double theta_in)
: x(x_in), y(y_in), theta(theta_in) {}

Eigen::Vector2d PoseSE2::position() const {

    return Eigen::Vector2d(x, y);
}

void PoseSE2::setTheta(double theta_in) {

    theta = thetaMod(theta_in);
}

double PoseSE2::thetaMod(double theta_in) const {

    return angles::normalize_angle(theta_in);
}

TimedPose::TimedPose(const PoseSE2 &pose_in, double dt_in)
: pose(pose_in), dt(dt_in) {}

MyTebController::MyTebController() = default;

MyTebController::~MyTebController() = default;

void MyTebController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

    node_ = parent;
    auto node = node_.lock();
    if (!node) {
        throw nav2_core::ControllerException("Failed to lock lifecycle node in MyTebController");
    }

    tf_ = tf;
    plugin_name_ = name;
    costmap_ros_ = costmap_ros;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    graph_optimizer_ = std::make_unique<TebGraphOptimizer>();

    auto declare_double_param = [&](const std::string &param_name, double default_value, double &target) {
        declare_parameter_if_not_declared(
            node, plugin_name_ + "." + param_name, rclcpp::ParameterValue(default_value));
        node->get_parameter(plugin_name_ + "." + param_name, target);
    };
    auto declare_int_param = [&](const std::string &param_name, int default_value, int &target) {
        declare_parameter_if_not_declared(
            node, plugin_name_ + "." + param_name, rclcpp::ParameterValue(default_value));
        node->get_parameter(plugin_name_ + "." + param_name, target);
    };
    auto declare_bool_param = [&](const std::string &param_name, bool default_value, bool &target) {
        declare_parameter_if_not_declared(
            node, plugin_name_ + "." + param_name, rclcpp::ParameterValue(default_value));
        node->get_parameter(plugin_name_ + "." + param_name, target);
    };

    declare_bool_param("teb_autosize", config_.teb_autosize, config_.teb_autosize);
    declare_double_param("dt_ref", config_.dt_ref, config_.dt_ref);
    declare_double_param("dt_hysteresis", config_.dt_hysteresis, config_.dt_hysteresis);
    declare_int_param("min_samples", config_.min_samples, config_.min_samples);
    declare_int_param("max_samples", config_.max_samples, config_.max_samples);
    declare_double_param("min_obstacle_dist", config_.min_obstacle_dist, config_.min_obstacle_dist);
    declare_double_param("inflation_dist", config_.inflation_dist, config_.inflation_dist);
    declare_double_param("costmap_weight", config_.costmap_weight, config_.costmap_weight);
    declare_int_param("no_iterations", config_.no_iterations, config_.no_iterations);
    declare_double_param("penalty_epsilon", config_.penalty_epsilon, config_.penalty_epsilon);
    declare_double_param("weight_obstacle", config_.weight_obstacle, config_.weight_obstacle);
    declare_double_param("weight_optimaltime", config_.weight_optimaltime, config_.weight_optimaltime);
    declare_double_param("weight_shortest_path", config_.weight_shortest_path, config_.weight_shortest_path);
    declare_double_param("weight_smoothness", config_.weight_smoothness, config_.weight_smoothness);
    declare_double_param("weight_kinematics", config_.weight_kinematics, config_.weight_kinematics);
    declare_double_param("max_vel_x", config_.max_vel_x, config_.max_vel_x);
    declare_double_param("max_vel_x_backwards", config_.max_vel_x_backwards, config_.max_vel_x_backwards);
    declare_double_param("max_vel_theta", config_.max_vel_theta, config_.max_vel_theta);
    declare_double_param("acc_lim_x", config_.acc_lim_x, config_.acc_lim_x);
    declare_double_param("acc_lim_theta", config_.acc_lim_theta, config_.acc_lim_theta);
    declare_double_param("min_turning_radius", config_.min_turning_radius, config_.min_turning_radius);
    declare_bool_param("is_holonomic", config_.is_holonomic, config_.is_holonomic);
    declare_bool_param("optimizer_verbose", config_.optimizer_verbose, config_.optimizer_verbose);
    declare_double_param("goal_tolerance_band", config_.goal_tolerance_band, config_.goal_tolerance_band);
    declare_double_param("reinit_pose_distance", config_.reinit_pose_distance, config_.reinit_pose_distance);

    double transform_tolerance = 0.1;
    declare_double_param("transform_tolerance", transform_tolerance, transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    teb_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        plugin_name_ + "/teb_markers", 10);

    RCLCPP_INFO(logger_, "基于g2o的TEB控制器配置完成");
}

void MyTebController::activate() { RCLCPP_INFO(logger_, "插件已激活"); }

void MyTebController::deactivate() { RCLCPP_INFO(logger_, "插件已停用"); }

void MyTebController::cleanup() {

    teb_trajectory_.clear();
    initial_teb_trajectory_.clear();
    obstacle_samples_.clear();
    needs_reinitialization_ = true;
    RCLCPP_INFO(logger_, "插件已清理");
}

void MyTebController::setPlan(const nav_msgs::msg::Path &path) {

    global_plan_ = path;
    needs_reinitialization_ = true;
}

geometry_msgs::msg::TwistStamped MyTebController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * /*goal_checker*/) {

    auto node = node_.lock();
    if (!node) {
        throw nav2_core::ControllerException("Failed to lock lifecycle node in computeVelocityCommands");
    }

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = "base_link";

    if (global_plan_.poses.empty()) {
        return cmd_vel;
    }

    const PoseSE2 robot_pose(
        pose.pose.position.x,
        pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation));

    local_plan_ = cropGlobalPlan(robot_pose);
    if (local_plan_.poses.empty()) {
        RCLCPP_WARN(logger_, "Local TEB plan is empty after cropping, returning zero velocity");
        return cmd_vel;
    }

    // 初始化轨迹数组
    if (shouldReinitializeBand(robot_pose, local_plan_)) {
        initializeTrajectory(local_plan_, robot_pose);
    }

    if (teb_trajectory_.size() < 2) {
        return cmd_vel;
    }

    if (config_.teb_autosize) {
        autoResizeTrajectory();
    }

    obstacle_samples_ = collectObstacleSamples();
    const auto &goal_msg = local_plan_.poses.back().pose;
    const PoseSE2 goal_pose(goal_msg.position.x, goal_msg.position.y, tf2::getYaw(goal_msg.orientation));

    // 开始进行teb规划算法
    optimizeTEB(goal_pose);

    if (!checkTrajectoryFeasibility()) {
        RCLCPP_WARN(logger_, "Optimized TEB trajectory is not feasible, returning zero velocity");
        return cmd_vel;
    }

    // 提取速度 并 可视化
    cmd_vel = extractVelocity(robot_pose, velocity);
    publishVisualization();

    last_v_x_ = cmd_vel.twist.linear.x;
    last_v_y_ = cmd_vel.twist.linear.y;
    last_omega_ = cmd_vel.twist.angular.z;
    return cmd_vel;
}

void MyTebController::initializeTrajectory(
    const nav_msgs::msg::Path &global_path,
    const PoseSE2 &start_pose) {

    teb_trajectory_.clear();
    initial_teb_trajectory_.clear();

    teb_trajectory_.emplace_back(start_pose, config_.dt_ref);

    for (const auto &pose_stamped : global_path.poses) {
        PoseSE2 next_pose(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            tf2::getYaw(pose_stamped.pose.orientation));

        const double distance =
            (next_pose.position() - teb_trajectory_.back().pose.position()).norm();
        if (distance < 1e-3) {
            continue;
        }

        const double dt_guess = std::max(config_.dt_ref, distance / std::max(config_.max_vel_x, 1e-2));
        teb_trajectory_.emplace_back(next_pose, dt_guess);
        if (static_cast<int>(teb_trajectory_.size()) >= config_.max_samples) {
            break;
        }
    }

    while (static_cast<int>(teb_trajectory_.size()) < config_.min_samples) {
        teb_trajectory_.emplace_back(teb_trajectory_.back().pose, config_.dt_ref);
    }

    if (!teb_trajectory_.empty()) {
        teb_trajectory_.back().dt = 0.0;
    }

    initial_teb_trajectory_ = teb_trajectory_;
    needs_reinitialization_ = false;
}

void MyTebController::optimizeTEB(const PoseSE2 &goal_pose) {

    if (!graph_optimizer_) {
        throw nav2_core::ControllerException("TEB graph optimizer is not initialized");
    }

    if (!graph_optimizer_->optimize(
            teb_trajectory_,
            initial_teb_trajectory_,
            goal_pose,
            obstacle_samples_,
            config_,
            config_.optimizer_verbose)) {
        RCLCPP_WARN(logger_, "g2o TEB optimization failed");
    }
}

geometry_msgs::msg::TwistStamped MyTebController::extractVelocity(
    const PoseSE2 &robot_pose,
    const geometry_msgs::msg::Twist & /*robot_vel*/) {

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = "base_link";

    if (teb_trajectory_.size() < 2) {
        return cmd_vel;
    }

    const PoseSE2 &target_pose = teb_trajectory_[1].pose;
    const double dt = std::max(teb_trajectory_[0].dt, 1e-3);
    const double dx_world = target_pose.x - robot_pose.x;
    const double dy_world = target_pose.y - robot_pose.y;

    const double cos_theta = std::cos(robot_pose.theta);
    const double sin_theta = std::sin(robot_pose.theta);
    const double dx_robot = cos_theta * dx_world + sin_theta * dy_world;
    const double dy_robot = -sin_theta * dx_world + cos_theta * dy_world;
    const double heading_error = angles::normalize_angle(target_pose.theta - robot_pose.theta);

    double desired_v_x = dx_robot / dt;
    double desired_v_y = config_.is_holonomic ? dy_robot / dt : 0.0;
    double desired_omega = heading_error / dt;

    desired_v_x = std::clamp(desired_v_x, -config_.max_vel_x_backwards, config_.max_vel_x);
    desired_v_y = std::clamp(desired_v_y, -config_.max_vel_x, config_.max_vel_x);
    desired_omega = std::clamp(desired_omega, -config_.max_vel_theta, config_.max_vel_theta);

    const double max_dv = config_.acc_lim_x * dt;
    const double max_dw = config_.acc_lim_theta * dt;
    desired_v_x = std::clamp(desired_v_x, last_v_x_ - max_dv, last_v_x_ + max_dv);
    desired_v_y = std::clamp(desired_v_y, last_v_y_ - max_dv, last_v_y_ + max_dv);
    desired_omega = std::clamp(desired_omega, last_omega_ - max_dw, last_omega_ + max_dw);

    cmd_vel.twist.linear.x = desired_v_x;
    cmd_vel.twist.linear.y = desired_v_y;
    cmd_vel.twist.angular.z = desired_omega;
    return cmd_vel;
}

bool MyTebController::checkTrajectoryFeasibility() {

    if (teb_trajectory_.size() < 2) {
        return false;
    }

    for (const auto &timed_pose : teb_trajectory_) {
        if (timed_pose.dt < 0.0) {
            return false;
        }

        const double obstacle_distance = getObstacleDistance(timed_pose.pose.x, timed_pose.pose.y);
        if (obstacle_distance < 0.0) {
            return false;
        }
        if (obstacle_distance < config_.min_obstacle_dist) {
            return false;
        }
    }

    return true;
}

void MyTebController::autoResizeTrajectory() {

    if (teb_trajectory_.empty()) {
        return;
    }

    std::vector<TimedPose> resized;
    resized.reserve(teb_trajectory_.size());
    resized.push_back(teb_trajectory_.front());

    for (size_t index = 1; index < teb_trajectory_.size(); ++index) {
        const double distance =
            (teb_trajectory_[index].pose.position() - resized.back().pose.position()).norm();

        if (distance > 1.5 * config_.dt_ref && static_cast<int>(resized.size()) < config_.max_samples) {
            PoseSE2 mid_pose(
                0.5 * (resized.back().pose.x + teb_trajectory_[index].pose.x),
                0.5 * (resized.back().pose.y + teb_trajectory_[index].pose.y),
                angles::normalize_angle(
                    0.5 * (resized.back().pose.theta + teb_trajectory_[index].pose.theta)));
            resized.emplace_back(mid_pose, 0.5 * resized.back().dt);
        }

        if (distance < std::max(0.05, config_.dt_hysteresis) && index + 1 < teb_trajectory_.size()) {
            continue;
        }

        resized.push_back(teb_trajectory_[index]);
    }

    while (static_cast<int>(resized.size()) < config_.min_samples) {
        resized.emplace_back(resized.back().pose, config_.dt_ref);
    }

    if (static_cast<int>(resized.size()) > config_.max_samples) {
        resized.resize(config_.max_samples);
    }

    if (!resized.empty()) {
        resized.back().dt = 0.0;
    }

    teb_trajectory_ = resized;
    initial_teb_trajectory_ = teb_trajectory_;
}

double MyTebController::getObstacleDistance(double x, double y) const {

    auto costmap = costmap_ros_->getCostmap();
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(x, y, mx, my)) {
        return -1.0;
    }

    const unsigned char cost = costmap->getCost(mx, my);
    if (cost == nav2_costmap_2d::FREE_SPACE) {
        return std::numeric_limits<double>::infinity();
    }
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        return 0.0;
    }
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return config_.inflation_dist;
    }

    const double ratio =
        static_cast<double>(cost) /
        static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    return std::max(0.0, config_.inflation_dist * (1.0 - ratio));
}

void MyTebController::publishVisualization() {

    if (!teb_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    auto make_line_marker = [&](int id, float r, float g, float b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = local_plan_.header.frame_id.empty() ? "map" : local_plan_.header.frame_id;
        marker.header.stamp = clock_->now();
        marker.ns = plugin_name_;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;
        marker.color.a = 1.0F;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        return marker;
    };

    auto optimized_marker = make_line_marker(0, 0.1F, 0.8F, 0.2F);
    auto initial_marker = make_line_marker(1, 0.9F, 0.7F, 0.1F);

    for (const auto &timed_pose : teb_trajectory_) {
        geometry_msgs::msg::Point point;
        point.x = timed_pose.pose.x;
        point.y = timed_pose.pose.y;
        optimized_marker.points.push_back(point);
    }

    for (const auto &timed_pose : initial_teb_trajectory_) {
        geometry_msgs::msg::Point point;
        point.x = timed_pose.pose.x;
        point.y = timed_pose.pose.y;
        initial_marker.points.push_back(point);
    }

    visualization_msgs::msg::Marker obstacle_marker = make_line_marker(2, 0.9F, 0.2F, 0.2F);
    obstacle_marker.type = visualization_msgs::msg::Marker::POINTS;
    obstacle_marker.scale.x = 0.05;
    obstacle_marker.scale.y = 0.05;
    obstacle_marker.points.clear();

    for (const auto &obstacle : obstacle_samples_) {
        geometry_msgs::msg::Point point;
        point.x = obstacle.x();
        point.y = obstacle.y();
        obstacle_marker.points.push_back(point);
    }

    visualization_msgs::msg::Marker goal_marker = make_line_marker(3, 0.2F, 0.4F, 1.0F);
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.scale.x = 0.12;
    goal_marker.scale.y = 0.12;
    goal_marker.scale.z = 0.12;
    goal_marker.points.clear();
    if (!local_plan_.poses.empty()) {
        goal_marker.pose.position.x = local_plan_.poses.back().pose.position.x;
        goal_marker.pose.position.y = local_plan_.poses.back().pose.position.y;
        goal_marker.pose.position.z = 0.0;
        goal_marker.pose.orientation.w = 1.0;
    }

    marker_array.markers.push_back(optimized_marker);
    marker_array.markers.push_back(initial_marker);
    marker_array.markers.push_back(obstacle_marker);
    marker_array.markers.push_back(goal_marker);
    teb_marker_pub_->publish(marker_array);
}

/**
 * @brief 裁剪全局路径
 * @param robot_pose 机器人当前位置
 * @return 裁剪后的全局路径
 */
nav_msgs::msg::Path MyTebController::cropGlobalPlan(const PoseSE2 &robot_pose) const {

    nav_msgs::msg::Path cropped_plan;
    cropped_plan.header = global_plan_.header;
    if (global_plan_.poses.empty()) {
        return cropped_plan;
    }

    size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < global_plan_.poses.size(); ++index) {
        const double dx = global_plan_.poses[index].pose.position.x - robot_pose.x;
        const double dy = global_plan_.poses[index].pose.position.y - robot_pose.y;
        const double distance = std::hypot(dx, dy);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_index = index;
        }
    }

    const double horizon =
        0.5 * std::min(costmap_ros_->getCostmap()->getSizeInMetersX(), costmap_ros_->getCostmap()->getSizeInMetersY());

    for (size_t index = nearest_index; index < global_plan_.poses.size(); ++index) {
        const auto &pose = global_plan_.poses[index];
        const double distance = std::hypot(
            pose.pose.position.x - robot_pose.x,
            pose.pose.position.y - robot_pose.y);

        if (!cropped_plan.poses.empty() && distance > horizon) {
            break;
        }

        cropped_plan.poses.push_back(pose);
        if (static_cast<int>(cropped_plan.poses.size()) >= config_.max_samples) {
            break;
        }
    }

    if (cropped_plan.poses.empty()) {
        cropped_plan.poses.push_back(global_plan_.poses.back());
    }

    return cropped_plan;
}

bool MyTebController::shouldReinitializeBand(
    const PoseSE2 &robot_pose, const nav_msgs::msg::Path &local_plan) const {

    if (needs_reinitialization_ || teb_trajectory_.size() < 2 || local_plan.poses.empty()) {
        return true;
    }

    const double start_shift =
        (teb_trajectory_.front().pose.position() - robot_pose.position()).norm();
    if (start_shift > config_.reinit_pose_distance) {
        return true;
    }

    const auto &goal_pose = local_plan.poses.back().pose;
    const double goal_shift = std::hypot(
        teb_trajectory_.back().pose.x - goal_pose.position.x,
        teb_trajectory_.back().pose.y - goal_pose.position.y);
    return goal_shift > config_.goal_tolerance_band;
}

ObstacleSamples MyTebController::collectObstacleSamples() const {

    ObstacleSamples obstacles;
    auto costmap = costmap_ros_->getCostmap();
    obstacles.reserve(costmap->getSizeInCellsX() * costmap->getSizeInCellsY() / 4);

    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
        for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
            const unsigned char cost = costmap->getCost(mx, my);
            if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                continue;
            }

            double wx = 0.0;
            double wy = 0.0;
            costmap->mapToWorld(mx, my, wx, wy);
            obstacles.emplace_back(wx, wy);
        }
    }

    return obstacles;
}

void MyTebController::setSpeedLimit(const double &speed_limit, const bool &percentage) {

    (void)speed_limit;
    (void)percentage;
    RCLCPP_INFO(logger_, "收到限速指令，当前g2o TEB控制器尚未实现动态限速逻辑");
}

} // namespace my_teb_controller

PLUGINLIB_EXPORT_CLASS(my_teb_controller::MyTebController, nav2_core::Controller)
