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

namespace {

/**
 * @brief 计算两个位姿之间的速度
 * @param from_pose 起始位姿，世界坐标系下的 (x, y, θ)
 * @param to_pose 终止位姿，世界坐标系下的 (x, y, θ)
 * @param dt 两帧之间的时间间隔
 * @return MecanumVelocity 机体坐标系下的速度 (v_x, v_y, omega)
 */
MecanumVelocity computeSegmentVelocity(const PoseSE2 &from_pose, const PoseSE2 &to_pose, double dt) {

    const double bounded_dt = std::max(dt, 1e-3);
    const double dx_world = to_pose.x - from_pose.x;
    const double dy_world = to_pose.y - from_pose.y;
    const double cos_theta = std::cos(from_pose.theta);
    const double sin_theta = std::sin(from_pose.theta);

    MecanumVelocity velocity;
    velocity.v_x = (cos_theta * dx_world + sin_theta * dy_world) / bounded_dt;
    velocity.v_y = (-sin_theta * dx_world + cos_theta * dy_world) / bounded_dt;
    velocity.omega = angles::normalize_angle(to_pose.theta - from_pose.theta) / bounded_dt;
    return velocity;
}

bool withinSymmetricLimit(double value, double limit) {

    return std::abs(value) <= limit + 1e-6;
}

double minimumTimeDiff(const TebConfig &config) {

    return std::max(1e-2, config.dt_ref - config.dt_hysteresis);
}

} // namespace

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
    declare_double_param("max_vel_y", config_.max_vel_y, config_.max_vel_y);
    declare_double_param("max_vel_theta", config_.max_vel_theta, config_.max_vel_theta);
    declare_double_param("acc_lim_x", config_.acc_lim_x, config_.acc_lim_x);
    declare_double_param("acc_lim_y", config_.acc_lim_y, config_.acc_lim_y);
    declare_double_param("acc_lim_theta", config_.acc_lim_theta, config_.acc_lim_theta);
    declare_bool_param("optimizer_verbose", config_.optimizer_verbose, config_.optimizer_verbose);
    declare_double_param("goal_tolerance_band", config_.goal_tolerance_band, config_.goal_tolerance_band);
    declare_double_param("reinit_pose_distance", config_.reinit_pose_distance, config_.reinit_pose_distance);

    double transform_tolerance = 0.1;
    declare_double_param("transform_tolerance", transform_tolerance, transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    teb_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        plugin_name_ + "/teb_markers", 10);

    RCLCPP_INFO(logger_, "基于g2o的麦克纳姆轮TEB控制器配置完成");
}

void MyTebController::activate() { RCLCPP_INFO(logger_, "麦克纳姆轮TEB插件已激活"); }

void MyTebController::deactivate() { RCLCPP_INFO(logger_, "麦克纳姆轮TEB插件已停用"); }

void MyTebController::cleanup() {

    teb_trajectory_.clear();
    initial_teb_trajectory_.clear();
    obstacle_samples_.clear();
    last_velocity_ = MecanumVelocity{};
    needs_reinitialization_ = true;
    RCLCPP_INFO(logger_, "麦克纳姆轮TEB插件已清理");
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
        last_velocity_ = MecanumVelocity{};
        return cmd_vel;
    }

    const PoseSE2 robot_pose(
        pose.pose.position.x,
        pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation));

    local_plan_ = cropGlobalPlan(robot_pose);
    if (local_plan_.poses.empty()) {
        RCLCPP_WARN(logger_, "Local TEB plan is empty after cropping, returning zero velocity");
        last_velocity_ = MecanumVelocity{};
        return cmd_vel;
    }

    // 初始化轨迹数组
    if (shouldReinitializeBand(robot_pose, local_plan_)) {
        initializeTrajectory(local_plan_, robot_pose);
    }

    if (teb_trajectory_.size() < 2) {
        last_velocity_ = MecanumVelocity{};
        return cmd_vel;
    }

    if (config_.teb_autosize) {
        autoResizeTrajectory();
    }

    obstacle_samples_ = collectObstacleSamples();
    const auto &goal_msg = local_plan_.poses.back().pose;
    const PoseSE2 goal_pose(goal_msg.position.x, goal_msg.position.y, tf2::getYaw(goal_msg.orientation));

    // 开始进行teb规划算法
    if (!optimizeTEB(goal_pose)) {
        RCLCPP_WARN(logger_, "TEB optimization failed, returning zero velocity");
        last_velocity_ = MecanumVelocity{};
        needs_reinitialization_ = true;
        return cmd_vel;
    }

    if (!checkTrajectoryFeasibility()) {
        RCLCPP_WARN(logger_, "Optimized TEB trajectory is not feasible, returning zero velocity");
        last_velocity_ = MecanumVelocity{};
        needs_reinitialization_ = true;
        return cmd_vel;
    }

    // 提取速度 并 可视化
    cmd_vel = extractVelocity(robot_pose, velocity);
    publishVisualization();

    last_velocity_.v_x = cmd_vel.twist.linear.x;
    last_velocity_.v_y = cmd_vel.twist.linear.y;
    last_velocity_.omega = cmd_vel.twist.angular.z;
    return cmd_vel;
}

void MyTebController::initializeTrajectory(
    const nav_msgs::msg::Path &global_path,
    const PoseSE2 &start_pose) {

    teb_trajectory_.clear();
    initial_teb_trajectory_.clear();

    teb_trajectory_.emplace_back(start_pose, config_.dt_ref);

    for (size_t pose_index = 0; pose_index < global_path.poses.size(); ++pose_index) {
        const auto &pose_stamped = global_path.poses[pose_index];
        PoseSE2 next_pose(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            tf2::getYaw(pose_stamped.pose.orientation));

        if (pose_index + 1 < global_path.poses.size()) {
            const auto &next_msg = global_path.poses[pose_index + 1].pose.position;
            const double dx = next_msg.x - pose_stamped.pose.position.x;
            const double dy = next_msg.y - pose_stamped.pose.position.y;
            if (std::hypot(dx, dy) > 1e-6) {
                next_pose.theta = std::atan2(dy, dx);
            }
        } else if (!teb_trajectory_.empty()) {
            const PoseSE2 &prev_pose = teb_trajectory_.back().pose;
            const double dx = next_pose.x - prev_pose.x;
            const double dy = next_pose.y - prev_pose.y;
            if (std::hypot(dx, dy) > 1e-6) {
                next_pose.theta = std::atan2(dy, dx);
            }
        }

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

bool MyTebController::optimizeTEB(const PoseSE2 &goal_pose) {

    if (!graph_optimizer_) {
        throw nav2_core::ControllerException("TEB graph optimizer is not initialized");
    }

    const bool ok = graph_optimizer_->optimize(
        teb_trajectory_,
        initial_teb_trajectory_,
        goal_pose,
        obstacle_samples_,
        config_,
        config_.optimizer_verbose);
    if (!ok) {
        RCLCPP_WARN(logger_, "g2o TEB optimization failed");
        return false;
    }

    initial_teb_trajectory_ = teb_trajectory_;
    return true;
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
    const double dt = std::max(teb_trajectory_[0].dt, minimumTimeDiff(config_));
    MecanumVelocity desired_velocity = computeSegmentVelocity(robot_pose, target_pose, dt);

    desired_velocity.v_x = std::clamp(desired_velocity.v_x, -config_.max_vel_x, config_.max_vel_x);
    desired_velocity.v_y = std::clamp(desired_velocity.v_y, -config_.max_vel_y, config_.max_vel_y);
    desired_velocity.omega =
        std::clamp(desired_velocity.omega, -config_.max_vel_theta, config_.max_vel_theta);

    const double max_dv_x = config_.acc_lim_x * dt;
    const double max_dv_y = config_.acc_lim_y * dt;
    const double max_domega = config_.acc_lim_theta * dt;
    desired_velocity.v_x = std::clamp(
        desired_velocity.v_x,
        last_velocity_.v_x - max_dv_x,
        last_velocity_.v_x + max_dv_x);
    desired_velocity.v_y = std::clamp(
        desired_velocity.v_y,
        last_velocity_.v_y - max_dv_y,
        last_velocity_.v_y + max_dv_y);
    desired_velocity.omega = std::clamp(
        desired_velocity.omega,
        last_velocity_.omega - max_domega,
        last_velocity_.omega + max_domega);

    cmd_vel.twist.linear.x = desired_velocity.v_x;
    cmd_vel.twist.linear.y = desired_velocity.v_y;
    cmd_vel.twist.angular.z = desired_velocity.omega;
    return cmd_vel;
}

/**
 * @brief 从车体坐标系速度推算世界坐标系下的位移增量
 * @param v_x 车体坐标系x方向速度
 * @param v_y 车体坐标系y方向速度
 * @param omega 车体坐标系转角速度
 * @param theta 车体坐标系当前朝向
 * @param dt 时间增量
 * @return 世界坐标系下的位移增量
 */
PoseSE2 MyTebController::mecanumForwardKinematics(
    double v_x,
    double v_y,
    double omega,
    double theta,
    double dt) const {

    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    return PoseSE2(
        (v_x * cos_theta - v_y * sin_theta) * dt,
        (v_x * sin_theta + v_y * cos_theta) * dt,
        omega * dt);
}

bool MyTebController::checkTrajectoryFeasibility() {

    if (teb_trajectory_.size() < 2) {
        return false;
    }

    for (const auto &timed_pose : teb_trajectory_) {
        if (!std::isfinite(timed_pose.dt) || timed_pose.dt < 0.0) {
            return false;
        }

        const double obstacle_distance = getObstacleDistance(timed_pose.pose.x, timed_pose.pose.y);
        if (obstacle_distance < 0.0) {
            continue;
        }
        if (obstacle_distance < config_.min_obstacle_dist) {
            return false;
        }
    }

    for (size_t index = 0; index + 1 < teb_trajectory_.size(); ++index) {
        const PoseSE2 &from_pose = teb_trajectory_[index].pose;
        const PoseSE2 &to_pose = teb_trajectory_[index + 1].pose;
        const double dt = teb_trajectory_[index].dt;
        if (!std::isfinite(dt) || dt <= 0.0) {
            return false;
        }

        const MecanumVelocity velocity = computeSegmentVelocity(from_pose, to_pose, dt);
        if (!withinSymmetricLimit(velocity.v_x, config_.max_vel_x) ||
            !withinSymmetricLimit(velocity.v_y, config_.max_vel_y) ||
            !withinSymmetricLimit(velocity.omega, config_.max_vel_theta)) {
            RCLCPP_DEBUG(
                logger_,
                "TEB segment exceeds mecanum velocity limits and will be clamped: vx=%.3f vy=%.3f omega=%.3f",
                velocity.v_x,
                velocity.v_y,
                velocity.omega);
        }

        const PoseSE2 predicted_delta = mecanumForwardKinematics(
            velocity.v_x,
            velocity.v_y,
            velocity.omega,
            from_pose.theta,
            dt);
        const double delta_error = std::hypot(
            predicted_delta.x - (to_pose.x - from_pose.x),
            predicted_delta.y - (to_pose.y - from_pose.y));
        if (!std::isfinite(delta_error) || delta_error > 1e-3) {
            return false;
        }
    }

    for (size_t index = 0; index + 2 < teb_trajectory_.size(); ++index) {
        const double dt_1 = teb_trajectory_[index].dt;
        const double dt_2 = teb_trajectory_[index + 1].dt;
        if (!std::isfinite(dt_1) || !std::isfinite(dt_2) || dt_1 <= 0.0 || dt_2 <= 0.0) {
            return false;
        }

        const MecanumVelocity velocity_1 = computeSegmentVelocity(
            teb_trajectory_[index].pose,
            teb_trajectory_[index + 1].pose,
            dt_1);
        const MecanumVelocity velocity_2 = computeSegmentVelocity(
            teb_trajectory_[index + 1].pose,
            teb_trajectory_[index + 2].pose,
            dt_2);
        const double avg_dt = std::max(0.5 * (dt_1 + dt_2), 1e-3);

        const double acc_x = (velocity_2.v_x - velocity_1.v_x) / avg_dt;
        const double acc_y = (velocity_2.v_y - velocity_1.v_y) / avg_dt;
        const double acc_theta = (velocity_2.omega - velocity_1.omega) / avg_dt;
        if (!withinSymmetricLimit(acc_x, config_.acc_lim_x) ||
            !withinSymmetricLimit(acc_y, config_.acc_lim_y) ||
            !withinSymmetricLimit(acc_theta, config_.acc_lim_theta)) {
            RCLCPP_DEBUG(
                logger_,
                "TEB segment exceeds mecanum acceleration limits and will be clamped: ax=%.3f ay=%.3f atheta=%.3f",
                acc_x,
                acc_y,
                acc_theta);
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
