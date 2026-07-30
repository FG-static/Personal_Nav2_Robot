#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_smoother.hpp"
#include "my_planning_metrics/path_metrics.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_nav2_smoother {

void EsdfG2oSmoother::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) {

    node_ = parent.lock();
    if (!node_) {
        throw std::runtime_error("Failed to lock lifecycle node in EsdfG2oSmoother");
    }

    name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_sub_ = std::move(costmap_sub);
    footprint_sub_ = std::move(footprint_sub);
    declareParameters();

    RCLCPP_INFO(node_->get_logger(), "ESDF g2o smoother configured");
}

void EsdfG2oSmoother::activate() {

    RCLCPP_INFO(node_->get_logger(), "ESDF g2o smoother activated");
}

void EsdfG2oSmoother::deactivate() {

    RCLCPP_INFO(node_->get_logger(), "ESDF g2o smoother deactivated");
}

void EsdfG2oSmoother::cleanup() {

    esdf_map_.clear();
    costmap_sub_.reset();
    footprint_sub_.reset();
    tf_.reset();
    RCLCPP_INFO(node_->get_logger(), "ESDF g2o smoother cleaned up");
}

bool EsdfG2oSmoother::smooth(
    nav_msgs::msg::Path &path,
    const rclcpp::Duration &/*max_time*/) {

    const auto total_start_time = std::chrono::steady_clock::now();
    const nav_msgs::msg::Path input_path = path;
    EsdfG2oOptimizationStats optimization_stats;
    auto logMetrics =
        [&](bool success, double backend_ms) {
            const auto smoothing_end_time = std::chrono::steady_clock::now();
            const double smoother_total_ms =
                std::chrono::duration<double, std::milli>(
                    smoothing_end_time - total_start_time).count();

            const auto metrics_start_time = std::chrono::steady_clock::now();
            const auto costmap = costmap_sub_ ? costmap_sub_->getCostmap() : nullptr;
            my_planning_metrics::ObstacleDistanceField distance_field;
            const bool distance_ready =
                costmap != nullptr && distance_field.build(costmap.get());
            const auto *field = distance_ready ? &distance_field : nullptr;
            const my_planning_metrics::PathMetrics input_metrics =
                my_planning_metrics::evaluatePath(input_path, field);
            const my_planning_metrics::PathMetrics output_metrics =
                my_planning_metrics::evaluatePath(path, field);
            const double metrics_eval_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - metrics_start_time).count();

            RCLCPP_INFO(
                node_->get_logger(),
                "BACKEND_METRICS algorithm=ESDFG2O success=%s "
                "backend_iterations=%d graph_vertices=%zu graph_edges=%zu "
                "initial_chi2=%.6f final_chi2=%.6f backend_ms=%.3f "
                "smoother_total_ms=%.3f metrics_eval_ms=%.3f "
                "input_points=%zu output_points=%zu "
                "input_length_m=%.3f output_length_m=%.3f "
                "input_max_curvature_1pm=%.3f output_max_curvature_1pm=%.3f "
                "input_min_lethal_obstacle_distance_m=%.3f "
                "output_min_lethal_obstacle_distance_m=%.3f",
                success ? "true" : "false",
                optimization_stats.iterations,
                optimization_stats.vertices,
                optimization_stats.edges,
                optimization_stats.initial_chi2,
                optimization_stats.final_chi2,
                backend_ms,
                smoother_total_ms,
                metrics_eval_ms,
                input_metrics.point_count,
                output_metrics.point_count,
                input_metrics.length_m,
                output_metrics.length_m,
                input_metrics.max_curvature_inv_m,
                output_metrics.max_curvature_inv_m,
                input_metrics.min_lethal_obstacle_distance_m,
                output_metrics.min_lethal_obstacle_distance_m);
        };

    if (path.poses.size() < 3) {
        logMetrics(true, 0.0);
        return true;
    }

    const auto backend_start_time = std::chrono::steady_clock::now();
    TrajectoryPoints trajectory;
    if (!initializeTrajectory(path, trajectory)) {
        const double backend_optimization_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - backend_start_time).count();
        logMetrics(false, backend_optimization_ms);
        return true;
    }

    auto costmap = costmap_sub_ ? costmap_sub_->getCostmap() : nullptr;
    if (!esdf_map_.buildFromCostmap(costmap.get(), path, config_.esdf_margin)) {
        RCLCPP_WARN(node_->get_logger(), "ESDF g2o smoother failed to build ESDF map, keeping raw path");
        const double backend_optimization_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - backend_start_time).count();
        logMetrics(false, backend_optimization_ms);
        return true;
    }

    const TrajectoryPoints reference_trajectory = trajectory;
    if (!optimizer_.optimize(trajectory, reference_trajectory, esdf_map_, config_)) {
        optimization_stats = optimizer_.lastStats();
        RCLCPP_DEBUG(node_->get_logger(), "ESDF g2o optimizer skeleton kept raw path");
        const double backend_optimization_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - backend_start_time).count();
        logMetrics(false, backend_optimization_ms);
        return true;
    }

    optimization_stats = optimizer_.lastStats();
    updatePathFromTrajectory(path, trajectory);
    const double backend_optimization_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - backend_start_time).count();
    logMetrics(true, backend_optimization_ms);
    return true;
}

void EsdfG2oSmoother::declareParameters() {

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".resample_resolution", rclcpp::ParameterValue(config_.resample_resolution));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".esdf_margin", rclcpp::ParameterValue(config_.esdf_margin));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".min_obstacle_dist", rclcpp::ParameterValue(config_.min_obstacle_dist));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".weight_obstacle", rclcpp::ParameterValue(config_.weight_obstacle));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".weight_anchor", rclcpp::ParameterValue(config_.weight_anchor));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".weight_smoothness", rclcpp::ParameterValue(config_.weight_smoothness));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".weight_length", rclcpp::ParameterValue(config_.weight_length));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".no_iterations", rclcpp::ParameterValue(config_.no_iterations));
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".optimizer_verbose", rclcpp::ParameterValue(config_.optimizer_verbose));

    node_->get_parameter(name_ + ".resample_resolution", config_.resample_resolution);
    node_->get_parameter(name_ + ".esdf_margin", config_.esdf_margin);
    node_->get_parameter(name_ + ".min_obstacle_dist", config_.min_obstacle_dist);
    node_->get_parameter(name_ + ".weight_obstacle", config_.weight_obstacle);
    node_->get_parameter(name_ + ".weight_anchor", config_.weight_anchor);
    node_->get_parameter(name_ + ".weight_smoothness", config_.weight_smoothness);
    node_->get_parameter(name_ + ".weight_length", config_.weight_length);
    node_->get_parameter(name_ + ".no_iterations", config_.no_iterations);
    node_->get_parameter(name_ + ".optimizer_verbose", config_.optimizer_verbose);
}

bool EsdfG2oSmoother::initializeTrajectory(
    const nav_msgs::msg::Path &path,
    TrajectoryPoints &trajectory) const {

    trajectory.clear();
    trajectory.reserve(path.poses.size());
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
        TrajectoryPoint point;
        point.x = path.poses[i].pose.position.x;
        point.y = path.poses[i].pose.position.y;
        point.fixed = (i == 0 || i + 1 == path.poses.size());
        trajectory.push_back(point);
    }

    return !trajectory.empty();
}

void EsdfG2oSmoother::updatePathFromTrajectory(
    nav_msgs::msg::Path &path,
    const TrajectoryPoints &trajectory) const {

    if (path.poses.size() != trajectory.size())
        return;

    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        path.poses[i].pose.position.x = trajectory[i].x;
        path.poses[i].pose.position.y = trajectory[i].y;
    }

    for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
        const double dx = trajectory[i + 1].x - trajectory[i].x;
        const double dy = trajectory[i + 1].y - trajectory[i].y;
        const double yaw = std::atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        path.poses[i].pose.orientation = tf2::toMsg(q);
    }

    if (path.poses.size() > 1)
        path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
}

} // namespace my_nav2_smoother

PLUGINLIB_EXPORT_CLASS(my_nav2_smoother::EsdfG2oSmoother, nav2_core::Smoother)
