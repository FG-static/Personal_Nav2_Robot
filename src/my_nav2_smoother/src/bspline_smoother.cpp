#include "my_nav2_smoother/bspline_smoother.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <nav2_util/node_utils.hpp>
#include <rclcpp/parameter_value.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

extern "C" {
#include "osqp/osqp.h"
}

namespace my_bspline_smoother {

    void MyBSplineSmoother::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/
    ) {

        costmap_sub_ = costmap_sub;
        node_ = parent.lock();
        name_ = name;

        nav2_util::declare_parameter_if_not_declared(node_, name + ".w_smooth", rclcpp::ParameterValue(10.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".w_guide", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".max_vel", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".max_acc", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".max_jerk", rclcpp::ParameterValue(2.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".min_dt", rclcpp::ParameterValue(0.05));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".max_dt", rclcpp::ParameterValue(1.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".max_outer_iterations", rclcpp::ParameterValue(3));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".time_inflation_factor", rclcpp::ParameterValue(1.15));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".max_time_scale", rclcpp::ParameterValue(2.0));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".corridor_max_expand_dist", rclcpp::ParameterValue(0.8));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".corridor_min_half_width", rclcpp::ParameterValue(0.05));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".corridor_overlap_threshold", rclcpp::ParameterValue(0.8));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".corridor_collision_check_resolution", rclcpp::ParameterValue(0.03));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".visualize_corridor_boxes", rclcpp::ParameterValue(true));
        nav2_util::declare_parameter_if_not_declared(node_, name + ".corridor_marker_z", rclcpp::ParameterValue(0.02));
        nav2_util::declare_parameter_if_not_declared(
            node_, name + ".max_overshoot_constraints_per_iter", rclcpp::ParameterValue(20));

        node_->get_parameter(name + ".w_smooth", w_smooth_);
        node_->get_parameter(name + ".w_guide", w_guide_);
        node_->get_parameter(name + ".max_vel", max_vel_);
        node_->get_parameter(name + ".max_acc", max_acc_);
        node_->get_parameter(name + ".max_jerk", max_jerk_);
        node_->get_parameter(name + ".min_dt", min_dt_);
        node_->get_parameter(name + ".max_dt", max_dt_);
        node_->get_parameter(name + ".max_outer_iterations", max_outer_iterations_);
        node_->get_parameter(name + ".time_inflation_factor", time_inflation_factor_);
        node_->get_parameter(name + ".max_time_scale", max_time_scale_);
        node_->get_parameter(name + ".corridor_max_expand_dist", corridor_max_expand_dist_);
        node_->get_parameter(name + ".corridor_min_half_width", corridor_min_half_width_);
        node_->get_parameter(name + ".corridor_overlap_threshold", corridor_overlap_threshold_);
        node_->get_parameter(name + ".corridor_collision_check_resolution", corridor_collision_check_resolution_);
        node_->get_parameter(name + ".visualize_corridor_boxes", visualize_corridor_boxes_);
        node_->get_parameter(name + ".corridor_marker_z", corridor_marker_z_);
        node_->get_parameter(name + ".max_overshoot_constraints_per_iter", max_overshoot_constraints_per_iter_);

        corridor_marker_pub_ =
            node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                "~/corridor_boxes",
                rclcpp::SystemDefaultsQoS());
    }
    void MyBSplineSmoother::activate() {

        if (corridor_marker_pub_)
            corridor_marker_pub_->on_activate();

        RCLCPP_INFO(node_->get_logger(), "插件已激活");
    }

    void MyBSplineSmoother::deactivate() {

        if (corridor_marker_pub_)
            corridor_marker_pub_->on_deactivate();

        RCLCPP_INFO(node_->get_logger(), "插件已停用");
    }

    void MyBSplineSmoother::cleanup() {

        corridor_marker_pub_.reset();
        RCLCPP_INFO(node_->get_logger(), "插件已清理");
    }
    bool MyBSplineSmoother::smooth(
        nav_msgs::msg::Path &path,
        const rclcpp::Duration &/*max_time*/
    ) {

        if (path.poses.size() < 4) return true;
        nav_msgs::msg::Path raw_path = path;
        applyBSplineAlgorithm(path, raw_path);

        return true;
    }
    void MyBSplineSmoother::applyBSplineAlgorithm(
        nav_msgs::msg::Path &path,
        const nav_msgs::msg::Path &raw_path
    ) {

        if (raw_path.poses.size() < 4) {

            path = raw_path;
            return;
        }
        std::vector<double>
            ref_path_x,
            ref_path_y,
            smooth_path_x,
            smooth_path_y;
        for (size_t i = 0; i < raw_path.poses.size(); ++ i) {

            ref_path_x.push_back(raw_path.poses[i].pose.position.x);
            ref_path_y.push_back(raw_path.poses[i].pose.position.y);
        }
        path_frame_id_ = raw_path.header.frame_id.empty() ? "map" : raw_path.header.frame_id;
        solveBSplineQP(ref_path_x, ref_path_y, w_smooth_, w_guide_, smooth_path_x, smooth_path_y);
        int n = smooth_path_x.size();
        for (int i = 0; i < n; ++ i) {

            double yaw;

            if (i < n - 1) {

                yaw = std::atan2(smooth_path_y[i + 1] - smooth_path_y[i],
                                smooth_path_x[i + 1] - smooth_path_x[i]);
            } else {

                yaw = std::atan2(smooth_path_y[i] - smooth_path_y[i - 1],
                                smooth_path_x[i] - smooth_path_x[i - 1]);
            }

            // 更新位姿
            path.poses[i].pose.position.x = smooth_path_x[i];
            path.poses[i].pose.position.y = smooth_path_y[i];
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw); // Roll=0, Pitch=0, Yaw=yaw
            path.poses[i].pose.orientation = tf2::toMsg(q);
        }
    }

    /**
     * @brief 状态约束超限检查
     * @param p_x 规划的轨迹x分量
     * @param p_y 规划的轨迹y分量
     * @param dt_segment 前一次分配好的时间
     * @return DynamicReport 报告超调的地方以及相关信息
     */
    DynamicReport MyBSplineSmoother::checkDynamicFeasibility(
        const std::vector<double> &p_x,
        const std::vector<double> &p_y,
        const std::vector<double> &dt_segment
    ) const {

        DynamicReport report;
        const int n = static_cast<int>(p_x.size());
        if (n < 2 || static_cast<int>(dt_segment.size()) != n - 1) {

            report.ok = false;
            return report;
        }

        std::vector<Eigen::Vector2d> vel;
        vel.reserve(n - 1);
        for (int i = 0; i + 1 < n; ++ i) {

            const double dt = std::max(dt_segment[i], min_dt_);

            Eigen::Vector2d v(
                (p_x[i + 1] - p_x[i]) / dt,
                (p_y[i + 1] - p_y[i]) / dt
            );
            const double speed = v.norm();
            report.max_vel = std::max(report.max_vel, speed);

            if (speed > max_vel_) {

                report.ok = false;
                report.vel_vios.push_back({
                    i,
                    speed / std::max(max_vel_, 1e-6)
                });
            }
            vel.push_back(v);
        }

        std::vector<Eigen::Vector2d> acc;
        acc.reserve(std::max(0, n - 2));
        for (int i = 0; i + 1 < static_cast<int>(vel.size()); ++ i) {

            const double dt = std::max(
                0.5 * (dt_segment[i] + dt_segment[i + 1]),
                min_dt_
            );

            Eigen::Vector2d a = (vel[i + 1] - vel[i]) / dt;
            const double _acc = a.norm();
            report.max_acc = std::max(report.max_acc, _acc);

            if (_acc > max_acc_) {

                report.ok = false;
                report.acc_vios.push_back({
                    i,
                    _acc / std::max(max_acc_, 1e-6)
                });
            }
            acc.push_back(a);
        }

        for (int i = 0; i + 1 < static_cast<int>(acc.size()); ++ i) {

            const double dt = std::max(
                0.5 * (dt_segment[i + 1] + dt_segment[i + 2]),
                min_dt_
            );

            Eigen::Vector2d j = (acc[i + 1] - acc[i]) / dt;
            const double jerk = j.norm();

            report.max_jerk = std::max(report.max_jerk, jerk);

            if (jerk > max_jerk_) {

                report.ok = false;
                report.jerk_vios.push_back({
                    i,
                    jerk / std::max(max_jerk_, 1e-6)
                });
            }
        }
        return report;
    }

    /**
     * @brief 第一次粗略时间分配
     * @param p_ref_x 参考路径x分量
     * @param p_ref_y 参考路径y分量
     * @param output 输出时间分配数组
     * @return bool 是否成功
     */
    bool MyBSplineSmoother::computeTimeAllocation(
        const std::vector<double> &p_ref_x,
        const std::vector<double> &p_ref_y,
        std::vector<double> &output
    ) const {

        const int n = static_cast<int>(p_ref_x.size());
        output.clear();
        output.reserve(std::max(0, n - 1));
        for (int i = 0; i + 1 < n; ++ i) {

            const double dx = p_ref_x[i + 1] - p_ref_x[i];
            const double dy = p_ref_y[i + 1] - p_ref_y[i];
            const double ds = std::hypot(dx, dy);

            const double dt_vel = ds / std::max(max_vel_, 1e-3);
            const double dt_acc = std::sqrt(ds / std::max(max_acc_, 1e-3));
            const double dt_jerk = std::cbrt(ds / std::max(max_jerk_, 1e-3));

            const double dt = std::clamp(
                std::max({dt_vel, dt_acc, dt_jerk, min_dt_}),
                min_dt_,
                max_dt_
            );

            output.push_back(dt);
        }
        return true;
    }

    void MyBSplineSmoother::inflateTimeAllocation(
        const DynamicReport &report,
        std::vector<double> &dt_segment
    ) const {

        auto inflate_segment =
            [&](int index, double scale) {

                if (index < 0 ||
                    index >= static_cast<int>(dt_segment.size()))
                    return;

                const double safe_scale = std::clamp(
                    scale * time_inflation_factor_,
                    1.0,
                    max_time_scale_
                );
                dt_segment[index] = std::clamp(
                    dt_segment[index] * safe_scale,
                    min_dt_,
                    max_dt_
                );
            };

        for (const auto &vio : report.vel_vios)
            inflate_segment(vio.index, vio.ratio);

        for (const auto &vio : report.acc_vios) {

            const double scale = std::sqrt(vio.ratio);
            inflate_segment(vio.index, scale);
            inflate_segment(vio.index + 1, scale);
        }

        for (const auto &vio : report.jerk_vios) {

            const double scale = std::cbrt(vio.ratio);
            inflate_segment(vio.index, scale);
            inflate_segment(vio.index + 1, scale);
            inflate_segment(vio.index + 2, scale);
        }
    }

    CorridorReport MyBSplineSmoother::checkCorridorFeasibility(
        const std::vector<double> &p_x,
        const std::vector<double> &p_y,
        const CorridorBounds &bounds
    ) const {

        CorridorReport report;
        const int n = static_cast<int>(p_x.size());
        if (n == 0 ||
            p_y.size() != p_x.size() ||
            bounds.lower_x.size() != p_x.size() ||
            bounds.upper_x.size() != p_x.size() ||
            bounds.lower_y.size() != p_x.size() ||
            bounds.upper_y.size() != p_x.size()) {

            report.ok = false;
            report.point_vios.push_back({-1, 0.0, 0.0, 0.0});
            return report;
        }

        for (int i = 0; i < n; ++ i) {

            double violation = 0.0, dx = 0.0, dy = 0.0;
            if (p_x[i] < bounds.lower_x[i]) {

                dx = p_x[i] - bounds.lower_x[i];
                violation = std::max(violation, -dx);
            }
            if (p_x[i] > bounds.upper_x[i]) {

                dx = p_x[i] - bounds.upper_x[i];
                violation = std::max(violation, dx);
            }
            if (p_y[i] < bounds.lower_y[i]) {

                dy = p_y[i] - bounds.lower_y[i];
                violation = std::max(violation, -dy);
            }
            if (p_y[i] > bounds.upper_y[i]){

                dy = p_y[i] - bounds.upper_y[i];
                violation = std::max(violation, dy);
            }

            if (violation > 1e-6) {

                report.ok = false;
                report.point_vios.push_back({i, dx, dy, violation});
            }
        }

        for (int i = 0; i + 1 < n; ++ i) {

            if (!isSegmentCollisionFree(p_x[i], p_y[i], p_x[i + 1], p_y[i + 1])) {

                report.ok = false;
                const double distance = std::hypot(p_x[i + 1] - p_x[i], p_y[i + 1] - p_y[i]);
                report.segment_vios.push_back({i, 0.0, 0.0, distance});
            }
        }

        return report;
    }

    bool MyBSplineSmoother::isColumnFree(
        int mx,
        int min_my,
        int max_my
    ) const {

        if (min_my > max_my)
            return false;

        for (int my = min_my; my <= max_my; ++ my)
            if (!isCellFree(mx, my))
                return false;

        return true;
    }

    bool MyBSplineSmoother::isRowFree(
        int my,
        int min_mx,
        int max_mx
    ) const {

        if (min_mx > max_mx)
            return false;

        for (int mx = min_mx; mx <= max_mx; ++ mx)
            if (!isCellFree(mx, my))
                return false;

        return true;
    }

    GridBox MyBSplineSmoother::expandBoxFromCell(
        int seed_mx,
        int seed_my,
        int max_expand_cells
    ) const {

        GridBox box;
        box.min_mx = seed_mx;
        box.max_mx = seed_mx;
        box.min_my = seed_my;
        box.max_my = seed_my;

        if (!isCellFree(seed_mx, seed_my)) return box;

        bool expanded = true;
        while (expanded) {

            expanded = false;

            if (box.max_mx - seed_mx < max_expand_cells &&
                isColumnFree(box.max_mx + 1, box.min_my, box.max_my)) {

                ++ box.max_mx;
                expanded = true;
            }

            if (seed_mx - box.min_mx < max_expand_cells &&
                isColumnFree(box.min_mx - 1, box.min_my, box.max_my)) {

                -- box.min_mx;
                expanded = true;
            }

            if (box.max_my - seed_my < max_expand_cells &&
                isRowFree(box.max_my + 1, box.min_mx, box.max_mx)) {

                ++ box.max_my;
                expanded = true;
            }

            if (seed_my - box.min_my < max_expand_cells &&
                isRowFree(box.min_my - 1, box.min_mx, box.max_mx)) {

                -- box.min_my;
                expanded = true;
            }
        }
        return box;
    }

    CorridorBounds MyBSplineSmoother::buildCorridorBounds(
        const std::vector<double> &p_ref_x,
        const std::vector<double> &p_ref_y
    ) const {

        CorridorBounds bounds;

        const int n = static_cast<int>(p_ref_x.size());
        bounds.lower_x.resize(n);
        bounds.upper_x.resize(n);
        bounds.lower_y.resize(n);
        bounds.upper_y.resize(n);

        auto setFallbackCorridorBounds =
            [&](
                int index,
                const std::vector<double> &p_ref_x,
                const std::vector<double> &p_ref_y,
                CorridorBounds &bounds
            ) {

                bounds.lower_x[index] = p_ref_x[index] - corridor_min_half_width_;
                bounds.upper_x[index] = p_ref_x[index] + corridor_min_half_width_;
                bounds.lower_y[index] = p_ref_y[index] - corridor_min_half_width_;
                bounds.upper_y[index] = p_ref_y[index] + corridor_min_half_width_;
            };

        if (n == 0 || p_ref_x.size() != p_ref_y.size())
            return bounds;

        if (!costmap_sub_) {

            for (int i = 0; i < n; ++ i)
                setFallbackCorridorBounds(
                    i,
                    p_ref_x,
                    p_ref_y,
                    bounds
                );
            return bounds;
        }
        auto costmap = costmap_sub_->getCostmap();
        if (!costmap) {

            for (int i = 0; i < n; ++ i)
                setFallbackCorridorBounds(
                    i,
                    p_ref_x,
                    p_ref_y,
                    bounds
                );
            return bounds;
        }

        const double resolution = std::max(costmap->getResolution(), 1e-3);
        const int max_expand_cells = std::max(
            1,
            static_cast<int>(
                std::ceil(corridor_max_expand_dist_ / resolution)
            )
        );
        for (int i = 0; i < n; ++ i) {

            unsigned int mx = 0, my = 0;
            if (!worldToMap(p_ref_x[i], p_ref_y[i], mx, my) ||
                !isCellFree(static_cast<int>(mx), static_cast<int>(my))) {

                setFallbackCorridorBounds(i, p_ref_x, p_ref_y, bounds);
                continue;
            }

            const GridBox box = expandBoxFromCell(
                static_cast<int>(mx),
                static_cast<int>(my),
                max_expand_cells
            );

            double min_x = 0.0;
            double min_y = 0.0;
            double max_x = 0.0;
            double max_y = 0.0;

            costmap->mapToWorld(
                static_cast<unsigned int>(box.min_mx),
                static_cast<unsigned int>(box.min_my),
                min_x,
                min_y);

            costmap->mapToWorld(
                static_cast<unsigned int>(box.max_mx),
                static_cast<unsigned int>(box.max_my),
                max_x,
                max_y);

            bounds.lower_x[i] = std::min(min_x, max_x) - 0.5 * resolution;
            bounds.upper_x[i] = std::max(min_x, max_x) + 0.5 * resolution;
            bounds.lower_y[i] = std::min(min_y, max_y) - 0.5 * resolution;
            bounds.upper_y[i] = std::max(min_y, max_y) + 0.5 * resolution;

            if (bounds.upper_x[i] - bounds.lower_x[i] < 2.0 * corridor_min_half_width_) {

                bounds.lower_x[i] = p_ref_x[i] - corridor_min_half_width_;
                bounds.upper_x[i] = p_ref_x[i] + corridor_min_half_width_;
            }

            if (bounds.upper_y[i] - bounds.lower_y[i] < 2.0 * corridor_min_half_width_) {

                bounds.lower_y[i] = p_ref_y[i] - corridor_min_half_width_;
                bounds.upper_y[i] = p_ref_y[i] + corridor_min_half_width_;
            }
        }
        for (int i = 0; i < n; ++ i) {

            if (i < 2 || i > n - 3) {

                bounds.lower_x[i] = p_ref_x[i];
                bounds.upper_x[i] = p_ref_x[i];
                bounds.lower_y[i] = p_ref_y[i];
                bounds.upper_y[i] = p_ref_y[i];
            }
        }

        return bounds;
    }

    // TODO：Understanf it
    std::vector<GridBox> MyBSplineSmoother::buildSegmentCorridorBoxes(
        const std::vector<double> &p_ref_x,
        const std::vector<double> &p_ref_y
    ) const {

        std::vector<GridBox> boxes;
        if (p_ref_x.size() != p_ref_y.size() || p_ref_x.size() < 4)
            return boxes;

        const int segment_count = static_cast<int>(p_ref_x.size()) - 3;
        boxes.reserve(segment_count);

        if (!costmap_sub_)
            return boxes;

        auto costmap = costmap_sub_->getCostmap();
        if (!costmap)
            return boxes;

        const double resolution = std::max(costmap->getResolution(), 1e-3);
        const int max_expand_cells = std::max(
            1,
            static_cast<int>(std::ceil(corridor_max_expand_dist_ / resolution))
        );

        for (int segment = 0; segment < segment_count; ++ segment) {

            // First version: use an interior reference point as the segment's corridor seed.
            const int seed_index = std::clamp(
                segment + 1,
                0,
                static_cast<int>(p_ref_x.size()) - 1
            );

            unsigned int mx = 0;
            unsigned int my = 0;
            if (!worldToMap(p_ref_x[seed_index], p_ref_y[seed_index], mx, my) ||
                !isCellFree(static_cast<int>(mx), static_cast<int>(my))) {

                boxes.push_back(GridBox{
                    static_cast<int>(mx),
                    static_cast<int>(mx),
                    static_cast<int>(my),
                    static_cast<int>(my)
                });
                continue;
            }

            boxes.push_back(expandBoxFromCell(
                static_cast<int>(mx),
                static_cast<int>(my),
                max_expand_cells
            ));
        }

        return boxes;
    }

    void MyBSplineSmoother::publishCorridorMarkers(
        const CorridorBounds &bounds,
        const std::string &frame_id
    ) const {

        if (!visualize_corridor_boxes_ || !corridor_marker_pub_ ||
            !corridor_marker_pub_->is_activated())
            return;

        const std::size_t n = bounds.lower_x.size();
        if (bounds.upper_x.size() != n ||
            bounds.lower_y.size() != n ||
            bounds.upper_y.size() != n)
            return;

        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = frame_id.empty() ? "map" : frame_id;
        clear_marker.header.stamp = node_->now();
        clear_marker.ns = "bspline_corridor_boxes";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear_marker);

        for (std::size_t i = 0; i < n; ++ i) {

            const double width = bounds.upper_x[i] - bounds.lower_x[i];
            const double height = bounds.upper_y[i] - bounds.lower_y[i];
            if (width < 0.0 || height < 0.0)
                continue;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = clear_marker.header.frame_id;
            marker.header.stamp = clear_marker.header.stamp;
            marker.ns = "bspline_corridor_boxes";
            marker.id = static_cast<int>(i + 1);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 0.5 * (bounds.lower_x[i] + bounds.upper_x[i]);
            marker.pose.position.y = 0.5 * (bounds.lower_y[i] + bounds.upper_y[i]);
            marker.pose.position.z = 0.5 * corridor_marker_z_;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = std::max(width, 1e-3);
            marker.scale.y = std::max(height, 1e-3);
            marker.scale.z = std::max(corridor_marker_z_, 1e-3);
            marker.color.r = 0.1F;
            marker.color.g = 0.75F;
            marker.color.b = 1.0F;
            marker.color.a = 0.18F;
            markers.markers.push_back(marker);
        }

        corridor_marker_pub_->publish(markers);
    }

    bool MyBSplineSmoother::worldToMap(
        double wx,
        double wy,
        unsigned int &mx,
        unsigned int &my
    ) const {

        if (!costmap_sub_)
            return false;

        auto costmap = costmap_sub_->getCostmap();
        if (!costmap)
            return false;

        return costmap->worldToMap(wx, wy, mx, my);
    }

    bool MyBSplineSmoother::isCellFree(int mx, int my) const {

        if (!costmap_sub_ || mx < 0 || my < 0)
            return false;

        auto costmap = costmap_sub_->getCostmap();
        if (!costmap)
            return false;

        const auto umx = static_cast<unsigned int>(mx);
        const auto umy = static_cast<unsigned int>(my);
        if (umx >= costmap->getSizeInCellsX() || umy >= costmap->getSizeInCellsY())
            return false;

        const unsigned char cost = costmap->getCost(umx, umy);
        if (cost == nav2_costmap_2d::NO_INFORMATION)
            return false;

        return cost < corridor_lethal_cost_threshold_;
    }

    bool MyBSplineSmoother::isSegmentCollisionFree(
        double x0,
        double y0,
        double x1,
        double y1
    ) const {

        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double length = std::hypot(dx, dy);
        const double step = std::max(corridor_collision_check_resolution_, 1e-3);
        const int steps = std::max(1, static_cast<int>(std::ceil(length / step)));

        for (int i = 0; i <= steps; ++ i) {

            const double ratio = static_cast<double>(i) / static_cast<double>(steps);
            const double x = x0 + ratio * dx;
            const double y = y0 + ratio * dy;

            unsigned int mx = 0;
            unsigned int my = 0;
            if (!worldToMap(x, y, mx, my))
                return false;
            if (!isCellFree(static_cast<int>(mx), static_cast<int>(my)))
                return false;
        }

        return true;
    }

    std::array<double, 4> MyBSplineSmoother::cubicBSplineBasis(double u) const {

        u = std::clamp(u, 0.0, 1.0);
        const double u2 = u * u;
        const double u3 = u2 * u;

        return {
            (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0,
            (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0,
            (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0,
            u3 / 6.0
        };
    }

    std::array<double, 4> MyBSplineSmoother::cubicBSplineBasisDerivative(
        double u
    ) const {

        u = std::clamp(u, 0.0, 1.0);
        const double u2 = u * u;

        return {
            (-3.0 + 6.0 * u - 3.0 * u2) / 6.0,
            (-12.0 * u + 9.0 * u2) / 6.0,
            (3.0 + 6.0 * u - 9.0 * u2) / 6.0,
            u2 / 2.0
        };
    }

    Eigen::Vector2d MyBSplineSmoother::evaluateSplinePoint(
        const std::vector<double> &p_x,
        const std::vector<double> &p_y,
        int segment_index,
        double u
    ) const {

        if (segment_index < 0 ||
            segment_index + 3 >= static_cast<int>(p_x.size()) ||
            p_x.size() != p_y.size())
            return Eigen::Vector2d::Zero();

        const auto basis = cubicBSplineBasis(u);

        double x = 0.0;
        double y = 0.0;
        for (int i = 0; i < 4; ++ i) {

            x += basis[i] * p_x[segment_index + i];
            y += basis[i] * p_y[segment_index + i];
        }

        return Eigen::Vector2d(x, y);
    }

    SplineOvershootReport MyBSplineSmoother::checkSplineOvershootByExtrema(
        const std::vector<double> &p_x,
        const std::vector<double> &p_y,
        const std::vector<GridBox> &segment_boxes
    ) const {

        SplineOvershootReport report;

        if (!costmap_sub_ || p_x.size() != p_y.size() || p_x.size() < 4) {

            report.ok = false;
            return report;
        }

        auto costmap = costmap_sub_->getCostmap();
        if (!costmap) {

            report.ok = false;
            return report;
        }

        const int segment_count = static_cast<int>(p_x.size()) - 3;
        if (static_cast<int>(segment_boxes.size()) < segment_count) {

            report.ok = false;
            return report;
        }

        const double resolution = std::max(costmap->getResolution(), 1e-3);

        for (int seg = 0; seg < segment_count; ++ seg) {

            std::vector<double> candidates = {0.0, 1.0};

            const auto x_extrema = findSplineExtremaU(p_x, seg);
            candidates.insert(candidates.end(), x_extrema.begin(), x_extrema.end());
            const auto y_extrema = findSplineExtremaU(p_y, seg);
            candidates.insert(candidates.end(), y_extrema.begin(), y_extrema.end());

            const GridBox box = segment_boxes[seg];

            double min_x = 0.0;
            double min_y = 0.0;
            double max_x = 0.0;
            double max_y = 0.0;

            costmap->mapToWorld(
                static_cast<unsigned int>(box.min_mx),
                static_cast<unsigned int>(box.min_my),
                min_x, min_y
            );
            costmap->mapToWorld(
                static_cast<unsigned int>(box.max_mx),
                static_cast<unsigned int>(box.max_my),
                max_x, max_y
            );
            const double lower_x = std::min(min_x, max_x) - 0.5 * resolution;
            const double upper_x = std::max(min_x, max_x) + 0.5 * resolution;
            const double lower_y = std::min(min_y, max_y) - 0.5 * resolution;
            const double upper_y = std::max(min_y, max_y) + 0.5 * resolution;

            for (const double u : candidates) {

                const Eigen::Vector2d point =
                    evaluateSplinePoint(p_x, p_y, seg, u);

                double dx = 0.0;
                double dy = 0.0;

                if (point.x() < lower_x)
                    dx = point.x() - lower_x;
                else if (point.x() > upper_x)
                    dx = point.x() - upper_x;

                if (point.y() < lower_y)
                    dy = point.y() - lower_y;
                else if (point.y() > upper_y)
                    dy = point.y() - upper_y;

                if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6) {

                    report.ok = false;
                    report.overshoots.push_back({
                        seg,
                        u,
                        point.x(),
                        point.y(),
                        dx,
                        dy
                    });
                }
            }
        }
        return report;
    }

    std::vector<double> MyBSplineSmoother::findSplineExtremaU(
        const std::vector<double> &p,
        int segment_index
    ) const {

        std::vector<double> roots;

        if (segment_index < 0 ||
            segment_index + 3 >= static_cast<int>(p.size())) {
            return roots;
        }

        const double p0 = p[segment_index];
        const double p1 = p[segment_index + 1];
        const double p2 = p[segment_index + 2];
        const double p3 = p[segment_index + 3];

        // dp/du = a*u^2 + b*u + c
        const double a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
        const double b = p0 - 2.0 * p1 + p2;
        const double c = -0.5 * p0 + 0.5 * p2;

        constexpr double eps = 1e-9; // 作0判断

        auto addRoot = [&](double u) {

            if (u <= eps || u >= 1.0 - eps)
                return;

            for (double existing : roots)
                if (std::abs(existing - u) < 1e-6)
                    return;

            roots.push_back(u);
        };

        if (std::abs(a) < eps) {

            if (std::abs(b) > eps)
                addRoot(-c / b);
            return roots;
        }

        const double delta = b * b - 4.0 * a * c;
        if (delta < -eps) return roots;

        const double sqrt_d = std::sqrt(std::max(0.0, delta));

        if (std::abs(sqrt_d) < eps) {

            addRoot(-b / (2.0 * a));
            return roots;
        }

        // x = -2c / (b + sgn(b) * sqrt(b^2 − 4ac))
        auto sgn = [&](double x) {

            if (x == 0.0) return 0.0;
            return (x > 0 ? 1.0 : -1.0);
        };
        const double q = -0.5 * (b + sgn(b) * sqrt_d);

        if (std::abs(q) < eps) {

            addRoot(-b / (2.0 * a));
            return roots;
        }

        addRoot(q / a);
        addRoot(c / q);

        return roots;
    }

    /**
     * @brief 在轨迹超调位置生成约束并添加到额外约束列表中
     * @param report 超调报告
     * @param segment_boxes 轨迹段的网格框列表
     * @param extra_constraints 额外约束列表
     * @return void
     */
    void MyBSplineSmoother::appendOvershootConstraints(
        const SplineOvershootReport &report,
        const std::vector<GridBox> &segment_boxes,
        std::vector<SplinePointConstraint> &extra_constraints
    ) const {

        if (!costmap_sub_)
            return;

        auto costmap = costmap_sub_->getCostmap();
        if (!costmap)
            return;

        const double resolution = std::max(costmap->getResolution(), 1e-3);
        int added_count = 0;
        const int max_added_count = std::max(0, max_overshoot_constraints_per_iter_);

        for (const auto &overshoot : report.overshoots) {

            if (added_count >= max_added_count)
                break;

            if (overshoot.segment_index < 0 ||
                overshoot.segment_index >= static_cast<int>(segment_boxes.size()))
                continue;

            bool duplicated = false;
            for (const auto &existing : extra_constraints) { // 防止多重相同约束

                if (existing.segment_index == overshoot.segment_index &&
                    std::abs(existing.u - overshoot.u) < 1e-3) {

                    duplicated = true;
                    break;
                }
            }

            if (duplicated)
                continue;

            const GridBox &box = segment_boxes[overshoot.segment_index];

            double min_x = 0.0;
            double min_y = 0.0;
            double max_x = 0.0;
            double max_y = 0.0;

            costmap->mapToWorld(
                static_cast<unsigned int>(box.min_mx),
                static_cast<unsigned int>(box.min_my),
                min_x,
                min_y);

            costmap->mapToWorld(
                static_cast<unsigned int>(box.max_mx),
                static_cast<unsigned int>(box.max_my),
                max_x,
                max_y);

            SplinePointConstraint constraint;
            constraint.segment_index = overshoot.segment_index;
            constraint.u = overshoot.u;
            constraint.lower_x = std::min(min_x, max_x) - 0.5 * resolution;
            constraint.upper_x = std::max(min_x, max_x) + 0.5 * resolution;
            constraint.lower_y = std::min(min_y, max_y) - 0.5 * resolution;
            constraint.upper_y = std::max(min_y, max_y) + 0.5 * resolution;

            extra_constraints.push_back(constraint);
            ++ added_count;
        }
    }

    /**
     * @brief 使用 B-Spline 二次规划平滑路径
     * @param p_ref 原始参考路径的坐标序列 (x 或 y)
     * @param w_s 平滑权重
     * @param w_g 引导项权重
     * @param p_smooth 输出的平滑后的坐标序列
     * @return 成功返回true
     */
    bool MyBSplineSmoother::solveBSplineQP(
        const std::vector<double> &p_ref_x,
        const std::vector<double> &p_ref_y,
        double w_s,
        double w_g,
        std::vector<double> &p_smooth_x,
        std::vector<double> &p_smooth_y
    ) {

        std::vector<double> dt_segment;
        computeTimeAllocation(p_ref_x, p_ref_y, dt_segment);
        const CorridorBounds bounds = buildCorridorBounds(p_ref_x, p_ref_y);
        const std::vector<GridBox> segment_boxes =
            buildSegmentCorridorBoxes(p_ref_x, p_ref_y);
        publishCorridorMarkers(bounds, path_frame_id_);
        std::vector<SplinePointConstraint> extra_constraints;

        const int max_iterations = std::max(1, max_outer_iterations_);
        for (int iter = 0; iter < max_iterations; ++ iter) {

            if (!solveBSplineQPOnce(
                    p_ref_x,
                    p_ref_y,
                    dt_segment,
                    bounds,
                    extra_constraints,
                    w_s,
                    w_g,
                    p_smooth_x,
                    p_smooth_y))
                return false;

            const DynamicReport dynamic_report =
                checkDynamicFeasibility(
                    p_smooth_x,
                    p_smooth_y,
                    dt_segment
                );
            const CorridorReport corridor_report =
                checkCorridorFeasibility(
                    p_smooth_x,
                    p_smooth_y,
                    bounds
                );
            SplineOvershootReport overshoot_report;
            const int segment_count = static_cast<int>(p_smooth_x.size()) - 3;
            if (segment_count > 0 &&
                static_cast<int>(segment_boxes.size()) >= segment_count) {

                overshoot_report =
                    checkSplineOvershootByExtrema(
                        p_smooth_x,
                        p_smooth_y,
                        segment_boxes
                    );
            }

            RCLCPP_DEBUG(
                node_->get_logger(),
                "B-Spline check iter=%d dyn_ok=%d corridor_ok=%d overshoot_ok=%d max_v=%.3f max_a=%.3f max_j=%.3f point_vios=%zu segment_vios=%zu overshoots=%zu extra_constraints=%zu",
                iter,
                dynamic_report.ok,
                corridor_report.ok,
                overshoot_report.ok,
                dynamic_report.max_vel,
                dynamic_report.max_acc,
                dynamic_report.max_jerk,
                corridor_report.point_vios.size(),
                corridor_report.segment_vios.size(),
                overshoot_report.overshoots.size(),
                extra_constraints.size());

            if (dynamic_report.ok && corridor_report.ok && overshoot_report.ok)
                return true;

            if (!dynamic_report.ok && iter + 1 < max_iterations)
                inflateTimeAllocation(dynamic_report, dt_segment);

            if (!overshoot_report.ok && iter + 1 < max_iterations)
                appendOvershootConstraints(
                    overshoot_report,
                    segment_boxes,
                    extra_constraints
                );
        }

        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "B-Spline constraints remain violated after %d iterations",
            max_iterations);
        return !p_smooth_x.empty() && !p_smooth_y.empty();
    }

    bool MyBSplineSmoother::solveBSplineQPOnce(
        const std::vector<double> &p_ref_x,
        const std::vector<double> &p_ref_y,
        const std::vector<double> &dt_segment,
        const CorridorBounds &bounds,
        const std::vector<SplinePointConstraint> &extra_constraints,
        double w_s,
        double w_g,
        std::vector<double> &p_smooth_x,
        std::vector<double> &p_smooth_y
    ) {

        int n = p_ref_x.size();
        if (n < 4 || static_cast<int>(dt_segment.size()) != n - 1) return false;
        if (bounds.lower_x.size() != p_ref_x.size() ||
            bounds.upper_x.size() != p_ref_x.size() ||
            bounds.lower_y.size() != p_ref_x.size() ||
            bounds.upper_y.size() != p_ref_x.size())
            return false;

        Eigen::SparseMatrix<double> H(n, n);
        std::vector<Eigen::Triplet<double>> triplets;
        for (int i = 0; i < n - 3; ++ i) {

            const double dt = std::max(
                (dt_segment[i] + dt_segment[i + 1] + dt_segment[i + 2]) / 3.0,
                min_dt_
            );
            const double time_w = w_s / std::pow(dt, 3);
            for (int r = 0; r < 4; ++ r) {

                for(int c = 0; c < 4; ++ c) {

                    triplets.push_back(Eigen::Triplet<double>(
                        i + r,
                        i + c,
                        time_w * Q_data[r][c]));
                }
            }
        } // w_s H_s

        // 引导项
        for (int i = 0; i < n; ++ i) {

            triplets.push_back(Eigen::Triplet<double>(i, i, w_g));
        } // w_g I
        H.setFromTriplets(triplets.begin(), triplets.end()); // H_{QP}

        Eigen::VectorXd f_x(n), f_y(n);
        for (int i = 0; i < n; ++ i) {

            f_x(i) = -w_g * p_ref_x[i];
            f_y(i) = -w_g * p_ref_y[i];
        } // f_{QP}

        const int constraint_count =
            n + static_cast<int>(extra_constraints.size());

        // 边界约束
        Eigen::VectorXd l_x(constraint_count), u_x(constraint_count);
        Eigen::VectorXd l_y(constraint_count), u_y(constraint_count);
        for (int i = 0; i < n; ++ i) {

            l_x(i) = bounds.lower_x[i];
            u_x(i) = bounds.upper_x[i];
            l_y(i) = bounds.lower_y[i];
            u_y(i) = bounds.upper_y[i];
        }

        for (std::size_t k = 0; k < extra_constraints.size(); ++ k) {

            const auto &constraint = extra_constraints[k];
            const int row = n + static_cast<int>(k);
            l_x(row) = constraint.lower_x;
            u_x(row) = constraint.upper_x;
            l_y(row) = constraint.lower_y;
            u_y(row) = constraint.upper_y;
        }

        // OSQP求解
        OSQPSettings *settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
        OSQPData *data = (OSQPData*)c_malloc(sizeof(OSQPData));
        Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int> H_csc = H.triangularView<Eigen::Upper>(); // csc形式
        Eigen::SparseMatrix<c_float, Eigen::ColMajor, c_int> A_csc(constraint_count, n);
        std::vector<Eigen::Triplet<c_float>> a_triplets;
        a_triplets.reserve(n + 4 * extra_constraints.size());
        for (int i = 0; i < n; ++ i)
            a_triplets.push_back(Eigen::Triplet<c_float>(i, i, 1.0));

        for (std::size_t k = 0; k < extra_constraints.size(); ++ k) {

            const auto &constraint = extra_constraints[k];
            const int segment_index = constraint.segment_index;
            if (segment_index < 0 || segment_index + 3 >= n)
                continue;

            const int row = n + static_cast<int>(k);
            const auto basis = cubicBSplineBasis(constraint.u);
            for (int j = 0; j < 4; ++ j) {

                a_triplets.push_back(Eigen::Triplet<c_float>(
                    row,
                    segment_index + j,
                    static_cast<c_float>(basis[j])));
            }
        }
        A_csc.setFromTriplets(a_triplets.begin(), a_triplets.end());
        A_csc.makeCompressed();
        H_csc.makeCompressed();

        data->n = n; // 变量
        data->m = constraint_count; // 约束
        data->P = csc_matrix( // csc矩阵信息
            n, n,
            H_csc.nonZeros(),
            H_csc.valuePtr(),
            (c_int*)H_csc.innerIndexPtr(),
            (c_int*)H_csc.outerIndexPtr()
        );
        data->q = f_x.data(); // 一次项
        data->A = csc_matrix( // 约束矩阵
            constraint_count, n,
            A_csc.nonZeros(),
            A_csc.valuePtr(),
            (c_int*)A_csc.innerIndexPtr(),
            (c_int*)A_csc.outerIndexPtr()
        );

        // 约束向量
        data->l = l_x.data();
        data->u = u_x.data();

        // 开始求解
        osqp_set_default_settings(settings);
        settings->warm_start = 1;
        settings->verbose = 0; // 生产环境关闭日志

        OSQPWorkspace *work = nullptr;
        c_int status = osqp_setup(&work, data, settings);
        bool success = false;
        if (status == 0 && work) {

            osqp_solve(work);
            if (work->info->status_val >= 0 && work->solution) {

                p_smooth_x.resize(n);
                for (int i = 0; i < n; ++ i) {

                    p_smooth_x[i] = work->solution->x[i];
                }
            }
            osqp_update_lin_cost(work, f_y.data());
            osqp_update_bounds(work, l_y.data(), u_y.data());
            osqp_solve(work);
            if (work->info->status_val >= 0 && work->solution) {

                p_smooth_y.resize(n);
                for (int i = 0; i < n; ++ i) {

                    p_smooth_y[i] = work->solution->x[i];
                }
                success = true;
            }
        } else {

            RCLCPP_INFO(node_->get_logger(), "B-Spline: OSQP初始化失败");
        }

        // 释放
        if (work) osqp_cleanup(work);
        if (data) {

            if (data->P) c_free(data->P);
            if (data->A) c_free(data->A);
            c_free(data);
        }
        if (settings) c_free(settings);

        return success;
    }
} // my_bspline_smoother

PLUGINLIB_EXPORT_CLASS(my_bspline_smoother::MyBSplineSmoother, nav2_core::Smoother)
