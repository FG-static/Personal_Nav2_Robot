#ifndef MY_PLANNING_METRICS__PATH_METRICS_HPP_
#define MY_PLANNING_METRICS__PATH_METRICS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace my_planning_metrics {

struct PathMetrics {
    std::size_t point_count = 0;
    double length_m = 0.0;
    double max_curvature_inv_m = 0.0;
    double min_lethal_obstacle_distance_m = -1.0;
};

class ObstacleDistanceField {
public:
    bool build(
        const nav2_costmap_2d::Costmap2D *costmap,
        unsigned char obstacle_threshold = nav2_costmap_2d::LETHAL_OBSTACLE
    ) {
        distances_m_.clear();
        size_x_ = 0;
        size_y_ = 0;
        resolution_ = 0.0;

        if (costmap == nullptr)
            return false;

        size_x_ = costmap->getSizeInCellsX();
        size_y_ = costmap->getSizeInCellsY();
        resolution_ = costmap->getResolution();
        origin_x_ = costmap->getOriginX();
        origin_y_ = costmap->getOriginY();
        if (size_x_ == 0 || size_y_ == 0 || resolution_ <= 0.0)
            return false;

        const std::size_t map_size = static_cast<std::size_t>(size_x_) * size_y_;
        std::vector<double> row_pass(map_size, kInfinity);
        std::vector<double> input(std::max(size_x_, size_y_), kInfinity);
        std::vector<double> output(std::max(size_x_, size_y_), kInfinity);

        bool has_obstacle = false;
        for (unsigned int my = 0; my < size_y_; ++my) {
            for (unsigned int mx = 0; mx < size_x_; ++mx) {
                const unsigned char cost = costmap->getCost(mx, my);
                input[mx] =
                    cost != nav2_costmap_2d::NO_INFORMATION &&
                    cost >= obstacle_threshold ? 0.0 : kInfinity;
                has_obstacle = has_obstacle || input[mx] == 0.0;
            }

            distanceTransform1D(input, size_x_, output);
            for (unsigned int mx = 0; mx < size_x_; ++mx)
                row_pass[toIndex(mx, my)] = output[mx];
        }

        if (!has_obstacle)
            return false;

        distances_m_.assign(map_size, kInfinity);
        for (unsigned int mx = 0; mx < size_x_; ++mx) {
            for (unsigned int my = 0; my < size_y_; ++my)
                input[my] = row_pass[toIndex(mx, my)];

            distanceTransform1D(input, size_y_, output);
            for (unsigned int my = 0; my < size_y_; ++my)
                distances_m_[toIndex(mx, my)] = std::sqrt(output[my]) * resolution_;
        }

        return true;
    }

    bool query(double wx, double wy, double &distance_m) const {
        if (distances_m_.empty() || wx < origin_x_ || wy < origin_y_)
            return false;

        const int mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
        const int my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
        if (mx < 0 || my < 0 ||
            mx >= static_cast<int>(size_x_) ||
            my >= static_cast<int>(size_y_))
            return false;

        distance_m = distances_m_[toIndex(
            static_cast<unsigned int>(mx),
            static_cast<unsigned int>(my))];
        return std::isfinite(distance_m);
    }

private:
    static constexpr double kInfinity = std::numeric_limits<double>::infinity();

    static void distanceTransform1D(
        const std::vector<double> &input,
        unsigned int count,
        std::vector<double> &output
    ) {
        std::fill(output.begin(), output.begin() + count, kInfinity);

        std::vector<int> sites(count);
        std::vector<double> boundaries(count + 1, kInfinity);
        int envelope_size = -1;

        for (unsigned int q = 0; q < count; ++q) {
            if (!std::isfinite(input[q]))
                continue;

            if (envelope_size < 0) {
                envelope_size = 0;
                sites[0] = static_cast<int>(q);
                boundaries[0] = -kInfinity;
                boundaries[1] = kInfinity;
                continue;
            }

            double boundary = 0.0;
            while (true) {
                const int previous = sites[envelope_size];
                const double q_value = static_cast<double>(q);
                boundary =
                    ((input[q] + q_value * q_value) -
                    (input[previous] +
                    static_cast<double>(previous) * static_cast<double>(previous))) /
                    (2.0 * (q_value - static_cast<double>(previous)));

                if (boundary > boundaries[envelope_size] || envelope_size == 0)
                    break;
                --envelope_size;
            }

            ++envelope_size;
            sites[envelope_size] = static_cast<int>(q);
            boundaries[envelope_size] = boundary;
            boundaries[envelope_size + 1] = kInfinity;
        }

        if (envelope_size < 0)
            return;

        int envelope_index = 0;
        for (unsigned int q = 0; q < count; ++q) {
            while (
                envelope_index < envelope_size &&
                boundaries[envelope_index + 1] < static_cast<double>(q))
                ++envelope_index;

            const double delta =
                static_cast<double>(q) - static_cast<double>(sites[envelope_index]);
            output[q] = delta * delta + input[sites[envelope_index]];
        }
    }

    std::size_t toIndex(unsigned int mx, unsigned int my) const {
        return static_cast<std::size_t>(my) * size_x_ + mx;
    }

    unsigned int size_x_ = 0;
    unsigned int size_y_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    std::vector<double> distances_m_;
};

inline PathMetrics evaluatePath(
    const nav_msgs::msg::Path &path,
    const ObstacleDistanceField *distance_field = nullptr
) {
    PathMetrics metrics;
    metrics.point_count = path.poses.size();
    if (path.poses.empty())
        return metrics;

    double minimum_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path.poses.size(); ++index) {
        const auto &position = path.poses[index].pose.position;

        if (distance_field != nullptr) {
            double obstacle_distance = 0.0;
            if (distance_field->query(position.x, position.y, obstacle_distance))
                minimum_distance = std::min(minimum_distance, obstacle_distance);
        }

        if (index == 0)
            continue;

        const auto &previous = path.poses[index - 1].pose.position;
        metrics.length_m += std::hypot(
            position.x - previous.x,
            position.y - previous.y);
    }

    if (std::isfinite(minimum_distance))
        metrics.min_lethal_obstacle_distance_m = minimum_distance;

    for (std::size_t index = 1; index + 1 < path.poses.size(); ++index) {
        const auto &p0 = path.poses[index - 1].pose.position;
        const auto &p1 = path.poses[index].pose.position;
        const auto &p2 = path.poses[index + 1].pose.position;

        const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
        const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
        const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
        const double denominator = a * b * c;
        if (denominator <= 1e-12)
            continue;

        const double cross =
            (p1.x - p0.x) * (p2.y - p0.y) -
            (p1.y - p0.y) * (p2.x - p0.x);
        metrics.max_curvature_inv_m = std::max(
            metrics.max_curvature_inv_m,
            2.0 * std::abs(cross) / denominator);
    }

    return metrics;
}

} // namespace my_planning_metrics

#endif // MY_PLANNING_METRICS__PATH_METRICS_HPP_
