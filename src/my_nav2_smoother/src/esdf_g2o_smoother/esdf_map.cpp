#include "my_nav2_smoother/esdf_g2o_smoother/esdf_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

#include "nav2_costmap_2d/cost_values.hpp"

namespace my_nav2_smoother {

bool EsdfMap::buildFromCostmap(
    const nav2_costmap_2d::Costmap2D *costmap,
    const nav_msgs::msg::Path &/*reference_path*/,
    double /*margin*/) {

    clear();
    if (costmap == nullptr)
        return false;

    size_x_ = costmap->getSizeInCellsX();
    size_y_ = costmap->getSizeInCellsY();
    resolution_ = costmap->getResolution();
    origin_x_ = costmap->getOriginX();
    origin_y_ = costmap->getOriginY();

    if (size_x_ == 0 || size_y_ == 0 || resolution_ <= 0.0)
        return false;

    cv::Mat binary(
        static_cast<int>(size_y_),
        static_cast<int>(size_x_),
        CV_8UC1,
        cv::Scalar(255));

    bool has_obstacle = false;
    for (unsigned int my = 0; my < size_y_; ++ my) {

        for (unsigned int mx = 0; mx < size_x_; ++ mx) {

            const unsigned char cost = costmap->getCost(mx, my);
            const bool occupied =
                cost != nav2_costmap_2d::NO_INFORMATION &&
                cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
            if (occupied) {

                binary.at<unsigned char>(static_cast<int>(my), static_cast<int>(mx)) = 0;
                has_obstacle = true;
            }
        }
    }

    const std::size_t map_size = static_cast<std::size_t>(size_x_) * size_y_;
    distances_.assign(map_size, 0.0);
    gradients_.assign(map_size, Eigen::Vector2d::Zero());

    if (!has_obstacle) {

        const double max_distance =
            std::hypot(static_cast<double>(size_x_), static_cast<double>(size_y_)) * resolution_;
        std::fill(distances_.begin(), distances_.end(), max_distance);
        ready_ = true;
        return true;
    }

    cv::Mat distance_cells;
    // 对图 binary 以 cv::DIST_L2 为距离类型，cv::DIST_MASK_PRECISE
    // 为距离计算精度来生成 ESDF 并将该地图存入 cv::Mat 类型变量
    // distance_cells 中
    cv::distanceTransform(binary, distance_cells, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // 索引采用一值除/模表示
    auto toIndex = [this](unsigned int mx, unsigned int my) {
        return static_cast<std::size_t>(my) * size_x_ + mx;
    };

    // 用索引将 ESDF 的值存入一级容器中
    for (unsigned int my = 0; my < size_y_; ++ my) {

        for (unsigned int mx = 0; mx < size_x_; ++ mx) {

            const float distance_cell =
                distance_cells.at<float>(static_cast<int>(my), static_cast<int>(mx));
            distances_[toIndex(mx, my)] = static_cast<double>(distance_cell) * resolution_;
        }
    }

    // 对应索引计算梯度
    for (unsigned int my = 0; my < size_y_; ++ my) {

        for (unsigned int mx = 0; mx < size_x_; ++ mx) {

            // 使用中心差分计算
            const unsigned int left = mx > 0 ? mx - 1 : mx;
            const unsigned int right = mx + 1 < size_x_ ? mx + 1 : mx;
            const unsigned int down = my > 0 ? my - 1 : my;
            const unsigned int up = my + 1 < size_y_ ? my + 1 : my;

            double grad_x = 0.0;
            if (right != left) {

                grad_x =
                    (distances_[toIndex(right, my)] - distances_[toIndex(left, my)]) /
                    (static_cast<double>(right - left) * resolution_);
            }

            double grad_y = 0.0;
            if (up != down) {

                grad_y =
                    (distances_[toIndex(mx, up)] - distances_[toIndex(mx, down)]) /
                    (static_cast<double>(up - down) * resolution_);
            }

            gradients_[toIndex(mx, my)] = Eigen::Vector2d(grad_x, grad_y);
        }
    }

    ready_ = true;
    return true;
}

bool EsdfMap::query(double wx, double wy, EsdfQueryResult &result) const {

    result = EsdfQueryResult{};
    if (!ready_ || resolution_ <= 0.0)
        return false;

    const int mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    const int my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(size_x_) ||
        my >= static_cast<int>(size_y_))
        return false;

    const std::size_t index =
        static_cast<std::size_t>(my) * size_x_ + static_cast<std::size_t>(mx);
    if (index >= distances_.size() || index >= gradients_.size())
        return false;

    result.distance = distances_[index];
    result.gradient = gradients_[index];
    result.valid = true;
    return true;
}

bool EsdfMap::isReady() const {

    return ready_;
}

void EsdfMap::clear() {

    ready_ = false;
    size_x_ = 0;
    size_y_ = 0;
    resolution_ = 0.0;
    origin_x_ = 0.0;
    origin_y_ = 0.0;
    distances_.clear();
    gradients_.clear();
}

} // namespace my_nav2_smoother
