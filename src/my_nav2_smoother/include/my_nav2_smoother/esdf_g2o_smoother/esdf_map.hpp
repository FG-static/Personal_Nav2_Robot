#ifndef MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_MAP
#define MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_MAP

#include <vector>

#include <Eigen/Dense>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace my_nav2_smoother {

struct EsdfQueryResult {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double distance = 0.0;
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
    bool valid = false;
};

class EsdfMap {

public:

    EsdfMap() = default;

    bool buildFromCostmap(
        const nav2_costmap_2d::Costmap2D *costmap,
        const nav_msgs::msg::Path &reference_path,
        double margin);

    bool query(double wx, double wy, EsdfQueryResult &result) const;
    bool isReady() const;
    void clear();

private:

    bool ready_ = false;
    unsigned int size_x_ = 0;
    unsigned int size_y_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    std::vector<double> distances_;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> gradients_;
};

} // namespace my_nav2_smoother

#endif // MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_MAP
