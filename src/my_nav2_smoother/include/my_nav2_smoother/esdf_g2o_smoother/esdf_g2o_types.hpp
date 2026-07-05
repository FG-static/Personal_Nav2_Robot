#ifndef MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_TYPES
#define MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_TYPES

#include <vector>

#include <Eigen/Dense>

namespace my_nav2_smoother {

struct EsdfG2oSmootherConfig {

    double resample_resolution = 0.1;
    double esdf_margin = 1.0;
    double min_obstacle_dist = 0.25;
    double weight_obstacle = 50.0;
    double weight_anchor = 1.0;
    double weight_smoothness = 10.0;
    double weight_length = 1.0;
    int no_iterations = 10;
    bool optimizer_verbose = false;
};

struct TrajectoryPoint {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool fixed = false;
};

using TrajectoryPoints = std::vector<TrajectoryPoint, Eigen::aligned_allocator<TrajectoryPoint>>;

} // namespace my_nav2_smoother

#endif // MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_TYPES
