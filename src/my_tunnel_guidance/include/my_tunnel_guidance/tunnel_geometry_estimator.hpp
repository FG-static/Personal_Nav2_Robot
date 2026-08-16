#ifndef MY_TUNNEL_GUIDANCE__TUNNEL_GEOMETRY_ESTIMATOR_HPP_
#define MY_TUNNEL_GUIDANCE__TUNNEL_GEOMETRY_ESTIMATOR_HPP_

#include <Eigen/Dense>

#include <vector>

#include "my_tunnel_guidance/tunnel_types.hpp"

namespace my_tunnel_guidance {

struct TunnelGeometryParams {
    int min_ground_points = 100;
    int min_wall_points = 100;
    double plane_max_rmse = 0.08;
    double min_tunnel_width = 2.5;
    double max_tunnel_width = 5.0;
    double min_direction_eigen_gap = 0.05;
    double centerline_length = 8.0;
    double centerline_point_spacing = 0.2;
};

class TunnelGeometryEstimator {

public:

    explicit TunnelGeometryEstimator(const TunnelGeometryParams & params = {})
    : params_(params) {}

    PlaneEstimate fitPlanePca(const std::vector<Eigen::Vector3d> & points) const;

    bool solveTunnelDirection(
        const PlaneEstimate & left,
        const PlaneEstimate & right,
        const PlaneEstimate & ground,
        Eigen::Vector3d & tangent,
        double & confidence) const;

    bool solveCenterPoint(
        const std::vector<Eigen::Vector3d> & left_points,
        const std::vector<Eigen::Vector3d> & right_points,
        const PlaneEstimate & ground,
        const Eigen::Vector3d & tangent,
        const Eigen::Vector3d & lateral,
        Eigen::Vector3d & center,
        double & width) const;

    TunnelFrameEstimate estimateFrame(
        const std::vector<Eigen::Vector3d> & left_points,
        const std::vector<Eigen::Vector3d> & right_points,
        const std::vector<Eigen::Vector3d> & ground_points) const;

    CenterlineEstimate buildStraightCenterline(
        const TunnelFrameEstimate & frame) const;
private:

    TunnelGeometryParams params_;
};

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__TUNNEL_GEOMETRY_ESTIMATOR_HPP_
