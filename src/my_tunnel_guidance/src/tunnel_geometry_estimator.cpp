#include "my_tunnel_guidance/tunnel_geometry_estimator.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <limits>

namespace my_tunnel_guidance {

namespace {

/**
 * @brief 计算点集在给定轴上的投影，并返回中位数
 * @param points 点集
 * @param axis 投影轴
 * @return 中位数投影值
 */
double medianProjection(
    const std::vector<Eigen::Vector3d> & points,
    const Eigen::Vector3d & axis
) {

    std::vector<double> projections;
    projections.reserve(points.size());
    for (const auto & point : points) {

        projections.push_back(axis.dot(point));
    }
    const auto middle = projections.begin() + static_cast<std::ptrdiff_t>(projections.size() / 2);
    std::nth_element(projections.begin(), middle, projections.end());
    return projections[static_cast<std::size_t>(middle - projections.begin())];
}

/**
 * @brief 计算平面权重
 * @param plane 平面估计
 * @return 平面权重
 */
double planeWeight(const PlaneEstimate & plane) {

    const double rmse = std::max(plane.rmse, 1e-3);
    const double flatness = std::clamp(
        1.0 - plane.eigenvalues.x() / std::max(plane.eigenvalues.z(), 1e-9),
        0.05, 1.0);
    return static_cast<double>(plane.point_count) * flatness / rmse;
}
}  // namespace

/**
 * @brief 使用 PCA 拟合平面
 * @param points 点集
 * @return 平面估计
 */
PlaneEstimate TunnelGeometryEstimator::fitPlanePca(
    const std::vector<Eigen::Vector3d> & points
) const {

    PlaneEstimate plane;
    if (points.size() < 3) {

        return plane;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & point : points) {

        // 累加点云的质心
        centroid += point;
    }
    centroid /= static_cast<double>(points.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto & point : points) {

        // 计算点到质心的差
        const Eigen::Vector3d diff = point - centroid;
        covariance += diff * diff.transpose();
    }
    covariance /= static_cast<double>(points.size());

    // 计算协方差矩阵的特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {

        return plane;
    }

    plane.normal = solver.eigenvectors().col(0).normalized();
    plane.centroid = centroid;
    plane.eigenvalues = solver.eigenvalues();
    plane.offset = -plane.normal.dot(centroid);

    double squared_error_sum = 0.0;
    for (const auto & point : points) {

        // 计算点到平面的误差
        const double error = plane.normal.dot(point) + plane.offset;
        squared_error_sum += error * error;
    }
    plane.rmse = std::sqrt(squared_error_sum / static_cast<double>(points.size()));
    plane.point_count = points.size();
    plane.valid = std::isfinite(plane.rmse) &&
        std::isfinite(plane.normal.norm()) &&
        plane.eigenvalues.allFinite();
    return plane;
}

/**
 * @brief 求解隧道方向
 * @param left 左平面
 * @param right 右平面
 * @param ground 地面平面
 * @param tangent 隧道方向
 * @param confidence 置信度
 * @return bool 是否成功求解
 */
bool TunnelGeometryEstimator::solveTunnelDirection(
    const PlaneEstimate & left,
    const PlaneEstimate & right,
    const PlaneEstimate & ground,
    Eigen::Vector3d & tangent,
    double & confidence
) const {

    const double w_left = planeWeight(left);
    const double w_right = planeWeight(right);
    const double w_ground = planeWeight(ground);

    Eigen::Matrix3d M =
        w_left * left.normal * left.normal.transpose() +
        w_right * right.normal * right.normal.transpose() +
        w_ground * ground.normal * ground.normal.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M);
    if (solver.info() != Eigen::Success) {

        return false;
    }

    // 特征值最小的
    tangent = solver.eigenvectors().col(0).normalized();
    const double adjacent_eigenvalue = std::max(solver.eigenvalues()[1], 1e-9);
    confidence = (solver.eigenvalues()[1] - solver.eigenvalues()[0]) /
        adjacent_eigenvalue;
    confidence = std::clamp(confidence, 0.0, 1.0);
    if (confidence < params_.min_direction_eigen_gap) {

        return false;
    }

    if (tangent.x() < 0.0) {

        tangent = -tangent;
    }
    return std::isfinite(tangent.norm()) && tangent.norm() > 0.5;
}

/**
 * @brief 求解隧道中心点
 * @param left_points 左点集
 * @param right_points 右点集
 * @param ground 地面平面
 * @param tangent 隧道方向
 * @param lateral 隧道横向方向
 * @param center 中心点
 * @param width 隧道宽度
 * @return bool 是否成功求解
 */
bool TunnelGeometryEstimator::solveCenterPoint(
    const std::vector<Eigen::Vector3d> & left_points,
    const std::vector<Eigen::Vector3d> & right_points,
    const PlaneEstimate & ground,
    const Eigen::Vector3d & tangent,
    const Eigen::Vector3d & lateral,
    Eigen::Vector3d & center,
    double & width
) const {

    // 计算左右点集在横向方向上的投影，方便求解中心点
    const double s_left = medianProjection(left_points, lateral);
    const double s_right = medianProjection(right_points, lateral);
    const double s_center = 0.5 * (s_left + s_right);
    width = std::abs(s_left - s_right);

    Eigen::Matrix3d A;
    A.row(0) = lateral.transpose();
    A.row(1) = ground.normal.transpose();
    A.row(2) = tangent.transpose();

    Eigen::Vector3d b;
    b.x() = s_center;
    b.y() = -ground.offset;
    b.z() = tangent.dot(Eigen::Vector3d::Zero());  // p_ref = base_link origin

    center = A.colPivHouseholderQr().solve(b);
    const double residual = (A * center - b).norm();
    return std::isfinite(center.norm()) && residual < 0.2;
}

/**
 * @brief 估计隧道框架
 * @param left_points 左点集
 * @param right_points 右点集
 * @param ground_points 地面点集
 * @return TunnelFrameEstimate 隧道框架估计结果
 */
TunnelFrameEstimate TunnelGeometryEstimator::estimateFrame(
    const std::vector<Eigen::Vector3d> & left_points,
    const std::vector<Eigen::Vector3d> & right_points,
    const std::vector<Eigen::Vector3d> & ground_points
) const {

    TunnelFrameEstimate frame;
    if (left_points.size() < static_cast<std::size_t>(params_.min_wall_points) ||
        right_points.size() < static_cast<std::size_t>(params_.min_wall_points) ||
        ground_points.size() < static_cast<std::size_t>(params_.min_ground_points))
        return frame;

    const PlaneEstimate left = fitPlanePca(left_points);
    const PlaneEstimate right = fitPlanePca(right_points);
    PlaneEstimate ground = fitPlanePca(ground_points);

    if (!left.valid || left.rmse > params_.plane_max_rmse ||
        !right.valid || right.rmse > params_.plane_max_rmse ||
        !ground.valid || ground.rmse > params_.plane_max_rmse)
        return frame;

    // 地面法向量统一朝上；墙体法向量应基本水平。
    if (ground.normal.z() < 0.0) {

        ground.normal = -ground.normal;
        ground.offset = -ground.offset;
    }
    if (ground.normal.z() < 0.5 ||
        std::abs(left.normal.z()) > 0.5 ||
        std::abs(right.normal.z()) > 0.5)
        return frame;

    Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
    double confidence = 0.0;
    if (!solveTunnelDirection(left, right, ground, tangent, confidence)) {

        return frame;
    }

    frame.up = ground.normal.normalized();
    frame.lateral = frame.up.cross(tangent).normalized();
    if (frame.lateral.y() < 0.0) {

        frame.lateral = -frame.lateral;
    }
    frame.tangent = frame.lateral.cross(frame.up).normalized();
    if (frame.tangent.x() < 0.0) {

        frame.tangent = -frame.tangent;
        frame.lateral = frame.up.cross(frame.tangent).normalized();
    }

    double width = 0.0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    if (
        !solveCenterPoint(
            left_points, right_points, ground, frame.tangent, frame.lateral, center, width
        )
    ) {

        return frame;
    }

    if (width < params_.min_tunnel_width || width > params_.max_tunnel_width) {

        return frame;
    }

    // 局部坐标系应基本正交。
    const double dot_products =
        std::abs(frame.tangent.dot(frame.lateral)) +
        std::abs(frame.tangent.dot(frame.up)) +
        std::abs(frame.lateral.dot(frame.up));
    if (dot_products > 0.2) {

        return frame;
    }

    frame.center = center;
    frame.width = width;
    frame.confidence = confidence;
    frame.valid = true;
    return frame;
}

/**
 * @brief 构建隧道中心线上的点集
 * @param frame 隧道框架估计结果
 * @return CenterlineEstimate 隧道中心线上的点集
 */
CenterlineEstimate TunnelGeometryEstimator::buildStraightCenterline(
    const TunnelFrameEstimate & frame
) const {

    CenterlineEstimate centerline;
    centerline.local_frame = frame;
    if (!frame.valid || params_.centerline_length <= 0.0 ||
        params_.centerline_point_spacing <= 0.0
    ) {

        return centerline;
    }

    const int point_count = static_cast<int>(
        params_.centerline_length / params_.centerline_point_spacing) + 1;
    centerline.points.reserve(static_cast<std::size_t>(point_count));
    centerline.tangents.reserve(static_cast<std::size_t>(point_count));
    for (int i = 0; i < point_count; ++ i) {

        const double u = static_cast<double>(i) * params_.centerline_point_spacing;
        centerline.points.push_back(frame.center + u * frame.tangent);
        centerline.tangents.push_back(frame.tangent);
    }
    centerline.valid = true;
    return centerline;
}

}  // namespace my_tunnel_guidance
