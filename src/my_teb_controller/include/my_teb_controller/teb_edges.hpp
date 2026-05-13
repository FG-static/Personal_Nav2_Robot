#ifndef MY_TEB_CONTROLLER__TEB_EDGES
#define MY_TEB_CONTROLLER__TEB_EDGES

#include <cmath>
#include <iostream>

#include "angles/angles.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_unary_edge.h"

#include "my_teb_controller/teb_types.hpp"
#include "my_teb_controller/teb_vertices.hpp"

namespace my_teb_controller {

// 障碍物约束边
class EdgeObstacle : public g2o::BaseUnaryEdge<1, double, VertexPose> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeObstacle(const Eigen::Vector2d &obstacle, double min_obstacle_dist)
    : obstacle_(obstacle), min_obstacle_dist_(min_obstacle_dist) {}

    void computeError() override {

        const auto *pose_vertex = static_cast<const VertexPose *>(_vertices[0]);
        const double distance = (pose_vertex->estimate().position() - obstacle_).norm();
        _error[0] = std::max(0.0, min_obstacle_dist_ - distance);
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }

private:

    Eigen::Vector2d obstacle_;
    double min_obstacle_dist_ = 0.0;
};

// 目标点约束边 类似启发式代价函数
class EdgeGoalPose : public g2o::BaseUnaryEdge<3, PoseSE2, VertexPose> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {

        const auto *pose_vertex = static_cast<const VertexPose *>(_vertices[0]);
        const PoseSE2 &estimate = pose_vertex->estimate();
        _error[0] = estimate.x - _measurement.x;
        _error[1] = estimate.y - _measurement.y;
        _error[2] = angles::normalize_angle(estimate.theta - _measurement.theta);
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }
};

// 原始路径点约束边 拉回，防止偏离过大
class EdgePathAnchor : public g2o::BaseUnaryEdge<3, PoseSE2, VertexPose> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {

        const auto *pose_vertex = static_cast<const VertexPose *>(_vertices[0]);
        const PoseSE2 &estimate = pose_vertex->estimate();
        _error[0] = estimate.x - _measurement.x;
        _error[1] = estimate.y - _measurement.y;
        _error[2] = angles::normalize_angle(estimate.theta - _measurement.theta);
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }
};

// 最短路径点约束边 相邻路径点之差为优化目标
class EdgeShortestPath : public g2o::BaseBinaryEdge<1, double, VertexPose, VertexPose> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {

        const auto *from_vertex = static_cast<const VertexPose *>(_vertices[0]);
        const auto *to_vertex = static_cast<const VertexPose *>(_vertices[1]);
        _error[0] =
            (to_vertex->estimate().position() - from_vertex->estimate().position()).norm();
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }
};

// 最优时间路径约束边
class EdgeTimeOptimal : public g2o::BaseUnaryEdge<1, double, VertexTimeDiff> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {

        const auto *dt_vertex = static_cast<const VertexTimeDiff *>(_vertices[0]);
        _error[0] = dt_vertex->estimate();
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }
};

// 速度限制约束边
class EdgeVelocityHolonomic : public g2o::BaseMultiEdge<3, Eigen::Vector3d> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeVelocityHolonomic(const TebConfig *config)
    : config_(config) {

        resize(3);
    }

    void computeError() override {

        const auto *from_vertex = static_cast<const VertexPose *>(_vertices[0]);
        const auto *to_vertex = static_cast<const VertexPose *>(_vertices[1]);
        const auto *dt_vertex = static_cast<const VertexTimeDiff *>(_vertices[2]);

        const PoseSE2 &from_pose = from_vertex->estimate();
        const PoseSE2 &to_pose = to_vertex->estimate();
        const double dt = std::max(dt_vertex->estimate(), 1e-3);

        // 车体系速度
        const double dx_world = to_pose.x - from_pose.x;
        const double dy_world = to_pose.y - from_pose.y;
        const double cos_theta = std::cos(from_pose.theta);
        const double sin_theta = std::sin(from_pose.theta);
        const double vx = (cos_theta * dx_world + sin_theta * dy_world) / dt;
        const double vy = (-sin_theta * dx_world + cos_theta * dy_world) / dt;
        const double omega = angles::normalize_angle(to_pose.theta - from_pose.theta) / dt;

        // 麦克纳姆轮速度约束
        _error[0] = symmetricBound(vx, config_->max_vel_x);
        _error[1] = symmetricBound(vy, config_->max_vel_y);
        _error[2] = symmetricBound(omega, config_->max_vel_theta);
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }

private:

    static double symmetricBound(double value, double bound) {

        if (value > bound) {
            return value - bound;
        }
        if (value < -bound) {
            return -bound - value;
        }
        return 0.0;
    }

    const TebConfig *config_ = nullptr;
};

// 加速度限制约束边
class EdgeAccelerationHolonomic : public g2o::BaseMultiEdge<3, Eigen::Vector3d> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgeAccelerationHolonomic(const TebConfig *config)
    : config_(config) {

        resize(5);
    }

    void computeError() override {

        const auto *pose_vertex_1 = static_cast<const VertexPose *>(_vertices[0]);
        const auto *pose_vertex_2 = static_cast<const VertexPose *>(_vertices[1]);
        const auto *dt_vertex_1 = static_cast<const VertexTimeDiff *>(_vertices[2]);
        const auto *pose_vertex_3 = static_cast<const VertexPose *>(_vertices[3]);
        const auto *dt_vertex_2 = static_cast<const VertexTimeDiff *>(_vertices[4]);

        const auto velocity_1 =
            computeVelocity(pose_vertex_1->estimate(), pose_vertex_2->estimate(), dt_vertex_1->estimate());
        const auto velocity_2 =
            computeVelocity(pose_vertex_2->estimate(), pose_vertex_3->estimate(), dt_vertex_2->estimate());

        const double avg_dt = std::max(0.5 * (dt_vertex_1->estimate() + dt_vertex_2->estimate()), 1e-3);
        const double ax = (velocity_2[0] - velocity_1[0]) / avg_dt;
        const double ay = (velocity_2[1] - velocity_1[1]) / avg_dt;
        const double alpha = (velocity_2[2] - velocity_1[2]) / avg_dt;

        _error[0] = symmetricBound(ax, config_->acc_lim_x);
        _error[1] = symmetricBound(ay, config_->acc_lim_y);
        _error[2] = symmetricBound(alpha, config_->acc_lim_theta);
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }

private:

    static Eigen::Vector3d computeVelocity(const PoseSE2 &from_pose, const PoseSE2 &to_pose, double dt) {

        dt = std::max(dt, 1e-3);
        const double dx_world = to_pose.x - from_pose.x;
        const double dy_world = to_pose.y - from_pose.y;
        const double cos_theta = std::cos(from_pose.theta);
        const double sin_theta = std::sin(from_pose.theta);

        return Eigen::Vector3d(
            (cos_theta * dx_world + sin_theta * dy_world) / dt,
            (-sin_theta * dx_world + cos_theta * dy_world) / dt,
            angles::normalize_angle(to_pose.theta - from_pose.theta) / dt);
    }

    static double symmetricBound(double value, double bound) {

        if (value > bound) {
            return value - bound;
        }
        if (value < -bound) {
            return -bound - value;
        }
        return 0.0;
    }

    const TebConfig *config_ = nullptr;
};

} // namespace my_teb_controller

#endif // MY_TEB_CONTROLLER__TEB_EDGES
