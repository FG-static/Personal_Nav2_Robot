#include "my_teb_controller/teb_graph_optimizer.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

namespace my_teb_controller {

/**
 * @brief 优化函数，构建图并求解
 * @param trajectory 优化后的轨迹
 * @param reference_trajectory 参考轨迹
 * @param goal_pose 目标位姿
 * @param obstacles 障碍物
 * @param config 配置
 * @param verbose 是否打印调试信息
 * @return 是否成功
 */
bool TebGraphOptimizer::optimize(
    std::vector<TimedPose> &trajectory,
    const std::vector<TimedPose> &reference_trajectory,
    const PoseSE2 &goal_pose,
    const ObstacleSamples &obstacles,
    const TebConfig &config,
    bool verbose) {

    if (trajectory.size() < 2 || reference_trajectory.size() != trajectory.size()) {
        return false;
    }

    if (!initializeSolver(verbose)) { // 初始化求解器
        return false;
    }

    // 路径点+约束边建图
    if (!buildGraph(trajectory, reference_trajectory, goal_pose, obstacles, config)) {
        return false;
    }

    optimizer_.initializeOptimization();
    const int iterations = optimizer_.optimize(config.no_iterations);
    if (iterations <= 0) {
        clear();
        return false;
    }

    copyBack(trajectory);
    clear();
    return true;
}

/**
 * @brief 初始化求解器
 * @param verbose 是否打印调试信息
 * @return 是否成功
 */
bool TebGraphOptimizer::initializeSolver(bool verbose) {

    clear();

    // 构建Hessian矩阵与线性求解器
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto linear_solver = std::make_unique<LinearSolverType>();
    auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    optimizer_.setAlgorithm(algorithm);
    optimizer_.setVerbose(verbose);
    return optimizer_.solver() != nullptr;
}

/**
 * @brief 构建图
 * @param trajectory 优化后的轨迹
 * @param reference_trajectory 参考轨迹
 * @param goal_pose 目标位姿
 * @param obstacles 障碍物
 * @param config 配置
 * @return 是否成功
 */
bool TebGraphOptimizer::buildGraph(
    const std::vector<TimedPose> &trajectory,
    const std::vector<TimedPose> &reference_trajectory,
    const PoseSE2 &goal_pose,
    const ObstacleSamples &obstacles,
    const TebConfig &config) {

    if (trajectory.size() < 2 || trajectory.size() != reference_trajectory.size()) {
        return false;
    }

    int next_vertex_id = 0;
    pose_vertices_.reserve(trajectory.size());
    dt_vertices_.reserve(trajectory.size() - 1);

    for (size_t index = 0; index < trajectory.size(); ++ index) {

        // 开始建图
        auto *pose_vertex = new VertexPose();
        pose_vertex->setId(next_vertex_id++); // 分配顶点编号
        pose_vertex->setEstimate(trajectory[index].pose); // 作为优化起点，使用上一帧的轨迹
        pose_vertex->setFixed(index == 0); // 第一个顶点固定
        optimizer_.addVertex(pose_vertex); // 加入图的顶点集合，参与Hessian构建
        pose_vertices_.push_back(pose_vertex);

        if (index == 0 || index + 1 == trajectory.size()) {
            continue;
        }

        auto *anchor_edge = new EdgePathAnchor(); // 原始路径约束边
        anchor_edge->setVertex(0, pose_vertex); // 设置待优化路径点集
        anchor_edge->setMeasurement(reference_trajectory[index].pose); // 设置参考（原始路径）
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
        information(0, 0) = config.weight_smoothness;
        information(1, 1) = config.weight_smoothness;
        information(2, 2) = 0.1 * config.weight_smoothness;
        anchor_edge->setInformation(information); // 权重
        optimizer_.addEdge(anchor_edge);
    }

    for (size_t index = 0; index + 1 < trajectory.size(); ++ index) {

        auto *dt_vertex = new VertexTimeDiff();
        dt_vertex->setId(next_vertex_id++);
        dt_vertex->setEstimate(std::max(trajectory[index].dt, 1e-3));
        optimizer_.addVertex(dt_vertex);
        dt_vertices_.push_back(dt_vertex);

        auto *time_edge = new EdgeTimeOptimal();
        time_edge->setVertex(0, dt_vertex);
        time_edge->setMeasurement(0.0);
        time_edge->setInformation(Eigen::Matrix<double, 1, 1>::Constant(config.weight_optimaltime));
        optimizer_.addEdge(time_edge);

        auto *path_edge = new EdgeShortestPath();
        path_edge->setVertex(0, pose_vertices_[index]);
        path_edge->setVertex(1, pose_vertices_[index + 1]);
        path_edge->setMeasurement(0.0);
        path_edge->setInformation(
            Eigen::Matrix<double, 1, 1>::Constant(std::max(config.weight_shortest_path, 1e-3)));
        optimizer_.addEdge(path_edge);

        auto *velocity_edge = new EdgeVelocityHolonomic(&config);
        velocity_edge->setVertex(0, pose_vertices_[index]);
        velocity_edge->setVertex(1, pose_vertices_[index + 1]);
        velocity_edge->setVertex(2, dt_vertex);
        velocity_edge->setMeasurement(Eigen::Vector3d::Zero());
        velocity_edge->setInformation(Eigen::Matrix3d::Identity() * config.weight_kinematics);
        optimizer_.addEdge(velocity_edge);
    }

    for (size_t index = 0; index + 2 < trajectory.size(); ++ index) {

        auto *acceleration_edge = new EdgeAccelerationHolonomic(&config);
        // 添加相邻两点与时间间隔点以便计算速度进行约束
        acceleration_edge->setVertex(0, pose_vertices_[index]);
        acceleration_edge->setVertex(1, pose_vertices_[index + 1]);
        acceleration_edge->setVertex(2, dt_vertices_[index]);
        acceleration_edge->setVertex(3, pose_vertices_[index + 2]);
        acceleration_edge->setVertex(4, dt_vertices_[index + 1]);
        acceleration_edge->setMeasurement(Eigen::Vector3d::Zero());
        acceleration_edge->setInformation(Eigen::Matrix3d::Identity() * config.weight_kinematics);
        optimizer_.addEdge(acceleration_edge);
    }

    if (!pose_vertices_.empty()) {
        auto *goal_edge = new EdgeGoalPose();
        goal_edge->setVertex(0, pose_vertices_.back());
        goal_edge->setMeasurement(goal_pose);
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
        information(0, 0) = config.weight_smoothness;
        information(1, 1) = config.weight_smoothness;
        information(2, 2) = config.weight_kinematics;
        goal_edge->setInformation(information);
        optimizer_.addEdge(goal_edge);
    }

    if (!obstacles.empty()) {
        for (size_t index = 1; index < pose_vertices_.size(); ++index) {
            const Eigen::Vector2d nearest = findNearestObstacle(pose_vertices_[index]->estimate(), obstacles);
            auto *obstacle_edge = new EdgeObstacle(nearest, config.min_obstacle_dist);
            obstacle_edge->setVertex(0, pose_vertices_[index]);
            obstacle_edge->setMeasurement(0.0);
            obstacle_edge->setInformation(
                Eigen::Matrix<double, 1, 1>::Constant(config.weight_obstacle));
            optimizer_.addEdge(obstacle_edge);
        }
    }

    return true;
}

/**
 * @brief 路径转换给外部调用轨迹数组
 * @param trajectory 轨迹数组
 */
void TebGraphOptimizer::copyBack(std::vector<TimedPose> &trajectory) const {

    for (size_t index = 0; index < pose_vertices_.size() && index < trajectory.size(); ++index) {
        trajectory[index].pose = pose_vertices_[index]->estimate();
        if (index < dt_vertices_.size()) {
            trajectory[index].dt = std::max(dt_vertices_[index]->estimate(), 1e-3);
        } else {
            trajectory[index].dt = 0.0;
        }
    }
}

void TebGraphOptimizer::clear() {

    optimizer_.clear();
    pose_vertices_.clear();
    dt_vertices_.clear();
}

Eigen::Vector2d TebGraphOptimizer::findNearestObstacle(
    const PoseSE2 &pose, const ObstacleSamples &obstacles) const {

    Eigen::Vector2d nearest = pose.position();
    double best_distance = std::numeric_limits<double>::infinity();

    for (const auto &obstacle : obstacles) {
        const double distance = (obstacle - pose.position()).squaredNorm();
        if (distance < best_distance) {
            best_distance = distance;
            nearest = obstacle;
        }
    }

    return nearest;
}

} // namespace my_teb_controller
