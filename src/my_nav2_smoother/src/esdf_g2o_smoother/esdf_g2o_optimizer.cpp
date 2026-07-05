#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_edges.hpp"

namespace my_nav2_smoother {

bool EsdfG2oOptimizer::optimize(
    TrajectoryPoints &trajectory,
    const TrajectoryPoints &reference_trajectory,
    const EsdfMap &esdf_map,
    const EsdfG2oSmootherConfig &config) {

    if (trajectory.empty() || reference_trajectory.size() != trajectory.size() ||
        !esdf_map.isReady())
        return false;

    if (!initializeSolver(config.optimizer_verbose))
        return false;

    if (!buildGraph(trajectory, reference_trajectory, esdf_map, config)) {
        clear();
        return false;
    }

    if (point_vertices_.empty() || optimizer_.vertices().empty()) {
        clear();
        return false;
    }

    optimizer_.initializeOptimization();
    optimizer_.setVerbose(config.optimizer_verbose);
    const int iterations = optimizer_.optimize(std::max(1, config.no_iterations));
    if (iterations <= 0) {
        clear();
        return false;
    }

    copyBack(trajectory);
    clear();
    return true;
}

bool EsdfG2oOptimizer::initializeSolver(bool verbose) {

    clear();

    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto linear_solver = std::make_unique<LinearSolverType>();
    auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    optimizer_.setAlgorithm(algorithm);
    optimizer_.setVerbose(verbose);
    return optimizer_.solver() != nullptr;
}

bool EsdfG2oOptimizer::buildGraph(
    const TrajectoryPoints &trajectory,
    const TrajectoryPoints &reference_trajectory,
    const EsdfMap &esdf_map,
    const EsdfG2oSmootherConfig &config) {

    if (trajectory.empty() || reference_trajectory.size() != trajectory.size() ||
        !esdf_map.isReady())
        return false;

    int next_vertex_id = 0;
    point_vertices_.reserve(trajectory.size());

    // 设定路径点顶点
    for (std::size_t index = 0; index < trajectory.size(); ++ index) {

        auto *point_vertex = new VertexPathPoint();
        point_vertex->setId(next_vertex_id ++);
        point_vertex->setEstimate(Eigen::Vector2d(trajectory[index].x, trajectory[index].y));
        point_vertex->setFixed(trajectory[index].fixed);

        if (!optimizer_.addVertex(point_vertex))
            return false;

        point_vertices_.push_back(point_vertex);
    }

    // 开始设定边
    if (config.weight_anchor > 0.0) {

        for (std::size_t index = 0; index < point_vertices_.size(); ++ index) {

            if (trajectory[index].fixed)
                continue;

            auto *anchor_edge = new EdgeAnchor();
            anchor_edge->setVertex(0, point_vertices_[index]);
            anchor_edge->setMeasurement(
                Eigen::Vector2d(reference_trajectory[index].x, reference_trajectory[index].y));
            anchor_edge->setInformation(
                Eigen::Matrix2d::Identity() * std::max(config.weight_anchor, 1e-9));

            if (!optimizer_.addEdge(anchor_edge))
                return false;
        }
    }

    if (config.weight_obstacle > 0.0) {

        for (std::size_t index = 0; index < point_vertices_.size(); ++ index) {

            if (trajectory[index].fixed)
                continue;

            auto *obstacle_edge = new EdgeEsdfObstacle();
            obstacle_edge->setVertex(0, point_vertices_[index]);
            obstacle_edge->setEsdfMap(&esdf_map);
            obstacle_edge->setMeasurement(config.min_obstacle_dist);
            obstacle_edge->setInformation(
                Eigen::Matrix<double, 1, 1>::Constant(std::max(config.weight_obstacle, 1e-9)));

            if (!optimizer_.addEdge(obstacle_edge))
                return false;
        }
    }

    if (config.weight_smoothness > 0.0 && point_vertices_.size() >= 3) {

        for (std::size_t index = 1; index + 1 < point_vertices_.size(); ++ index) {

            if (trajectory[index].fixed)
                continue;

            auto *smoothness_edge = new EdgeSmoothness();
            smoothness_edge->setVertex(0, point_vertices_[index - 1]);
            smoothness_edge->setVertex(1, point_vertices_[index]);
            smoothness_edge->setVertex(2, point_vertices_[index + 1]);
            smoothness_edge->setMeasurement(Eigen::Vector2d::Zero());
            smoothness_edge->setInformation(
                Eigen::Matrix2d::Identity() * std::max(config.weight_smoothness, 1e-9));

            if (!optimizer_.addEdge(smoothness_edge))
                return false;
        }
    }

    return true;
}

void EsdfG2oOptimizer::copyBack(TrajectoryPoints &trajectory) const {

    const std::size_t count = std::min(point_vertices_.size(), trajectory.size());
    for (std::size_t index = 0; index < count; ++ index) {

        const Eigen::Vector2d estimate = point_vertices_[index]->estimate();
        trajectory[index].x = estimate.x();
        trajectory[index].y = estimate.y();
    }

    for (std::size_t index = 0; index + 1 < trajectory.size(); ++ index) {

        const double dx = trajectory[index + 1].x - trajectory[index].x;
        const double dy = trajectory[index + 1].y - trajectory[index].y;
        trajectory[index].yaw = std::atan2(dy, dx);
    }

    if (trajectory.size() > 1)
        trajectory.back().yaw = trajectory[trajectory.size() - 2].yaw;
}

void EsdfG2oOptimizer::clear() {

    optimizer_.clear();
    point_vertices_.clear();
}

} // namespace my_nav2_smoother
