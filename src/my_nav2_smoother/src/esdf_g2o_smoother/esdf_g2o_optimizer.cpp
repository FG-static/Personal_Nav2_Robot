#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_optimizer.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

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

    point_vertices_.reserve(trajectory.size());

    return true;
}

void EsdfG2oOptimizer::copyBack(TrajectoryPoints &/*trajectory*/) const {
}

void EsdfG2oOptimizer::clear() {

    optimizer_.clear();
    point_vertices_.clear();
}

} // namespace my_nav2_smoother
