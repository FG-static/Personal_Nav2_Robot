#ifndef MY_TEB_CONTROLLER__TEB_GRAPH_OPTIMIZER
#define MY_TEB_CONTROLLER__TEB_GRAPH_OPTIMIZER

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "g2o/core/sparse_optimizer.h"

#include "my_teb_controller/teb_edges.hpp"

namespace my_teb_controller {

class TebGraphOptimizer {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TebGraphOptimizer() = default;

    bool optimize(
        std::vector<TimedPose> &trajectory,
        const std::vector<TimedPose> &reference_trajectory,
        const PoseSE2 &goal_pose,
        const ObstacleSamples &obstacles,
        const TebConfig &config,
        bool verbose);

private:

    bool initializeSolver(bool verbose);
    bool buildGraph(
        const std::vector<TimedPose> &trajectory,
        const std::vector<TimedPose> &reference_trajectory,
        const PoseSE2 &goal_pose,
        const ObstacleSamples &obstacles,
        const TebConfig &config);
    void copyBack(std::vector<TimedPose> &trajectory, const TebConfig &config) const;
    void clear();
    Eigen::Vector2d findNearestObstacle(const PoseSE2 &pose, const ObstacleSamples &obstacles) const;

    g2o::SparseOptimizer optimizer_;
    std::vector<VertexPose *> pose_vertices_;
    std::vector<VertexTimeDiff *> dt_vertices_;
};

} // namespace my_teb_controller

#endif // MY_TEB_CONTROLLER__TEB_GRAPH_OPTIMIZER
