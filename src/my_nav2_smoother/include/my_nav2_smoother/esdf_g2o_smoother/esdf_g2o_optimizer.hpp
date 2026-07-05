#ifndef MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_OPTIMIZER
#define MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_OPTIMIZER

#include <vector>

#include "g2o/core/sparse_optimizer.h"

#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_types.hpp"
#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_vertices.hpp"
#include "my_nav2_smoother/esdf_g2o_smoother/esdf_map.hpp"

namespace my_nav2_smoother {

class EsdfG2oOptimizer {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EsdfG2oOptimizer() = default;

    bool optimize(
        TrajectoryPoints &trajectory,
        const TrajectoryPoints &reference_trajectory,
        const EsdfMap &esdf_map,
        const EsdfG2oSmootherConfig &config);
private:

    bool initializeSolver(bool verbose);
    bool buildGraph(
        const TrajectoryPoints &trajectory,
        const TrajectoryPoints &reference_trajectory,
        const EsdfMap &esdf_map,
        const EsdfG2oSmootherConfig &config);
    void copyBack(TrajectoryPoints &trajectory) const;
    void clear();

    g2o::SparseOptimizer optimizer_;
    std::vector<VertexPathPoint *> point_vertices_;
};

} // namespace my_nav2_smoother

#endif // MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_OPTIMIZER
