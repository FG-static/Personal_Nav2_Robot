#ifndef MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_VERTICES
#define MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_VERTICES

#include <iosfwd>

#include <Eigen/Dense>

#include "g2o/core/base_vertex.h"

namespace my_nav2_smoother {

class VertexPathPoint : public g2o::BaseVertex<2, Eigen::Vector2d> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override {
        _estimate.setZero();
    }

    void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector2d(update[0], update[1]);
    }

    bool read(std::istream &/*is*/) override {
        return false;
    }

    bool write(std::ostream &/*os*/) const override {
        return false;
    }
};

} // namespace my_nav2_smoother

#endif // MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_VERTICES
