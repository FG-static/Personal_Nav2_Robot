#ifndef MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_EDGES
#define MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_EDGES

#include <iosfwd>

#include <Eigen/Dense>

#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_unary_edge.h"

#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_vertices.hpp"
#include "my_nav2_smoother/esdf_g2o_smoother/esdf_map.hpp"

namespace my_nav2_smoother {

class EdgeEsdfObstacle : public g2o::BaseUnaryEdge<1, double, VertexPathPoint> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setEsdfMap(const EsdfMap *esdf_map) {
        esdf_map_ = esdf_map;
    }

    void computeError() override {

        _error[0] = 0.0;
        if (esdf_map_ == nullptr)
            return;

        const auto *vertex = static_cast<const VertexPathPoint *>(_vertices[0]);
        EsdfQueryResult result;
        if (!esdf_map_->query(vertex->estimate().x(), vertex->estimate().y(), result) ||
            !result.valid)
            return;

        // TSDF 做法
        const double min_obstacle_dist = _measurement;
        if (result.distance < min_obstacle_dist)
            _error[0] = min_obstacle_dist - result.distance;
    }

    // 手写解析 Jacobian ，避免 g2o 使用默认的数值扰动计算数值 Jacobian
    void linearizeOplus() override {

        _jacobianOplusXi.setZero();
        if (esdf_map_ == nullptr)
            return;

        const auto *vertex = static_cast<const VertexPathPoint *>(_vertices[0]);
        EsdfQueryResult result;
        if (!esdf_map_->query(vertex->estimate().x(), vertex->estimate().y(), result) ||
            !result.valid)
            return;

        const double min_obstacle_dist = _measurement;
        if (result.distance >= min_obstacle_dist || !result.gradient.allFinite())
            return;

        _jacobianOplusXi(0, 0) = -result.gradient.x();
        _jacobianOplusXi(0, 1) = -result.gradient.y();
    }

    bool read(std::istream &/*is*/) override {
        return false;
    }

    bool write(std::ostream &/*os*/) const override {
        return false;
    }

private:

    const EsdfMap *esdf_map_ = nullptr;
};

class EdgeAnchor : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPathPoint> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {

        const auto *vertex = static_cast<const VertexPathPoint *>(_vertices[0]);
        _error = vertex->estimate() - _measurement;
    }

    void linearizeOplus() override {

        _jacobianOplusXi.setIdentity();
    }

    bool read(std::istream &/*is*/) override {
        return false;
    }

    bool write(std::ostream &/*os*/) const override {
        return false;
    }
};

class EdgeSmoothness : public g2o::BaseMultiEdge<2, Eigen::Vector2d> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSmoothness() {
        resize(3);
    }

    void computeError() override {

        const auto *prev = static_cast<const VertexPathPoint *>(_vertices[0]);
        const auto *current = static_cast<const VertexPathPoint *>(_vertices[1]);
        const auto *next = static_cast<const VertexPathPoint *>(_vertices[2]);
        _error = prev->estimate() - 2.0 * current->estimate() + next->estimate();
    }

    void linearizeOplus() override {

        _jacobianOplus[0].setIdentity();
        _jacobianOplus[1].setIdentity();
        _jacobianOplus[1] *= -2.0;
        _jacobianOplus[2].setIdentity();
    }

    bool read(std::istream &/*is*/) override {
        return false;
    }

    bool write(std::ostream &/*os*/) const override {
        return false;
    }
};

} // namespace my_nav2_smoother

#endif // MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_EDGES
