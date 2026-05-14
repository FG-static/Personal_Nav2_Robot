#ifndef MY_TEB_CONTROLLER__TEB_VERTICES
#define MY_TEB_CONTROLLER__TEB_VERTICES

#include <iostream>

#include "angles/angles.h"
#include "g2o/core/base_vertex.h"

#include "my_teb_controller/teb_types.hpp"

// 重写g2o::BaseVertex的oplusImpl和oplusImpl方法，适配该项目
namespace my_teb_controller {

class VertexPose : public g2o::BaseVertex<3, PoseSE2> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPose() = default;

    void setToOriginImpl() override {

        _estimate = PoseSE2();
    }

    void oplusImpl(const double *update) override {

        _estimate.x += update[0];
        _estimate.y += update[1];
        _estimate.theta = angles::normalize_angle(_estimate.theta + update[2]);
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }
};

// 同理
class VertexTimeDiff : public g2o::BaseVertex<1, double> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexTimeDiff() = default;

    void setToOriginImpl() override {

        _estimate = 0.1;
    }

    void oplusImpl(const double *update) override {

        _estimate = std::max(1e-3, _estimate + update[0]);
    }

    bool read(std::istream & /*is*/) override { return false; }
    bool write(std::ostream & /*os*/) const override { return false; }
};

} // namespace my_teb_controller

#endif // MY_TEB_CONTROLLER__TEB_VERTICES
