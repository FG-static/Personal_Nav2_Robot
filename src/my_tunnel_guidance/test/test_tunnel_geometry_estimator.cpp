#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <random>
#include <vector>

#include "my_tunnel_guidance/tunnel_geometry_estimator.hpp"

using my_tunnel_guidance::TunnelGeometryEstimator;
using my_tunnel_guidance::TunnelGeometryParams;

namespace
{

void makeTunnelPoints(
    std::vector<Eigen::Vector3d> & ground,
    std::vector<Eigen::Vector3d> & left,
    std::vector<Eigen::Vector3d> & right,
    double yaw,
    double noise)
{
    std::mt19937 gen(42);
    std::normal_distribution<double> noise_dist(0.0, noise);
    Eigen::AngleAxisd rotation(yaw, Eigen::Vector3d::UnitZ());

    for (double x = -0.5; x <= 8.0; x += 0.1) {
        for (double y = -2.0; y <= 2.0; y += 0.1) {
            Eigen::Vector3d point(x, y, 0.0);
            point.x() += noise_dist(gen);
            point.y() += noise_dist(gen);
            point.z() += noise_dist(gen);
            ground.push_back(rotation * point);
        }
        for (double z = 0.3; z <= 2.0; z += 0.1) {
            Eigen::Vector3d lp(x, 2.0, z);
            lp.x() += noise_dist(gen);
            lp.z() += noise_dist(gen);
            left.push_back(rotation * lp);

            Eigen::Vector3d rp(x, -2.0, z);
            rp.x() += noise_dist(gen);
            rp.z() += noise_dist(gen);
            right.push_back(rotation * rp);
        }
    }
}

}  // namespace

TEST(TunnelGeometryEstimator, StraightTunnelCenterline)
{
    std::vector<Eigen::Vector3d> ground, left, right;
    makeTunnelPoints(ground, left, right, 0.0, 0.005);

    TunnelGeometryEstimator estimator;
    const auto frame = estimator.estimateFrame(left, right, ground);
    ASSERT_TRUE(frame.valid);
    EXPECT_NEAR(frame.center.y(), 0.0, 0.05);
    EXPECT_NEAR(frame.width, 4.0, 0.10);
    EXPECT_NEAR(frame.tangent.x(), 1.0, 0.03);
    EXPECT_NEAR(std::abs(frame.tangent.y()), 0.0, 0.03);

    const auto centerline = estimator.buildStraightCenterline(frame);
    ASSERT_TRUE(centerline.valid);
    EXPECT_GT(centerline.points.size(), 10U);
}

TEST(TunnelGeometryEstimator, RotatedTunnelKeepsRelativeCenter)
{
    std::vector<Eigen::Vector3d> ground, left, right;
    const double yaw = 0.3;
    makeTunnelPoints(ground, left, right, yaw, 0.005);

    TunnelGeometryEstimator estimator;
    const auto frame = estimator.estimateFrame(left, right, ground);
    ASSERT_TRUE(frame.valid);
    EXPECT_NEAR(frame.width, 4.0, 0.10);
    // 中心点应仍落在两面墙的横向中间。
    EXPECT_NEAR(frame.center.norm(), 0.0, 0.10);
    EXPECT_NEAR(frame.tangent.dot(Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0)), 1.0, 0.05);
}

TEST(TunnelGeometryEstimator, MissingWallIsInvalid)
{
    std::vector<Eigen::Vector3d> ground, left, right;
    makeTunnelPoints(ground, left, right, 0.0, 0.005);
    right.clear();

    TunnelGeometryEstimator estimator;
    const auto frame = estimator.estimateFrame(left, right, ground);
    EXPECT_FALSE(frame.valid);
}
