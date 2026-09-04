#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"

#include "my_nav2_smoother/bspline_smoother.hpp"

namespace {

using my_bspline_smoother::MyBSplineSmoother;

class BsplineSmootherTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        node_ = std::make_shared<nav2_util::LifecycleNode>(
            "test_bspline_smoother", "", rclcpp::NodeOptions());
        smoother_ = std::make_shared<MyBSplineSmoother>();
        // 无 costmap 的配置：走廊退化为参考路径 ±corridor_min_half_width，
        // 用于隔离验证 QP 与端点钉扎行为
        smoother_->configure(node_, "test_bspline", nullptr, nullptr, nullptr);
    }

    nav2_util::LifecycleNode::SharedPtr node_;
    std::shared_ptr<MyBSplineSmoother> smoother_;
};

nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & points)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (const auto & [x, y] : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    return path;
}

TEST_F(BsplineSmootherTest, ShortPathPassthrough)
{
    auto path = makePath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
    const auto original = path;

    const bool ok = smoother_->smooth(path, rclcpp::Duration::from_seconds(1.0));

    ASSERT_TRUE(ok);
    ASSERT_EQ(path.poses.size(), original.poses.size());
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
        EXPECT_NEAR(
            path.poses[i].pose.position.x,
            original.poses[i].pose.position.x, 1e-9);
        EXPECT_NEAR(
            path.poses[i].pose.position.y,
            original.poses[i].pose.position.y, 1e-9);
    }
}

TEST_F(BsplineSmootherTest, StraightLineStaysStraightWithEndpointFidelity)
{
    constexpr std::size_t kPoints = 30;
    std::vector<std::pair<double, double>> points;
    for (std::size_t i = 0; i < kPoints; ++i) {
        points.emplace_back(0.1 * static_cast<double>(i), 0.02 * static_cast<double>(i));
    }
    auto path = makePath(points);

    const bool ok = smoother_->smooth(path, rclcpp::Duration::from_seconds(1.0));

    ASSERT_TRUE(ok);
    // 输出点数恒等于输入点数（控制点直出）
    ASSERT_EQ(path.poses.size(), kPoints);
    // 端点钉扎：首末点偏差须远小于走廊检查容差
    EXPECT_NEAR(path.poses.front().pose.position.x, 0.0, 1e-3);
    EXPECT_NEAR(path.poses.front().pose.position.y, 0.0, 1e-3);
    EXPECT_NEAR(
        path.poses.back().pose.position.x, 0.1 * (kPoints - 1), 1e-3);
    EXPECT_NEAR(
        path.poses.back().pose.position.y, 0.02 * (kPoints - 1), 1e-3);

    // 直线输入平滑后仍贴直线（到直线 y=0.2x 的垂距足够小）
    for (const auto & pose : path.poses) {
        const double x = pose.pose.position.x;
        const double y = pose.pose.position.y;
        const double line_distance = std::abs(0.2 * x - y) / std::sqrt(1.0 + 0.04);
        EXPECT_LT(line_distance, 1e-2) << "point (" << x << "," << y << ")";
    }
}

TEST_F(BsplineSmootherTest, OutputStaysInsideDegradedCorridor)
{
    // 无 costmap 时走廊为 ±0.05：锯齿幅值 0.04 的输入平滑后
    // 每个输出点仍须落在参考点附近走廊内
    constexpr std::size_t kPoints = 40;
    std::vector<std::pair<double, double>> points;
    for (std::size_t i = 0; i < kPoints; ++i) {
        const double wobble = (i % 2 == 0) ? 0.04 : -0.04;
        points.emplace_back(0.1 * static_cast<double>(i), wobble);
    }
    auto path = makePath(points);

    const bool ok = smoother_->smooth(path, rclcpp::Duration::from_seconds(1.0));

    ASSERT_TRUE(ok);
    ASSERT_EQ(path.poses.size(), kPoints);
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const double dx = path.poses[i].pose.position.x - points[i].first;
        const double dy = path.poses[i].pose.position.y - points[i].second;
        EXPECT_LT(std::hypot(dx, dy), 0.06)
            << "point " << i << " outside corridor band";
    }
}

TEST_F(BsplineSmootherTest, RepeatedSmoothCallsStable)
{
    std::vector<std::pair<double, double>> points;
    for (std::size_t i = 0; i < 20; ++i) {
        points.emplace_back(0.1 * static_cast<double>(i), 0.0);
    }
    auto path_first = makePath(points);

    ASSERT_TRUE(smoother_->smooth(path_first, rclcpp::Duration::from_seconds(1.0)));

    // 连续第二次调用（metrics 节流窗口内）不应崩溃或改变端点行为
    auto path_second = makePath(points);
    ASSERT_TRUE(smoother_->smooth(path_second, rclcpp::Duration::from_seconds(1.0)));
    ASSERT_EQ(path_second.poses.size(), points.size());
    EXPECT_NEAR(path_second.poses.front().pose.position.x, 0.0, 1e-3);
    EXPECT_NEAR(
        path_second.poses.back().pose.position.x,
        0.1 * (points.size() - 1), 1e-3);
}

}  // namespace
