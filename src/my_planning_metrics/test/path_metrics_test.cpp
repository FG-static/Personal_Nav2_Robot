#include <cmath>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "my_planning_metrics/path_metrics.hpp"

namespace my_planning_metrics {
namespace {

TEST(PathMetrics, ComputesDistanceToLethalObstacle) {
    nav2_costmap_2d::Costmap2D costmap(
        5, 5, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
    costmap.setCost(2, 2, nav2_costmap_2d::LETHAL_OBSTACLE);

    ObstacleDistanceField distance_field;
    ASSERT_TRUE(distance_field.build(&costmap));

    double distance_m = 0.0;
    ASSERT_TRUE(distance_field.query(2.5, 2.5, distance_m));
    EXPECT_DOUBLE_EQ(distance_m, 0.0);

    ASSERT_TRUE(distance_field.query(4.5, 2.5, distance_m));
    EXPECT_NEAR(distance_m, 2.0, 1e-9);

    ASSERT_TRUE(distance_field.query(4.5, 4.5, distance_m));
    EXPECT_NEAR(distance_m, std::sqrt(8.0), 1e-9);
}

TEST(PathMetrics, ComputesLengthCurvatureAndClearance) {
    nav2_costmap_2d::Costmap2D costmap(
        5, 5, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
    costmap.setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);

    ObstacleDistanceField distance_field;
    ASSERT_TRUE(distance_field.build(&costmap));

    nav_msgs::msg::Path path;
    for (const auto &point : {
        std::pair<double, double>{1.5, 1.5},
        std::pair<double, double>{2.5, 1.5},
        std::pair<double, double>{2.5, 2.5}}) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        path.poses.push_back(pose);
    }

    const PathMetrics metrics = evaluatePath(path, &distance_field);
    EXPECT_EQ(metrics.point_count, 3U);
    EXPECT_NEAR(metrics.length_m, 2.0, 1e-9);
    EXPECT_NEAR(metrics.max_curvature_inv_m, std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(metrics.min_lethal_obstacle_distance_m, std::sqrt(2.0), 1e-9);
}

} // namespace
} // namespace my_planning_metrics
