#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "my_tunnel_guidance/tunnel_guidance_search.hpp"

namespace {

using my_tunnel_guidance::GridState;
using my_tunnel_guidance::TunnelGrid;
using my_tunnel_guidance::TunnelGuidanceSearch;
using my_tunnel_guidance::TunnelGuidanceSearchParams;

TunnelGuidanceSearchParams testParams()
{
    TunnelGuidanceSearchParams params;
    params.resolution = 0.1;
    params.min_x = -1.0;
    params.max_x = 11.0;
    params.half_width = 4.0;
    params.robot_clearance = 0.2;
    params.clearance_weight = 2.0;
    params.clearance_decay = 0.5;
    params.minimum_frontier_distance = 2.0;
    params.goal_distance = 4.0;
    params.debug_expansion_interval = 10U;
    return params;
}

TunnelGrid makeCurvedGrid()
{
    TunnelGrid grid;
    grid.width = 120;
    grid.height = 80;
    grid.resolution = 0.1;
    grid.origin = Eigen::Vector2d(-1.0, -4.0);
    grid.states.assign(
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height), GridState::Unknown);

    for (int mx = 0; mx < grid.width; ++mx) {
        const double x = grid.origin.x() +
            (static_cast<double>(mx) + 0.5) * grid.resolution;
        if (x < -0.1 || x > 10.0) {
            continue;
        }

        const double center_y = 0.45 * std::sin(0.55 * x);
        for (int my = 0; my < grid.height; ++my) {
            const double y = grid.origin.y() +
                (static_cast<double>(my) + 0.5) * grid.resolution;
            const double lateral_distance = std::abs(y - center_y);
            const std::size_t index = static_cast<std::size_t>(my * grid.width + mx);
            if (lateral_distance <= 0.75) {
                grid.states[index] = GridState::Free;
            } else if (lateral_distance <= 0.85) {
                grid.states[index] = GridState::Occupied;
            }
        }
    }
    return grid;
}

TunnelGrid makeShortKnownGrid()
{
    TunnelGrid grid;
    grid.width = 120;
    grid.height = 80;
    grid.resolution = 0.1;
    grid.origin = Eigen::Vector2d(-1.0, -4.0);
    grid.states.assign(
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height), GridState::Unknown);

    for (int mx = 0; mx < grid.width; ++mx) {
        const double x = grid.origin.x() +
            (static_cast<double>(mx) + 0.5) * grid.resolution;
        if (x < -0.1 || x > 1.2) {
            continue;
        }
        for (int my = 0; my < grid.height; ++my) {
            const double y = grid.origin.y() +
                (static_cast<double>(my) + 0.5) * grid.resolution;
            if (std::abs(y) <= 0.8) {
                grid.states[static_cast<std::size_t>(my * grid.width + mx)] =
                    GridState::Free;
            }
        }
    }
    return grid;
}

std::size_t pointToIndex(const TunnelGrid & grid, const Eigen::Vector3d & point)
{
    const int mx = static_cast<int>(std::floor(
        (point.x() - grid.origin.x()) / grid.resolution));
    const int my = static_cast<int>(std::floor(
        (point.y() - grid.origin.y()) / grid.resolution));
    return static_cast<std::size_t>(my * grid.width + mx);
}

double pathLengthToGoal(
    const std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector3d & goal)
{
    double length = 0.0;
    for (std::size_t i = 1U; i < path.size(); ++i) {
        length += (path[i] - path[i - 1U]).head<2>().norm();
        if ((path[i] - goal).head<2>().norm() < 1e-6) {
            return length;
        }
    }
    return length;
}

std::vector<Eigen::Vector3d> makeWallAndRoofPoints()
{
    std::vector<Eigen::Vector3d> points;
    for (double x = 0.3; x <= 8.0; x += 0.1) {
        for (double z = 0.2; z <= 1.2; z += 0.2) {
            points.emplace_back(x, 2.0, z);
            points.emplace_back(x, -2.0, z);
        }
        for (double y = -2.2; y <= 2.2; y += 0.2) {
            points.emplace_back(x, y, 1.8);
            points.emplace_back(x, y, 0.0);
        }
    }
    return points;
}

}  // namespace

TEST(TunnelGuidanceSearch, FollowsCurvedKnownCorridor)
{
    TunnelGuidanceSearchParams params = testParams();
    TunnelGuidanceSearch search(params);
    const TunnelGrid grid = makeCurvedGrid();

    const auto result = search.searchGrid(grid);

    ASSERT_TRUE(result.valid);
    ASSERT_GT(result.path.size(), 20U);
    EXPECT_GE(result.goal_clearance, params.robot_clearance);
    EXPECT_NEAR(result.goal_tangent.head<2>().norm(), 1.0, 1e-6);
    EXPECT_GT(result.path.back().x(), 8.0);

    for (const Eigen::Vector3d & point : result.path) {
        const std::size_t index = pointToIndex(grid, point);
        ASSERT_LT(index, grid.states.size());
        EXPECT_EQ(grid.states[index], GridState::Free);
    }

    const double goal_length = pathLengthToGoal(result.path, result.goal);
    EXPECT_GE(goal_length, params.goal_distance);
    EXPECT_LT(goal_length, params.goal_distance + 0.25);
}

TEST(TunnelGuidanceSearch, AvoidsOccupiedBlockUsingClearanceField)
{
    TunnelGuidanceSearchParams params = testParams();
    params.goal_distance = 5.0;
    TunnelGuidanceSearch search(params);
    TunnelGrid grid = makeCurvedGrid();

    for (int mx = 40; mx <= 50; ++mx) {
        for (int my = 35; my <= 45; ++my) {
            grid.states[static_cast<std::size_t>(my * grid.width + mx)] =
                GridState::Occupied;
        }
    }

    const auto result = search.searchGrid(grid);

    ASSERT_TRUE(result.valid);
    EXPECT_GT(result.path.back().x(), 8.0);
    for (const Eigen::Vector3d & point : result.path) {
        const std::size_t index = pointToIndex(grid, point);
        ASSERT_LT(index, grid.states.size());
        EXPECT_EQ(grid.states[index], GridState::Free);
    }
}

TEST(TunnelGuidanceSearch, IgnoresRoofReturnsWhenBuildingGrid)
{
    TunnelGuidanceSearchParams params = testParams();
    params.max_x = 8.5;
    params.half_width = 3.0;
    params.minimum_frontier_distance = 2.0;
    params.goal_distance = 3.0;
    params.obstacle_max_height = 1.3;
    TunnelGuidanceSearch search(params);

    const auto result = search.search(makeWallAndRoofPoints());

    ASSERT_TRUE(result.valid);
    EXPECT_GT(result.goal.x(), 2.5);
    EXPECT_GE(result.goal_clearance, params.robot_clearance);
}

TEST(TunnelGuidanceSearch, RejectsCorridorWithoutSufficientForwardReach)
{
    TunnelGuidanceSearchParams params = testParams();
    params.minimum_frontier_distance = 3.0;
    params.goal_distance = 4.0;
    TunnelGuidanceSearch search(params);

    const auto result = search.searchGrid(makeShortKnownGrid());

    EXPECT_FALSE(result.valid);
}

TEST(TunnelGuidanceSearch, EmitsDeterministicDebugSnapshots)
{
    TunnelGuidanceSearchParams params = testParams();
    params.debug_expansion_interval = 25U;
    TunnelGuidanceSearch search(params);
    std::size_t frame_count = 0U;
    std::size_t last_expanded_count = 0U;
    std::size_t last_closed_count = 0U;
    const auto result = search.searchGrid(
        makeCurvedGrid(),
        [&frame_count, &last_expanded_count, &last_closed_count](
            const my_tunnel_guidance::SearchDebugFrame & frame) {
            ++frame_count;
            last_expanded_count = frame.expanded_count;
            last_closed_count = frame.closed.size();
            EXPECT_EQ(frame.costs.size(), frame.esdf_distances.size());
            EXPECT_EQ(frame.costs.size(), frame.closed.size());
        });

    ASSERT_TRUE(result.valid);
    EXPECT_GT(frame_count, 0U);
    EXPECT_GT(last_expanded_count, 0U);
    EXPECT_EQ(last_closed_count, 80U * 120U);
}

TEST(TunnelGuidanceSearch, ApplyFreeClosingBridgesGapsKeepsOccupied)
{
    // 三行夹一条单个 Unknown 缝隙，闭运算应把它提升为 Free；
    // 周边已有状态（Free/Occupied）保持不变，kernel < 3 时为空操作。
    TunnelGrid grid;
    grid.width = 5;
    grid.height = 3;
    grid.resolution = 0.1;
    grid.origin = Eigen::Vector2d(-0.25, -0.15);
    grid.states.assign(
        static_cast<std::size_t>(grid.width) * grid.height, GridState::Free);

    const std::size_t gap_index =
        static_cast<std::size_t>(1 * grid.width + 2);
    const std::size_t occupied_index =
        static_cast<std::size_t>(2 * grid.width + 3);
    grid.states[gap_index] = GridState::Unknown;
    grid.states[occupied_index] = GridState::Occupied;

    TunnelGuidanceSearch::applyFreeClosing(grid, 1);
    EXPECT_EQ(grid.states[gap_index], GridState::Unknown);

    TunnelGuidanceSearch::applyFreeClosing(grid, 3);
    EXPECT_EQ(grid.states[gap_index], GridState::Free);
    EXPECT_EQ(grid.states[occupied_index], GridState::Occupied);
}

TEST(TunnelGuidanceSearch, SearchCrossesUnknownEndpointStaysObserved)
{
    // 左右两块确认空地被整列 Unknown 隔开：路径必须穿越未知区，
    // 但终点只能落在右侧确认空地上；占用车界外框封闭洪水。
    TunnelGuidanceSearchParams params = testParams();
    params.robot_clearance = 0.05;
    params.minimum_frontier_distance = 0.15;
    params.goal_distance = 0.25;
    TunnelGuidanceSearch search(params);

    TunnelGrid grid;
    grid.width = 12;
    grid.height = 7;
    grid.resolution = 0.1;
    grid.origin = Eigen::Vector2d(-0.55, -0.30);
    grid.states.assign(
        static_cast<std::size_t>(grid.width) * grid.height, GridState::Unknown);

    auto setColumnRange = [&grid](
        int mx, int my_begin, int my_end, GridState state) {
        for (int my = my_begin; my <= my_end; ++my) {
            grid.states[static_cast<std::size_t>(my * grid.width + mx)] =
                state;
        }
    };
    auto setState = [&grid](int mx, int my, GridState state) {
        grid.states[static_cast<std::size_t>(my * grid.width + mx)] = state;
    };

    for (int mx = 0; mx < grid.width; ++mx) {
        setState(mx, 0, GridState::Occupied);
        setState(mx, grid.height - 1, GridState::Occupied);
    }
    for (int my = 0; my < grid.height; ++my) {
        setState(0, my, GridState::Occupied);
        setState(grid.width - 1, my, GridState::Occupied);
    }

    for (int mx = 1; mx <= 4; ++mx) {
        setColumnRange(mx, 1, grid.height - 2, GridState::Free);
    }
    // 整列 Unknown 作为必经的未观测隔离带（起点格 (5,3) 由 SearchData 覆盖）
    setColumnRange(5, 1, grid.height - 2, GridState::Unknown);
    setColumnRange(6, 1, grid.height - 2, GridState::Unknown);
    for (int mx = 7; mx <= 10; ++mx) {
        setColumnRange(mx, 1, grid.height - 2, GridState::Free);
    }
    // 起点在世界原点附近且必须是可通行格
    setState(5, 3, GridState::Free);

    const auto result = search.searchGrid(grid);

    ASSERT_TRUE(result.valid);
    EXPECT_GE(result.goal_clearance, params.robot_clearance);
    EXPECT_EQ(
        grid.states[pointToIndex(grid, result.goal)], GridState::Free);
    bool crossed_unknown = false;
    for (const Eigen::Vector3d & point : result.path) {
        if (grid.states[pointToIndex(grid, point)] == GridState::Unknown) {
            crossed_unknown = true;
        }
    }
    EXPECT_TRUE(crossed_unknown);
}

TEST(TunnelGuidanceSearch, RejectsInvalidUnknownCostFactor)
{
    TunnelGuidanceSearchParams params = testParams();
    params.unknown_cost_factor = 0.9;
    TunnelGuidanceSearch search(params);

    const auto result = search.searchGrid(makeCurvedGrid());
    EXPECT_FALSE(result.valid);
}

TunnelGrid makeForwardWindowGrid(
    bool left_wall,
    bool right_wall,
    bool closing_wall)
{
    TunnelGrid grid;
    grid.width = 90;
    grid.height = 60;
    grid.resolution = 0.1;
    grid.origin = Eigen::Vector2d(-0.5, -3.0);
    grid.states.assign(
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height), GridState::Unknown);

    for (int mx = 0; mx < grid.width; ++mx) {
        const double x = grid.origin.x() +
            (static_cast<double>(mx) + 0.5) * grid.resolution;
        for (int my = 0; my < grid.height; ++my) {
            const double y = grid.origin.y() +
                (static_cast<double>(my) + 0.5) * grid.resolution;
            const std::size_t index =
                static_cast<std::size_t>(my * grid.width + mx);
            if (x >= 0.0 && x <= 7.0 && std::abs(y) < 1.9) {
                grid.states[index] = GridState::Free;
            }
            if (left_wall && x >= 0.0 && x <= 7.0 && y >= 1.95 && y <= 2.15) {
                grid.states[index] = GridState::Occupied;
            }
            if (right_wall && x >= 0.0 && x <= 7.0 && y <= -1.95 && y >= -2.15) {
                grid.states[index] = GridState::Occupied;
            }
            if (closing_wall && x >= 4.5 && x <= 5.5 && std::abs(y) <= 0.7) {
                grid.states[index] = GridState::Occupied;
            }
        }
    }
    return grid;
}

TEST(TunnelGuidanceSearch, ExitObserveDetectsTwoSidedCorridor)
{
    const auto observation = TunnelGuidanceSearch::observeExit(
        makeForwardWindowGrid(true, true, false));
    ASSERT_TRUE(observation.valid);
    EXPECT_TRUE(observation.corridor_present);
    EXPECT_FALSE(observation.open_ahead);
}

TEST(TunnelGuidanceSearch, ExitObserveDetectsOpenMouth)
{
    const auto observation = TunnelGuidanceSearch::observeExit(
        makeForwardWindowGrid(false, false, false));
    ASSERT_TRUE(observation.valid);
    EXPECT_FALSE(observation.corridor_present);
    EXPECT_TRUE(observation.open_ahead);
}

TEST(TunnelGuidanceSearch, ExitObserveRejectsClosingWall)
{
    const auto observation = TunnelGuidanceSearch::observeExit(
        makeForwardWindowGrid(false, false, true));
    ASSERT_TRUE(observation.valid);
    EXPECT_FALSE(observation.open_ahead);
}

TEST(TunnelGuidanceSearch, ExitObserveIgnoresOneSidedWall)
{
    const auto observation = TunnelGuidanceSearch::observeExit(
        makeForwardWindowGrid(true, false, false));
    ASSERT_TRUE(observation.valid);
    EXPECT_FALSE(observation.corridor_present);
    EXPECT_FALSE(observation.open_ahead);
}

TEST(TunnelGuidanceSearch, ExitObserveIgnoresWallsBehindRobot)
{
    TunnelGrid grid = makeForwardWindowGrid(false, false, false);
    for (int mx = 0; mx < grid.width; ++mx) {
        const double x = grid.origin.x() +
            (static_cast<double>(mx) + 0.5) * grid.resolution;
        if (x < -0.05 || x > 0.8) {
            continue;
        }
        for (int my = 0; my < grid.height; ++my) {
            const double y = grid.origin.y() +
                (static_cast<double>(my) + 0.5) * grid.resolution;
            const std::size_t index =
                static_cast<std::size_t>(my * grid.width + mx);
            if (y >= 1.95 && y <= 2.15) {
                grid.states[index] = GridState::Occupied;
            }
            if (y <= -1.95 && y >= -2.15) {
                grid.states[index] = GridState::Occupied;
            }
        }
    }

    const auto observation = TunnelGuidanceSearch::observeExit(grid);
    ASSERT_TRUE(observation.valid);
    EXPECT_TRUE(observation.open_ahead);
}


