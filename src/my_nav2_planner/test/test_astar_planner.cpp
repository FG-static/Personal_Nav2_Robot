#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "my_nav2_planner/astar_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace {

using my_nav2_planner::MyAStarPlanner;

constexpr unsigned char kUnknown = nav2_costmap_2d::NO_INFORMATION;
constexpr unsigned char kLethal = nav2_costmap_2d::LETHAL_OBSTACLE;

/// 建一个全 Free 的 costmap
nav2_costmap_2d::Costmap2D makeFreeMap(unsigned int width, unsigned int height)
{
    return nav2_costmap_2d::Costmap2D(width, height, 0.1, 0.0, 0.0);
}

/// 校验路径每一步：不穿障碍格；对角步的两个正交邻格也不是硬障碍（防穿角）
void expectPathLegal(
    const nav2_costmap_2d::Costmap2D & map,
    const std::vector<std::uint64_t> & cells)
{
    ASSERT_GE(cells.size(), 2U);
    const int width = static_cast<int>(map.getSizeInCellsX());

    for (std::size_t i = 0; i < cells.size(); ++i) {
        const int cx = static_cast<int>(cells[i] % width);
        const int cy = static_cast<int>(cells[i] / width);
        ASSERT_LT(map.getCost(
            static_cast<unsigned int>(cx), static_cast<unsigned int>(cy)), kLethal)
            << "path cell (" << cx << "," << cy << ") is lethal";

        if (i + 1 >= cells.size()) {
            continue;
        }
        const int nx = static_cast<int>(cells[i + 1] % width);
        const int ny = static_cast<int>(cells[i + 1] / width);
        const int dx = nx - cx;
        const int dy = ny - cy;
        ASSERT_LE(std::abs(dx), 1);
        ASSERT_LE(std::abs(dy), 1);
        if (dx != 0 && dy != 0) {
            const unsigned char cost_a =
                map.getCost(static_cast<unsigned int>(cx + dx), static_cast<unsigned int>(cy));
            const unsigned char cost_b =
                map.getCost(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy + dy));
            EXPECT_LT(cost_a, kLethal) << "corner-cut via (" << cx + dx << "," << cy << ")";
            EXPECT_LT(cost_b, kLethal) << "corner-cut via (" << cx << "," << cy + dy << ")";
        }
    }
}

TEST(AStarPlanner, StraightCorridorShortestPath)
{
    MyAStarPlanner planner;
    const auto map = makeFreeMap(20, 9);

    const auto result = planner.searchCells(map, 2, 4, 17, 4);

    ASSERT_TRUE(result.found);
    ASSERT_FALSE(result.cells.empty());
    // 8 邻域下直线最短路径为 15 步、16 个格
    EXPECT_EQ(result.cells.size(), 16U);
    EXPECT_EQ(result.cells.front(), 4U * 20U + 2U);
    EXPECT_EQ(result.cells.back(), 4U * 20U + 17U);
    expectPathLegal(map, result.cells);
}

TEST(AStarPlanner, DiagonalGapNotCrossed)
{
    MyAStarPlanner planner;
    auto map = makeFreeMap(22, 9);
    // 在直行通道上放两个斜对贴紧的致命格，直线不可行，
    // 穿角路径会试图从 (9,4)->(10,3) 挤过缝隙
    map.setCost(10, 4, kLethal);
    map.setCost(9, 3, kLethal);

    const auto result = planner.searchCells(map, 2, 4, 17, 4);

    ASSERT_TRUE(result.found);
    expectPathLegal(map, result.cells);
}

TEST(AStarPlanner, GoalOccupiedThrows)
{
    MyAStarPlanner planner;
    auto map = makeFreeMap(20, 9);
    map.setCost(17, 4, kLethal);

    EXPECT_THROW(
        planner.searchCells(map, 2, 4, 17, 4), nav2_core::GoalOccupied);
}

TEST(AStarPlanner, GoalEqualsStartSucceedsEvenOccupied)
{
    MyAStarPlanner planner;
    auto map = makeFreeMap(20, 9);
    map.setCost(17, 4, kLethal);

    // 与 start 重合时保留历史宽容行为：返回单点成功
    const auto result = planner.searchCells(map, 17, 4, 17, 4);

    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.cells.size(), 1U);
}

TEST(AStarPlanner, UnknownPenaltyPrefersDetour)
{
    MyAStarPlanner planner;
    auto map = makeFreeMap(30, 11);
    // 中部横向未知带（默认 treat_unknown_as_free=false，每步加 unknown_cost），
    // 上下方向留出全 Free 绕行空间
    for (unsigned int mx = 10; mx <= 19; ++mx) {
        for (unsigned int my = 4; my <= 6; ++my) {
            map.setCost(mx, my, kUnknown);
        }
    }

    const auto result = planner.searchCells(map, 5, 5, 25, 5);

    ASSERT_TRUE(result.found);
    const int width = static_cast<int>(map.getSizeInCellsX());
    for (const std::uint64_t idx : result.cells) {
        const unsigned int my = static_cast<unsigned int>(idx / width);
        const unsigned int mx = static_cast<unsigned int>(idx % width);
        EXPECT_NE(map.getCost(mx, my), kUnknown)
            << "path crosses unknown cell (" << mx << "," << my << ")";
    }
}

TEST(AStarPlanner, CancelCheckerAbortsSearch)
{
    MyAStarPlanner planner;
    // 带整墙的地图迫使 A* 在绕行到缺口前扩展整片开阔区
    // （约 1600 格 > 1024 的取消检查间隔），保证取消检查被触发
    auto map = makeFreeMap(40, 80);
    for (unsigned int my = 0; my < 78; ++my) {
        map.setCost(20, my, kLethal);
    }

    const std::function<bool()> cancelled = []() {return true;};
    const auto result = planner.searchCells(map, 0, 0, 21, 0, cancelled);

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.cells.empty());
}

TEST(AStarPlanner, OutOfBoundsCellsRejected)
{
    MyAStarPlanner planner;
    const auto map = makeFreeMap(10, 10);

    const auto result = planner.searchCells(map, 0, 0, 10, 5);

    EXPECT_FALSE(result.found);
    EXPECT_TRUE(result.cells.empty());
}

}  // namespace
