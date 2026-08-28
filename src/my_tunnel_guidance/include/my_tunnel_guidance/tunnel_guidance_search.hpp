#ifndef MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_SEARCH_HPP_
#define MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_SEARCH_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "my_tunnel_guidance/tunnel_types.hpp"

namespace my_tunnel_guidance {

struct TunnelGuidanceSearchParams {

    double resolution = 0.10;
    double min_x = -1.0;
    double max_x = 10.0;
    double half_width = 4.0;
    double obstacle_min_height = 0.15;
    double obstacle_max_height = 1.30;
    double robot_clearance = 0.30;
    double clearance_weight = 2.0;
    double clearance_decay = 0.50;
    double minimum_frontier_distance = 3.0;
    double goal_distance = 5.0;
    // 未知区边代价乘子（>=1）：路径允许穿过未观测区域，但更倾向留在确认空地
    double unknown_cost_factor = 1.3;
    // Free 掩码形态学闭运算核边长（单位：格，建议奇数；<3 表示关闭）
    int free_close_kernel = 3;
    std::size_t debug_expansion_interval = 50U;
};

struct TunnelGuidanceSearchResult {

    std::vector<Eigen::Vector3d> path;
    Eigen::Vector3d goal = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_tangent = Eigen::Vector3d::UnitX();
    double goal_clearance = 0.0;
    bool valid = false;
};

// Forward-looking exit cue from the occupancy grid (base_link).
// Ignores walls behind the robot, which is why PCA point counts fail at the mouth.
struct TunnelExitWindow {

    double min_x = 2.0;
    double max_x = 6.0;
    double wall_inner_y = 1.0;
    double wall_outer_y = 2.8;
    double front_half_width = 0.8;
    double min_side_column_ratio = 0.35;
    double max_open_column_ratio = 0.18;
    // Absolute cap, not a ratio: a thin end-wall is only a few x-columns.
    int max_front_columns = 2;
};

struct TunnelExitObservation {

    int columns = 0;
    int left_columns = 0;
    int right_columns = 0;
    int front_columns = 0;
    double left_column_ratio = 0.0;
    double right_column_ratio = 0.0;
    double front_column_ratio = 0.0;
    bool valid = false;
    bool corridor_present = false;
    bool open_ahead = false;
};

struct SearchDebugFrame {

    int width = 0;
    int height = 0;
    double resolution = 0.0;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    std::vector<double> esdf_distances;
    std::vector<double> costs;
    std::vector<std::uint8_t> closed;
    std::size_t expanded_count = 0U;
};

using SearchDebugCallback =
    std::function<void(const SearchDebugFrame & frame)>;

class TunnelGuidanceSearch {

public:

    explicit TunnelGuidanceSearch(const TunnelGuidanceSearchParams & params = {});

    TunnelGuidanceSearchResult search(
        const std::vector<Eigen::Vector3d> & base_points) const;

    /**
     * @brief 对占用栅格的 Free 掩码做形态学闭运算（先膨胀后腐蚀），
     *        将相邻射线之间小于核尺寸的假性 Unknown 缝隙提升为 Free。
     *        Occupied 永远不会被修改，真实的大块未观测区也基本保留。
     * @param grid 待处理的占用栅格（就地修改）
     * @param kernel 闭运算核边长（单位：格，建议奇数；<3 表示关闭）
     */
    static void applyFreeClosing(TunnelGrid & grid, int kernel);

    TunnelGuidanceSearchResult searchGrid(
        const TunnelGrid & grid,
        const SearchDebugCallback & debug_callback = {}) const;

    TunnelExitObservation observeExit(
        const std::vector<Eigen::Vector3d> & base_points,
        const TunnelExitWindow & window = {}) const;

    static TunnelExitObservation observeExit(
        const TunnelGrid & grid,
        const TunnelExitWindow & window = {});

private:

    struct SearchData {

        std::vector<double> esdf_distances;
        std::vector<double> costs;
        std::vector<double> path_lengths;
        std::vector<int> parents;
        std::vector<std::uint8_t> closed;
        std::size_t expanded_count = 0U;
    };

    bool parametersValid() const;

    TunnelGrid buildGridFromPoints(
        const std::vector<Eigen::Vector3d> & base_points) const;

    std::vector<double> computeEsdf(const TunnelGrid & grid) const;

    static bool isValidGrid(const TunnelGrid & grid);

    static std::size_t toIndex(const TunnelGrid & grid, int mx, int my);

    static bool isInside(const TunnelGrid & grid, int mx, int my);

    static bool worldToCell(
        const TunnelGrid & grid,
        const Eigen::Vector2d & point,
        int & mx,
        int & my);

    static Eigen::Vector2d cellToWorld(
        const TunnelGrid & grid,
        int mx,
        int my);

    bool isSearchCellTraversable(
        const TunnelGrid & grid,
        const SearchData & data,
        std::size_t index,
        std::size_t start_index) const;

    bool isFrontierCell(
        const TunnelGrid & grid,
        int mx,
        int my) const;

    std::vector<int> reconstructCellPath(
        const SearchData & data,
        int endpoint,
        std::size_t cell_count) const;

    TunnelGuidanceSearchResult makeResult(
        const TunnelGrid & grid,
        const SearchData & data,
        int endpoint) const;

    void emitDebugFrame(
        const TunnelGrid & grid,
        const SearchData & data,
        const SearchDebugCallback & debug_callback) const;

    TunnelGuidanceSearchParams params_;
};

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_SEARCH_HPP_
