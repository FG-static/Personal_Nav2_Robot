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
    std::size_t debug_expansion_interval = 50U;
};

struct TunnelGuidanceSearchResult {

    std::vector<Eigen::Vector3d> path;
    Eigen::Vector3d goal = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_tangent = Eigen::Vector3d::UnitX();
    double goal_clearance = 0.0;
    bool valid = false;
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

    TunnelGuidanceSearchResult searchGrid(
        const TunnelGrid & grid,
        const SearchDebugCallback & debug_callback = {}) const;

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
