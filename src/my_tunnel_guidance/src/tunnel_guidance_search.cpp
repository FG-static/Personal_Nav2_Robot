#include "my_tunnel_guidance/tunnel_guidance_search.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace my_tunnel_guidance {

namespace {

// 数值比较时的容差，避免浮点误差导致判断不稳定
constexpr double kEpsilon = 1e-9;

// 栅格单元偏移量，表示从当前格到相邻格的 (列, 行) 增量
using CellOffset = std::pair<int, int>;

// 八邻域搜索偏移：先水平/垂直，后对角，共 8 个方向
constexpr std::array<CellOffset, 8> kNeighborOffsets = {
    CellOffset{1, 0},
    CellOffset{1, 1},
    CellOffset{0, 1},
    CellOffset{-1, 1},
    CellOffset{-1, 0},
    CellOffset{-1, -1},
    CellOffset{0, -1},
    CellOffset{1, -1}};

/**
 * @brief 判断三维点是否为有限数值，用于过滤 NaN/Inf 点
 * @param point 输入三维点
 * @return 所有分量均有限时返回 true
 */
bool isFinitePoint(const Eigen::Vector3d & point)
{
    return point.allFinite();
}

}  // namespace

/**
 * @brief 构造函数，保存搜索参数副本
 * @param params 搜索参数（分辨率、范围、代价权重等）
 */
TunnelGuidanceSearch::TunnelGuidanceSearch(
    const TunnelGuidanceSearchParams & params)
: params_(params)
{
}

/**
 * @brief 校验搜索参数是否可用
 * @return 所有参数均有限且满足取值范围时返回 true
 */
bool TunnelGuidanceSearch::parametersValid() const
{
    return std::isfinite(params_.resolution) && params_.resolution > 0.0 &&
           std::isfinite(params_.min_x) && std::isfinite(params_.max_x) &&
           params_.max_x > params_.min_x &&
           std::isfinite(params_.half_width) && params_.half_width > 0.0 &&
           std::isfinite(params_.obstacle_min_height) &&
           std::isfinite(params_.obstacle_max_height) &&
           params_.obstacle_max_height >= params_.obstacle_min_height &&
           std::isfinite(params_.robot_clearance) &&
           params_.robot_clearance >= 0.0 &&
           std::isfinite(params_.clearance_weight) &&
           params_.clearance_weight >= 0.0 &&
           std::isfinite(params_.clearance_decay) &&
           params_.clearance_decay > 0.0 &&
           std::isfinite(params_.minimum_frontier_distance) &&
           params_.minimum_frontier_distance >= 0.0 &&
           std::isfinite(params_.goal_distance) &&
           params_.goal_distance > 0.0;
}

/**
 * @brief 由 base_link 系点云构建 2D 占用栅格
 * 
 * 每个有效点先按布雷森汉姆直线算法从机器人位置（原点）向该点
 * 扫描，沿途格子标记为 Free；落点在障碍物高度带内的格子标记为
 * Occupied，其余高度的点只提供自由空间证据。
 * @param base_points base_link 系下的三维点云
 * @return TunnelGrid 包含状态数组的占用栅格，参数或输入非法时返回空栅格
 */
TunnelGrid TunnelGuidanceSearch::buildGridFromPoints(
    const std::vector<Eigen::Vector3d> & base_points) const
{
    TunnelGrid grid;
    if (!parametersValid()) {
        return grid;
    }

    // 根据参数范围计算栅格尺寸（列/行数向上取整）
    const double map_width = params_.max_x - params_.min_x;
    const double map_height = 2.0 * params_.half_width;
    const double resolution = params_.resolution;
    const double width_cells = std::ceil(map_width / resolution);
    const double height_cells = std::ceil(map_height / resolution);
    if (width_cells <= 0.0 || height_cells <= 0.0 ||
        width_cells > static_cast<double>(std::numeric_limits<int>::max()) ||
        height_cells > static_cast<double>(std::numeric_limits<int>::max())) {
        return grid;
    }

    grid.width = static_cast<int>(width_cells);
    grid.height = static_cast<int>(height_cells);
    grid.resolution = resolution;
    grid.origin = Eigen::Vector2d(params_.min_x, -params_.half_width);
    grid.states.assign(
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height), GridState::Unknown);

    // 机器人位于栅格原点，作为全部射线扫描的起点，并初始化为 Free
    int start_x = 0;
    int start_y = 0;
    if (!worldToCell(grid, Eigen::Vector2d::Zero(), start_x, start_y)) {
        grid = TunnelGrid{};
        return grid;
    }

    const std::size_t start_index = toIndex(grid, start_x, start_y);
    grid.states[start_index] = GridState::Free;

    // 将某个格子标记为 Free，已占用的格子保持不变（冲突时占用优先）
    const auto markFree = [&grid](int mx, int my) {
        if (!isInside(grid, mx, my)) {
            return;
        }
        const std::size_t index = TunnelGuidanceSearch::toIndex(grid, mx, my);
        if (grid.states[index] != GridState::Occupied) {
            grid.states[index] = GridState::Free;
        }
    };

    // 布雷森汉姆直线算法：从起点到端点逐格推进，沿途标记为 Free，
    // 遇到 Occupied 格子提前停止，避免穿过障碍物形成假通路
    const auto raytrace = [&markFree, &grid, start_x, start_y](
        int end_x, int end_y) {
        int current_x = start_x;
        int current_y = start_y;
        const int delta_x = std::abs(end_x - current_x);
        const int delta_y = std::abs(end_y - current_y);
        const int step_x = current_x < end_x ? 1 : -1;
        const int step_y = current_y < end_y ? 1 : -1;
        int error = delta_x - delta_y;

        while (current_x != end_x || current_y != end_y) {
            if (grid.states[TunnelGuidanceSearch::toIndex(
                    grid, current_x, current_y)] == GridState::Occupied) {
                break;
            }
            markFree(current_x, current_y);
            const int double_error = 2 * error;
            if (double_error > -delta_y) {
                error -= delta_y;
                current_x += step_x;
            }
            if (double_error < delta_x) {
                error += delta_x;
                current_y += step_y;
            }
        }
    };

    // 逐点建图：先扫射标记自由空间，再按高度带决定端点是否为障碍物
    for (const Eigen::Vector3d & point : base_points) {
        if (!isFinitePoint(point)) {
            continue;
        }

        int end_x = 0;
        int end_y = 0;
        if (!worldToCell(
                grid, Eigen::Vector2d(point.x(), point.y()), end_x, end_y)) {
            continue;
        }

        // Every finite return clears the ray.  Only returns in the robot-height
        // band become 2D occupied cells; floor and ceiling returns still provide
        // useful free-space evidence without becoming obstacles.
        raytrace(end_x, end_y);
        const std::size_t endpoint = toIndex(grid, end_x, end_y);
        if (point.z() >= params_.obstacle_min_height &&
            point.z() <= params_.obstacle_max_height) {
            grid.states[endpoint] = GridState::Occupied;
        } else if (grid.states[endpoint] != GridState::Occupied) {
            grid.states[endpoint] = GridState::Free;
        }
    }

    return grid;
}

/**
 * @brief 计算带符号障碍物距离场（ESDF）
 * 
 * 将占用栅格转为二值图像后，使用 OpenCV 的精确 L2 距离变换，
 * 得到每个格子到最近障碍物的距离（米），供避障代价与可通行判断使用。
 * @param grid 占用栅格
 * @return 与栅格等长的距离数组（米），无占用物时填充最大距离，非法栅格返回空
 */
std::vector<double> TunnelGuidanceSearch::computeEsdf(
    const TunnelGrid & grid) const
{
    if (!isValidGrid(grid)) {
        return {};
    }

    // 构造二值图：255 表示空，0 表示障碍物
    cv::Mat binary(
        grid.height,
        grid.width,
        CV_8UC1,
        cv::Scalar(255));
    bool has_occupied_cell = false;
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const std::size_t index = toIndex(grid, mx, my);
            if (grid.states[index] == GridState::Occupied) {
                binary.at<unsigned char>(my, mx) = 0U;
                has_occupied_cell = true;
            }
        }
    }

    const std::size_t cell_count =
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height);
    std::vector<double> distances(cell_count, 0.0);
    if (!has_occupied_cell) {
        const double max_distance = std::hypot(
            static_cast<double>(grid.width),
            static_cast<double>(grid.height)) * grid.resolution;
        std::fill(distances.begin(), distances.end(), max_distance);
        return distances;
    }

    // 精确 L2 距离变换，输出每个格子到最近障碍物的距离（以格子数为单位）
    cv::Mat distance_cells;
    cv::distanceTransform(
        binary,
        distance_cells,
        cv::DIST_L2,
        cv::DIST_MASK_PRECISE);

    // 将格数距离乘分辨率转换为米制距离，存入与栅格一致的数组
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const float distance_in_cells =
                distance_cells.at<float>(my, mx);
            distances[toIndex(grid, mx, my)] =
                static_cast<double>(distance_in_cells) * grid.resolution;
        }
    }
    return distances;
}

/**
 * @brief 校验栅格尺寸、分辨率、原点以及状态数组长度是否一致
 * @param grid 待校验栅格
 * @return 栅格结构完整时返回 true
 */
bool TunnelGuidanceSearch::isValidGrid(const TunnelGrid & grid)
{
    if (grid.width <= 0 || grid.height <= 0 ||
        !std::isfinite(grid.resolution) || grid.resolution <= 0.0 ||
        !grid.origin.allFinite()) {
        return false;
    }

    const std::size_t expected_size =
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height);
    return grid.states.size() == expected_size;
}

/**
 * @brief 将栅格坐标映射为一维数组下标（行优先）
 * @param grid 栅格，提供列数
 * @param mx 列号
 * @param my 行号
 * @return 一维下标
 */
std::size_t TunnelGuidanceSearch::toIndex(
    const TunnelGrid & grid,
    int mx,
    int my)
{
    return static_cast<std::size_t>(my) *
        static_cast<std::size_t>(grid.width) +
        static_cast<std::size_t>(mx);
}

/**
 * @brief 判断栅格坐标是否在网格范围内
 * @param grid 栅格
 * @param mx 列号
 * @param my 行号
 * @return 在范围内返回 true
 */
bool TunnelGuidanceSearch::isInside(
    const TunnelGrid & grid,
    int mx,
    int my)
{
    return mx >= 0 && my >= 0 && mx < grid.width && my < grid.height;
}

/**
 * @brief 世界坐标（base_link 平面）转换到栅格坐标
 * @param grid 栅格，提供原点和分辨率
 * @param point 世界坐标点
 * @param mx 输出的列号
 * @param my 输出的行号
 * @return 点在栅格范围内时返回 true
 */
bool TunnelGuidanceSearch::worldToCell(
    const TunnelGrid & grid,
    const Eigen::Vector2d & point,
    int & mx,
    int & my)
{
    if (!isValidGrid(grid) || !point.allFinite()) {
        return false;
    }

    const double cell_x = (point.x() - grid.origin.x()) / grid.resolution;
    const double cell_y = (point.y() - grid.origin.y()) / grid.resolution;
    if (!std::isfinite(cell_x) || !std::isfinite(cell_y) ||
        cell_x < 0.0 || cell_y < 0.0 ||
        cell_x >= static_cast<double>(grid.width) ||
        cell_y >= static_cast<double>(grid.height)) {
        return false;
    }

    mx = static_cast<int>(std::floor(cell_x));
    my = static_cast<int>(std::floor(cell_y));
    return isInside(grid, mx, my);
}

/**
 * @brief 栅格坐标转换到世界坐标，取格子中心点
 * @param grid 栅格，提供原点和分辨率
 * @param mx 列号
 * @param my 行号
 * @return 格子中心的世界坐标
 */
Eigen::Vector2d TunnelGuidanceSearch::cellToWorld(
    const TunnelGrid & grid,
    int mx,
    int my)
{
    return grid.origin + Eigen::Vector2d(
        (static_cast<double>(mx) + 0.5) * grid.resolution,
        (static_cast<double>(my) + 0.5) * grid.resolution);
}

/**
 * @brief 判断某个格子是否允许搜索通过
 * 
 * 要求：在网格内、非占用、是 Free 或起点、且到最近障碍物的
 * 距离不小于机器人安全间隙。未知格子不允许通过。
 * @param grid 占用栅格
 * @param data 搜索数据，提供 ESDF 距离
 * @param index 待判断格子的下标
 * @param start_index 起点下标，起点不受 Free/未知限制
 * @return 可通过时返回 true
 */
bool TunnelGuidanceSearch::isSearchCellTraversable(
    const TunnelGrid & grid,
    const SearchData & data,
    std::size_t index,
    std::size_t start_index) const
{
    if (index >= grid.states.size() ||
        index >= data.esdf_distances.size()) {
        return false;
    }
    if (grid.states[index] == GridState::Occupied) {
        return false;
    }
    if (grid.states[index] != GridState::Free && index != start_index) {
        return false;
    }
    return data.esdf_distances[index] + kEpsilon >= params_.robot_clearance;
}

/**
 * @brief 判断格子是否为前沿（Free 且至少一个八邻域格子为 Unknown）
 * @param grid 占用栅格
 * @param mx 列号
 * @param my 行号
 * @return 是前沿格子时返回 true
 */
bool TunnelGuidanceSearch::isFrontierCell(
    const TunnelGrid & grid,
    int mx,
    int my) const
{
    if (!isInside(grid, mx, my) ||
        grid.states[toIndex(grid, mx, my)] != GridState::Free) {
        return false;
    }

    for (const CellOffset & offset : kNeighborOffsets) {
        const int neighbor_x = mx + offset.first;
        const int neighbor_y = my + offset.second;
        if (isInside(grid, neighbor_x, neighbor_y) &&
            grid.states[toIndex(grid, neighbor_x, neighbor_y)] ==
            GridState::Unknown) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 沿 parent 链从终点回溯到起点，还原栅格路径
 * @param data 搜索数据，提供 parent 信息
 * @param endpoint 终点下标
 * @param cell_count 栅格单元总数，用于边界校验
 * @return 从起点到终点的下标序列；回溯失败时返回空
 */
std::vector<int> TunnelGuidanceSearch::reconstructCellPath(
    const SearchData & data,
    int endpoint,
    std::size_t cell_count) const
{
    std::vector<int> path;
    if (endpoint < 0 || static_cast<std::size_t>(endpoint) >= cell_count ||
        data.parents.size() != cell_count) {
        return path;
    }

    // 从终点沿 parent 链回退，visited 防止数据异常时陷入死循环
    std::vector<std::uint8_t> visited(cell_count, 0U);
    int current = endpoint;
    while (current >= 0 && static_cast<std::size_t>(current) < cell_count &&
           visited[static_cast<std::size_t>(current)] == 0U) {
        path.push_back(current);
        visited[static_cast<std::size_t>(current)] = 1U;
        current = data.parents[static_cast<std::size_t>(current)];
    }

    if (current >= 0) {
        path.clear();
        return path;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * @brief 从搜索数据中构建结果
 * 
 * 将栅格路径转换为世界坐标路径，按 goal_distance 截取局部目标点，
 * 并估计目标点的前进方向（切线）与安全间隙。
 * @param grid 搜索网格
 * @param data 搜索数据
 * @param endpoint 搜索终点
 * @return TunnelGuidanceSearchResult 构建的结果
 */
TunnelGuidanceSearchResult TunnelGuidanceSearch::makeResult(
    const TunnelGrid & grid,
    const SearchData & data,
    int endpoint) const
{
    TunnelGuidanceSearchResult result;
    const std::size_t cell_count =
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height);
    const std::vector<int> cell_path =
        reconstructCellPath(data, endpoint, cell_count);
    if (cell_path.size() < 2U) {
        return result;
    }

    // 栅格路径转世界坐标路径（z 固定为 0，平面搜索）
    result.path.reserve(cell_path.size());
    for (const int index : cell_path) {
        const int mx = index % grid.width;
        const int my = index / grid.width;
        const Eigen::Vector2d point = cellToWorld(grid, mx, my);
        result.path.emplace_back(point.x(), point.y(), 0.0);
    }

    // 计算路径累计长度，用于按 goal_distance 定位局部目标点
    double total_length = 0.0;
    std::vector<double> cumulative_lengths(result.path.size(), 0.0);
    for (std::size_t i = 1U; i < result.path.size(); ++i) {
        total_length += (result.path[i] - result.path[i - 1U]).head<2>().norm();
        cumulative_lengths[i] = total_length;
    }

    const double target_length = std::min(
        std::max(params_.goal_distance, grid.resolution), total_length);
    std::size_t goal_path_index = result.path.size() - 1U;
    for (std::size_t i = 1U; i < cumulative_lengths.size(); ++i) {
        if (cumulative_lengths[i] + kEpsilon >= target_length) {
            goal_path_index = i;
            break;
        }
    }

    // 取到达目标距离处（不超过终点）的路径点作为局部目标
    const Eigen::Vector3d & goal = result.path[goal_path_index];
    const int goal_cell = cell_path[goal_path_index];
    if (goal_cell < 0 ||
        static_cast<std::size_t>(goal_cell) >= data.esdf_distances.size()) {
        return TunnelGuidanceSearchResult{};
    }
    result.goal = goal;
    result.goal_clearance = data.esdf_distances[static_cast<std::size_t>(goal_cell)];

    // 用目标点前后窗口内的路径段方向作为目标朝向（切线），
    // 窗口退化时退回上一点到目标点的方向
    const std::size_t tangent_window = 3U;
    const std::size_t tangent_begin =
        goal_path_index > tangent_window ? goal_path_index - tangent_window : 0U;
    const std::size_t tangent_end = std::min(
        goal_path_index + tangent_window,
        result.path.size() - 1U);
    Eigen::Vector2d tangent =
        (result.path[tangent_end] - result.path[tangent_begin]).head<2>();
    if (tangent.norm() <= kEpsilon && goal_path_index > 0U) {
        tangent = (result.path[goal_path_index] -
            result.path[goal_path_index - 1U]).head<2>();
    }
    if (tangent.norm() <= kEpsilon) {
        tangent = Eigen::Vector2d::UnitX();
    } else {
        tangent.normalize();
    }
    result.goal_tangent = Eigen::Vector3d(tangent.x(), tangent.y(), 0.0);
    result.valid = result.goal_clearance + kEpsilon >= params_.robot_clearance;
    return result;
}

/**
 * @brief 将当前搜索状态打包为调试帧并回调给调用方（如可视化）
 * @param grid 搜索网格
 * @param data 搜索数据（距离场、代价、闭合集合等）
 * @param debug_callback 调试回调，为空时不发送
 */
void TunnelGuidanceSearch::emitDebugFrame(
    const TunnelGrid & grid,
    const SearchData & data,
    const SearchDebugCallback & debug_callback) const
{
    if (!debug_callback) {
        return;
    }

    SearchDebugFrame frame;
    frame.width = grid.width;
    frame.height = grid.height;
    frame.resolution = grid.resolution;
    frame.origin = grid.origin;
    frame.esdf_distances = data.esdf_distances;
    frame.costs = data.costs;
    frame.closed = data.closed;
    frame.expanded_count = data.expanded_count;
    debug_callback(frame);
}

/**
 * @brief 一键搜索入口：由 base_link 点云建图后直接在栅格上搜索
 * @param base_points base_link 系下的三维点云
 * @return 搜索结果（路径、局部目标等）
 */
TunnelGuidanceSearchResult TunnelGuidanceSearch::search(
    const std::vector<Eigen::Vector3d> & base_points) const
{
    if (!parametersValid()) {
        return {};
    }
    const TunnelGrid grid = buildGridFromPoints(base_points);
    return searchGrid(grid);
}

/**
 * @brief 在占用栅格上执行搜索
 * 
 * 流程：计算 ESDF 距离场 -> 以机器人为起点执行带避障代价的
 * Dijkstra 扩展 -> 在所有可到达格中挑选最优前沿（或最远可到达格）
 * 作为终点 -> 回溯并构建结果。
 * @param grid 搜索网格
 * @param debug_callback 调试回调
 * @return TunnelGuidanceSearchResult 搜索结果
 */
TunnelGuidanceSearchResult TunnelGuidanceSearch::searchGrid(
    const TunnelGrid & grid,
    const SearchDebugCallback & debug_callback) const
{
    TunnelGuidanceSearchResult result;
    if (!parametersValid() || !isValidGrid(grid)) {
        return result;
    }

    // 初始化搜索数据：距离场、代价值、路径长度、父节点、闭合集合
    const std::size_t cell_count =
        static_cast<std::size_t>(grid.width) *
        static_cast<std::size_t>(grid.height);
    SearchData data;
    data.esdf_distances = computeEsdf(grid);
    data.costs.assign(cell_count, std::numeric_limits<double>::infinity());
    data.path_lengths.assign(cell_count, std::numeric_limits<double>::infinity());
    data.parents.assign(cell_count, -1);
    data.closed.assign(cell_count, 0U);
    if (data.esdf_distances.size() != cell_count) {
        return result;
    }

    // 机器人位于原点，作为搜索起点；起点本身必须可通行
    int start_x = 0;
    int start_y = 0;
    if (!worldToCell(grid, Eigen::Vector2d::Zero(), start_x, start_y)) {
        return result;
    }
    const std::size_t start_index = toIndex(grid, start_x, start_y);
    if (!isSearchCellTraversable(grid, data, start_index, start_index)) {
        return result;
    }

    // 最小堆优先队列，按代价值从小到大扩展
    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    data.costs[start_index] = 0.0;
    data.path_lengths[start_index] = 0.0;
    open.emplace(0.0, static_cast<int>(start_index));

    // 主搜索循环：每次取出代价最小的格子，若已闭合或代价过期则跳过
    std::size_t last_debug_expansion = 0U;
    while (!open.empty()) {
        const QueueEntry current = open.top();
        open.pop();
        const double current_cost = current.first;
        const int current_index = current.second;
        if (current_index < 0 ||
            static_cast<std::size_t>(current_index) >= cell_count ||
            current_cost > data.costs[static_cast<std::size_t>(current_index)] + kEpsilon ||
            data.closed[static_cast<std::size_t>(current_index)] != 0U) {
            continue;
        }

        // 标记闭合并计数，按设定间隔发送调试帧
        data.closed[static_cast<std::size_t>(current_index)] = 1U;
        ++data.expanded_count;
        if (debug_callback && params_.debug_expansion_interval > 0U &&
            data.expanded_count % params_.debug_expansion_interval == 0U) {
            emitDebugFrame(grid, data, debug_callback);
            last_debug_expansion = data.expanded_count;
        }

        // 遍历八邻域：对角移动要求两个相邻正方向格子也可通行，防止穿墙切角
        const int current_x = current_index % grid.width;
        const int current_y = current_index / grid.width;
        for (const CellOffset & offset : kNeighborOffsets) {
            const int neighbor_x = current_x + offset.first;
            const int neighbor_y = current_y + offset.second;
            if (!isInside(grid, neighbor_x, neighbor_y)) {
                continue;
            }

            const bool diagonal = offset.first != 0 && offset.second != 0;
            if (diagonal) {
                const std::size_t horizontal_index =
                    toIndex(grid, current_x + offset.first, current_y);
                const std::size_t vertical_index =
                    toIndex(grid, current_x, current_y + offset.second);
                if (!isSearchCellTraversable(
                        grid, data, horizontal_index, start_index) ||
                    !isSearchCellTraversable(
                        grid, data, vertical_index, start_index)) {
                    continue;
                }
            }

            const std::size_t neighbor_index = toIndex(grid, neighbor_x, neighbor_y);
            if (!isSearchCellTraversable(
                    grid, data, neighbor_index, start_index)) {
                continue;
            }

            // 边代价 = 移动距离 x (1 + 避障惩罚)，惩罚随到障碍物距离
            // 指数衰减，引导路径尽量远离障碍物
            const double move_distance = diagonal ?
                std::sqrt(2.0) * grid.resolution : grid.resolution;
            const double clearance = data.esdf_distances[neighbor_index];
            const double clearance_penalty =
                params_.clearance_weight *
                std::exp(-clearance / params_.clearance_decay);
            const double edge_cost = move_distance * (1.0 + clearance_penalty);
            const double new_cost =
                data.costs[static_cast<std::size_t>(current_index)] + edge_cost;
            if (new_cost + kEpsilon >= data.costs[neighbor_index]) {
                continue;
            }

            // 找到更优代价时更新并重新入队（Dijkstra 松弛）
            data.costs[neighbor_index] = new_cost;
            data.path_lengths[neighbor_index] =
                data.path_lengths[static_cast<std::size_t>(current_index)] +
                move_distance;
            data.parents[neighbor_index] = current_index;
            open.emplace(new_cost, static_cast<int>(neighbor_index));
        }
    }

    if (debug_callback && data.expanded_count != last_debug_expansion) {
        emitDebugFrame(grid, data, debug_callback);
    }

    // 候选终点比较：优先路径更长，其次安全间隙更大，
    // 最后优先靠近隧道中轴线（|y| 更小），全部相同则取 x 更远者
    int best_frontier = -1;
    int best_reachable = -1;
    const auto isBetterCandidate = [&data, &grid](int candidate, int current) {
        if (current < 0) {
            return true;
        }
        const std::size_t candidate_index = static_cast<std::size_t>(candidate);
        const std::size_t current_index = static_cast<std::size_t>(current);
        const double candidate_length = data.path_lengths[candidate_index];
        const double current_length = data.path_lengths[current_index];
        if (candidate_length > current_length + kEpsilon) {
            return true;
        }
        if (std::abs(candidate_length - current_length) > kEpsilon) {
            return false;
        }

        const double candidate_esdf =
            data.esdf_distances[candidate_index];
        const double current_esdf =
            data.esdf_distances[current_index];
        if (candidate_esdf > current_esdf + kEpsilon) {
            return true;
        }
        if (std::abs(candidate_esdf - current_esdf) > kEpsilon) {
            return false;
        }
        const int candidate_x = candidate % grid.width;
        const int candidate_y = candidate / grid.width;
        const int current_x = current % grid.width;
        const int current_y = current / grid.width;
        const double candidate_world_y =
            cellToWorld(grid, candidate_x, candidate_y).y();
        const double current_world_y =
            cellToWorld(grid, current_x, current_y).y();
        const double candidate_lateral =
            std::abs(candidate_world_y);
        const double current_lateral =
            std::abs(current_world_y);
        if (candidate_lateral + kEpsilon < current_lateral) {
            return true;
        }
        if (std::abs(candidate_lateral - current_lateral) > kEpsilon) {
            return false;
        }
        return candidate_x > current_x ||
               (candidate_x == current_x && candidate < current);
    };

    // 扫描全部已扩展的格子，筛选出满足距离门槛的前沿与最远可到达格：
    // 前沿作为首选终点，找不到前沿时退化为最远可到达格
    for (int my = 0; my < grid.height; ++my) {
        for (int mx = 0; mx < grid.width; ++mx) {
            const int index = my * grid.width + mx;
            const std::size_t unsigned_index = static_cast<std::size_t>(index);
            if (data.closed[unsigned_index] == 0U ||
                !isSearchCellTraversable(grid, data, unsigned_index, start_index) ||
                !std::isfinite(data.path_lengths[unsigned_index])) {
                continue;
            }

            const Eigen::Vector2d point = cellToWorld(grid, mx, my);
            if (point.x() < 0.0 ||
                data.path_lengths[unsigned_index] + kEpsilon <
                params_.minimum_frontier_distance) {
                continue;
            }

            // 前沿格与普通可达格分开竞争，均取候选比较中最优者
            if (isFrontierCell(grid, mx, my)) {
                if (isBetterCandidate(index, best_frontier)) {
                    best_frontier = index;
                }
            }
            if (isBetterCandidate(index, best_reachable)) {
                best_reachable = index;
            }
        }
    }

    // 优先使用前沿终点；无前沿时退回最远可到达格，避免原地无路可走
    const int endpoint = best_frontier >= 0 ? best_frontier : best_reachable;
    if (endpoint < 0) {
        return result;
    }
    return makeResult(grid, data, endpoint);
}

}  // namespace my_tunnel_guidance
