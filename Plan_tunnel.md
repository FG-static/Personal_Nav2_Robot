# 涵洞巡检点搜索第一版实现计划

## 1. 目标

将当前“拟合一个涵洞方向，再沿直线向前取固定距离目标点”的方法，改为：

1. 根据当前 Mid-360 点云构建机器人前方的局部二维栅格。
2. 利用 ESDF 让搜索结果倾向于远离墙体和障碍物。
3. 从机器人位置向所有可达的前沿区域运行一次 Dijkstra 搜索。
4. 回溯得到一条局部引导路径，并沿路径弧长选择巡检点。
5. 先在独立二维长廊代价地图中展示完整搜索动画和最终结果。
6. 验证通过后只把巡检点交给 Nav2，实际路径规划、平滑和控制仍由现有插件完成。

第一版优先验证弯曲涵洞中的实际效果，不提前建设完整的涵洞语义地图系统。

## 2. 为什么第一版使用 Dijkstra

### 2.1 普通 A*

普通 A* 需要预先给定一个确定目标：

```text
start -> 已知 goal
```

启发式函数 `h(n)` 用来估计当前格子到该目标的剩余距离，因此 A* 适合“目标已知，搜索路径”的问题。

当前涵洞节点要解决的却是“目标应该选在哪里”，在搜索开始前没有唯一目标，因此不能直接使用普通 A* 完成全部工作。

### 2.2 多目标 A*

多目标 A* 预先提供一组候选目标：

```text
goals = {g1, g2, g3, ...}
h(n) = min(distance(n, gi))
```

它可以找到通向某个候选目标的低代价路径。它适合候选目标数量明确，而且目标评分规则比较简单的情况。

但是，如果不同候选目标还要比较前进距离、净空、历史一致性等额外评分，仅找到第一个目标并不一定等于找到最适合的巡检点，实现上还需要继续搜索或修改终止条件。

### 2.3 Dijkstra

Dijkstra 不使用目标启发式，从机器人所在格子向外扩展一次，就能得到：

- 到所有可达格子的最低累计代价。
- 每个格子的父节点。
- 所有前沿候选是否可达。
- 到任意候选前沿的完整回溯路径。

当前局部地图预计只有约一万到两万个格子，单次 Dijkstra 的开销可以接受，而且实现和调试明显简单。因此第一版使用 Dijkstra；确定候选策略稳定且搜索开销成为问题后，再考虑多目标 A*。

## 3. 第一版范围

### 3.1 本轮实现

- 使用当前单帧或最新一帧点云构建 `base_link` 下的局部二维栅格。
- 对点云做高度带过滤，排除地面和顶板。
- 通过射线遍历区分已知自由格、占用格和未知格。
- 使用 OpenCV `cv::distanceTransform()` 计算自由空间 ESDF。
- 在已知自由区域内执行八邻域 Dijkstra。
- 从可达前沿中选择一个前方候选并回溯引导路径。
- 沿引导路径累计弧长，在指定前瞻距离处选择巡检点。
- 提供独立 OpenCV 二维演示程序，输出搜索动画和结果图。
- 复用现有 `/tunnel_guidance/centerline`、`local_goal`、Marker 和 Nav2 Action 逻辑。
- 保留现有点云时间戳处理、TF 查询和目标执行状态机。

### 3.2 本轮暂不实现

- 不做多帧累积地图或全局涵洞地图。
- 不做完整 2.5D 高程地图；第一版默认地面坡度较缓，机器人 `base_link` 能近似跟随地面。
- 不区分墙体、静态障碍物和动态障碍物，第一版统一作为占用区域。
- 不做分支涵洞的拓扑决策，只处理单通道和缓弯通道。
- 不新增自定义消息、Action 或 BT 节点。
- 不拆成多个 ROS 节点。
- 不对引导路径做 B-Spline 或 g2o 优化。
- 不改 Nav2 规划器、平滑器和控制器。

## 4. 第一版数据流

```text
/livox/lidar
      |
      v
点云转换到 base_link
      |
      v
高度带过滤 + 局部栅格 + 射线清空
      |
      v
已知自由 / 占用 / 未知栅格
      |
      v
ESDF 距离场
      |
      v
从机器人格子运行一次 Dijkstra
      |
      v
选择可达的前方 frontier
      |
      v
回溯局部引导路径
      |
      v
按路径弧长选择巡检点
      |
      v
NavigateToPose -> 现有 Nav2 导航链路
```

这里生成的引导路径只用于判断巡检点位于涵洞的哪个位置，不直接作为控制器轨迹。

## 5. 栅格和 ESDF

### 5.1 局部栅格

第一版使用以 `base_link` 为中心、方向固定的滚动局部栅格，每次搜索直接根据最新点云重建：

```text
x: -1.0m ~ 10.0m
y: -4.0m ~ 4.0m
resolution: 0.10m
```

每个格子只保留三种状态：

```cpp
enum class GridState : std::uint8_t {
    Unknown,
    Free,
    Occupied
};

struct TunnelGrid {
    int width = 0;
    int height = 0;
    double resolution = 0.10;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    std::vector<GridState> states;
};
```

点云过滤建议初值：

```text
obstacle_min_height: 0.15m
obstacle_max_height: 1.30m
```

这样可以排除当前涵洞的地面和最低约 `1.6m` 的顶板，同时保留两侧墙和车身高度范围内的障碍物。

### 5.2 自由空间标记

不能只把点云落点标记为占用，因为这样无法区分自由区域和未知区域。

对每个有效点，从雷达原点到落点执行二维 Bresenham 射线遍历：

- 射线经过的格子标记为 `Free`。
- 射线终点标记为 `Occupied`。
- 未被射线经过的格子保持 `Unknown`。

搜索只允许经过 `Free`。未知区域本身不可通行，但与未知区域相邻的自由格可以成为前沿候选。

### 5.3 ESDF

构造 OpenCV 二值图：

```text
Occupied = 0
其他格子 = 255
```

使用：

```cpp
cv::distanceTransform(binary, distance_cells, cv::DIST_L2, cv::DIST_MASK_PRECISE);
```

将格子距离乘以分辨率得到米制净空距离。满足以下条件的自由格才允许进入搜索：

```text
esdf_distance >= robot_clearance
```

## 6. Dijkstra 搜索和候选选择

### 6.1 搜索节点

```cpp
struct SearchCell {
    double cost = infinity;
    int parent_index = -1;
    bool closed = false;
};
```

使用八邻域扩展，直移长度为 `resolution`，斜移长度为 `sqrt(2) * resolution`。

第一版边代价使用：

```text
edge_cost = move_distance *
            (1 + clearance_weight * exp(-esdf_distance / clearance_decay))
```

靠近墙体或障碍物时 ESDF 较小，搜索代价增大；自由空间中部 ESDF 较大，搜索代价接近普通距离代价。

第一版不把转角加入 Dijkstra 状态，避免立刻引入方向维度。引导路径不直接控制机器人，其离散折线只用于选目标点和估计局部朝向。

### 6.2 前沿候选

前沿格定义为：

- 当前格是已知自由格。
- 至少有一个相邻格是未知格。
- 已通过机器人净空检查。
- 能由 Dijkstra 从机器人位置到达。
- 与机器人的路径距离大于最低前进距离。

第一版假设涵洞没有岔路，在所有可达前沿中优先选择：

1. 路径距离足够长的候选。
2. 位于机器人前方的候选。
3. ESDF 净空较大的候选。

暂不处理分叉口语义选择。

### 6.3 回溯路径和巡检点

从选中的前沿格沿 `parent_index` 回溯到机器人格子，再反转得到局部引导路径。

巡检点不能按数组下标或路径中点选取，而应累计路径弧长：

```text
goal_distance = 5.0m
```

选择第一个累计弧长达到 `goal_distance` 的路径点。如果路径不足，则选择最远的安全路径点。

最终点还需要检查：

- 格子仍为已知自由区域。
- ESDF 净空满足机器人外接半径和安全余量。
- 与机器人距离大于最低目标距离。

如果目标不满足条件，就沿引导路径向机器人方向回退。目标朝向使用目标前后若干路径点计算的平均切线，避免直接使用单个八邻域方向。

## 7. 最小文件组织

第一版只增加一个算法模块，不拆分局部地图、ESDF、搜索和目标选择类：

```text
src/my_tunnel_guidance/
├── include/my_tunnel_guidance/
│   ├── tunnel_guidance_search.hpp      # 新增：栅格、ESDF、Dijkstra 和目标选择
│   ├── tunnel_guidance_node.hpp        # 修改：持有搜索器和最新搜索结果
│   └── tunnel_types.hpp                # 修改：补充栅格和搜索结果结构
├── src/
│   ├── tunnel_guidance_search.cpp      # 新增：第一版全部核心算法
│   └── tunnel_guidance_node.cpp        # 修改：调用搜索器并复用发布/Action逻辑
├── tools/
│   └── tunnel_guidance_demo.cpp        # 新增：OpenCV二维搜索动画和结果输出
├── test/
│   └── test_tunnel_guidance_search.cpp # 新增：直道、弯道、顶板和不可达测试
├── config/tunnel_guidance.yaml         # 修改：增加搜索参数
├── CMakeLists.txt                      # 修改：加入 OpenCV 和新源文件
└── package.xml                         # 修改：补充 OpenCV 依赖
```

现有 `tunnel_geometry_estimator.*` 暂时保留，用于对照、调试或后续提供局部方向先验，但第一版搜索成功时不再调用 `buildStraightCenterline()` 生成巡检目标。

## 8. 需要新增的最小接口

```cpp
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
    std::size_t expanded_count = 0;
};

using SearchDebugCallback =
    std::function<void(const SearchDebugFrame & frame)>;

class TunnelGuidanceSearch {
public:
    explicit TunnelGuidanceSearch(const TunnelGuidanceSearchParams & params);

    TunnelGuidanceSearchResult search(
        const std::vector<Eigen::Vector3d> & base_points) const;

    TunnelGuidanceSearchResult searchGrid(
        const TunnelGrid & grid,
        const SearchDebugCallback & debug_callback = {}) const;
};
```

内部栅格构建、射线遍历、ESDF、Dijkstra、前沿提取和目标回退先作为该类的私有函数，不再继续拆文件。

`searchGrid()` 是纯二维搜索入口，供单元测试和 OpenCV 演示程序使用。`search()` 负责把点云转换为 `TunnelGrid` 后复用 `searchGrid()`，避免演示程序和 ROS 节点各写一套算法。

`SearchDebugCallback` 只在演示程序传入。Dijkstra 每扩展指定数量的格子后触发一次回调，正式节点不传回调，因此不会承担生成动画的运行时开销。

## 9. OpenCV 二维快速可视化测试

这一阶段位于纯算法完成之后、接入现有 ROS 节点之前，用来排除点云、TF、时间戳和 Nav2 对算法调试的干扰。

### 9.1 测试地图

`tunnel_guidance_demo` 使用 OpenCV 直接生成一张长廊二维代价地图，第一版至少包含：

- 一段直线入口。
- 一段缓弯或 S 形通道。
- 通道外部占用区域。
- 通道末端的未知区域，用于形成前沿候选。
- 一个简单盒状障碍物，用于确认 ESDF 代价和可达性有效。
- 明确的机器人起点，但不预先给定最终巡检点。

演示程序把 OpenCV 图像转换为 `TunnelGrid`，调用正式算法的 `searchGrid()`。测试程序不能复制一份简化版 Dijkstra，否则无法证明节点将要使用的算法正确。

### 9.2 搜索动画

Dijkstra 搜索过程中每隔例如 `50~100` 次格子扩展保存一帧，画面至少显示：

- 黑色：占用区域。
- 灰色：未知区域。
- 白色：已知自由区域。
- 由冷到暖的颜色：已经扩展格子的累计代价或扩展顺序。
- 蓝色圆点：机器人起点。
- 黄色区域：候选前沿。
- 绿色折线：搜索完成后回溯出的引导路径。
- 红色圆点和箭头：最终巡检点及朝向。

优先使用 OpenCV `VideoWriter` 输出 `dijkstra_search.avi`。如果当前环境的视频编码器不可用，至少保留连续编号的 PNG 帧，不能因为视频写入失败而使测试失败。

### 9.3 最终输出

演示程序接受输出目录参数，例如：

```bash
ros2 run my_tunnel_guidance tunnel_guidance_demo \
    --output-dir /tmp/tunnel_guidance_demo
```

输出至少包括：

```text
input_costmap.png          # 原始长廊代价地图
esdf_heatmap.png           # ESDF距离场热力图
search_cost.png            # Dijkstra累计代价或扩展顺序图
frontier_candidates.png    # 所有可达前沿候选
final_guidance.png         # 最终路径、巡检点和净空信息
dijkstra_search.avi        # 搜索过程动画，编码器不可用时输出frames/*.png
```

终端同时输出：

- 栅格尺寸和分辨率。
- 自由、占用和未知格数量。
- Dijkstra 扩展格数量和搜索耗时。
- 可达前沿数量。
- 引导路径长度。
- 巡检点坐标、路径弧长和 ESDF 净空。

### 9.4 快速测试通过条件

- 引导路径始终位于已知自由空间内。
- 引导路径能够沿缓弯长廊延伸，不穿过墙体。
- 搜索在障碍物附近向高 ESDF 区域偏移。
- 巡检点通过路径弧长选定，而不是直接取欧氏距离或数组中点。
- 巡检点净空不小于 `robot_clearance`。
- 动画最后一帧和 `final_guidance.png` 中的路径、目标一致。
- 关闭调试回调后，算法不执行任何图像和视频生成代码。

只有这一步通过后，才进入 ROS 节点集成阶段。

## 10. 节点集成方式

1. `pointCloudCallback()` 保留原有点云转换、标定、墙模型更新、出口检测和分类点云发布流程。
2. 新增 `search_requested_` 作为一次性搜索请求；节点启动时置位，首次有效点云到来时规划巡检点。
3. 搜索成功时，将局部引导路径转换到 `output_frame`，并发布 `centerline` 和 `local_goal`。
4. 向 Nav2 发送该巡检点后进入 `GoalActive`，活动目标期间不运行 Dijkstra。
5. 到达巡检点后进入 `Dwelling`，等待 `auto_goal_dwell_time` 表示本次巡检完成。
6. 巡检完成后进入 `PlanningRequested`，在下一帧有效点云上重新运行一次完整搜索。
7. 新搜索结果直接使用算法给出的 `goal`，不沿旧路径按候选索引或剩余距离继续推进目标。
8. 目标被拒绝或导航失败时请求一次全新搜索；目标被取消时停止自动巡检。
9. 搜索失败时保持请求标志，在下一帧点云上重试，不在定时器中重复转换或搜索同一帧。

节点状态保持为：

```text
WaitingForCloud
    -> PlanningRequested
    -> GoalReady
    -> GoalActive
    -> Dwelling
    -> PlanningRequested
```

该状态机表达“规划一次、行驶、巡检、再规划一次”，不能退化为点云回调中的连续滚动规划。

## 11. 实施阶段

### 阶段 1：纯算法和单元测试

- 新增栅格、射线标记和坐标转换。
- 新增 ESDF 计算。
- 新增 Dijkstra、前沿提取、路径回溯和弧长选点。
- 构造直涵洞和缓弯涵洞测试数据。
- 加入倾斜顶板点，确认顶板不会被投影成二维障碍物。

### 阶段 2：OpenCV 二维快速测试

- 新增 `tunnel_guidance_demo` 独立可执行程序。
- 生成包含缓弯、未知末端和简单障碍物的长廊代价地图。
- 通过 `searchGrid()` 调用正式 Dijkstra 算法。
- 输出搜索过程动画、ESDF 热力图、搜索代价图和最终结果图。
- 根据可视化结果先修正搜索和选点问题。

### 阶段 3：接入现有节点

- 复用现有点云回调和 TF 结果，仅在 `search_requested_` 置位时调用搜索器。
- 替换固定直线中心线和固定索引选点。
- 删除候选索引、旧路径剩余距离推进和逐帧搜索失败计数。
- 到点停留结束后使用最新点云重新规划下一巡检点。
- 复用现有话题、Marker 和 Nav2 Action 接口。
- YAML 增加搜索参数。

### 阶段 4：仿真和 rosbag 验证

- 在直涵洞中确认引导路径位于通道中部。
- 增加缓弯涵洞模型，确认路径能够跟随弯道。
- 使用 `/home/goose/rosbags/culvert_mid360_01` 验证顶板不会阻止搜索。
- 检查目标执行期间不会持续重复发送 `NavigateToPose`。

当前 rosbag 只有 `/ground_truth` 和 `/tf_static`，没有动态 `/tf`。整节点回放时需要同步启动现有 ground-truth 到 `odom -> base_footprint` 的 TF 发布链路；纯搜索器单元测试不依赖 TF。

## 12. 验证标准

### 构建和测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_tunnel_guidance --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select my_tunnel_guidance
colcon test-result --verbose
```

### OpenCV 快速测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run my_tunnel_guidance tunnel_guidance_demo \
    --output-dir /tmp/tunnel_guidance_demo
```

检查生成的搜索动画、`esdf_heatmap.png` 和 `final_guidance.png`，并确认终端统计中的路径长度、目标净空和搜索耗时合理。

### 行为标准

- 带顶板点云下可以生成有效引导路径和巡检点。
- 直涵洞中的目标点位于通道中部附近。
- 缓弯涵洞中的目标点沿自由空间连通路径前进，不穿过墙体。
- 目标点净空不小于 `robot_clearance`。
- 目标按照路径弧长选择，而不是按照数组下标选择。
- Nav2 仍负责最终路径规划、平滑和控制。
- 活动目标未完成时不会因为点云逐帧变化而反复重发目标。

## 13. 后续升级条件

只有第一版运行后出现对应问题，才增加以下内容：

- 搜索耗时明显偏高：将 Dijkstra 改为多目标 A*。
- 单帧点云看不到完整弯道：增加短时间窗口的局部栅格累积。
- 地面坡度使固定高度带失效：升级为 2.5D 地面高度和净空地图。
- 障碍物导致引导路径偏离涵洞结构中心：拆分结构墙体层和通行障碍物层。
- 分叉涵洞选错方向：增加历史路线、拓扑图或任务级方向约束。
- 引导路径或目标抖动：增加历史一致性代价和目标切换滞回。
