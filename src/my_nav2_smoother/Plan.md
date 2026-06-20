# BSpline 平滑器 2D 几何安全走廊计划

## 目标

给 `MyBSplineSmoother` 增加 2D 几何安全走廊，让平滑后的路径始终位于 Nav2 costmap 表示的无碰撞区域内。

当前平滑器已经具备：

- B-Spline / QP 路径平滑。
- 初始时间分配。
- 速度、加速度、jerk 动力学可行性检测。
- 动力学超限后的时间膨胀迭代。

本计划补齐几何侧约束：

- 基于 2D costmap 生成局部安全矩形 box。
- 使用这些 box 作为 OSQP 的位置上下界。
- 每次求解后检查轨迹是否仍在安全走廊内。
- 最后把几何修正和现有动力学迭代合并到同一个外层循环。

## 设计选择

使用 2D AABB 矩形安全走廊，不使用 OctoMap 或 3D 隔离箱。

原因：

- Nav2 平滑器输入输出是 `nav_msgs::msg::Path`，当前机器人路径主要是 2D 平面轨迹。
- `MyBSplineSmoother` 已经通过插件接口拿到了 `nav2_costmap_2d::CostmapSubscriber`。
- 安全性应该和 Nav2 规划/控制使用的 inflated costmap 保持一致。
- AABB 矩形可以直接映射到当前 OSQP 的简单上下界约束：

```text
lower_x[i] <= x_i <= upper_x[i]
lower_y[i] <= y_i <= upper_y[i]
```

不要对重叠矩形做几何差集。矩形相减后可能变成非凸区域或 L 形区域，无法继续用当前简单上下界 QP 表达。

## 新增数据结构

在 `bspline_smoother.hpp` 中新增：

```cpp
struct CorridorBounds {
    std::vector<double> lower_x;
    std::vector<double> upper_x;
    std::vector<double> lower_y;
    std::vector<double> upper_y;
};

struct GridBox {
    int min_mx = 0;
    int max_mx = 0;
    int min_my = 0;
    int max_my = 0;
};

struct CorridorViolation {
    int index = -1;
    double distance = 0.0;
};

struct CorridorReport {
    bool ok = true;
    std::vector<CorridorViolation> point_violations;
    std::vector<CorridorViolation> segment_violations;
};
```

语义：

- `CorridorBounds`：最终给 OSQP 使用的每个轨迹点的 x/y 上下界。
- `GridBox`：costmap 栅格坐标下的安全矩形。
- `CorridorReport`：几何安全检测结果，告诉外层循环哪些点或线段违反安全走廊。

## 新增参数

在 `configure()` 中新增参数：

```cpp
double corridor_max_expand_dist_ = 0.8;
double corridor_min_half_width_ = 0.05;
double corridor_overlap_threshold_ = 0.8;
double corridor_collision_check_resolution_ = 0.03;
unsigned char corridor_lethal_cost_threshold_ =
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
```

建议 YAML 初始值：

```yaml
corridor_max_expand_dist: 0.8
corridor_min_half_width: 0.05
corridor_overlap_threshold: 0.8
corridor_collision_check_resolution: 0.03
```

第一版使用 inflated costmap 作为安全来源。建议把 cost 大于等于 `INSCRIBED_INFLATED_OBSTACLE` 的 cell 视为不可通行。

`NO_INFORMATION` 第一版建议视为不可通行，这样更保守。

## 阶段 1：Costmap 辅助函数

新增以下辅助函数：

```cpp
bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const;

bool isCellFree(int mx, int my) const;

bool isColumnFree(int mx, int min_my, int max_my) const;

bool isRowFree(int my, int min_mx, int max_mx) const;
```

实现规则：

- costmap 不可用时返回 false。
- 地图索引越界时返回 false。
- lethal / inscribed / inflated obstacle cell 返回 false。
- unknown cell 第一版返回 false。
- 行列检查只检查“新增边界”，不要每次全 box 扫描，降低开销。

## 阶段 2：从路径点膨胀局部安全 box

对每个参考路径点生成一个原始 AABB。

新增函数：

```cpp
GridBox expandBoxFromCell(int seed_mx, int seed_my, int max_expand_cells) const;
```

扩张流程：

```text
以 seed cell 初始化 1-cell box
while 还能扩张:
    尝试向 +x 增加一列
    尝试向 -x 增加一列
    尝试向 +y 增加一行
    尝试向 -y 增加一行
    只有新增行/列全部 free 才接受扩张
    任一方向达到 max_expand_cells 就停止该方向
```

重要约束：

- box 只围绕路径点做局部扩张。
- 不要扩张到整个 free space。
- 遇到地图边界、障碍物、unknown cell 或最大扩张距离后停止。

把最大扩张距离转换成栅格数：

```cpp
int max_expand_cells = static_cast<int>(
    std::ceil(corridor_max_expand_dist_ / costmap->getResolution()));
```

## 阶段 3：安全 box 去重策略

第一版优化约束层不做 box 复用。

关键原则：

- 每个路径点直接使用自己膨胀出来的 `raw_box_i` 生成 `bounds[i]`。
- 不需要在优化约束层查找旧 box，也不需要决定某个点是否复用旧 box。
- 这样最简单、最稳定，也避免旧 box 过大导致平滑路径自由度过大。
- box 去重只用于 RViz 可视化或调试统计，不参与 OSQP 约束生成。

优化约束层第一版只需要：

```text
for each raw_box_i:
    bounds[i] = raw_box_i 对应的世界坐标边界
```

可视化去重可选函数：

```cpp
bool contains(const GridBox &a, const GridBox &b) const;

bool hasIntersection(const GridBox &a, const GridBox &b) const;

double overlapRatio(const GridBox &a, const GridBox &b) const;

GridBox mergeByIntersection(const GridBox &a, const GridBox &b) const;

std::vector<GridBox> deduplicateBoxesForVisualization(
    const std::vector<GridBox> &raw_boxes) const;
```

可视化层可以做这些处理：

- 如果已有 box 包含当前 box，可不显示当前 box。
- 如果当前 box 包含已有 box，可替换显示用 box。
- 如果两个 box 高度重叠，可以用交集合并作为显示用 box。
- 如果两个 box 没有交集，保留为两个独立显示 box。

为什么可视化高重叠时取交集：

- 安全走廊是约束，应该保守。
- 取并集可能包含障碍物附近的不安全区域。
- 取交集仍然是矩形，能继续作为保守显示区域。

注意：可视化去重结果不能反过来修改 `bounds[i]`。优化约束必须按路径点逐个赋值。

## 阶段 4：生成 `CorridorBounds`

新增函数：

```cpp
CorridorBounds buildCorridorBounds(
    const std::vector<double> &p_ref_x,
    const std::vector<double> &p_ref_y) const;
```

流程：

```text
for 每个参考路径点 i:
    world 坐标转 costmap 坐标
    从该 cell 膨胀 raw_box_i
    bounds[i] = raw_box_i 对应的世界坐标边界

固定前两个点和最后两个点到原始路径位置
```

注意：第一版不复用旧 box，不做路径点范围合并。每个路径点的 OSQP 上下界只由该点自己的安全 box 决定。

把 `GridBox` 转成世界坐标边界：

```cpp
costmap->mapToWorld(box.min_mx, box.min_my, min_x, min_y);
costmap->mapToWorld(box.max_mx, box.max_my, max_x, max_y);
```

确保边界至少有最小宽度：

```cpp
upper_x[i] - lower_x[i] >= 2.0 * corridor_min_half_width_
upper_y[i] - lower_y[i] >= 2.0 * corridor_min_half_width_
```

如果路径点在 costmap 外，或者 seed cell 本身不可通行，第一版不要直接失败。可以 fallback 到参考点附近的小 box，保证 smoother 在 costmap 暂时不完整时仍然鲁棒。

## 阶段 5：替换 OSQP 固定边界

当前代码中固定边界类似：

```cpp
l_x(i) = p_ref_x[i] - 1.0;
u_x(i) = p_ref_x[i] + 1.0;
l_y(i) = p_ref_y[i] - 1.0;
u_y(i) = p_ref_y[i] + 1.0;
```

应替换为 `CorridorBounds`。

修改 `solveBSplineQPOnce()` 签名：

```cpp
bool solveBSplineQPOnce(
    const std::vector<double> &p_ref_x,
    const std::vector<double> &p_ref_y,
    const std::vector<double> &dt_segment,
    const CorridorBounds &bounds,
    double w_s,
    double w_g,
    std::vector<double> &p_smooth_x,
    std::vector<double> &p_smooth_y);
```

x 方向求解时使用：

```cpp
data->l = bounds.lower_x.data();
data->u = bounds.upper_x.data();
```

y 方向求解时使用：

```cpp
osqp_update_bounds(work, bounds.lower_y.data(), bounds.upper_y.data());
```

## 阶段 6：几何安全检测

先做点约束检测，再做线段碰撞检测。

新增点检测函数：

```cpp
CorridorReport checkCorridorFeasibility(
    const std::vector<double> &p_x,
    const std::vector<double> &p_y,
    const CorridorBounds &bounds) const;
```

点检测逻辑：

```text
for 每个平滑后路径点:
    检查 x 是否在 [lower_x, upper_x]
    检查 y 是否在 [lower_y, upper_y]
```

然后新增线段检测函数：

```cpp
bool isSegmentCollisionFree(
    double x0, double y0,
    double x1, double y1) const;
```

线段检测逻辑：

- 按 `corridor_collision_check_resolution_` 沿线段采样。
- 每个采样点转 costmap cell。
- 调用 `isCellFree()` 检查是否可通行。

## 阶段 7：几何失败处理

几何失败不要改 `dt_segment`。

几何失败应该修改：

- `CorridorBounds`
- 或局部 guide 权重
- 或后续插入锚点

第一版先只做收紧 bounds。

新增函数：

```cpp
void tightenCorridorBounds(
    const CorridorReport &report,
    const std::vector<double> &p_ref_x,
    const std::vector<double> &p_ref_y,
    CorridorBounds &bounds) const;
```

第一版处理策略：

- 对 point violation，收紧该点和相邻点的 bounds，使其更靠近参考路径点。
- 对 segment violation，收紧线段两端点的 bounds。
- 暂时不引入局部 guide 权重矩阵，避免一次改动过大。

目标是：当平滑曲线试图离开局部安全区域时，把它拉回参考路径附近。

## 阶段 8：统一外层迭代

不要分别做“动力学迭代”和“几何迭代”。应该使用一个统一外层循环。

目标结构：

```cpp
bool MyBSplineSmoother::solveBSplineQP(...) {
    std::vector<double> dt_segment;
    computeTimeAllocation(p_ref_x, p_ref_y, dt_segment);

    CorridorBounds bounds = buildCorridorBounds(p_ref_x, p_ref_y);

    for (int iter = 0; iter < max_outer_iterations_; ++iter) {
        if (!solveBSplineQPOnce(
                p_ref_x, p_ref_y,
                dt_segment, bounds,
                w_s, w_g,
                p_smooth_x, p_smooth_y)) {
            return false;
        }

        DynamicReport dyn_report =
            checkDynamicFeasibility(p_smooth_x, p_smooth_y, dt_segment);

        CorridorReport corridor_report =
            checkCorridorFeasibility(p_smooth_x, p_smooth_y, bounds);

        if (dyn_report.ok && corridor_report.ok) {
            return true;
        }

        if (!dyn_report.ok) {
            inflateTimeAllocation(dyn_report, dt_segment);
        }

        if (!corridor_report.ok) {
            tightenCorridorBounds(corridor_report, p_ref_x, p_ref_y, bounds);
        }
    }

    return !p_smooth_x.empty() && !p_smooth_y.empty();
}
```

动力学问题修改时间分配。

几何问题修改安全边界。

这两类修正必须在同一个外层循环中统一处理，避免两个独立迭代互相打架。

## 验证计划

构建验证：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_nav2_smoother
```

静态检查：

- 确认 `solveBSplineQPOnce()` 接收 `CorridorBounds`。
- 确认固定 `+-1.0` 位置边界已经删除。
- 确认前两个点和最后两个点仍然固定。
- 确认 unknown / 越界 costmap cell 有明确处理。

运行检查：

- 在窄通道场景中规划，确认平滑路径不会切角穿障碍。
- 在 RViz 中同时观察原始路径、平滑路径和安全 box。
- 打印每轮外层迭代的最大速度、最大加速度、最大 jerk、几何 violation 数量。
- costmap 暂时不可用时，smoother 应该返回原始路径或保守平滑结果，而不是崩溃。

## 推荐实现顺序

1. 添加数据结构和参数。
2. 添加 costmap helper 函数。
3. 实现 `expandBoxFromCell()`。
4. 实现可选的 box 可视化去重函数。
5. 实现 `buildCorridorBounds()`。
6. 修改 `solveBSplineQPOnce()` 使用 `CorridorBounds`。
7. 实现 `checkCorridorFeasibility()` 点检测。
8. 实现线段碰撞检测。
9. 实现 `tightenCorridorBounds()`。
10. 把动力学 report 和几何 report 合并进同一个外层循环。

## 注意事项

- 第一版只做 AABB 矩形走廊。
- 允许相邻 box 重叠。
- 不做 box 差集。
- 不引入 OctoMap，除非后续路径规划变成真正 3D。
- 动力学 violation 修改 `dt_segment`。
- 几何 violation 修改 `CorridorBounds`。
- 如果几何约束过紧导致 OSQP 失败，应 fallback 到原始路径或上一轮可行路径。

## 阶段 9：升级为笔记中的后验超调约束法

前面阶段 1-8 是第一版工程实现：只给每个路径点设置 AABB 上下界，并在求解后用点检测和线段采样检查是否碰撞。这个版本能工作，但它不是笔记里最严格的“轨迹超调后添加约束”方法。

笔记里的方法要求：

- 先求解一次轨迹。
- 再检查连续曲线是否超出 Bounding Box。
- 如果越界，在越界位置添加新的硬约束。
- 重新求解 QP，直到没有越界或达到最大迭代次数。

这个阶段的核心不是继续收紧 `bounds[i]`，而是修改 OSQP 的约束矩阵 `A`，让它能约束 B-Spline 曲线中间位置：

```text
l <= A x <= u
```

当前 `A = I`，只能表达：

```text
lower[i] <= x_i <= upper[i]
```

升级后，`A` 需要包含额外行，表达某个 B-Spline 段在参数 `u` 处的位置约束：

```text
box_min <= b0(u) * x_i
        + b1(u) * x_{i+1}
        + b2(u) * x_{i+2}
        + b3(u) * x_{i+3}
        <= box_max
```

其中 `b0..b3` 是三次 uniform B-Spline 基函数。

### 9.1 新增曲线约束结构

在 `bspline_smoother.hpp` 中新增：

```cpp
struct SplinePointConstraint {
    int segment_index = -1;
    double u = 0.0;
    double lower_x = 0.0;
    double upper_x = 0.0;
    double lower_y = 0.0;
    double upper_y = 0.0;
};

struct SplineOvershoot {
    int segment_index = -1;
    double u = 0.0;
    double x = 0.0;
    double y = 0.0;
    double dx = 0.0;
    double dy = 0.0;
};

struct SplineOvershootReport {
    bool ok = true;
    std::vector<SplineOvershoot> overshoots;
};
```

语义：

- `SplinePointConstraint`：写入 OSQP 约束矩阵 `A` 的一条曲线中间点硬约束。
- `SplineOvershoot`：一次求解后检测到的越界位置。
- `SplineOvershootReport`：连续曲线超调检测结果。

### 9.2 新增 B-Spline 基函数工具

新增函数：

```cpp
std::array<double, 4> cubicBSplineBasis(double u) const;

std::array<double, 4> cubicBSplineBasisDerivative(double u) const;

PlannerPoint evaluateSplinePoint(
    const std::vector<double> &p_x,
    const std::vector<double> &p_y,
    int segment_index,
    double u) const;
```

其中 `u` 范围是 `[0.0, 1.0]`。

三次 uniform B-Spline 基函数：

```text
b0 = (1 - 3u + 3u^2 - u^3) / 6
b1 = (4 - 6u^2 + 3u^3) / 6
b2 = (1 + 3u + 3u^2 - 3u^3) / 6
b3 = u^3 / 6
```

这些函数后续同时用于：

- 构建 OSQP 的额外约束行。
- 在求解后评估曲线中间点。
- 做密集采样或极值点检测。

### 9.3 先做密集采样版超调检测

第一版不要立刻解析求 `dx/du = 0` 和 `dy/du = 0` 的根。先实现密集采样版，代码更简单，也更容易验证。

新增函数：

```cpp
SplineOvershootReport checkSplineOvershootBySampling(
    const std::vector<double> &p_x,
    const std::vector<double> &p_y,
    const std::vector<GridBox> &segment_boxes) const;
```

检测流程：

```text
for 每个 B-Spline 段 i:
    获取该段对应的安全 box
    for u = 0.0 到 1.0，按 overshoot_check_resolution 采样:
        用 B-Spline 基函数计算曲线点 p(u)
        如果 p(u) 超出 box，记录 overshoot
```

新增参数：

```cpp
double overshoot_check_resolution_ = 0.05;
int max_overshoot_constraints_per_iter_ = 20;
```

建议 YAML 初始值：

```yaml
overshoot_check_resolution: 0.05
max_overshoot_constraints_per_iter: 20
```

注意：采样间隔越小越安全，但 QP 约束行会增加，计算量也会上升。

### 9.4 再做极值点版超调检测

密集采样验证稳定后，再升级成笔记里的极值点法。

新增函数：

```cpp
std::vector<double> findSplineExtremaU(
    const std::vector<double> &p,
    int segment_index) const;
```

对 x 方向和 y 方向分别求：

```text
dp(u) / du = 0
```

因为三次 B-Spline 的导数是二次多项式，所以求根可以直接用二次方程公式。只保留 `u ∈ [0, 1]` 的根。

最终每段需要检查的 `u` 集合：

```text
{0.0, 1.0}
+ x 方向极值点
+ y 方向极值点
+ 可选的少量中间采样点
```

这样才严格对应笔记里：

```text
使用 dp(t)/dt 求轨迹多项式函数的极值点，判断是否超过 Bounding Box。
```

### 9.5 修改 `solveBSplineQPOnce()` 支持额外约束

修改函数签名：

```cpp
bool solveBSplineQPOnce(
    const std::vector<double> &p_ref_x,
    const std::vector<double> &p_ref_y,
    const std::vector<double> &dt_segment,
    const CorridorBounds &bounds,
    const std::vector<SplinePointConstraint> &extra_constraints,
    double w_s,
    double w_g,
    std::vector<double> &p_smooth_x,
    std::vector<double> &p_smooth_y);
```

OSQP 约束数量从：

```cpp
data->m = n;
```

改成：

```cpp
data->m = n + extra_constraints.size();
```

约束矩阵 `A` 从单位矩阵改成：

```text
前 n 行：单位矩阵，约束控制点/路径点本身
后 extra_constraints.size() 行：B-Spline 基函数线性组合，约束曲线中间点
```

构造规则：

```cpp
// 前 n 行
A(row=i, col=i) = 1.0;

// 额外约束行
const int row = n + k;
const int i = constraint.segment_index;
const auto basis = cubicBSplineBasis(constraint.u);
A(row, i + 0) = basis[0];
A(row, i + 1) = basis[1];
A(row, i + 2) = basis[2];
A(row, i + 3) = basis[3];
```

x 方向上下界：

```text
前 n 行：bounds.lower_x / bounds.upper_x
额外行：constraint.lower_x / constraint.upper_x
```

y 方向上下界：

```text
前 n 行：bounds.lower_y / bounds.upper_y
额外行：constraint.lower_y / constraint.upper_y
```

### 9.6 从超调报告生成额外约束

新增函数：

```cpp
void appendOvershootConstraints(
    const SplineOvershootReport &report,
    const std::vector<GridBox> &segment_boxes,
    std::vector<SplinePointConstraint> &extra_constraints) const;
```

处理策略：

- 对每个 overshoot，根据 `segment_index` 找到对应 box。
- 在该 `u` 位置添加一条 `SplinePointConstraint`。
- 上下界使用对应 box 的世界坐标边界。
- 每轮最多添加 `max_overshoot_constraints_per_iter_` 条，避免 QP 规模无限增长。
- 添加前检查是否已有相同 `segment_index` 和近似相同 `u` 的约束，避免重复。

### 9.7 外层迭代流程

阶段 9 后，外层循环变成：

```cpp
std::vector<SplinePointConstraint> extra_constraints;

for (int iter = 0; iter < max_outer_iterations_; ++iter) {
    solveBSplineQPOnce(
        p_ref_x,
        p_ref_y,
        dt_segment,
        bounds,
        extra_constraints,
        w_s,
        w_g,
        p_smooth_x,
        p_smooth_y);

    DynamicReport dyn_report =
        checkDynamicFeasibility(p_smooth_x, p_smooth_y, dt_segment);

    SplineOvershootReport overshoot_report =
        checkSplineOvershootBySampling(p_smooth_x, p_smooth_y, segment_boxes);

    if (dyn_report.ok && overshoot_report.ok) {
        return true;
    }

    if (!dyn_report.ok) {
        inflateTimeAllocation(dyn_report, dt_segment);
    }

    if (!overshoot_report.ok) {
        appendOvershootConstraints(
            overshoot_report,
            segment_boxes,
            extra_constraints);
    }
}
```

这里几何失败不再优先使用 `tightenCorridorBounds()`，而是添加新的 `A` 矩阵行约束。`tightenCorridorBounds()` 可以保留为 fallback。

### 9.8 与安全走廊 box 的关系

阶段 9 需要保留每段曲线对应的 box，而不只是每个点的 `CorridorBounds`。

新增或调整函数：

```cpp
std::vector<GridBox> buildSegmentCorridorBoxes(
    const std::vector<double> &p_ref_x,
    const std::vector<double> &p_ref_y) const;
```

第一版段 box 可以这样选：

```text
segment i 使用 raw_box[i + 1] 或 raw_box[i + 2]
```

更保守的方式：

```text
segment i 使用 raw_box[i], raw_box[i+1], raw_box[i+2], raw_box[i+3] 的交集
```

如果交集为空，fallback 到该段中点对应的扩张 box。

注意：用并集不安全，因为并集可能跨过障碍物附近区域，而且不一定是凸矩形。

### 9.9 验证顺序

先验证简单场景：

- 空旷地图中，extra constraints 应该很少或为 0。
- 窄通道切角场景中，第一次 QP 可能超调，第二次或第三次 QP 应添加额外约束并回到走廊内。
- 打印每轮 `extra_constraints.size()` 和 `overshoot_report.overshoots.size()`。

再验证压力场景：

- S 形窄通道。
- 贴障碍物转弯。
- 原始路径点稀疏导致 B-Spline 明显切角的场景。

构建验证：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_nav2_smoother
```

运行观察：

- RViz 中显示原始路径、平滑路径、segment box。
- 如果平滑路径切角，下一轮应出现 overshoot constraint。
- 如果约束太多导致 OSQP 失败，应退回上一轮可行路径或原始路径。

## 阶段 10：Bezier 凸包硬约束版本

这是更严格但改动更大的版本，对应笔记里的 Bezier 走廊约束。

核心思想：

- 把每段三次 B-Spline 转成等价 Bezier 控制点。
- 利用 Bezier 凸包性质，只要该段所有 Bezier 控制点都在 box 内，整段曲线就在 box 内。
- 因此不需要密集采样，也不需要只靠后验修正。

这要求新增 B-Spline 到 Bezier 的线性变换：

```text
Q_bezier = M * P_bspline
```

然后把每个 Bezier 控制点约束写成 OSQP 的额外线性约束行。

第一阶段不建议直接做这个版本，因为需要更仔细地处理：

- 每段 box 分配。
- 相邻段 box 的 overlap region。
- 端点连续性和控制点约束。
- OSQP 约束数量显著增加。

推荐路线：

```text
先做阶段 9 的采样/极值点后验约束
稳定后再升级阶段 10 的 Bezier 凸包硬约束
```
