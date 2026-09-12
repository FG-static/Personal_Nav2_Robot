# MEMORY

Working notes for this machine and the `humble` branch. Not a substitute for [`src/HUMBLE_PORTING.md`](src/HUMBLE_PORTING.md) or [`src/README.md`](src/README.md).

Last updated: 2026-09-12.

## People

- User: **FGoose**, software engineer. GitHub: `FG-static`. Email: `meis38@126.com`.
- Prefers working directly in code: implement, debug, iterate. Do not commit or push unless asked.

## Machine

- Host: `nav-THUNDEROBOT`, Ubuntu **22.04**, ROS 2 **Humble**, Gazebo **Classic 11**.
- Default chassis on this branch: **diff**, not mecanum.
- Miniconda Python **3.13** is often first on `PATH`. Humble `rclpy` / launch need **`/usr/bin/python3` (3.10)**. Always `conda deactivate` (or use `quick_source.sh`) before build/launch.
- Sophus is installed under `/usr/local`. OSQP comes from `osqp_vendor` in this workspace plus Humble `osqp`.
- Livox-SDK2 is already installed (`/usr/local/lib/liblivox_lidar_sdk_shared.so`). That is **not** the ROS driver.

## Repos and workspaces

| Path | What |
| --- | --- |
| `/home/nav/nav2_test` | This repo. Git: `https://github.com/FG-static/Personal_Nav2_Robot.git`. **Work here.** |
| `/home/nav/culvert_nav` | Historical Humble run workspace, **not git**. Do not treat as source of truth. |
| `/home/nav/bievr_ws` | BIEVR-LIO. Has its own `quick_source.sh` (Humble + user Ceres 2.2). |
| `/home/nav/livox_ws` | Sibling ROS 2 workspace for `livox_ros_driver2` only. Not part of this git repo. |
| `FG-static/small_point_lio_lc` | Upstream LIO repo. Its `quick_source.sh` is Jazzy + `/home/goose/fastlio/livox_ws`. Cannot be used as-is here. |

Related GitHub: `FG-static/small_point_lio_lc`.

## Git policy

- **`main`**: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic. Do not run it on this machine.
- **`humble`**: Ubuntu 22.04 + Humble + Gazebo Classic. Current checkout.
- When `main` moves, **do not cherry-pick blindly**. Port to Humble-runnable form first, then apply locally. User said so explicitly.
- Local `main` may lag `origin/main`. That is expected; this tree tracks `humble`.
- Last pushed humble tip at last check: `2eae617` *Fit inspection goal heading over a 2 m path window.*
- `origin/main` had two commits **not** yet on `origin/humble`:
  - `702af0f` Improve A* search and B-Spline smoother robustness
  - `8eb147b` Add Mid360 hardware bringup support
  These were ported locally (uncommitted working tree) on 2026-09-11/12.

## Humble API that must not be overwritten from `main`

- A* plugin: `createPlan(start, goal)` only. Humble `nav2_core::GlobalPlanner` has **no** `cancel_checker` argument. Unit tests may pass a checker into `searchCells()`.
- Exceptions: only `nav2_core::PlannerException` in `nav2_core/exceptions.hpp`. No `planner_exceptions.hpp`, no `GoalOccupied` / `StartOutsideMapBounds` / `GoalOutsideMapBounds` / `NoValidPathCouldBeFound`.
- B-Spline: keep Humble `getActiveCostmap()` snapshot cache. Do not switch back to `costmap_sub_->getCostmap()` on every query.
- Humble B-Spline already pins **only start and goal** and uses chord-based yaw. Main’s older “pin last two A* cells” is worse; do not restore it.
- `allow_capture_done` yaml default is **false** for sim (no MCU handshake).
- Inspection dwell recording uses **raw Mid360** `/livox/lidar` → `map`, not the navigation ROI. Output dir `/tmp/tunnel_inspections`.
- Inspection goal heading: `search_goal_heading_window: 2.0` (least-squares over ~2 m of path). Already on humble as `2eae617`.
- Simulation stays Gazebo Classic. Do **not** add Jazzy `gz_plugin_vendor` / `ros_gz_*`.
- `tunnel_guidance.launch.py` on humble keeps launch args (`use_sim_time`, `enable_auto_goal` default true, `allow_capture_done` false, `dataset_output_dir`). Do not strip it to yaml-only like later `main`.
- `full_navigation_real.launch.py` was previously **not** ported by request. Hardware bringup is a **different** file (`hardware_bringup.launch.py`) and **was** ported.

## Local uncommitted port (as of 2026-09-12)

Working tree on `humble` (not committed unless FGoose asks):

- A*: 8-neighbor search extracted as `searchCells()` + `isBlockedCell()`; no diagonal corner-cut; goal-on-lethal throws `PlannerException`; start-on-lethal warns and continues; start==goal occupied is a one-cell success; metrics throttle `metrics_min_interval` (default 1.0 s); failures throw instead of empty path.
- B-Spline: invalid corridor seed → `GridBox.valid=false`; `corridor_violation_tolerance` default `1e-4`; QP fail falls back to raw path and **returns true** (avoid BT `ClearEntireCostmap`); OSQP `solutionUsable`; if x solve fails, skip y.
- Tests: `src/my_nav2_planner/test/test_astar_planner.cpp`, `src/my_nav2_smoother/test/test_bspline_smoother.cpp`. Both passed after the port. CMake skips uncrustify (K&R vs ROS default). Keep OSQP include-dir workaround in smoother CMake.
- YAML (all four param files that already had A*/BSpline): `metrics_min_interval: 1.0`; BSpline also `corridor_violation_tolerance: 1.0e-4`.
- Hardware bringup files from `main`, Humble-tweaked:
  - `src/my_nav2_robot/launch/hardware_bringup.launch.py` (default `chassis:=diff`, `robot_description` wrapped in `ParameterValue(..., value_type=str)`)
  - `src/my_nav2_robot/launch/livox_mid360.launch.py`
  - `src/my_nav2_robot/config/MID360_config.json` (**strict JSON**, no `//` comments; RapidJSON in the official driver rejects comments)
  - `src/my_nav2_robot/scripts/check_mid360_net.sh`
- `package.xml`: `exec_depend` `joint_state_publisher` only for this bringup. No `gz_plugin_vendor`.

## How to source on this machine

**Nav2 + Mid360 driver (this workspace):**

```bash
source /home/nav/nav2_test/quick_source.sh
```

That script is a Humble rewrite of `small_point_lio_lc/quick_source.sh`. It:

1. Drops conda from `PATH`
2. Sources `/opt/ros/humble/setup.bash`
3. Sources `/home/nav/livox_ws/install/setup.bash`
4. Sources `/home/nav/nav2_test/install/setup.bash`

**Do not** source `/home/nav/bievr_ws` in the same Nav2 terminal. BIEVR’s Ceres 2.2 (`libceres.so.4` under `bievr_ws/deps/ceres`) conflicts with the distro Ceres Nav2/plugins expect. Run BIEVR in a **separate** terminal with `source /home/nav/bievr_ws/quick_source.sh`.

Build this workspace:

```bash
conda deactivate
source /opt/ros/humble/setup.bash
cd /home/nav/nav2_test
colcon build --symlink-install --packages-select <pkgs>
```

## Livox ROS driver

`livox_mid360.launch.py` needs package `livox_ros_driver2`. It is **not** in `nav2_test` on purpose: official `build.sh` deletes `../../build` and `../../install`.

- Workspace: `/home/nav/livox_ws`
- Source: `/home/nav/livox_ws/src/livox_ros_driver2`
- Pinned to tag **`1.2.4`** (`b6ff7d1`). Master (`1.2.6+`) uses `kLivoxLidarTypeMid360s` / double-echo types that this machine’s Livox-SDK2 (installed 2025-07-06) does not have.
- Executable: `livox_ros_driver2_node`

Launch after sourcing:

```bash
source /home/nav/nav2_test/quick_source.sh
ros2 launch my_nav2_robot livox_mid360.launch.py
# or
ros2 launch my_nav2_robot hardware_bringup.launch.py
```

Hardware JSON (`src/my_nav2_robot/config/MID360_config.json`):

- `lidar_type: 8` (SDK2 protocol)
- Host IPs must exist on a local NIC or the SDK prints `bind failed` / `Failed to init livox lidar sdk`
- Last seen values (user-editable): host `192.168.1.50`, lidar `192.168.1.161`
- Check: `ros2 run my_nav2_robot check_mid360_net.sh <lidar_ip> <host_ip>`

`/scan` is **not** a physical 2D lidar. `pointcloud_to_laserscan_node` in `my_nav2_robot` projects `/livox/lidar` (`livox_frame`) to `/scan` (`laser_link`).

## Simulation / LIO split

- Sim Nav2: `full_navigation.launch.py` / `gazebo_sim.launch.py`, Classic Gazebo, `chassis:=diff`, `use_sim_time:=true`.
- Do not wire BIEVR into `full_navigation.launch.py` (that was tried and reverted). Two terminals.
- BIEVR consumes `/livox/lidar` + `/livox/imu`, publishes `/bievr_lio/odom`. If using it, Nav2 `odom_topic` should be `/bievr_lio/odom`, and **do not** also run `use_ground_truth_odom:=true` (two `odom→base_footprint` TFs).
- Mid360 sim timestamps: if point `t` exceeds ~0.2 s BIEVR drops the cloud and IMU-only odom flies. Sim plugin should stamp against the nominal 10 Hz period.

## Coding conventions in this tree

- K&R braces. Humble ament uncrustify fights that; planner/smoother tests skip uncrustify.
- Humble `robot_description` from xacro must be `ParameterValue(..., value_type=str)` or launch treats the URDF as YAML.

## What not to do

- Do not `source bievr_ws` into Nav2.
- Do not copy Jazzy `createPlan(..., cancel_checker)` as an override.
- Do not overwrite Humble B-Spline with main’s older file.
- Do not put `livox_ros_driver2` under `nav2_test/src` and run its `build.sh`.
- Do not leave `//` comments in `MID360_config.json`.
- Do not commit unless FGoose asks.
