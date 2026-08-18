# AGENTS.md

## Cursor Cloud specific instructions

This repo is a **ROS 2 Jazzy** navigation workspace (Ubuntu 24.04) built with `colcon`.
It contains a `my_nav2_robot` core package plus custom Nav2 plugins (planners,
controllers, smoothers, BT nodes) that are simulated in **Gazebo (gz-sim 8 / Harmonic)**
and visualized in **RViz2**. See `src/README.md` (Chinese) for the author's overview.

The base environment (ROS 2 Jazzy desktop, Nav2, `ros-gz`, and the extra dependencies
noted below) is already installed in the VM snapshot. The startup update script only
refreshes declared workspace dependencies via `rosdep`; it does **not** build.

### Build / source / test (run from `/workspace`)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2   # limit workers: VM has 4 cores / 15 GB
source install/setup.bash                             # source AFTER building, in every new shell
colcon test                                           # then: colcon test-result --all
```

- Only the functional **gtest** suites matter: `path_metrics_test`,
  `test_pointcloud_timing`, `test_tunnel_geometry_estimator` — these pass.
  The ~87 `colcon test` "failures" are **ament code-style linters**
  (uncrustify / flake8 / cpplint / cppcheck / lint_cmake) that flag the repo's
  pre-existing style; they are not environment problems.

### Running the application (Gazebo + Nav2)

- A virtual X display is available at `DISPLAY=:1`; there is **no GPU**, so use software
  GL: `export DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1` before launching GUI nodes (RViz/Gazebo).
- Default nav mode (`ros2 launch my_nav2_robot full_navigation.launch.py`) expects an
  **external LIO odometry** source (BIEVR-LIO, not in this repo) to publish
  `odom -> base_footprint`, so it will not localize on its own.
- **Self-contained run** (SLAM + Gazebo ground-truth odom TF, no external LIO needed):

  ```bash
  ros2 launch my_nav2_robot full_navigation.launch.py slam:=true use_gazebo_odom_tf:=true
  ```

  Wait for both lifecycle managers to log `Managed nodes are active`, then drive the robot:

  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
  ```

  The robot spawns at the culvert entrance (`x=-11.8`); `/ground_truth` gives its pose.
  You can also move it directly by publishing `geometry_msgs/msg/Twist` on `/cmd_vel`.

### Non-obvious environment setup baked into the snapshot

These fix mismatches between the repo and the stock apt packages. They are already
applied in the snapshot; recreate them only if a future base image loses them.

- **`Sophus` (undeclared dependency of `my_nav2_robot`).** The code does
  `find_package(Sophus)` + `ament_target_dependencies(... Sophus)`, which does **not**
  link the modern `Sophus::Sophus` target, so the apt `ros-jazzy-sophus` (headers under
  `/opt/ros/...`, and its `fmt` usage) fails to compile/link. Sophus 1.22.10 is installed
  **header-only to `/usr/local`** with `common.hpp` patched to default to
  `SOPHUS_USE_BASIC_LOGGING` (no `libfmt` needed).
- **`small_gicp` (used by `nav2_visual_processing`).** `find_package(small_gicp)` needs a
  config on the default CMake path; the ROS `small-gicp-vendor` hides it under an `opt/`
  dir. Upstream `small_gicp` is installed from source to `/usr/local`.
- **Hardcoded developer path.** `config/nav2_params_*.yaml` reference
  `/home/goose/nav2_test/src/my_nav2_robot/behavior_trees/test_nav.xml` (the author's
  workspace). A symlink makes it resolve: `sudo ln -sfn /workspace /home/goose/nav2_test`.
  Without it, `bt_navigator` fails to activate.
- **Toolchain.** The default `cc`/`c++` is clang, which targets gcc-14; `libstdc++-14-dev`
  is installed so linking works.
- Benign warnings during launch: `Cannot load scan pattern '.../mid360.csv'` (the custom
  Livox scan pattern file is not shipped; the gpu_lidar `/scan` still works) and PCL/FLANN
  CMake policy `CMP0144` warnings.
