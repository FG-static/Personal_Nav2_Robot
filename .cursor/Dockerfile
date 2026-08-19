# ROS 2 Humble (Ubuntu 22.04) development image for the `humble` branch.
#
# The upstream `main` targets Ubuntu 24.04 + ROS 2 Jazzy; this image provides the
# Ubuntu 22.04 + ROS 2 Humble toolchain that the Humble port (see src/HUMBLE_PORTING.md)
# builds against. Simulation backends for both Humble schemes are installed:
# Gazebo Classic 11 + gazebo_ros_pkgs (A) and Ignition Fortress + ros_gz (B).
# small_gicp remains omitted (nav2_visual_processing is COLCON_IGNORE).
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# --- System toolchain + ROS 2 Humble packages ---
# Covers: Nav2, slam-toolbox, BehaviorTree.CPP v3, g2o, Eigen, PCL, OpenCV,
# pointcloud_to_laserscan, osqp-vendor, Gazebo Classic, ros_gz, and rviz2.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      curl \
      sudo \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      libeigen3-dev \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-slam-toolbox \
      ros-humble-behaviortree-cpp-v3 \
      ros-humble-libg2o \
      ros-humble-pcl-ros \
      ros-humble-pcl-conversions \
      ros-humble-cv-bridge \
      ros-humble-angles \
      ros-humble-xacro \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-osqp-vendor \
      gazebo \
      libgazebo-dev \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros \
      ros-humble-gazebo-plugins \
      ros-humble-ros-gz \
      ros-humble-rviz2 \
      ros-humble-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# --- Sophus (header-only, basic logging, no libfmt) installed to /usr/local ---
# The workspace consumes Sophus via `ament_target_dependencies(... Sophus)`, which does
# not link the modern CMake target, so the header must resolve on the default include
# path and must not pull libfmt symbols. Patch common.hpp to default to basic logging.
RUN git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && python3 -c "p='/tmp/Sophus/sophus/common.hpp'; s=open(p).read(); s=s.replace('#ifdef SOPHUS_USE_BASIC_LOGGING', '#if !defined(SOPHUS_USE_BASIC_LOGGING) && !defined(SOPHUS_USE_FMT)\n#define SOPHUS_USE_BASIC_LOGGING\n#endif\n#ifdef SOPHUS_USE_BASIC_LOGGING', 1); open(p,'w').write(s)" \
    && cmake -S /tmp/Sophus -B /tmp/Sophus/build -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DSOPHUS_USE_BASIC_LOGGING=ON \
    && cmake --install /tmp/Sophus/build \
    && rm -rf /tmp/Sophus

# --- Non-root user matching Cursor's standard cloud environment (uid 1000) ---
RUN if ! id ubuntu >/dev/null 2>&1; then useradd -m -s /bin/bash -u 1000 ubuntu; fi \
    && echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu

# rosdep is already initialized in the base image; refresh the cache for the build user.
USER ubuntu
RUN rosdep update
