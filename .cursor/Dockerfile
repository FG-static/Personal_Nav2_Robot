# ROS 2 Humble (Ubuntu 22.04) development image for the `humble` branch.
#
# The upstream `main` targets Ubuntu 24.04 + ROS 2 Jazzy; this image provides the
# Ubuntu 22.04 + ROS 2 Humble toolchain that the Humble port (see src/HUMBLE_PORTING.md)
# builds against. Simulation backends (Gazebo Classic / Ignition Fortress, small_gicp)
# are intentionally NOT installed yet — they are added when the simulation layer is ported.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# --- System toolchain + ROS 2 Humble packages required by the non-simulation workspace ---
# Covers: Nav2 (incl. MPPI, collision-monitor, AMCL with Omni model, bringup),
# slam-toolbox, BehaviorTree.CPP v3, g2o, Eigen, PCL, OpenCV/cv_bridge,
# pointcloud_to_laserscan and osqp-vendor.
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
