#ifndef MY_TEB_CONTROLLER__TEB_TYPES
#define MY_TEB_CONTROLLER__TEB_TYPES

#include <vector>

#include <Eigen/Dense>

namespace my_teb_controller {

// SE2 2D Pose: 
// 原始路径点存储结构
struct PoseSE2 {

    // 含Eigen内数据结构固定大小的变量需要加
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    PoseSE2() = default;
    PoseSE2(double x, double y, double theta);

    Eigen::Vector2d position() const;
    void setTheta(double theta);
    double thetaMod(double theta) const;
};

struct TimedPose {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseSE2 pose;
    double dt = 0.0; // 上一个 PoseSE2 到当前 PoseSE2 的时间间隔

    TimedPose() = default;
    TimedPose(const PoseSE2 &pose, double dt);
};

struct MecanumVelocity {

    double v_x = 0.0;
    double v_y = 0.0;
    double omega = 0.0;
};

struct TebConfig {

    bool teb_autosize = true;
    double dt_ref = 0.3;
    double dt_hysteresis = 0.1;
    int min_samples = 3;
    int max_samples = 500;

    double min_obstacle_dist = 0.3;
    double inflation_dist = 0.5;
    double costmap_weight = 100.0;

    int no_iterations = 5;
    double penalty_epsilon = 0.1;
    double weight_obstacle = 50.0;
    double weight_optimaltime = 1.0;
    double weight_shortest_path = 0.0;
    double weight_smoothness = 100.0;
    double weight_kinematics = 500.0;

    double max_vel_x = 0.5;
    double max_vel_y = 0.5;
    double max_vel_theta = 1.0;
    double acc_lim_x = 2.0;
    double acc_lim_y = 2.0;
    double acc_lim_theta = 1.5;

    bool optimizer_verbose = false;
    double goal_tolerance_band = 0.25;
    double reinit_pose_distance = 0.3;
};

using ObstacleSamples = std::vector<Eigen::Vector2d>;

} // namespace my_teb_controller

#endif // MY_TEB_CONTROLLER__TEB_TYPES
