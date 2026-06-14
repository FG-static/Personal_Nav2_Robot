#ifndef MY_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER
#define MY_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace my_hybrid_astar_planner {

struct PlannerPose {

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// 运动模式
enum class MotionDirection {

    FORWARD = 0,
    REVERSE = 1,
    LATERAL_LEFT = 2,
    LATERAL_RIGHT = 3,
    ROTATE = 4
};

struct StateKey {

    int mx = 0;
    int my = 0;
    int theta_id = 0;

    bool operator==(const StateKey &other) const {

        return mx == other.mx &&
               my == other.my &&
               theta_id == other.theta_id;
    }
};

struct StateKeyHasher {

    std::size_t operator()(const StateKey &key) const;
};

struct GridNode2D {

    int index = -1;
    double g = std::numeric_limits<double>::infinity();
    int parent_index = -1;
    bool closed = false;
};

struct MotionSample {

    PlannerPose pose;
    double time_from_start = 0.0;
};

struct MotionPrimitive {

    int id = -1;
    double v_x = 0.0;
    double v_y = 0.0;
    double omega = 0.0;
    double duration = 0.0;
    double travel_cost = 0.0;
    MotionDirection direction = MotionDirection::FORWARD;
    std::vector<MotionSample> samples;
};

struct HybridNode {

    StateKey key;
    PlannerPose pose;
    double g = std::numeric_limits<double>::infinity(); // 累计
    double h_grid = std::numeric_limits<double>::infinity(); // 剩余估计代价
    double h_yaw = 0.0;
    double h_rs = 0.0; // reed_shepp 代价
    double f = std::numeric_limits<double>::infinity();
    int parent_index = -1;
    int parent_primitive_id = -1;
    bool closed = false;
};

struct PlannerParams {

    double unknown_cost = 5.0;
    double interpolation_resolution = 0.1;
    double xy_resolution = 0.1;
    int yaw_bin_count = 72;
    double step_time = 0.1;
    double primitive_duration = 0.5;
    double max_vel_x = 0.6;
    double max_vel_y = 0.6;
    double max_vel_theta = 1.0;
    double acc_lim_x = 1.0;
    double acc_lim_y = 1.0;
    double acc_lim_theta = 1.5;
    bool allow_reverse = true;
    double obstacle_cost_weight = 1.0;
    double goal_tolerance_xy = 0.2;
    double goal_tolerance_yaw = 0.2;
    double heuristic_grid_weight = 1.0;
    double heuristic_yaw_weight = 0.2;
    double rs_weight = 0.15;
    double rs_reverse_penalty = 0.4;
    double rs_gear_switch_penalty = 0.2;
    double analytic_expansion_distance = 2.0;
};

class MyHybridAStarPlanner : public nav2_core::GlobalPlanner {

public:

    MyHybridAStarPlanner() = default;
    ~MyHybridAStarPlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> cancel_checker) override;

private:

    void buildMotionPrimitives();
    MotionPrimitive makePrimitive(
        int id,
        double v_x,
        double v_y,
        double omega,
        MotionDirection direction) const;
    PlannerPose integrateMecanumMotion(
        const PlannerPose &start_pose,
        double v_x,
        double v_y,
        double omega,
        double dt) const;
    StateKey discretizeState(const PlannerPose &pose) const;
    bool validatePose(
        const geometry_msgs::msg::PoseStamped &pose,
        unsigned int &mx,
        unsigned int &my,
        const char *label) const;
    double poseToYaw(const geometry_msgs::msg::PoseStamped &pose) const;
    double normalizeAngle(double angle) const;
    bool isGoalReached(const PlannerPose &current, const PlannerPose &goal) const;
    bool computeGridHeuristic(unsigned int goal_mx, unsigned int goal_my);
    double getGridHeuristic(unsigned int mx, unsigned int my) const;
    bool isCellTraversable(unsigned int mx, unsigned int my) const;
    double computeNodeHeuristic(const PlannerPose &pose, const PlannerPose &goal) const;
    const MotionPrimitive * findMotionPrimitiveById(int primitive_id) const;
    bool simulatePrimitive(
        const HybridNode &current,
        const MotionPrimitive &primitive,
        PlannerPose &end_pose,
        std::vector<PlannerPose> &samples,
        double &transition_cost
    ) const;
    nav_msgs::msg::Path reconstructPath(
        const std::vector<HybridNode> &nodes,
        int goal_index,
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal
    ) const;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D *costmap_{nullptr};
    std::string global_frame_;
    std::string name_;
    PlannerParams params_;
    std::vector<MotionPrimitive> motion_primitives_;
    std::vector<double> heuristic_grid_;
    bool heuristic_ready_{false};
};

} // namespace my_hybrid_astar_planner

#endif // MY_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER
