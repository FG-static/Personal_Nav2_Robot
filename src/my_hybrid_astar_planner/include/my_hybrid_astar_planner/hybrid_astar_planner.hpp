#ifndef MY_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER
#define MY_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/replan_event.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace my_hybrid_astar_planner {

struct PlannerPose {

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// 运动模式
enum class MotionDirection {

    TRANSLATE = 0,
    ROTATE = 1
};

struct StateKey {

    int mx = 0;
    int my = 0;
    int theta_id = 0;
    int direction_id = 0;

    bool operator==(const StateKey &other) const {

        return mx == other.mx &&
               my == other.my &&
               theta_id == other.theta_id &&
               direction_id == other.direction_id;
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
    MotionDirection direction = MotionDirection::TRANSLATE;
    std::vector<MotionSample> samples;
};

struct HybridNode {

    StateKey key;
    PlannerPose pose;
    double g = std::numeric_limits<double>::infinity(); // 累计
    double h_grid = std::numeric_limits<double>::infinity(); // 剩余估计代价
    double h_yaw = 0.0;
    double h_goal_dist = 0.0; // 到目标点的欧氏距离启发
    double f = std::numeric_limits<double>::infinity(); // 总
    int parent_index = -1;
    int parent_primitive_id = -1;
    bool closed = false;
};

// 单个 primitive 在当前扩展节点下的真实代价分解
struct PrimitiveCostDebug {

    int primitive_id = -1;
    bool feasible = false;
    bool accepted = false;
    double travel_cost = 0.0;
    double obstacle_cost = 0.0;
    double switch_cost = 0.0;
    double goal_directed_cost = 0.0;
    double tangent_change_cost = 0.0;
    double delta_g = std::numeric_limits<double>::infinity();
    double h_grid = std::numeric_limits<double>::infinity();
    double h_yaw = 0.0;
    double h_goal_dist = 0.0;
    double score = std::numeric_limits<double>::infinity();
};

struct PlannerParams {

    double unknown_cost = 5.0;
    bool treat_unknown_as_free = false;
    double interpolation_resolution = 0.05;
    double xy_resolution = 0.05;
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
    double heuristic_goal_dist_weight = 0.3;
    double path_tangent_change_weight = 0.2;
    double primitive_switch_weight = 0.2;
    double direction_switch_weight = 0.5;
    double velocity_direction_change_weight = 0.5;
    double primitive_omega_change_weight = 0.3;
    double goal_progress_weight = 2.0;
    double goal_direction_weight = 0.5;
    double angular_motion_weight = 0.3;
    int primitive_translation_angle_count = 16;
    int primitive_omega_sample_count = 3;
    double primitive_speed_ratio = 1.0;
    bool include_pure_rotation_primitives = true;
    double analytic_expansion_distance = 2.0;
    double replan_time_threshold = 2.0;
    double path_prune_distance = 0.15;
    int max_iterations = 20000;
    double max_planning_time = 1.0;
    bool immediate_replan_if_blocked = true;
    bool reuse_path_if_valid = true;
    bool visualize_primitive_costs = true;
    std::string primitive_cost_frame = "base_link";
    bool primitive_cost_show_text = true;
    double primitive_cost_marker_z = 0.05;
    double primitive_cost_publish_rate = 5.0;
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
        const geometry_msgs::msg::PoseStamped &goal) override;

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
    void updateHeuristicTerms(
        HybridNode &node,
        const PlannerPose &pose,
        const PlannerPose &goal
    ) const;
    const MotionPrimitive* findMotionPrimitiveById(int primitive_id) const;
    double computePrimitiveSwitchCost(
        const HybridNode &current,
        const MotionPrimitive &next_primitive
    ) const;
    double computeGoalDirectedCost(
        const PlannerPose &current,
        const PlannerPose &next,
        const PlannerPose &goal
    ) const;
    double computePathTangentChangeCost(
        const PlannerPose &previous,
        const PlannerPose &current,
        const PlannerPose &next
    ) const;
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
    bool isCachedPathBlocked(const nav_msgs::msg::Path &path) const;
    nav_msgs::msg::Path pruneCachedPath(
        const nav_msgs::msg::Path &path,
        const geometry_msgs::msg::PoseStamped &cur_pose
    ) const;
    int findBestPruneIndex(
        const nav_msgs::msg::Path &path,
        const geometry_msgs::msg::PoseStamped &cur_pose
    ) const;
    double pointDistance2D(
        const geometry_msgs::msg::PoseStamped &a,
        const geometry_msgs::msg::PoseStamped &b
    ) const;
    void publishReplanEvent(
        uint8_t reason,
        const geometry_msgs::msg::PoseStamped &goal,
        const builtin_interfaces::msg::Time &candidate_path_stamp,
        const rclcpp::Time &trigger_time,
        double path_block_detection_ms,
        double front_end_ms
    );
    int findFirstPrimitiveId(
        const std::vector<HybridNode> &nodes,
        int goal_index
    ) const;
    std::vector<PrimitiveCostDebug> evaluatePrimitiveCostsAtPose(
        const PlannerPose &current_pose,
        const PlannerPose &goal_pose
    ) const;
    void updatePrimitiveCostVisualization();
    void publishPrimitiveCostVisualization(
        const std::vector<PrimitiveCostDebug> &costs,
        int selected_primitive_id,
        bool realtime = false
    ) const;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<rm_interfaces::msg::ReplanEvent>::SharedPtr replan_event_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr primitive_cost_pub_;
    rclcpp::TimerBase::SharedPtr primitive_cost_timer_;
    nav2_costmap_2d::Costmap2D *costmap_{nullptr};
    std::string global_frame_;
    std::string name_;
    PlannerParams params_;
    std::vector<MotionPrimitive> motion_primitives_;
    std::vector<double> heuristic_grid_;
    bool heuristic_ready_{false};
    std::size_t last_grid_expanded_cells_{0};
    nav_msgs::msg::Path last_path_;
    bool has_last_path_{false};
    rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
    uint64_t replan_event_id_{0};
    PlannerPose primitive_cost_goal_;
    bool has_primitive_cost_goal_{false};
    std::mutex planning_mutex_;
};

} // namespace my_hybrid_astar_planner

#endif // MY_HYBRID_ASTAR_PLANNER__HYBRID_ASTAR_PLANNER
