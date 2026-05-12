#ifndef MY_TEB_CONTROLLER__TEB_CONTROLLER
#define MY_TEB_CONTROLLER__TEB_CONTROLLER

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "my_teb_controller/teb_types.hpp"

// 适配层节点
namespace my_teb_controller {

class TebGraphOptimizer;

class MyTebController : public nav2_core::Controller {

public:

    MyTebController();
    ~MyTebController() override;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    void setPlan(const nav_msgs::msg::Path &path) override;

    void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &v,
        nav2_core::GoalChecker *goal_checker) override;

protected:

    void initializeTrajectory(
        const nav_msgs::msg::Path &global_path,
        const PoseSE2 &start_pose);

    void optimizeTEB(const PoseSE2 &goal_pose);

    geometry_msgs::msg::TwistStamped extractVelocity(
        const PoseSE2 &robot_pose,
        const geometry_msgs::msg::Twist &robot_vel);

    bool checkTrajectoryFeasibility();

    void autoResizeTrajectory();

    double getObstacleDistance(double x, double y) const;

    void publishVisualization();

    nav_msgs::msg::Path cropGlobalPlan(const PoseSE2 &robot_pose) const;
    bool shouldReinitializeBand(const PoseSE2 &robot_pose, const nav_msgs::msg::Path &local_plan) const;
    ObstacleSamples collectObstacleSamples() const;

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp::Logger logger_{rclcpp::get_logger("MyTebController")};
    rclcpp::Clock::SharedPtr clock_;
    std::string plugin_name_;
    nav_msgs::msg::Path global_plan_;
    nav_msgs::msg::Path local_plan_;
    std::vector<TimedPose> teb_trajectory_;
    std::vector<TimedPose> initial_teb_trajectory_;
    ObstacleSamples obstacle_samples_;
    TebConfig config_;
    double last_v_x_ = 0.0;
    double last_v_y_ = 0.0;
    double last_omega_ = 0.0;
    bool needs_reinitialization_ = true;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr teb_marker_pub_;
    rclcpp::Duration transform_tolerance_{0, 0};
    std::unique_ptr<TebGraphOptimizer> graph_optimizer_;
};

} // namespace my_teb_controller

#endif // MY_TEB_CONTROLLER__TEB_CONTROLLER
