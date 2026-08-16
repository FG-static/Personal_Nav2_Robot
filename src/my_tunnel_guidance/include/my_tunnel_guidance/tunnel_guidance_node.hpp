#ifndef MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_NODE_HPP_
#define MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

#include "my_tunnel_guidance/tunnel_geometry_estimator.hpp"
#include "my_tunnel_guidance/tunnel_types.hpp"

namespace my_tunnel_guidance {

class TunnelGuidanceNode : public rclcpp::Node {
    
public:
    
    explicit TunnelGuidanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
    
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

    void publishClassifiedPointClouds(
        const rclcpp::Time & stamp,
        const std::vector<Eigen::Vector3d> & left_points,
        const std::vector<Eigen::Vector3d> & right_points,
        const std::vector<Eigen::Vector3d> & ground_points);

    bool transformCloudToBase(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
        std::vector<Eigen::Vector3d> & points);

    bool getBaseToOutputTransform(
        const rclcpp::Time & stamp,
        Eigen::Isometry3d & transform) const;

    bool lookupTransformWithFallback(
        const std::string & target_frame,
        const std::string & source_frame,
        const rclcpp::Time & stamp,
        geometry_msgs::msg::TransformStamped & transform) const;

    TunnelFrameEstimate frameToOutputFrame(
        const TunnelFrameEstimate & frame,
        const Eigen::Isometry3d & base_to_output) const;

    bool initializeWallModel(
        const TunnelFrameEstimate & output_frame,
        const Eigen::Isometry3d & base_to_output);

    void classifyWithWallModel(
        const std::vector<Eigen::Vector3d> & output_points,
        std::vector<Eigen::Vector3d> & left_points,
        std::vector<Eigen::Vector3d> & right_points,
        std::vector<Eigen::Vector3d> & ground_points) const;

    bool updateWallModel(const TunnelFrameEstimate & output_frame);

    void maybeSendAutoGoal();
    void autoGoalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void autoGoalResultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
    bool transformGoalToMap(
        const geometry_msgs::msg::PoseStamped & input,
        geometry_msgs::msg::PoseStamped & output) const;

    void publishResults(
        const rclcpp::Time & stamp,
        const CenterlineEstimate & centerline,
        bool valid);

    double yawFromTangent(const Eigen::Vector3d & tangent) const;

    TunnelGeometryParams geometry_params_;
    TunnelGeometryEstimator estimator_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr auto_goal_client_;
    rclcpp::TimerBase::SharedPtr auto_goal_timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_points_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string input_cloud_topic_;
    std::string estimation_frame_;
    std::string output_frame_;
    double min_forward_distance_;
    double max_forward_distance_;
    double max_backward_distance_;
    double max_lateral_distance_;
    double min_side_lateral_distance_;
    double min_height_;
    double max_height_;
    double ground_max_z_;
    double voxel_leaf_size_;
    double lookahead_distance_;
    double filter_alpha_;
    int valid_frame_count_;
    double result_hold_time_;
    double wall_band_ = 0.25;
    double wall_model_update_alpha_ = 0.1;
    double wall_position_jump_limit_ = 0.3;
    int init_required_frames_ = 3;
    double ground_band_ = 0.15;
    bool enable_auto_goal_ = false;
    std::string auto_goal_frame_id_ = "map";
    double min_goal_send_interval_ = 1.0;
    double goal_moved_resend_distance_ = 0.5;

    TunnelWallModel wall_model_;
    int calibration_valid_frames_ = 0;

    bool has_last_result_ = false;
    rclcpp::Time last_valid_time_{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d filtered_tangent_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d filtered_center_ = Eigen::Vector3d::Zero();
    double filtered_width_ = 0.0;
    int consecutive_valid_ = 0;
    CenterlineEstimate last_centerline_;

    bool last_published_valid_ = false;
    bool has_latest_auto_goal_ = false;
    bool has_sent_auto_goal_ = false;
    bool waiting_for_auto_goal_result_ = false;
    geometry_msgs::msg::PoseStamped latest_auto_goal_;
    geometry_msgs::msg::PoseStamped last_sent_auto_goal_;
    rclcpp::Time last_auto_goal_send_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_NODE_HPP_
