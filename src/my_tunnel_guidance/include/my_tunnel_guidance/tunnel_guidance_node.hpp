#ifndef MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_NODE_HPP_
#define MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    
    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

    bool transformCloudToBase(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
        std::vector<Eigen::Vector3d> & points);

    bool getBaseToOutputTransform(
        const rclcpp::Time & stamp,
        Eigen::Isometry3d & transform) const;

    void publishResults(
        const rclcpp::Time & stamp,
        const CenterlineEstimate & centerline,
        bool valid);

    double yawFromTangent(const Eigen::Vector3d & tangent) const;

    TunnelGeometryParams geometry_params_;
    TunnelGeometryEstimator estimator_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;

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

    bool has_last_result_ = false;
    rclcpp::Time last_valid_time_{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d filtered_tangent_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d filtered_center_ = Eigen::Vector3d::Zero();
    double filtered_width_ = 0.0;
    int consecutive_valid_ = 0;
    CenterlineEstimate last_centerline_;
};

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__TUNNEL_GUIDANCE_NODE_HPP_
