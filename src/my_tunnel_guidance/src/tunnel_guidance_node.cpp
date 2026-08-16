#include "my_tunnel_guidance/tunnel_guidance_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace my_tunnel_guidance {

TunnelGuidanceNode::TunnelGuidanceNode(const rclcpp::NodeOptions & options)
: Node("tunnel_guidance", options
) {

    input_cloud_topic_ = declare_parameter("input_cloud_topic", "/livox/lidar");
    estimation_frame_ = declare_parameter("estimation_frame", "base_link");
    output_frame_ = declare_parameter("output_frame", "odom");

    min_height_ = declare_parameter("min_height", -0.5);
    max_height_ = declare_parameter("max_height", 2.2);
    ground_max_z_ = declare_parameter("ground_max_z", 0.0);
    min_forward_distance_ = declare_parameter("min_forward_distance", 0.2);
    max_forward_distance_ = declare_parameter("max_forward_distance", 8.0);
    max_backward_distance_ = declare_parameter("max_backward_distance", 0.5);
    max_lateral_distance_ = declare_parameter("max_lateral_distance", 3.0);
    min_side_lateral_distance_ = declare_parameter("min_side_lateral_distance", 1.0);
    voxel_leaf_size_ = declare_parameter("voxel_leaf_size", 0.05);
    lookahead_distance_ = declare_parameter("lookahead_distance", 5.0);
    filter_alpha_ = declare_parameter("filter_new_measurement_weight", 0.2);
    valid_frame_count_ = declare_parameter("valid_frame_count", 3);
    result_hold_time_ = declare_parameter("result_hold_time", 0.5);

    geometry_params_.min_ground_points =
        declare_parameter("min_ground_points", 100);
    geometry_params_.min_wall_points = declare_parameter("min_wall_points", 100);
    geometry_params_.plane_max_rmse = declare_parameter("plane_max_rmse", 0.08);
    geometry_params_.min_tunnel_width = declare_parameter("min_tunnel_width", 2.5);
    geometry_params_.max_tunnel_width = declare_parameter("max_tunnel_width", 5.0);
    geometry_params_.min_direction_eigen_gap =
        declare_parameter("min_direction_eigen_gap", 0.05);
    geometry_params_.centerline_length =
        declare_parameter("centerline_length", 8.0);
    geometry_params_.centerline_point_spacing =
        declare_parameter("centerline_point_spacing", 0.2);
    estimator_ = TunnelGeometryEstimator(geometry_params_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TunnelGuidanceNode::pointCloudCallback, this, std::placeholders::_1));

    centerline_pub_ = create_publisher<nav_msgs::msg::Path>(
        "~/centerline", rclcpp::QoS(1));
    local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/local_goal", rclcpp::QoS(1));
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers", rclcpp::QoS(1));
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("~/valid", rclcpp::QoS(1));

    RCLCPP_INFO(get_logger(), "Tunnel guidance node started");
}

/**
 * @brief 将点云从传感器坐标系转换到机器人基坐标系
 * @param cloud_msg 输入点云消息
 * @param points 转换后的点云坐标
 * @return 转换成功返回true，否则返回false
 */
bool TunnelGuidanceNode::transformCloudToBase(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    std::vector<Eigen::Vector3d> & points
) {

    points.clear();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    if (cloud.empty()) {

        return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {

        transform = tf_buffer_->lookupTransform(
            estimation_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp,
            rclcpp::Duration::from_seconds(0.2)
        );
    } catch (const tf2::TransformException & ex) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot transform cloud to %s at stamp %.3f: %s",
            estimation_frame_.c_str(),
            rclcpp::Time(cloud_msg->header.stamp).seconds(), ex.what());
        return false;
    }

    const Eigen::Isometry3d sensor_to_base = tf2::transformToEigen(transform);
    points.reserve(cloud.size());
    for (const auto & point : cloud) {

        if (!pcl::isFinite(point)) {

            continue;
        }
        const Eigen::Vector3d p_base =
            sensor_to_base * Eigen::Vector3d(point.x, point.y, point.z);
        if (p_base.x() > max_forward_distance_ ||
            p_base.x() < -max_backward_distance_ ||
            (p_base.x() >= 0.0 && p_base.x() < min_forward_distance_) ||
            std::abs(p_base.y()) > max_lateral_distance_ ||
            p_base.z() < min_height_ || p_base.z() > max_height_)
            continue;
        points.push_back(p_base);
    }

    // 对点云进行体素滤波，减少点云密度
    if (voxel_leaf_size_ > 1e-3 && points.size() > 20) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        dense_cloud->resize(points.size());
        for (std::size_t i = 0; i < points.size(); ++ i) {

            dense_cloud->points[i].x = static_cast<float>(points[i].x());
            dense_cloud->points[i].y = static_cast<float>(points[i].y());
            dense_cloud->points[i].z = static_cast<float>(points[i].z());
        }
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(dense_cloud);
        voxel.setLeafSize(
            static_cast<float>(voxel_leaf_size_),
            static_cast<float>(voxel_leaf_size_),
            static_cast<float>(voxel_leaf_size_));
        pcl::PointCloud<pcl::PointXYZ> filtered;
        voxel.filter(filtered);
        points.clear();
        points.reserve(filtered.size());
        for (const auto & point : filtered) {

            points.emplace_back(point.x, point.y, point.z);
        }
    }
    return !points.empty();
}

/**
 * @brief 获取从估计坐标系到输出坐标系的变换，主要用于 publish
 * @param stamp 时间戳
 * @param transform 输出的变换矩阵
 * @return 是否成功获取变换
 */
bool TunnelGuidanceNode::getBaseToOutputTransform(
    const rclcpp::Time & stamp,
    Eigen::Isometry3d & transform
) const {

    try {

        const geometry_msgs::msg::TransformStamped tf_msg =
            tf_buffer_->lookupTransform(output_frame_, estimation_frame_, stamp,
                rclcpp::Duration::from_seconds(0.2));
        transform = tf2::transformToEigen(tf_msg);
        return true;
    } catch (const tf2::TransformException & ex) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot transform centerline to %s: %s", output_frame_.c_str(), ex.what());
        return false;
    }
}

void TunnelGuidanceNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg
) {

    std::vector<Eigen::Vector3d> base_points;
    if (!transformCloudToBase(cloud_msg, base_points))
        return;

    const rclcpp::Time stamp(cloud_msg->header.stamp);
    std::vector<Eigen::Vector3d> left_points, right_points, ground_points;
    left_points.reserve(base_points.size() / 4);
    right_points.reserve(base_points.size() / 4);
    ground_points.reserve(base_points.size() / 4);

    // 按照点的高度将点云分为地面点、左点和右点
    for (const auto & point : base_points) {

        if (point.z() <= ground_max_z_) {

            ground_points.push_back(point);
        } else if (point.y() > min_side_lateral_distance_) {

            left_points.push_back(point);
        } else if (point.y() < -min_side_lateral_distance_) {

            right_points.push_back(point);
        }
    }

    const TunnelFrameEstimate frame =
        estimator_.estimateFrame(left_points, right_points, ground_points);
    const rclcpp::Time now = get_clock()->now();

    if (!frame.valid) {

        if (has_last_result_ &&
            (now - last_valid_time_).seconds() <= result_hold_time_
        ) {

            publishResults(stamp, last_centerline_, false);
        }
        return;
    }

    Eigen::Vector3d tangent = frame.tangent;
    if (has_last_result_ && tangent.dot(filtered_tangent_) < 0.0) {

        tangent = -tangent;
    }

    if (!has_last_result_) {

        filtered_tangent_ = tangent;
        filtered_center_ = frame.center;
        filtered_width_ = frame.width;
    } else {

        filtered_tangent_ =
            ((1.0 - filter_alpha_) * filtered_tangent_ + filter_alpha_ * tangent).normalized();
        filtered_center_ =
            (1.0 - filter_alpha_) * filtered_center_ + filter_alpha_ * frame.center;
        filtered_width_ =
            (1.0 - filter_alpha_) * filtered_width_ + filter_alpha_ * frame.width;
    }

    TunnelFrameEstimate filtered_frame = frame;
    filtered_frame.tangent = filtered_tangent_;
    filtered_frame.center = filtered_center_;
    filtered_frame.width = filtered_width_;
    filtered_frame.lateral = filtered_frame.up.cross(filtered_tangent_).normalized();

    CenterlineEstimate centerline =
        estimator_.buildStraightCenterline(filtered_frame);
    if (!centerline.valid) {

        return;
    }

    last_centerline_ = centerline;
    last_valid_time_ = now;
    has_last_result_ = true;
    consecutive_valid_++;
    const bool publish_valid = consecutive_valid_ >= valid_frame_count_;
    publishResults(stamp, centerline, publish_valid);
}

double TunnelGuidanceNode::yawFromTangent(
    const Eigen::Vector3d & tangent
) const{

    return std::atan2(tangent.y(), tangent.x());
}

void TunnelGuidanceNode::publishResults(
    const rclcpp::Time & stamp,
    const CenterlineEstimate & centerline,
    bool valid
) {

    Eigen::Isometry3d base_to_output = Eigen::Isometry3d::Identity();
    if (!getBaseToOutputTransform(stamp, base_to_output)) {

        return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = output_frame_;
    for (const auto & point : centerline.points) {

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        const Eigen::Vector3d output_point = base_to_output * point;
        const Eigen::Vector3d output_tangent =
            base_to_output.linear() * centerline.tangents[0];
        pose.pose.position.x = output_point.x();
        pose.pose.position.y = output_point.y();
        pose.pose.position.z = output_point.z();
        pose.pose.orientation = tf2::toMsg(
            Eigen::Quaterniond(
                Eigen::AngleAxisd(yawFromTangent(output_tangent), Eigen::Vector3d::UnitZ())));
        path_msg.poses.push_back(pose);
    }
    centerline_pub_->publish(path_msg);

    geometry_msgs::msg::PoseStamped goal;
    goal.header = path_msg.header;
    const std::size_t goal_index = std::min(
        centerline.points.size() - 1,
        static_cast<std::size_t>(
            lookahead_distance_ / geometry_params_.centerline_point_spacing));
    const Eigen::Vector3d goal_point =
        base_to_output * centerline.points[goal_index];
    goal.pose.position.x = goal_point.x();
    goal.pose.position.y = goal_point.y();
    goal.pose.position.z = goal_point.z();
    goal.pose.orientation = path_msg.poses[goal_index].pose.orientation;
    local_goal_pub_->publish(goal);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker line;
    line.header = path_msg.header;
    line.ns = "centerline";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 0.0F;
    line.color.g = 1.0F;
    line.color.b = 0.0F;
    line.color.a = 1.0F;
    for (const auto & pose : path_msg.poses) {

        line.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(line);

    const Eigen::Vector3d center_output =
        base_to_output * centerline.local_frame.center;
    const std::array<Eigen::Vector3d, 3> axes = {
        centerline.local_frame.tangent,
        centerline.local_frame.lateral,
        centerline.local_frame.up};
    const std::array<double, 3> lengths = {1.5, 1.0, 1.0};
    for (std::size_t i = 0; i < axes.size(); ++ i) {

        visualization_msgs::msg::Marker arrow;
        arrow.header = path_msg.header;
        arrow.ns = "frame_axis";
        arrow.id = static_cast<int>(i);
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.1;
        arrow.color.a = 1.0F;
        arrow.color.r = i == 0 ? 1.0F : 0.0F;
        arrow.color.g = i == 1 ? 1.0F : 0.0F;
        arrow.color.b = i == 2 ? 1.0F : 0.0F;
        arrow.points.resize(2);
        arrow.points[0].x = center_output.x();
        arrow.points[0].y = center_output.y();
        arrow.points[0].z = center_output.z();
        const Eigen::Vector3d tip_output =
            center_output + base_to_output.linear() * axes[i] * lengths[i];
        arrow.points[1].x = tip_output.x();
        arrow.points[1].y = tip_output.y();
        arrow.points[1].z = tip_output.z();
        markers.markers.push_back(arrow);
    }
    markers_pub_->publish(markers);

    std_msgs::msg::Bool valid_msg;
    valid_msg.data = valid;
    valid_pub_->publish(valid_msg);
}

}  // namespace my_tunnel_guidance

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_tunnel_guidance::TunnelGuidanceNode>());
    rclcpp::shutdown();
    return 0;
}
