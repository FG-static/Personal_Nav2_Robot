#include "my_tunnel_guidance/tunnel_guidance_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cstdint>
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
    wall_band_ = declare_parameter("wall_band", 0.25);
    wall_model_update_alpha_ = declare_parameter("wall_model_update_alpha", 0.1);
    wall_position_jump_limit_ = declare_parameter("wall_position_jump_limit", 0.3);
    init_required_frames_ = declare_parameter("init_required_frames", 3);

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
    left_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/left_points", rclcpp::SensorDataQoS());
    right_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/right_points", rclcpp::SensorDataQoS());
    ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/ground_points", rclcpp::SensorDataQoS());

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

void TunnelGuidanceNode::publishClassifiedPointClouds(
    const rclcpp::Time & stamp,
    const std::vector<Eigen::Vector3d> & left_points,
    const std::vector<Eigen::Vector3d> & right_points,
    const std::vector<Eigen::Vector3d> & ground_points
) {

    const auto toPointCloudMessage = [this, &stamp](
        const std::vector<Eigen::Vector3d> & points) {

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.points.reserve(points.size());
        for (const Eigen::Vector3d & point : points) {

            pcl_cloud.points.emplace_back(
                static_cast<float>(point.x()),
                static_cast<float>(point.y()),
                static_cast<float>(point.z()));
        }
        pcl_cloud.width = static_cast<std::uint32_t>(pcl_cloud.points.size());
        pcl_cloud.height = 1U;
        pcl_cloud.is_dense = true;
        pcl_cloud.header.frame_id = output_frame_;

        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(pcl_cloud, message);
        message.header.stamp = stamp;
        message.header.frame_id = output_frame_;
        return message;
    };

    left_points_pub_->publish(toPointCloudMessage(left_points));
    right_points_pub_->publish(toPointCloudMessage(right_points));
    ground_points_pub_->publish(toPointCloudMessage(ground_points));
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

/**
 * @brief 将隧道帧从基坐标系转换到输出坐标系
 * @param frame 输入的隧道帧
 * @param base_to_output 基坐标系到输出坐标系的变换
 * @return TunnelFrameEstimate 转换后的隧道帧
 */
TunnelFrameEstimate TunnelGuidanceNode::frameToOutputFrame(
    const TunnelFrameEstimate & frame,
    const Eigen::Isometry3d & base_to_output
) const {

    TunnelFrameEstimate output_frame = frame;
    output_frame.center = base_to_output * frame.center;
    output_frame.tangent = (base_to_output.linear() * frame.tangent).normalized();
    output_frame.up = (base_to_output.linear() * frame.up).normalized();
    output_frame.lateral = output_frame.up.cross(output_frame.tangent).normalized();
    if (output_frame.lateral.dot(
            base_to_output.linear() * Eigen::Vector3d::UnitY()) < 0.0) {

        output_frame.lateral = -output_frame.lateral;
    }
    output_frame.tangent = output_frame.lateral.cross(output_frame.up).normalized();
    return output_frame;
}

bool TunnelGuidanceNode::initializeWallModel(
    const TunnelFrameEstimate & output_frame,
    const Eigen::Isometry3d & base_to_output
) {

    (void)base_to_output;
    if (!output_frame.valid || output_frame.width <= 0.0) {

        return false;
    }

    wall_model_.initialized = true;
    wall_model_.center = output_frame.center;
    wall_model_.tangent = output_frame.tangent.normalized();
    wall_model_.up = output_frame.up.normalized();
    wall_model_.lateral = wall_model_.up.cross(wall_model_.tangent).normalized();
    wall_model_.width = output_frame.width;
    wall_model_.left_l = 0.5 * wall_model_.width;
    wall_model_.right_l = -0.5 * wall_model_.width;

    RCLCPP_INFO(
        get_logger(),
        "Wall model calibrated: width=%.2f center=(%.2f %.2f %.2f) tangent=(%.2f %.2f)",
        wall_model_.width,
        wall_model_.center.x(), wall_model_.center.y(), wall_model_.center.z(),
        wall_model_.tangent.x(), wall_model_.tangent.y());
    return true;
}

void TunnelGuidanceNode::classifyWithWallModel(
    const std::vector<Eigen::Vector3d> & output_points,
    std::vector<Eigen::Vector3d> & left_points,
    std::vector<Eigen::Vector3d> & right_points,
    std::vector<Eigen::Vector3d> & ground_points
) const {

    for (const auto & point : output_points) {

        if (point.z() <= ground_max_z_) {

            ground_points.push_back(point);
            continue;
        }

        const double lateral_offset =
            wall_model_.lateral.dot(point - wall_model_.center);
        if (std::abs(lateral_offset - wall_model_.left_l) <= wall_band_) {

            left_points.push_back(point);
        } else if (std::abs(lateral_offset - wall_model_.right_l) <= wall_band_) {

            right_points.push_back(point);
        }
    }
}

bool TunnelGuidanceNode::updateWallModel(
    const TunnelFrameEstimate & output_frame
) {

    if (!output_frame.valid || !wall_model_.initialized) {

        return false;
    }

    Eigen::Vector3d tangent = output_frame.tangent;
    if (tangent.dot(wall_model_.tangent) < 0.0) {

        tangent = -tangent;
    }

    const double center_offset =
        wall_model_.lateral.dot(output_frame.center - wall_model_.center);
    const double left_obs = center_offset + 0.5 * output_frame.width;
    const double right_obs = center_offset - 0.5 * output_frame.width;

    if (std::abs(left_obs - wall_model_.left_l) > wall_position_jump_limit_ ||
        std::abs(right_obs - wall_model_.right_l) > wall_position_jump_limit_) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Wall observation jump too large, keep previous wall model");
        return false;
    }

    const double alpha = std::clamp(wall_model_update_alpha_, 0.0, 1.0);
    wall_model_.left_l += alpha * (left_obs - wall_model_.left_l);
    wall_model_.right_l += alpha * (right_obs - wall_model_.right_l);
    wall_model_.width = wall_model_.left_l - wall_model_.right_l;

    wall_model_.tangent =
        ((1.0 - alpha) * wall_model_.tangent + alpha * tangent).normalized();
    wall_model_.up =
        ((1.0 - alpha) * wall_model_.up + alpha * output_frame.up).normalized();
    wall_model_.lateral =
        wall_model_.up.cross(wall_model_.tangent).normalized();
    return true;
}

void TunnelGuidanceNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg
) {

    std::vector<Eigen::Vector3d> base_points;
    if (!transformCloudToBase(cloud_msg, base_points))
        return;

    const rclcpp::Time stamp(cloud_msg->header.stamp);
    Eigen::Isometry3d base_to_output = Eigen::Isometry3d::Identity();
    if (!getBaseToOutputTransform(stamp, base_to_output)) {

        return;
    }

    std::vector<Eigen::Vector3d> output_points;
    output_points.reserve(base_points.size());
    for (const auto & point : base_points) {

        output_points.push_back(base_to_output * point);
    }

    std::vector<Eigen::Vector3d> left_points, right_points, ground_points;
    left_points.reserve(base_points.size() / 4);
    right_points.reserve(base_points.size() / 4);
    ground_points.reserve(base_points.size() / 4);

    TunnelFrameEstimate output_frame;
    const rclcpp::Time now = get_clock()->now();

    if (!wall_model_.initialized) {

        // 初始化阶段：仍使用 base_link 下的粗分区完成首次标定。
        for (const auto & point : base_points) {

            if (point.z() <= ground_max_z_) {

                ground_points.push_back(point);
            } else if (point.y() > min_side_lateral_distance_) {

                left_points.push_back(point);
            } else if (point.y() < -min_side_lateral_distance_) {

                right_points.push_back(point);
            }
        }

        // 将点云转换到输出坐标系
        const auto to_output = [&](const std::vector<Eigen::Vector3d> & points) {

            std::vector<Eigen::Vector3d> converted;
            converted.reserve(points.size());
            for (const auto & point : points) {

                converted.push_back(base_to_output * point);
            }
            return converted;
        };
        publishClassifiedPointClouds(
            stamp,
            to_output(left_points),
            to_output(right_points),
            to_output(ground_points)
        );

        // 估计隧道帧
        const TunnelFrameEstimate base_frame =
            estimator_.estimateFrame(left_points, right_points, ground_points);
        if (!base_frame.valid) {

            calibration_valid_frames_ = 0;
            if (has_last_result_ &&
                (now - last_valid_time_).seconds() <= result_hold_time_
            ) {

                publishResults(stamp, last_centerline_, false);
            }
            return;
        }

        // 将隧道帧转换到输出坐标系
        output_frame = frameToOutputFrame(base_frame, base_to_output);
        calibration_valid_frames_++;

        // 初始化墙模型
        if (calibration_valid_frames_ >= init_required_frames_ &&
            initializeWallModel(output_frame, base_to_output)) {

            consecutive_valid_ = 0;
        } else {

            const CenterlineEstimate calibration_centerline =
                estimator_.buildStraightCenterline(output_frame);
            if (calibration_centerline.valid) {

                publishResults(stamp, calibration_centerline, false);
            }
            return;
        }
    } else {

        // 标定完成后，使用 odom 下的墙体模型分割点云。
        classifyWithWallModel(
            output_points, left_points, right_points, ground_points);
        publishClassifiedPointClouds(
            stamp, left_points, right_points, ground_points);

        const TunnelFrameEstimate observed_frame =
            estimator_.estimateFrame(left_points, right_points, ground_points);
        if (!observed_frame.valid) {

            consecutive_valid_ = 0;
            if (has_last_result_ &&
                (now - last_valid_time_).seconds() <= result_hold_time_
            ) {

                publishResults(stamp, last_centerline_, false);
            }
            return;
        }

        updateWallModel(observed_frame);

        // 用墙体模型在机器人当前纵向位置生成中心线。
        const Eigen::Vector3d robot_position = base_to_output.translation();
        const double s_robot =
            wall_model_.tangent.dot(robot_position - wall_model_.center);
        const double center_lateral =
            0.5 * (wall_model_.left_l + wall_model_.right_l);

        output_frame.tangent = wall_model_.tangent;
        output_frame.up = wall_model_.up;
        output_frame.lateral = wall_model_.lateral;
        output_frame.width = wall_model_.width;
        output_frame.center =
            wall_model_.center +
            wall_model_.tangent * s_robot +
            wall_model_.lateral * center_lateral;
        output_frame.confidence = observed_frame.confidence;
        output_frame.valid = true;
    }

    const CenterlineEstimate centerline =
        estimator_.buildStraightCenterline(output_frame);
    if (!centerline.valid) {

        return;
    }

    last_centerline_ = centerline;
    last_valid_time_ = now;
    has_last_result_ = true;
    consecutive_valid_++;
    const bool publish_valid =
        wall_model_.initialized && consecutive_valid_ >= valid_frame_count_;
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

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = output_frame_;
    for (std::size_t i = 0; i < centerline.points.size(); ++ i) {

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        const Eigen::Vector3d & point = centerline.points[i];
        const Eigen::Vector3d & tangent = centerline.tangents[i];
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation = tf2::toMsg(
            Eigen::Quaterniond(
                Eigen::AngleAxisd(yawFromTangent(tangent), Eigen::Vector3d::UnitZ())));
        path_msg.poses.push_back(pose);
    }
    centerline_pub_->publish(path_msg);

    geometry_msgs::msg::PoseStamped goal;
    goal.header = path_msg.header;
    const std::size_t goal_index = std::min(
        centerline.points.size() - 1,
        static_cast<std::size_t>(
            lookahead_distance_ / geometry_params_.centerline_point_spacing));
    goal.pose = path_msg.poses[goal_index].pose;
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

    const Eigen::Vector3d center_output = centerline.local_frame.center;
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
        const Eigen::Vector3d tip_output = center_output + axes[i] * lengths[i];
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
