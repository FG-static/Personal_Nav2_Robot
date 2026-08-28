#include "my_tunnel_guidance/tunnel_guidance_node.hpp"
#include "my_tunnel_guidance/pointcloud_timing.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <functional>
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

    search_params_.resolution = declare_parameter("search_resolution", 0.10);
    search_params_.min_x = declare_parameter(
        "search_min_x", -max_backward_distance_);
    search_params_.max_x = declare_parameter(
        "search_max_x", max_forward_distance_);
    search_params_.half_width = declare_parameter(
        "search_half_width", max_lateral_distance_);
    search_params_.obstacle_min_height = declare_parameter(
        "search_obstacle_min_height", 0.15);
    search_params_.obstacle_max_height = declare_parameter(
        "search_obstacle_max_height", 1.30);
    search_params_.robot_clearance = declare_parameter(
        "search_robot_clearance", 0.30);
    search_params_.clearance_weight = declare_parameter(
        "search_clearance_weight", 2.0);
    search_params_.clearance_decay = declare_parameter(
        "search_clearance_decay", 0.50);
    search_params_.unknown_cost_factor = declare_parameter(
        "search_unknown_cost_factor", 1.3);
    search_params_.free_close_kernel = declare_parameter(
        "search_free_close_kernel", 3);
    search_params_.minimum_frontier_distance = declare_parameter(
        "search_minimum_frontier_distance", 3.0);
    search_params_.goal_distance = declare_parameter(
        "search_goal_distance", lookahead_distance_);
    search_params_.debug_expansion_interval = 0U;

    filter_alpha_ = declare_parameter("filter_new_measurement_weight", 0.2);
    valid_frame_count_ = declare_parameter("valid_frame_count", 3);
    result_hold_time_ = declare_parameter("result_hold_time", 0.5);
    wall_band_ = declare_parameter("wall_band", 0.25);
    wall_model_update_alpha_ = declare_parameter("wall_model_update_alpha", 0.1);
    wall_position_jump_limit_ = declare_parameter("wall_position_jump_limit", 0.3);
    init_required_frames_ = declare_parameter("init_required_frames", 3);
    ground_band_ = declare_parameter("ground_band", 0.15);
    exit_detection_enabled_ = declare_parameter("exit_detection_enabled", true);
    exit_confirm_frames_ = declare_parameter("exit_confirm_frames", 5);
    exit_window_.min_x = declare_parameter("exit_forward_min_x", 2.0);
    exit_window_.max_x = declare_parameter("exit_forward_max_x", 6.0);
    exit_window_.wall_inner_y = declare_parameter("exit_wall_inner_y", 1.0);
    exit_window_.wall_outer_y = declare_parameter("exit_wall_outer_y", 2.8);
    exit_window_.front_half_width = declare_parameter("exit_front_half_width", 0.8);
    exit_window_.min_side_column_ratio =
        declare_parameter("exit_min_side_column_ratio", 0.35);
    exit_window_.max_open_column_ratio =
        declare_parameter("exit_max_open_column_ratio", 0.18);
    exit_window_.max_front_columns =
        declare_parameter("exit_max_front_columns", 2);
    if (exit_confirm_frames_ < 1) {

        exit_confirm_frames_ = 1;
    }
    enable_auto_goal_ = declare_parameter("enable_auto_goal", false);
    auto_goal_frame_id_ = declare_parameter("auto_goal_frame_id", "map");
    min_goal_send_interval_ = declare_parameter("min_goal_send_interval", 1.0);
    auto_goal_dwell_time_ = declare_parameter("auto_goal_dwell_time", 8.0);
    allow_capture_done_ = declare_parameter("allow_capture_done", true);
    enable_dataset_recording_ = declare_parameter("enable_dataset_recording", true);
    wait_for_dataset_ = declare_parameter("wait_for_dataset", true);
    dataset_output_dir_ = declare_parameter(
        "dataset_output_dir", std::string("/tmp/tunnel_inspections"));
    dataset_voxel_size_ = declare_parameter("dataset_voxel_size", 0.03);
    if (auto_goal_dwell_time_ < 0.0) {

        auto_goal_dwell_time_ = 0.0;
    }

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
    searcher_ = std::make_unique<TunnelGuidanceSearch>(search_params_);
    search_requested_ = enable_auto_goal_;

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
    exit_detected_pub_ = create_publisher<std_msgs::msg::Bool>(
        "~/exit_detected", rclcpp::QoS(1).reliable().transient_local());
    capture_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/capture_enable", rclcpp::QoS(1).reliable().transient_local());
    dataset_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
        "~/dataset_ready", rclcpp::QoS(1).reliable().transient_local());
    can_depart_pub_ = create_publisher<std_msgs::msg::Bool>(
        "~/can_depart", rclcpp::QoS(1).reliable().transient_local());
    merged_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/merged_map", rclcpp::QoS(1).reliable().transient_local());
    gimbal_sub_ = create_subscription<rm_interfaces::msg::Gimbal>(
        "/tracker/gimbal", rclcpp::QoS(10),
        std::bind(&TunnelGuidanceNode::onGimbal, this, std::placeholders::_1));
    left_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/left_points", rclcpp::SensorDataQoS());
    right_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/right_points", rclcpp::SensorDataQoS());
    ground_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/ground_points", rclcpp::SensorDataQoS());

    std_msgs::msg::Bool exit_msg;
    exit_msg.data = false;
    exit_detected_pub_->publish(exit_msg);
    setCaptureEnable(false);

    if (enable_dataset_recording_ &&
        !dataset_recorder_.openMission(
            std::filesystem::path(dataset_output_dir_),
            auto_goal_frame_id_, dataset_voxel_size_))
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to open inspection dataset directory %s",
            dataset_output_dir_.c_str());
        enable_dataset_recording_ = false;
    }
    publishHandshakeFlags();

    if (enable_auto_goal_) {

        auto_goal_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");
        auto_goal_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TunnelGuidanceNode::maybeSendAutoGoal, this));
    }

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
    if (!lookupTransformWithFallback(
            estimation_frame_, cloud_msg->header.frame_id,
            cloud_msg->header.stamp, transform)) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot transform cloud to %s at stamp %.3f",
            estimation_frame_.c_str(),
            rclcpp::Time(cloud_msg->header.stamp).seconds());
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
bool TunnelGuidanceNode::lookupTransformWithFallback(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform
) const {

    try {

        transform = tf_buffer_->lookupTransform(
            target_frame, source_frame, stamp,
            rclcpp::Duration::from_seconds(0.5));
        return true;
    } catch (const tf2::TransformException &) {

        // 动态 TF 启动阶段或者时间戳不完全同步时，使用当前最新 TF 兜底。
        try {

            transform = tf_buffer_->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero);
            return true;
        } catch (const tf2::TransformException &) {

            return false;
        }
    }
}

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

        // 标定后使用墙体模型中的地面平面分类，不依赖 odom 的绝对 Z 值。
        const double ground_distance =
            wall_model_.up.dot(point - wall_model_.center);
        if (std::abs(ground_distance) <= ground_band_) {

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

bool TunnelGuidanceNode::planNextInspectionGoal(
    const std::vector<Eigen::Vector3d> & base_points,
    const rclcpp::Time & stamp,
    const Eigen::Isometry3d & base_to_output
) {

    const TunnelGuidanceSearchResult search_result =
        searcher_->search(base_points);
    if (!search_result.valid) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "No valid inspection goal in the latest point cloud");
        return false;
    }

    CenterlineEstimate centerline;
    centerline.points.reserve(search_result.path.size());
    centerline.tangents.reserve(search_result.path.size());
    for (std::size_t i = 0; i < search_result.path.size(); ++ i) {

        const bool is_goal =
            (search_result.path[i] - search_result.goal).norm() < 1e-6;
        const Eigen::Vector3d tangent =
            !is_goal && i + 1 < search_result.path.size() ?
            search_result.path[i + 1] - search_result.path[i] :
            search_result.goal_tangent;
        centerline.points.push_back(base_to_output * search_result.path[i]);
        centerline.tangents.push_back(
            (base_to_output.linear() * tangent).normalized());
        if (is_goal) break;
    }

    centerline.local_frame.center = base_to_output.translation();
    centerline.local_frame.tangent = centerline.tangents.front();
    centerline.local_frame.up =
        (base_to_output.linear() * Eigen::Vector3d::UnitZ()).normalized();
    centerline.local_frame.lateral =
        centerline.local_frame.up.cross(centerline.local_frame.tangent).normalized();
    centerline.local_frame.valid = true;
    centerline.valid = true;
    publishResults(stamp, centerline, true, true);

    const Eigen::Vector3d goal_output = centerline.points.back();
    RCLCPP_INFO(
        get_logger(),
        "Planned next inspection goal once: x=%.2f y=%.2f clearance=%.2f",
        goal_output.x(), goal_output.y(), search_result.goal_clearance);
    return true;
}

void TunnelGuidanceNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg
) {

    std::vector<Eigen::Vector3d> base_points;
    if (!transformCloudToBase(cloud_msg, base_points))
        return;

    if (exit_detection_enabled_) {

        updateExitDetection(base_points);
    }

    // BIEVR-LIO publishes odometry at the scan end, not at header.stamp.
    const rclcpp::Time stamp = pointCloudEndStamp(*cloud_msg);
    Eigen::Isometry3d base_to_output = Eigen::Isometry3d::Identity();
    if (!getBaseToOutputTransform(stamp, base_to_output)) {

        return;
    }

    // 累积输出的离线数据
    accumulateInspectionDataset(base_points, stamp);

    if (enable_auto_goal_ && search_requested_ &&
        planNextInspectionGoal(base_points, stamp, base_to_output)) {

        search_requested_ = false;
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
            if (!enable_auto_goal_ && has_last_result_ &&
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
            if (!enable_auto_goal_ && calibration_centerline.valid) {

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
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Wall observation invalid: left=%zu right=%zu ground=%zu, "
                "continue using calibrated wall model",
                left_points.size(), right_points.size(), ground_points.size());
        } else {

            updateWallModel(observed_frame);
        }

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
        !exit_detected_ &&
        wall_model_.initialized && consecutive_valid_ >= valid_frame_count_;
    if (!enable_auto_goal_) {

        publishResults(stamp, centerline, publish_valid);
    }
}

double TunnelGuidanceNode::yawFromTangent(
    const Eigen::Vector3d & tangent
) const{

    return std::atan2(tangent.y(), tangent.x());
}

void TunnelGuidanceNode::publishResults(
    const rclcpp::Time & stamp,
    const CenterlineEstimate & centerline,
    bool valid,
    bool use_path_endpoint_as_goal
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
    if (use_path_endpoint_as_goal) {

        goal.pose = path_msg.poses.back().pose;
    } else {

        const std::size_t goal_index = std::min(
            centerline.points.size() - 1,
            static_cast<std::size_t>(
                std::llround(
                    lookahead_distance_ /
                    geometry_params_.centerline_point_spacing)));
        goal.pose = path_msg.poses[goal_index].pose;
    }
    local_goal_pub_->publish(goal);

    if (enable_auto_goal_) {

        if (exit_detected_) {

            // 出口已确认：取消正在执行的 goal，并且不再提供新的候选目标，
            // 避免小车在涵洞外继续沿过期墙体模型前进。
            if (waiting_for_auto_goal_result_ && auto_goal_client_) {

                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Tunnel exit detected, canceling active auto goal");
                auto_goal_client_->async_cancel_all_goals();
                waiting_for_auto_goal_result_ = false;
                has_sent_auto_goal_ = false;
            }
            resetInspectionHandshake();
            has_latest_auto_goal_ = false;
        } else {

            latest_auto_goal_ = goal;
            has_latest_auto_goal_ = true;
            last_published_valid_ = valid;
            // 墙体模型已经标定后，单帧观测失败不应打断正在执行的 Nav2 目标；
            // 当前目标仍由 Nav2 的 costmap/局部规划器负责避障。
            if (!valid && !wall_model_.initialized &&
                waiting_for_auto_goal_result_) {

                auto_goal_client_->async_cancel_all_goals();
                waiting_for_auto_goal_result_ = false;
                has_sent_auto_goal_ = false;
                resetInspectionHandshake();
            }
        }
    }

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

void TunnelGuidanceNode::updateExitDetection(
    const std::vector<Eigen::Vector3d> & base_points)
{
    const TunnelExitObservation observation =
        searcher_->observeExit(base_points, exit_window_);
    if (!observation.valid) {
        return;
    }

    if (observation.corridor_present) {
        had_corridor_ = true;
    }

    const bool exit_candidate = had_corridor_ && observation.open_ahead;
    if (exit_candidate) {
        ++exit_candidate_frames_;
    } else if (!exit_detected_) {
        exit_candidate_frames_ = 0;
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Exit observe L=%.2f R=%.2f F=%.2f corridor=%d open=%d had=%d frames=%d/%d",
        observation.left_column_ratio,
        observation.right_column_ratio,
        observation.front_column_ratio,
        static_cast<int>(observation.corridor_present),
        static_cast<int>(observation.open_ahead),
        static_cast<int>(had_corridor_),
        exit_candidate_frames_,
        exit_confirm_frames_);

    if (!exit_detected_ && exit_candidate_frames_ >= exit_confirm_frames_) {
        exit_detected_ = true;
        RCLCPP_WARN(
            get_logger(),
            "Tunnel exit detected after %d frames (L=%.2f R=%.2f F=%.2f)",
            exit_candidate_frames_,
            observation.left_column_ratio,
            observation.right_column_ratio,
            observation.front_column_ratio);
    }

    std_msgs::msg::Bool exit_msg;
    exit_msg.data = exit_detected_;
    exit_detected_pub_->publish(exit_msg);
}

void TunnelGuidanceNode::maybeSendAutoGoal() {

    if (!enable_auto_goal_ || !auto_goal_client_) {

        return;
    }

    // 退出检测时取消所有目标，重置状态
    if (exit_detected_) {

        if (waiting_for_auto_goal_result_) {

            auto_goal_client_->async_cancel_all_goals();
            waiting_for_auto_goal_result_ = false;
            has_sent_auto_goal_ = false;
        }
        resetInspectionHandshake();
        has_latest_auto_goal_ = false;
        search_requested_ = false;
        return;
    }

    // 执行中不改发新目标，避免滑动前瞻把巡检点不断往前推。
    if (waiting_for_auto_goal_result_) {

        return;
    }

    const rclcpp::Time now = get_clock()->now();
    if (auto_goal_dwelling_) {

        if ((now - dwell_start_time_).seconds() < auto_goal_dwell_time_) {

            return;
        }
        setCaptureEnable(false);
        auto_goal_dwelling_ = false;
        waiting_to_depart_ = true;
        finishInspectionDataset(now);
        RCLCPP_INFO(
            get_logger(),
            "Auto goal dwell finished, capture_enable=false, waiting for MCU "
            "capture_done and local dataset_ready");
        return;
    }

    if (waiting_to_depart_) {

        if (!inspectionCanDepart()) {

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting to depart: mcu_done=%d (need=%d) dataset_ready=%d (need=%d)",
                static_cast<int>(mcu_capture_done_),
                static_cast<int>(allow_capture_done_),
                static_cast<int>(dataset_ready_),
                static_cast<int>(wait_for_dataset_));
            return;
        }
        RCLCPP_INFO(
            get_logger(),
            "Inspection handshake complete (mcu=%d dataset=%d), requesting next search",
            static_cast<int>(mcu_capture_done_ || !allow_capture_done_),
            static_cast<int>(dataset_ready_ || !wait_for_dataset_));
        waiting_to_depart_ = false;
        has_sent_auto_goal_ = false;
        has_latest_auto_goal_ = false;
        search_requested_ = true;
        return;
    }

    if (search_requested_ || !has_latest_auto_goal_) {

        return;
    }

    if (has_sent_auto_goal_) {

        return;
    }

    if ((now - last_auto_goal_send_time_).seconds() < min_goal_send_interval_ &&
        last_auto_goal_send_time_.nanoseconds() > 0)
    {

        return;
    }

    geometry_msgs::msg::PoseStamped goal_in_map;
    if (!transformGoalToMap(latest_auto_goal_, goal_in_map)) {

        return;
    }

    if (!auto_goal_client_->wait_for_action_server(std::chrono::seconds(2))) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "NavigateToPose action server is not available");
        return;
    }

    NavigateToPose::Goal action_goal;
    action_goal.pose = goal_in_map;
    action_goal.behavior_tree = "";

    const uint64_t seq = ++ auto_goal_seq_;
    active_auto_goal_seq_ = seq;

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this, seq](
            const GoalHandleNavigateToPose::SharedPtr & goal_handle
        ) {
            autoGoalResponseCallback(goal_handle, seq);
        };

    send_goal_options.result_callback =
        [this, seq](
            const GoalHandleNavigateToPose::WrappedResult & result
        ) {
            autoGoalResultCallback(result, seq);
        };

    last_sent_auto_goal_ = latest_auto_goal_;
    has_sent_auto_goal_ = true;
    last_auto_goal_send_time_ = now;
    waiting_for_auto_goal_result_ = true;

    RCLCPP_INFO(
        get_logger(),
        "Sending auto goal: x=%.2f y=%.2f yaw=%.2f",
        action_goal.pose.pose.position.x,
        action_goal.pose.pose.position.y,
        std::atan2(
            2.0 * (action_goal.pose.pose.orientation.w * action_goal.pose.pose.orientation.z +
                   action_goal.pose.pose.orientation.x * action_goal.pose.pose.orientation.y),
            1.0 - 2.0 * (action_goal.pose.pose.orientation.y * action_goal.pose.pose.orientation.y +
                         action_goal.pose.pose.orientation.z * action_goal.pose.pose.orientation.z)));
    auto_goal_client_->async_send_goal(action_goal, send_goal_options);
}

void TunnelGuidanceNode::autoGoalResponseCallback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle,
    uint8_t seq
) {

    if (seq != active_auto_goal_seq_) return;

    if (!goal_handle) {

        RCLCPP_WARN(get_logger(), "Auto goal was rejected by NavigateToPose");
        waiting_for_auto_goal_result_ = false;
        has_sent_auto_goal_ = false;
        has_latest_auto_goal_ = false;
        search_requested_ = true;
    }
}

void TunnelGuidanceNode::autoGoalResultCallback(
    const GoalHandleNavigateToPose::WrappedResult & result,
    uint8_t seq
) {

    if (seq != active_auto_goal_seq_) {

        RCLCPP_DEBUG(
            get_logger(),
            "Ignoring result of preempted auto goal");
        return;
    }

    waiting_for_auto_goal_result_ = false;
    switch (result.code) {

        case rclcpp_action::ResultCode::SUCCEEDED:
            auto_goal_dwelling_ = true;
            waiting_to_depart_ = false;
            mcu_capture_done_ = false;
            dwell_start_time_ = get_clock()->now();
            setCaptureEnable(true);
            startInspectionDataset(dwell_start_time_);
            RCLCPP_INFO(
                get_logger(),
                "Auto goal succeeded, capture_enable=true, dwelling for %.1f s",
                auto_goal_dwell_time_);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Auto goal aborted, planning a new inspection goal");
            has_sent_auto_goal_ = false;
            has_latest_auto_goal_ = false;
            resetInspectionHandshake();
            search_requested_ = true;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Auto goal canceled");
            has_sent_auto_goal_ = false;
            has_latest_auto_goal_ = false;
            resetInspectionHandshake();
            search_requested_ = false;
            break;
        default:
            has_sent_auto_goal_ = false;
            has_latest_auto_goal_ = false;
            resetInspectionHandshake();
            search_requested_ = true;
            break;
    }
}

void TunnelGuidanceNode::onGimbal(const rm_interfaces::msg::Gimbal::SharedPtr msg)
{
    if ((auto_goal_dwelling_ || waiting_to_depart_) &&
        msg->capture_done && !mcu_capture_done_)
    {
        mcu_capture_done_ = true;
        RCLCPP_INFO(get_logger(), "Received MCU capture_done=true");
        publishHandshakeFlags();
    }
}

void TunnelGuidanceNode::setCaptureEnable(bool enable)
{
    std_msgs::msg::Bool msg;
    msg.data = enable;
    capture_enable_pub_->publish(msg);
}

void TunnelGuidanceNode::resetInspectionHandshake() {

    auto_goal_dwelling_ = false;
    waiting_to_depart_ = false;
    mcu_capture_done_ = false;
    dataset_recorder_.abortStation();
    dataset_ready_ = true;
    setCaptureEnable(false);
    publishHandshakeFlags();
}

// 发布握手标志
void TunnelGuidanceNode::publishHandshakeFlags() {

    std_msgs::msg::Bool dataset_msg;
    dataset_msg.data = dataset_ready_;
    dataset_ready_pub_->publish(dataset_msg);

    std_msgs::msg::Bool depart_msg;
    depart_msg.data = inspectionCanDepart();
    can_depart_pub_->publish(depart_msg);
}

// 检查是否可以离开
bool TunnelGuidanceNode::inspectionCanDepart() const {

    return (mcu_capture_done_ || !allow_capture_done_) &&
        (dataset_ready_ || !wait_for_dataset_);
}

bool TunnelGuidanceNode::lookupMapPose(
    const rclcpp::Time & stamp,
    Eigen::Isometry3d & pose_map_base) const {

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransformWithFallback(
            auto_goal_frame_id_, estimation_frame_, stamp, transform)
    ) {

        return false;
    }
    pose_map_base = tf2::transformToEigen(transform);
    return true;
}

void TunnelGuidanceNode::startInspectionDataset(const rclcpp::Time & stamp) {

    dataset_ready_ = false;
    if (!enable_dataset_recording_) {

        dataset_ready_ = true;
        publishHandshakeFlags();
        return;
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (!lookupMapPose(stamp, pose)) {

        RCLCPP_WARN(
            get_logger(),
            "No %s pose at inspection start, recording with identity pose",
            auto_goal_frame_id_.c_str());
    }
    if (!dataset_recorder_.beginStation(stamp.nanoseconds(), pose)) {

        RCLCPP_ERROR(get_logger(), "Failed to begin inspection dataset station");
        dataset_ready_ = false;
        publishHandshakeFlags();
        return;
    }
    publishHandshakeFlags();
}

void TunnelGuidanceNode::accumulateInspectionDataset(
    const std::vector<Eigen::Vector3d> & base_points,
    const rclcpp::Time & stamp
) {

    if (!dataset_recorder_.stationActive()) {

        return;
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (!lookupMapPose(stamp, pose)) {

        return;
    }

    std::vector<Eigen::Vector3d> map_points;
    map_points.reserve(base_points.size());
    for (const Eigen::Vector3d & point : base_points) {

        map_points.push_back(pose * point);
    }
    dataset_recorder_.addScan(map_points, pose);
}

void TunnelGuidanceNode::finishInspectionDataset(const rclcpp::Time & stamp) {

    if (!enable_dataset_recording_ || !dataset_recorder_.stationActive()) {

        dataset_ready_ = true;
        publishHandshakeFlags();
        return;
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    lookupMapPose(stamp, pose);
    InspectionStationSummary summary;
    if (!dataset_recorder_.finishStation(stamp.nanoseconds(), pose, summary)) {

        RCLCPP_ERROR(
            get_logger(),
            "Failed to save inspection dataset station, holding departure");
        dataset_ready_ = false;
        publishHandshakeFlags();
        return;
    }

    dataset_ready_ = true;
    RCLCPP_INFO(
        get_logger(),
        "Saved inspection station %d: %s points=%zu scans=%d drift=%.3f m merged=%zu",
        summary.id, summary.cloud_relpath.c_str(), summary.point_count,
        summary.scan_count, summary.max_drift_m, summary.merged_point_count);
    sensor_msgs::msg::PointCloud2 merged_msg;
    pcl::toROSMsg(dataset_recorder_.mergedCloud(), merged_msg);
    merged_msg.header.stamp = stamp;
    merged_msg.header.frame_id = auto_goal_frame_id_;
    merged_map_pub_->publish(merged_msg);
    publishHandshakeFlags();
}

bool TunnelGuidanceNode::transformGoalToMap(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output
) const {

    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransformWithFallback(
            auto_goal_frame_id_, input.header.frame_id,
            input.header.stamp, transform)) {

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Cannot transform local goal to %s", auto_goal_frame_id_.c_str());
        return false;
    }

    tf2::doTransform(input, output, transform);
    return true;
}

}  // namespace my_tunnel_guidance


int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_tunnel_guidance::TunnelGuidanceNode>());
    rclcpp::shutdown();
    return 0;
}
