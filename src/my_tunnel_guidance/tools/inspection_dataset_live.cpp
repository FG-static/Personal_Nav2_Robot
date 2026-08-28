#include "my_tunnel_guidance/inspection_dataset_recorder.hpp"
#include "my_tunnel_guidance/pointcloud_timing.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("inspection_dataset_live");
    const int station_count = node->declare_parameter("station_count", 2);
    const double station_duration = node->declare_parameter("station_duration", 1.5);
    const double station_gap = node->declare_parameter("station_gap", 4.0);
    const double gap_speed = node->declare_parameter("gap_speed", 0.35);
    const double voxel_size = node->declare_parameter("voxel_size", 0.03);
    const std::string frame_id = node->declare_parameter("frame_id", std::string("map"));
    const std::string output_dir = node->declare_parameter(
        "output_dir", std::string("/tmp/tunnel_inspections"));
    const std::string cloud_topic = node->declare_parameter(
        "cloud_topic", std::string("/livox/lidar"));

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    my_tunnel_guidance::InspectionDatasetRecorder recorder;
    if (!recorder.openMission(output_dir, frame_id, voxel_size)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open %s", output_dir.c_str());
        return 1;
    }

    auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    pcl::PointCloud<pcl::PointXYZ> latest;
    std::string latest_frame;
    rclcpp::Time latest_stamp(0, 0, RCL_ROS_TIME);
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            pcl::fromROSMsg(*msg, latest);
            latest_frame = msg->header.frame_id;
            latest_stamp = my_tunnel_guidance::pointCloudEndStamp(*msg);
        });

    const auto waitForCloud = [&]() {
        const auto wall_start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && latest.empty()) {
            rclcpp::spin_some(node);
            if (std::chrono::steady_clock::now() - wall_start >
                std::chrono::seconds(20))
            {
                return false;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
        return !latest.empty();
    };

    if (!waitForCloud()) {
        RCLCPP_ERROR(node->get_logger(), "No point cloud on %s", cloud_topic.c_str());
        return 1;
    }

    for (int i = 0; i < station_count && rclcpp::ok(); ++i) {
        rclcpp::spin_some(node);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        try {
            pose = tf2::transformToEigen(
                tf_buffer->lookupTransform(
                    frame_id, "base_link", latest_stamp, rclcpp::Duration::from_seconds(0.3)));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node->get_logger(), "TF at station start: %s", ex.what());
        }
        if (!recorder.beginStation(latest_stamp.nanoseconds(), pose)) {
            RCLCPP_ERROR(node->get_logger(), "beginStation failed");
            return 1;
        }

        const rclcpp::Time window_start = node->now();
        while (rclcpp::ok() &&
            (node->now() - window_start).seconds() < station_duration)
        {
            rclcpp::spin_some(node);
            if (latest.empty()) {
                continue;
            }
            try {
                const Eigen::Isometry3d sensor_to_map = tf2::transformToEigen(
                    tf_buffer->lookupTransform(
                        frame_id, latest_frame, latest_stamp,
                        rclcpp::Duration::from_seconds(0.2)));
                pose = tf2::transformToEigen(
                    tf_buffer->lookupTransform(
                        frame_id, "base_link", latest_stamp,
                        rclcpp::Duration::from_seconds(0.2)));
                std::vector<Eigen::Vector3d> map_points;
                map_points.reserve(latest.size());
                for (const auto & point : latest) {
                    if (!std::isfinite(point.x) ||
                        !std::isfinite(point.y) ||
                        !std::isfinite(point.z)) {
                        continue;
                    }
                    map_points.push_back(
                        sensor_to_map * Eigen::Vector3d(point.x, point.y, point.z));
                }
                recorder.addScan(map_points, pose);
            } catch (const tf2::TransformException &) {
            }
            latest.clear();
        }

        my_tunnel_guidance::InspectionStationSummary summary;
        if (!recorder.finishStation(node->now().nanoseconds(), pose, summary)) {
            RCLCPP_ERROR(node->get_logger(), "finishStation failed");
            return 1;
        }
        RCLCPP_INFO(
            node->get_logger(),
            "Station %d saved %s points=%zu merged=%zu",
            summary.id, summary.cloud_relpath.c_str(),
            summary.point_count, summary.merged_point_count);

        if (i + 1 < station_count && station_gap > 0.0 && gap_speed > 0.0) {
            RCLCPP_INFO(
                node->get_logger(),
                "Driving %.1f s at %.2f m/s before next station",
                station_gap, gap_speed);
            const rclcpp::Time drive_start = node->now();
            geometry_msgs::msg::Twist twist;
            twist.linear.x = gap_speed;
            while (rclcpp::ok() &&
                (node->now() - drive_start).seconds() < station_gap)
            {
                cmd_pub->publish(twist);
                rclcpp::spin_some(node);
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            twist.linear.x = 0.0;
            cmd_pub->publish(twist);
        }
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Merged map %s/%s points=%zu",
        output_dir.c_str(),
        my_tunnel_guidance::InspectionDatasetRecorder::mergedMapRelpath(),
        recorder.mergedPointCount());
    rclcpp::shutdown();
    return 0;
}
