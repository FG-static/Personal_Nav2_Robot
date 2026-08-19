#include "my_nav2_robot/pointcloud_to_laserscan_node.hpp"

#include <cmath>
#include <cstdint>

#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/create_timer_ros.h>

namespace my_nav2_robot {

PointCloudToLaserScanNode::PointCloudToLaserScanNode()
    : Node("pointcloud_to_laserscan")
{
    target_frame_ = declare_parameter<std::string>("target_frame", "laser_link");
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.3);
    min_height_ = declare_parameter<double>("min_height", -0.15);
    max_height_ = declare_parameter<double>("max_height", 0.40);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = declare_parameter<double>("angle_increment", M_PI / 180.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.1);
    range_min_ = declare_parameter<double>("range_min", 0.2);
    range_max_ = declare_parameter<double>("range_max", 12.0);
    inf_epsilon_ = declare_parameter<double>("inf_epsilon", 1.0);
    use_inf_ = declare_parameter<bool>("use_inf", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", sensor_qos);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in", sensor_qos,
        std::bind(&PointCloudToLaserScanNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Projecting cloud_in to scan in frame '%s' (z in [%.3f, %.3f], range [%.2f, %.2f])",
        target_frame_.c_str(), min_height_, max_height_, range_min_, range_max_);
}

void PointCloudToLaserScanNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
{
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header = cloud_msg->header;
    if (!target_frame_.empty()) {
        scan_msg->header.frame_id = target_frame_;
    }
    scan_msg->angle_min = angle_min_;
    scan_msg->angle_max = angle_max_;
    scan_msg->angle_increment = angle_increment_;
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time_;
    scan_msg->range_min = range_min_;
    scan_msg->range_max = range_max_;

    if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Invalid angle_min/angle_max/angle_increment");
        return;
    }

    const uint32_t ranges_size = static_cast<uint32_t>(std::ceil(
        (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment));
    if (use_inf_) {
        scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
    } else {
        scan_msg->ranges.assign(
            ranges_size, static_cast<float>(scan_msg->range_max + inf_epsilon_));
    }

    Eigen::Isometry3d cloud_to_target = Eigen::Isometry3d::Identity();
    const bool need_transform =
        !target_frame_.empty() && target_frame_ != cloud_msg->header.frame_id;
    if (need_transform) {
        try {
            const geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(
                    target_frame_,
                    cloud_msg->header.frame_id,
                    cloud_msg->header.stamp,
                    tf2::durationFromSec(transform_tolerance_));
            cloud_to_target = tf2::transformToEigen(transform);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Transform failure: %s", ex.what());
            return;
        }
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Eigen::Vector3d point(static_cast<double>(*iter_x),
            static_cast<double>(*iter_y), static_cast<double>(*iter_z));
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
            !std::isfinite(point.z())) {
            continue;
        }
        if (need_transform) {
            point = cloud_to_target * point;
        }
        if (point.z() > max_height_ || point.z() < min_height_) {
            continue;
        }

        const double range = std::hypot(point.x(), point.y());
        if (range < range_min_ || range > range_max_) {
            continue;
        }

        const double angle = std::atan2(point.y(), point.x());
        if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
            continue;
        }

        int index = static_cast<int>(
            (angle - scan_msg->angle_min) / scan_msg->angle_increment);
        if (index < 0) {
            index = 0;
        } else if (index >= static_cast<int>(ranges_size)) {
            index = static_cast<int>(ranges_size) - 1;
        }
        if (range < scan_msg->ranges[static_cast<size_t>(index)]) {
            scan_msg->ranges[static_cast<size_t>(index)] = static_cast<float>(range);
        }
    }

    scan_publisher_->publish(std::move(scan_msg));
}

}  // namespace my_nav2_robot

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_nav2_robot::PointCloudToLaserScanNode>());
    rclcpp::shutdown();
    return 0;
}
