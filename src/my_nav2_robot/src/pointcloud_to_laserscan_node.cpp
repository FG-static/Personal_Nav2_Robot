#include "my_nav2_robot/pointcloud_to_laserscan_node.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

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
    min_height_ = declare_parameter<double>("min_height", 0.02);
    max_height_ = declare_parameter<double>("max_height", 0.40);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = declare_parameter<double>("angle_increment", M_PI / 180.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.1);
    range_min_ = declare_parameter<double>("range_min", 0.2);
    range_max_ = declare_parameter<double>("range_max", 12.0);
    inf_epsilon_ = declare_parameter<double>("inf_epsilon", 1.0);
    use_inf_ = declare_parameter<bool>("use_inf", true);
    accumulation_duration_ = declare_parameter<double>("accumulation_duration", 0.0);

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
        "Projecting cloud_in to scan in frame '%s' (z in [%.3f, %.3f], range [%.2f, %.2f], "
        "accumulate %.2fs)",
        target_frame_.c_str(), min_height_, max_height_, range_min_, range_max_,
        accumulation_duration_);
}

void PointCloudToLaserScanNode::projectCloud(
    const sensor_msgs::msg::PointCloud2 &cloud_msg,
    std::vector<float> &ranges) const
{
    Eigen::Isometry3d cloud_to_target = Eigen::Isometry3d::Identity();
    const bool need_transform =
        !target_frame_.empty() && target_frame_ != cloud_msg.header.frame_id;
    if (need_transform) {
        try {
            const geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(
                    target_frame_,
                    cloud_msg.header.frame_id,
                    cloud_msg.header.stamp,
                    tf2::durationFromSec(transform_tolerance_));
            cloud_to_target = tf2::transformToEigen(transform);
        } catch (const tf2::TransformException &) {
            return;
        }
    }

    const uint32_t ranges_size = static_cast<uint32_t>(ranges.size());
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
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
        if (angle < angle_min_ || angle > angle_max_) {
            continue;
        }

        int index = static_cast<int>((angle - angle_min_) / angle_increment_);
        if (index < 0) {
            index = 0;
        } else if (index >= static_cast<int>(ranges_size)) {
            index = static_cast<int>(ranges_size) - 1;
        }
        const auto bin = static_cast<size_t>(index);
        if (!std::isfinite(ranges[bin]) || range < static_cast<double>(ranges[bin])) {
            ranges[bin] = static_cast<float>(range);
        }
    }
}

void PointCloudToLaserScanNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
{
    const auto clock_type = get_clock()->get_clock_type();
    const rclcpp::Time latest(cloud_msg->header.stamp, clock_type);
    if (!cloud_buffer_.empty()) {
        const rclcpp::Time previous(cloud_buffer_.back()->header.stamp, clock_type);
        if (latest < previous) {
            cloud_buffer_.clear();
        }
    }
    cloud_buffer_.push_back(cloud_msg);
    if (accumulation_duration_ <= 0.0) {
        while (cloud_buffer_.size() > 1U) {
            cloud_buffer_.pop_front();
        }
    } else {
        while (!cloud_buffer_.empty()) {
            const rclcpp::Time oldest(cloud_buffer_.front()->header.stamp, clock_type);
            if ((latest - oldest).seconds() > accumulation_duration_) {
                cloud_buffer_.pop_front();
            } else {
                break;
            }
        }
    }

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
        scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
    }

    for (const auto &cloud : cloud_buffer_) {
        projectCloud(*cloud, scan_msg->ranges);
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
