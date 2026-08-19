#ifndef MY_NAV2_ROBOT__POINTCLOUD_TO_LASERSCAN_NODE_HPP
#define MY_NAV2_ROBOT__POINTCLOUD_TO_LASERSCAN_NODE_HPP

#include <limits>
#include <memory>
#include <string>

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace my_nav2_robot {

// In-tree replacement for ros-humble-pointcloud-to-laserscan.
// Projects /livox/lidar (PointCloud2) onto a virtual 2D LaserScan for Nav2.
class PointCloudToLaserScanNode : public rclcpp::Node {
public:
    PointCloudToLaserScanNode();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg);

    std::string target_frame_;
    double transform_tolerance_ = 0.3;
    double min_height_ = std::numeric_limits<double>::lowest();
    double max_height_ = std::numeric_limits<double>::max();
    double angle_min_ = -M_PI;
    double angle_max_ = M_PI;
    double angle_increment_ = M_PI / 180.0;
    double scan_time_ = 0.1;
    double range_min_ = 0.0;
    double range_max_ = std::numeric_limits<double>::max();
    double inf_epsilon_ = 1.0;
    bool use_inf_ = true;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
};

}  // namespace my_nav2_robot

#endif  // MY_NAV2_ROBOT__POINTCLOUD_TO_LASERSCAN_NODE_HPP
