#ifndef MY_TUNNEL_GUIDANCE__POINTCLOUD_TIMING_HPP_
#define MY_TUNNEL_GUIDANCE__POINTCLOUD_TIMING_HPP_

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>

namespace my_tunnel_guidance {

std::uint32_t maxPointTimeOffsetNanoseconds(
    const sensor_msgs::msg::PointCloud2 & cloud);

rclcpp::Time pointCloudEndStamp(
    const sensor_msgs::msg::PointCloud2 & cloud);

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__POINTCLOUD_TIMING_HPP_
