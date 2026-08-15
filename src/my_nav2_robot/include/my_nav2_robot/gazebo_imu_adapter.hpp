#ifndef MY_NAV2_ROBOT__GAZEBO_IMU_ADAPTER_HPP
#define MY_NAV2_ROBOT__GAZEBO_IMU_ADAPTER_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace my_nav2_robot {

class GazeboImuAdapter : public rclcpp::Node {
public:
    GazeboImuAdapter();

private:
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &message);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &message);

    std::string input_topic_;
    std::string output_topic_;
    std::string kinematic_odometry_topic_;
    bool use_kinematic_acceleration_ = true;
    bool have_previous_odometry_ = false;
    bool have_kinematic_acceleration_ = false;
    std::int64_t previous_odometry_stamp_ns_ = 0;
    geometry_msgs::msg::Vector3 previous_linear_velocity_;
    geometry_msgs::msg::Vector3 kinematic_acceleration_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

}  // namespace my_nav2_robot

#endif  // MY_NAV2_ROBOT__GAZEBO_IMU_ADAPTER_HPP
