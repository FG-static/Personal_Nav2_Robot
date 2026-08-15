#ifndef MY_NAV2_ROBOT__SIMULATED_IMU_PUBLISHER_HPP
#define MY_NAV2_ROBOT__SIMULATED_IMU_PUBLISHER_HPP

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace my_nav2_robot {

class SimulatedImuPublisher : public rclcpp::Node {
public:
    SimulatedImuPublisher();

private:
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &message);
    bool isOdometryFinite(const nav_msgs::msg::Odometry &message) const;
    geometry_msgs::msg::Vector3 computeGravityInBody(
        const geometry_msgs::msg::Quaternion &orientation) const;

    std::string odometry_topic_;
    std::string output_topic_;
    std::string frame_id_;
    double gravity_magnitude_ = 9.8;
    double maximum_time_step_ = 0.1;

    bool have_previous_odometry_ = false;
    std::int64_t previous_stamp_ns_ = 0;
    geometry_msgs::msg::Vector3 previous_linear_velocity_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

}  // namespace my_nav2_robot

#endif  // MY_NAV2_ROBOT__SIMULATED_IMU_PUBLISHER_HPP
