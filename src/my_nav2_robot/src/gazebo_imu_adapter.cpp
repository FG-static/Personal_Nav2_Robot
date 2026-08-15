#include "my_nav2_robot/gazebo_imu_adapter.hpp"

#include <cmath>
#include <functional>
#include <memory>

namespace my_nav2_robot {

GazeboImuAdapter::GazeboImuAdapter()
    : Node("gazebo_imu_adapter")
{
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/imu_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/imu");
    kinematic_odometry_topic_ =
        declare_parameter<std::string>("kinematic_odometry_topic", "/ground_truth");
    use_kinematic_acceleration_ =
        declare_parameter<bool>("use_kinematic_acceleration", true);

    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, sensor_qos);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        input_topic_, sensor_qos,
        std::bind(&GazeboImuAdapter::imuCallback, this, std::placeholders::_1));
    if (use_kinematic_acceleration_) {
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            kinematic_odometry_topic_, sensor_qos,
            std::bind(&GazeboImuAdapter::odometryCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
        get_logger(),
        "Adapting Gazebo IMU %s -> %s (kinematic acceleration from %s: %s)",
        input_topic_.c_str(), output_topic_.c_str(),
        kinematic_odometry_topic_.c_str(), use_kinematic_acceleration_ ? "true" : "false");
}

void GazeboImuAdapter::imuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr &message)
{
    const geometry_msgs::msg::Vector3 &acceleration = message->linear_acceleration;
    const geometry_msgs::msg::Vector3 &angular_velocity = message->angular_velocity;
    if (!std::isfinite(acceleration.x) || !std::isfinite(acceleration.y) ||
        !std::isfinite(acceleration.z) || !std::isfinite(angular_velocity.x) ||
        !std::isfinite(angular_velocity.y) || !std::isfinite(angular_velocity.z))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping Gazebo IMU sample containing non-finite values");
        return;
    }

    sensor_msgs::msg::Imu corrected = *message;
    if (use_kinematic_acceleration_ && have_kinematic_acceleration_) {
        corrected.linear_acceleration.x = kinematic_acceleration_.x;
        corrected.linear_acceleration.y = kinematic_acceleration_.y;
    }
    imu_publisher_->publish(corrected);
}

void GazeboImuAdapter::odometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &message)
{
    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(message->header.stamp.nanosec);
    const geometry_msgs::msg::Vector3 &linear_velocity = message->twist.twist.linear;
    const geometry_msgs::msg::Vector3 &angular_velocity = message->twist.twist.angular;

    if (!std::isfinite(linear_velocity.x) || !std::isfinite(linear_velocity.y) ||
        !std::isfinite(angular_velocity.z))
    {
        return;
    }

    if (have_previous_odometry_) {
        const double dt = static_cast<double>(stamp_ns - previous_odometry_stamp_ns_) * 1.0e-9;
        if (dt > 1.0e-4 && dt < 0.5) {
            // Odometry twist is expressed in the child frame. Include omega x v
            // so the finite difference is the body-frame inertial acceleration.
            kinematic_acceleration_.x =
                (linear_velocity.x - previous_linear_velocity_.x) / dt -
                angular_velocity.z * linear_velocity.y;
            kinematic_acceleration_.y =
                (linear_velocity.y - previous_linear_velocity_.y) / dt +
                angular_velocity.z * linear_velocity.x;
            have_kinematic_acceleration_ = true;
        } else {
            have_kinematic_acceleration_ = false;
        }
    }

    previous_odometry_stamp_ns_ = stamp_ns;
    previous_linear_velocity_ = linear_velocity;
    have_previous_odometry_ = true;
}

}  // namespace my_nav2_robot

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_nav2_robot::GazeboImuAdapter>());
    rclcpp::shutdown();
    return 0;
}
