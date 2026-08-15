#include "my_nav2_robot/simulated_imu_publisher.hpp"

#include <cmath>
#include <functional>
#include <memory>

namespace my_nav2_robot {

SimulatedImuPublisher::SimulatedImuPublisher()
    : Node("simulated_imu_publisher")
{
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/ground_truth");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/imu");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    gravity_magnitude_ = declare_parameter<double>("gravity_magnitude", 9.8);
    maximum_time_step_ = declare_parameter<double>("maximum_time_step", 0.1);

    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, sensor_qos);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, sensor_qos,
        std::bind(&SimulatedImuPublisher::odometryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(), "Synthesizing IMU %s from %s at the odometry message rate",
        output_topic_.c_str(), odometry_topic_.c_str());
}

bool SimulatedImuPublisher::isOdometryFinite(
    const nav_msgs::msg::Odometry &message) const
{
    const geometry_msgs::msg::Quaternion &orientation = message.pose.pose.orientation;
    const geometry_msgs::msg::Vector3 &linear_velocity = message.twist.twist.linear;
    const geometry_msgs::msg::Vector3 &angular_velocity = message.twist.twist.angular;
    return std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
        std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
        std::isfinite(linear_velocity.x) && std::isfinite(linear_velocity.y) &&
        std::isfinite(linear_velocity.z) && std::isfinite(angular_velocity.x) &&
        std::isfinite(angular_velocity.y) && std::isfinite(angular_velocity.z);
}

geometry_msgs::msg::Vector3 SimulatedImuPublisher::computeGravityInBody(
    const geometry_msgs::msg::Quaternion &orientation) const
{
    const double norm = std::sqrt(
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w);
    geometry_msgs::msg::Vector3 gravity;
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        gravity.z = gravity_magnitude_;
        return gravity;
    }

    const double x = orientation.x / norm;
    const double y = orientation.y / norm;
    const double z = orientation.z / norm;
    const double w = orientation.w / norm;

    // R_world_body^T * [0, 0, g] is the specific force measured at rest.
    gravity.x = 2.0 * (x * z - w * y) * gravity_magnitude_;
    gravity.y = 2.0 * (y * z + w * x) * gravity_magnitude_;
    gravity.z = (1.0 - 2.0 * (x * x + y * y)) * gravity_magnitude_;
    return gravity;
}

void SimulatedImuPublisher::odometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &message)
{
    if (!isOdometryFinite(*message)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping ground-truth odometry containing non-finite values");
        return;
    }

    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(message->header.stamp.nanosec);
    const geometry_msgs::msg::Vector3 &linear_velocity = message->twist.twist.linear;
    const geometry_msgs::msg::Vector3 &angular_velocity = message->twist.twist.angular;
    const geometry_msgs::msg::Vector3 gravity =
        computeGravityInBody(message->pose.pose.orientation);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = message->header.stamp;
    imu.header.frame_id = frame_id_;
    imu.orientation = message->pose.pose.orientation;
    imu.angular_velocity = angular_velocity;
    imu.linear_acceleration = gravity;

    if (have_previous_odometry_) {
        const double dt = static_cast<double>(stamp_ns - previous_stamp_ns_) * 1.0e-9;
        if (dt > 1.0e-5 && dt <= maximum_time_step_) {
            const double acceleration_x =
                (linear_velocity.x - previous_linear_velocity_.x) / dt;
            const double acceleration_y =
                (linear_velocity.y - previous_linear_velocity_.y) / dt;
            const double acceleration_z =
                (linear_velocity.z - previous_linear_velocity_.z) / dt;

            // Twist is in the child frame, so add omega x v to obtain body-frame acceleration.
            imu.linear_acceleration.x += acceleration_x +
                angular_velocity.y * linear_velocity.z -
                angular_velocity.z * linear_velocity.y;
            imu.linear_acceleration.y += acceleration_y +
                angular_velocity.z * linear_velocity.x -
                angular_velocity.x * linear_velocity.z;
            imu.linear_acceleration.z += acceleration_z +
                angular_velocity.x * linear_velocity.y -
                angular_velocity.y * linear_velocity.x;
        }
    }

    imu_publisher_->publish(imu);
    previous_stamp_ns_ = stamp_ns;
    previous_linear_velocity_ = linear_velocity;
    have_previous_odometry_ = true;
}

}  // namespace my_nav2_robot

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_nav2_robot::SimulatedImuPublisher>());
    rclcpp::shutdown();
    return 0;
}
