// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/gimbal.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void receiveData();

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

  void onChassisCmd(const rm_interfaces::msg::Target::SharedPtr msg);

  void onCaptureEnable(const std_msgs::msg::Bool::SharedPtr msg);

  void latchCommand(float vx, float wz);

  void latchCaptureEnable(bool enable);

  void sendCmdTick();

  void transmit(float vx, float wz, bool capture_enable);

  void reopenPort();

  void getParams();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr chassis_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_enable_sub_;
  rclcpp::Publisher<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr cmd_tx_timer_;

  std::mutex send_mutex_;
  std::mutex cmd_mutex_;
  float vx_{0.f};
  float wz_{0.f};
  bool capture_enable_{false};
  bool has_cmd_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string chassis_cmd_topic_{"/chassis_cmd"};
  std::string capture_enable_topic_{"/capture_enable"};
  double cmd_send_hz_{20.0};
  double cmd_timeout_sec_{0.3};

  std::thread receive_thread_;
};

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
