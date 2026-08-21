/*
  安装依赖
  sudo apt install ros-humble-serial-driver
  以下仅供参考
*/

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
namespace
{
std::string toHex(const std::vector<uint8_t> & data)
{
  std::string out;
  out.reserve(data.size() * 3);
  char buf[4];
  for (size_t i = 0; i < data.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%s%02X", i == 0 ? "" : " ", data[i]);
    out += buf;
  }
  return out;
}
}  // namespace

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!");

  getParams();

  // publisher/subscriber 必须在接收线程启动前初始化，否则线程调用 publish() 时
  // gimbal_pub_ 还是空指针，导致 SIGSEGV
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(1),
    std::bind(&RMSerialDriver::onCmdVel, this, std::placeholders::_1));

  chassis_cmd_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
    chassis_cmd_topic_, rclcpp::QoS(1),
    std::bind(&RMSerialDriver::onChassisCmd, this, std::placeholders::_1));

  capture_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    capture_enable_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&RMSerialDriver::onCaptureEnable, this, std::placeholders::_1));

  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::Gimbal>(
    "/tracker/gimbal", 10);

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
    }
    receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    RCLCPP_INFO(
      get_logger(),
      "Opened %s. TX: AA + vx + wz + capture_enable + 55 (%zu B). "
      "RX: AA + temp + capture_done + 55 (%zu B). cmd_vel=%s chassis_cmd=%s capture_enable=%s",
      device_name_.c_str(), TX_FRAME_LEN, RX_FRAME_LEN,
      cmd_vel_topic_.c_str(), chassis_cmd_topic_.c_str(), capture_enable_topic_.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  if (cmd_send_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / cmd_send_hz_);
    cmd_tx_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RMSerialDriver::sendCmdTick, this));
    RCLCPP_INFO(
      get_logger(), "Chassis TX %.1f Hz, timeout %.2f s", cmd_send_hz_, cmd_timeout_sec_);
  }
}

RMSerialDriver::~RMSerialDriver()
{
  if (cmd_tx_timer_) {
    cmd_tx_timer_->cancel();
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  // RX：0xAA + uint32 Temp + uint8 capture_done + 0x55。Linux CDC 可能拆包，搜帧头后再读满。
  std::vector<uint8_t> hdr(1);
  std::vector<uint8_t> rest(RX_FRAME_LEN - 1);

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(hdr);
      if (hdr[0] != FRAME_HEADER) {
        continue;
      }

      serial_driver_->port()->receive(rest);
      if (rest.back() != FRAME_TAIL) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 200,
          "frame tail mismatch: got 0x%02X", rest.back());
        continue;
      }

      ReceiveFrame frame{};
      frame.header = FRAME_HEADER;
      std::copy(rest.begin(), rest.end(), reinterpret_cast<uint8_t *>(&frame) + 1);
      if (frame.tail != FRAME_TAIL) {
        continue;
      }

      rm_interfaces::msg::Gimbal gimbal_msg;
      gimbal_msg.header.stamp = this->get_clock()->now();
      gimbal_msg.temp = frame.temp;
      if (frame.capture_done == 0U || frame.capture_done == 1U) {
        gimbal_msg.capture_done = (frame.capture_done == 1U);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "illegal capture_done=%u, expected 0 or 1", frame.capture_done);
      }
      gimbal_pub_->publish(gimbal_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "RX temp=%u capture_done=%u", frame.temp, frame.capture_done);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latchCommand(static_cast<float>(msg->linear.x), static_cast<float>(msg->angular.z));
}

void RMSerialDriver::onChassisCmd(const rm_interfaces::msg::Target::SharedPtr msg)
{
  latchCommand(msg->vx, msg->wz);
  latchCaptureEnable(msg->capture_enable);
}

void RMSerialDriver::onCaptureEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  latchCaptureEnable(msg->data);
}

void RMSerialDriver::latchCommand(float vx, float wz)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  vx_ = vx;
  wz_ = wz;
  last_cmd_time_ = this->now();
  has_cmd_ = true;
}

void RMSerialDriver::latchCaptureEnable(bool enable)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  capture_enable_ = enable;
}

void RMSerialDriver::sendCmdTick()
{
  float vx = 0.f;
  float wz = 0.f;
  bool capture_enable = false;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (has_cmd_ && (this->now() - last_cmd_time_).seconds() <= cmd_timeout_sec_) {
      vx = vx_;
      wz = wz_;
    }
    capture_enable = capture_enable_;
  }
  transmit(vx, wz, capture_enable);
}

void RMSerialDriver::transmit(float vx, float wz, bool capture_enable)
{
  try {
    if (!serial_driver_->port()->is_open()) {
      return;
    }

    SendFrame frame{};
    frame.header = FRAME_HEADER;
    frame.vx = vx;
    frame.wz = wz;
    frame.capture_enable = boolToU8(capture_enable);
    frame.tail = FRAME_TAIL;

    std::lock_guard<std::mutex> lock(send_mutex_);
    const std::vector<uint8_t> data = toVector(frame);
    const size_t written = serial_driver_->port()->send(data);
    if (written != data.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 200,
        "short serial write: %zu / %zu", written, data.size());
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TX vx=%.3f wz=%.3f capture_enable=%u raw=[%s]",
      vx, wz, static_cast<unsigned>(boolToU8(capture_enable)), toHex(data).c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity parameter provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  chassis_cmd_topic_ = declare_parameter<std::string>("chassis_cmd_topic", "/chassis_cmd");
  capture_enable_topic_ = declare_parameter<std::string>("capture_enable_topic", "/capture_enable");
  cmd_send_hz_ = declare_parameter<double>("cmd_send_hz", 20.0);
  cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.3);

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
