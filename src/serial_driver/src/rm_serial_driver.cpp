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
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!");

  getParams();

  // publisher/subscriber 必须在接收线程启动前初始化，否则线程调用 publish() 时
  // gimbal_pub_ 还是空指针，导致 SIGSEGV
  target_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::Gimbal>(
    "/tracker/gimbal", 10);

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
}

RMSerialDriver::~RMSerialDriver()
{
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
  // 帧格式：
  // [A5][5A][seq:1][len:1][payload:46][CRC16-CCITT:2] = 52 bytes
  // CRC 覆盖前 50 字节（包含帧头），小端序存储在帧尾。
  //
  // 解帧策略：逐字节扫描找 A5 5A，再读剩余 50 字节，
  // 校验 len 字段与 CRC16-CCITT，通过后用 memcpy 解析 payload。
  // 若 len 或 CRC 不匹配则丢弃本次同步点，重新扫描。

  std::vector<uint8_t> hdr(1);
  // rest = seq(1) + len(1) + payload(46) + CRC16(2) = 50 bytes
  std::vector<uint8_t> rest(FRAME_TOTAL_LEN - 2);

  while (rclcpp::ok()) {
    try {
      // ---- 找帧头第一字节 A5 ----
      serial_driver_->port()->receive(hdr);
      if (hdr[0] != FRAME_SOF0) {
        continue;
      }

      // ---- 确认帧头第二字节 5A ----
      serial_driver_->port()->receive(hdr);
      if (hdr[0] != FRAME_SOF1) {
        // 当前字节可能是下一帧的 A5，不丢弃——但 serial API 不支持 unread，
        // 直接 continue 重新扫描（最多丢一个同步机会，CRC 兜底）
        continue;
      }

      // ---- 读剩余 50 字节（port阻塞直到全部到达）----
      serial_driver_->port()->receive(rest);

      // ---- 校验 len 字段（rest[1] 即 frame[3]）----
      if (rest[1] != FRAME_PAYLOAD_LEN) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 200,
          "len field mismatch: got %u, expected %u", rest[1], FRAME_PAYLOAD_LEN);
        continue;
      }

      // ---- 拼完整帧，做 CRC16-CCITT 校验 ----
      // frame = [A5, 5A, rest[0..49]]，共 52 字节
      // CRC 覆盖 frame[0..49]（前 50 字节），CRC 值存于 frame[50..51]（小端）
      std::array<uint8_t, FRAME_TOTAL_LEN> frame;
      frame[0] = FRAME_SOF0;
      frame[1] = FRAME_SOF1;
      std::copy(rest.begin(), rest.end(), frame.begin() + 2);

      const uint16_t rx_crc =
        static_cast<uint16_t>(frame[FRAME_TOTAL_LEN - 2]) |
        (static_cast<uint16_t>(frame[FRAME_TOTAL_LEN - 1]) << 8);
      const uint16_t calc_crc =
        crc16::Calc_CRC16_CCITT(frame.data(), FRAME_TOTAL_LEN - 2);

      if (rx_crc != calc_crc) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 200,
          "CRC mismatch: rx=0x%04X calc=0x%04X", rx_crc, calc_crc);
        continue;
      }

      // ---- 解析 payload（frame[4..49]）----
      ReceivePacket packet;
      std::memcpy(&packet, frame.data() + 4, sizeof(ReceivePacket));

      // TODO: 与电控确认 flags bit 定义后启用有效位检查
      // if (!(packet.flags & 0x0001U)) {
      //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 200, "IMU data invalid");
      //   continue;
      // }

      // ---- 填充并发布 Gimbal 消息 ----
      rm_interfaces::msg::Gimbal gimbal_msg;
      gimbal_msg.header.stamp = this->get_clock()->now(); // ROS 生态（TF/odom）用
      gimbal_msg.t_ms = packet.t_ms;                      // MCU 采样戳，供 ESKF 算 dt

      gimbal_msg.angular_velocity.x  = packet.gyro_x;
      gimbal_msg.angular_velocity.y  = packet.gyro_y;
      gimbal_msg.angular_velocity.z  = packet.gyro_z;
      gimbal_msg.linear_acceleration.x = packet.acc_x;
      gimbal_msg.linear_acceleration.y = packet.acc_y;
      gimbal_msg.linear_acceleration.z = packet.acc_z;
      // wheel_velocity 借用 Quaternion 的四个字段：1-x=fl, 2-y=fr, 3-z=rl, 4-w=rr
      gimbal_msg.wheel_velocity.x = packet.w_fl;
      gimbal_msg.wheel_velocity.y = packet.w_fr;
      gimbal_msg.wheel_velocity.z = packet.w_rl;
      gimbal_msg.wheel_velocity.w = packet.w_rr;

      gimbal_pub_->publish(gimbal_msg);

    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}




void RMSerialDriver::sendData(const rm_interfaces::msg::Target::SharedPtr msg)
{
  

  try {
    SendPacket packet;

    packet.test = msg->test;
    
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);


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
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
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
