// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
// InterfaceNotice.md：0xAA + payload + 0x55，无 len / seq / CRC。小端、1 字节对齐。
// 线上布尔量为 uint8_t：0=false，1=true。
//
// 与电控 packed 结构对应（上位机视角）：
//   电控 TX / 上位机 RX：帧头 + Capture_Done + Chassis_Vx + Chassis_Wz + 帧尾
//   电控 RX / 上位机 TX：帧头 + Chassis_Vx + Chassis_Wz + Capture_Enable + 帧尾
static constexpr uint8_t FRAME_HEADER = 0xAA;
static constexpr uint8_t FRAME_TAIL = 0x55;

inline uint8_t boolToU8(bool value)
{
  return value ? 1U : 0U;
}

#pragma pack(push, 1)
// 电控 → 上位机（电控 Serial_TX_Frame_t）
struct ReceiveFrame
{
  uint8_t header = FRAME_HEADER;
  uint8_t capture_done = 0;  // 0/1
  float vx = 0.f;            // m/s，电控上报的实际底盘线速度
  float wz = 0.f;            // rad/s，电控上报的实际底盘角速度
  uint8_t tail = FRAME_TAIL;
};

// 上位机 → 电控（电控 Serial_RX_Frame_t）
struct SendFrame
{
  uint8_t header = FRAME_HEADER;
  float vx = 0.f;              // m/s
  float wz = 0.f;              // rad/s
  uint8_t capture_enable = 0;  // 0/1，到位停留期间为 1
  uint8_t tail = FRAME_TAIL;
};
#pragma pack(pop)

static constexpr size_t RX_FRAME_LEN = 11;
static constexpr size_t TX_FRAME_LEN = 11;
static_assert(sizeof(ReceiveFrame) == RX_FRAME_LEN, "RX frame size mismatch with protocol");
static_assert(sizeof(SendFrame) == TX_FRAME_LEN, "TX frame size mismatch with protocol");

template<typename FrameT>
inline std::vector<uint8_t> toVector(const FrameT & data)
{
  std::vector<uint8_t> packet(sizeof(FrameT));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(FrameT), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
