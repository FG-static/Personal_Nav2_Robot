// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <vector>

namespace rm_serial_driver
{
// InterfaceNotice.md：0xAA + payload + 0x55，无 len / seq / CRC。小端、1 字节对齐。
// 线上布尔量为 uint8_t：0=false，1=true。
static constexpr uint8_t FRAME_HEADER = 0xAA;
static constexpr uint8_t FRAME_TAIL = 0x55;

inline uint8_t boolToU8(bool value)
{
  return value ? 1U : 0U;
}

#pragma pack(push, 1)
// 电控 → 上位机
struct ReceiveFrame
{
  uint8_t header = FRAME_HEADER;
  uint32_t temp = 0;
  uint8_t capture_done = 0;  // 0/1
  uint8_t tail = FRAME_TAIL;
};

// 上位机 → 电控
struct SendFrame
{
  uint8_t header = FRAME_HEADER;
  float vx = 0.f;              // m/s
  float wz = 0.f;              // rad/s
  uint8_t capture_enable = 0;  // 0/1，到位停留期间为 1
  uint8_t tail = FRAME_TAIL;
};
#pragma pack(pop)

static constexpr size_t RX_FRAME_LEN = 7;
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
