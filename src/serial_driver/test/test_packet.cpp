// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rm_serial_driver/packet.hpp"

using rm_serial_driver::FRAME_HEADER;
using rm_serial_driver::FRAME_TAIL;
using rm_serial_driver::ReceiveFrame;
using rm_serial_driver::RX_FRAME_LEN;
using rm_serial_driver::SendFrame;
using rm_serial_driver::toVector;
using rm_serial_driver::TX_FRAME_LEN;

TEST(SerialPacket, ReceiveFrameMatchesMcuTxLayout)
{
  EXPECT_EQ(sizeof(ReceiveFrame), 11U);
  EXPECT_EQ(RX_FRAME_LEN, 11U);
  EXPECT_EQ(offsetof(ReceiveFrame, header), 0U);
  EXPECT_EQ(offsetof(ReceiveFrame, capture_done), 1U);
  EXPECT_EQ(offsetof(ReceiveFrame, vx), 2U);
  EXPECT_EQ(offsetof(ReceiveFrame, wz), 6U);
  EXPECT_EQ(offsetof(ReceiveFrame, tail), 10U);

  ReceiveFrame frame{};
  frame.capture_done = 1;
  frame.vx = 0.5f;
  frame.wz = -0.25f;

  const std::vector<uint8_t> bytes = toVector(frame);
  ASSERT_EQ(bytes.size(), 11U);
  EXPECT_EQ(bytes[0], FRAME_HEADER);
  EXPECT_EQ(bytes[1], 1U);
  EXPECT_EQ(bytes[10], FRAME_TAIL);

  float vx = 0.f;
  float wz = 0.f;
  std::memcpy(&vx, bytes.data() + 2, sizeof(float));
  std::memcpy(&wz, bytes.data() + 6, sizeof(float));
  EXPECT_FLOAT_EQ(vx, 0.5f);
  EXPECT_FLOAT_EQ(wz, -0.25f);
}

TEST(SerialPacket, SendFrameMatchesMcuRxLayout)
{
  EXPECT_EQ(sizeof(SendFrame), 11U);
  EXPECT_EQ(TX_FRAME_LEN, 11U);
  EXPECT_EQ(offsetof(SendFrame, header), 0U);
  EXPECT_EQ(offsetof(SendFrame, vx), 1U);
  EXPECT_EQ(offsetof(SendFrame, wz), 5U);
  EXPECT_EQ(offsetof(SendFrame, capture_enable), 9U);
  EXPECT_EQ(offsetof(SendFrame, tail), 10U);

  SendFrame frame{};
  frame.vx = 1.25f;
  frame.wz = 0.5f;
  frame.capture_enable = 1;

  const std::vector<uint8_t> bytes = toVector(frame);
  ASSERT_EQ(bytes.size(), 11U);
  EXPECT_EQ(bytes[0], FRAME_HEADER);
  EXPECT_EQ(bytes[9], 1U);
  EXPECT_EQ(bytes[10], FRAME_TAIL);

  float vx = 0.f;
  float wz = 0.f;
  std::memcpy(&vx, bytes.data() + 1, sizeof(float));
  std::memcpy(&wz, bytes.data() + 5, sizeof(float));
  EXPECT_FLOAT_EQ(vx, 1.25f);
  EXPECT_FLOAT_EQ(wz, 0.5f);
}

TEST(SerialPacket, ScaleWzForMcu)
{
  using rm_serial_driver::scaleWzForMcu;

  EXPECT_FLOAT_EQ(scaleWzForMcu(0.f), 0.f);
  EXPECT_FLOAT_EQ(scaleWzForMcu(0.3f), 11.1f);
  EXPECT_FLOAT_EQ(scaleWzForMcu(0.1f), 3.9f);
  EXPECT_FLOAT_EQ(scaleWzForMcu(0.2f), 7.4f);
  EXPECT_FLOAT_EQ(scaleWzForMcu(-0.1f), -3.9f);
  EXPECT_FLOAT_EQ(scaleWzForMcu(-0.5f), -18.5f);
}
