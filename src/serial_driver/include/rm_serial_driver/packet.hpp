// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <cstdint>
#include <cstddef>
#include <vector>

namespace rm_serial_driver
{
    // 帧格式（imu_coordinate.md）:
    //   SOF0(1) + SOF1(1) + seq(1) + len(1) + payload(46) + CRC16-CCITT(2) = 52 bytes
    static constexpr uint8_t  FRAME_SOF0        = 0xA5;
    static constexpr uint8_t  FRAME_SOF1        = 0x5A;
    static constexpr uint8_t  FRAME_PAYLOAD_LEN = 46;
    static constexpr size_t   FRAME_TOTAL_LEN   = 52;  // 2+1+1+46+2

    // payload 内存布局：u32 t_ms + 10×f32 + u16 flags = 4+40+2 = 46 bytes
    struct ReceivePacket
    {
        uint32_t t_ms;     // MCU 时间戳 ms

        float w_fl;        // 前左轮角速度 rad/s (ESC ID 2)
        float w_fr;        // 前右轮角速度 rad/s (ESC ID 1)
        float w_rl;        // 后左轮角速度 rad/s (ESC ID 3)
        float w_rr;        // 后右轮角速度 rad/s (ESC ID 4)

        float gyro_x;      // 底盘车体系角速度 rad/s
        float gyro_y;
        float gyro_z;

        float acc_x;       // 底盘车体系加速度 m/s^2
        float acc_y;
        float acc_z;

        uint16_t flags;    // 有效性/故障状态位（与电控确认 bit 定义）
    } __attribute__((packed));

    static_assert(sizeof(ReceivePacket) == 46, "ReceivePacket size mismatch with protocol");

    struct SendPacket
    {
        uint8_t header = 0xFF;
        uint8_t test;      // 0～255，通信测试
        uint8_t checksum = 0xFE;
    } __attribute__((packed));

    inline std::vector<uint8_t> toVector(const SendPacket &data)
    {
        std::vector<uint8_t> packet(sizeof(SendPacket));
        std::copy(
            reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
        return packet;
    }

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
