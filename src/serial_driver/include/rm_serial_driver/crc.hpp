// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <cstddef>
#include <cstdint>

namespace crc16
{
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);

/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

/**
  * @brief CRC16-CCITT, init=0xFFFF, poly=0x1021, MSB-first（非反射）
  *        搬运于 imu_coordinate.md Python 示例算法
  * @param[in] data   : 待校验数据
  * @param[in] len    : 数据长度（字节），不含 CRC 自身
  * @return : 16-bit CRC 值（小端序写入帧尾）
  */
uint16_t Calc_CRC16_CCITT(const uint8_t * data, size_t len);

}  // namespace crc16

#endif  // RM_SERIAL_DRIVER__CRC_HPP_
