#include "my_tunnel_guidance/pointcloud_timing.hpp"

#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace my_tunnel_guidance {
namespace {

constexpr std::uint32_t MAX_SUPPORTED_POINT_OFFSET_NS = 200000000U;

std::uint32_t byteSwap(std::uint32_t value) {
    return ((value & 0x000000FFU) << 24U) |
           ((value & 0x0000FF00U) << 8U) |
           ((value & 0x00FF0000U) >> 8U) |
           ((value & 0xFF000000U) >> 24U);
}

bool hostIsBigEndian() {
    const std::uint16_t value = 0x0102U;
    std::uint8_t first_byte = 0U;
    std::memcpy(&first_byte, &value, sizeof(first_byte));
    return first_byte == 0x01U;
}

}  // namespace

/**
 * @brief 计算点云时间偏移的最大值
 * @param cloud 点云数据
 * @return 时间偏移的最大值（纳秒）
 */
std::uint32_t maxPointTimeOffsetNanoseconds(
    const sensor_msgs::msg::PointCloud2 & cloud
) {

    const auto field = std::find_if(
        cloud.fields.begin(), cloud.fields.end(),
        [](const sensor_msgs::msg::PointField & candidate) {
            return candidate.name == "t";
        });
    if (field == cloud.fields.end() ||
        field->datatype != sensor_msgs::msg::PointField::UINT32 ||
        field->count == 0U ||
        cloud.point_step < sizeof(std::uint32_t) ||
        field->offset > cloud.point_step - sizeof(std::uint32_t))
        return 0U;

    const std::size_t point_count =
        static_cast<std::size_t>(cloud.width) * cloud.height;
    if (point_count == 0U || cloud.row_step == 0U) {

        return 0U;
    }

    std::uint32_t max_offset_ns = 0U;
    const bool swap_bytes = cloud.is_bigendian != hostIsBigEndian();
    for (std::uint32_t row = 0U; row < cloud.height; ++ row) {

        const std::size_t row_offset =
            static_cast<std::size_t>(row) * cloud.row_step;
        for (std::uint32_t column = 0U; column < cloud.width; ++ column) {

            const std::size_t data_offset =
                row_offset + static_cast<std::size_t>(column) * cloud.point_step +
                field->offset;
            if (data_offset + sizeof(std::uint32_t) > cloud.data.size()) {

                return max_offset_ns;
            }

            std::uint32_t point_offset_ns = 0U;
            std::memcpy(
                &point_offset_ns, cloud.data.data() + data_offset,
                sizeof(point_offset_ns));
            if (swap_bytes) {

                point_offset_ns = byteSwap(point_offset_ns);
            }
            if (point_offset_ns <= MAX_SUPPORTED_POINT_OFFSET_NS) {

                max_offset_ns = std::max(max_offset_ns, point_offset_ns);
            }
        }
    }
    return max_offset_ns;
}

rclcpp::Time pointCloudEndStamp(
    const sensor_msgs::msg::PointCloud2 & cloud
) {

    const std::uint32_t offset_ns = maxPointTimeOffsetNanoseconds(cloud);
    return rclcpp::Time(cloud.header.stamp) +
           rclcpp::Duration::from_nanoseconds(offset_ns);
}

}  // namespace my_tunnel_guidance
