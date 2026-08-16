#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cstdint>

#include "my_tunnel_guidance/pointcloud_timing.hpp"

namespace {

sensor_msgs::msg::PointCloud2 makeTimedCloud() {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp.sec = 12;
    cloud.header.stamp.nanosec = 345000000U;
    cloud.height = 1U;
    cloud.width = 3U;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "t", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(3U);

    sensor_msgs::PointCloud2Iterator<std::uint32_t> time_iterator(cloud, "t");
    *time_iterator = 0U;
    ++time_iterator;
    *time_iterator = 50000000U;
    ++time_iterator;
    *time_iterator = 100000000U;
    return cloud;
}

}  // namespace

TEST(PointCloudTiming, UsesMaximumUint32NanosecondOffset) {
    const sensor_msgs::msg::PointCloud2 cloud = makeTimedCloud();

    EXPECT_EQ(
        my_tunnel_guidance::maxPointTimeOffsetNanoseconds(cloud),
        100000000U);
    EXPECT_EQ(
        my_tunnel_guidance::pointCloudEndStamp(cloud).nanoseconds(),
        12445000000LL);
}

TEST(PointCloudTiming, FallsBackToHeaderWhenTimeFieldIsMissing) {
    sensor_msgs::msg::PointCloud2 cloud = makeTimedCloud();
    cloud.fields.pop_back();

    EXPECT_EQ(
        my_tunnel_guidance::maxPointTimeOffsetNanoseconds(cloud), 0U);
    EXPECT_EQ(
        my_tunnel_guidance::pointCloudEndStamp(cloud).nanoseconds(),
        12345000000LL);
}

TEST(PointCloudTiming, IgnoresOffsetsUnsupportedByBievr) {
    sensor_msgs::msg::PointCloud2 cloud = makeTimedCloud();
    sensor_msgs::PointCloud2Iterator<std::uint32_t> time_iterator(cloud, "t");
    *time_iterator = 250000000U;
    ++time_iterator;
    *time_iterator = 300000000U;
    ++time_iterator;
    *time_iterator = 400000000U;

    EXPECT_EQ(
        my_tunnel_guidance::maxPointTimeOffsetNanoseconds(cloud), 0U);
}
