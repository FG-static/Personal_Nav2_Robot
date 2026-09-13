#include <gtest/gtest.h>

#include "my_tunnel_guidance/vision_capture_handshake.hpp"

TEST(VisionCaptureHandshake, IdleUntilBegin)
{
    my_tunnel_guidance::VisionCaptureHandshake handshake;
    EXPECT_EQ(handshake.command(), 0x00);
    EXPECT_FALSE(handshake.waiting());
    handshake.onStatus(0x02);
    EXPECT_FALSE(handshake.captureFinished());
}

TEST(VisionCaptureHandshake, RequiresCapturingBeforeDone)
{
    my_tunnel_guidance::VisionCaptureHandshake handshake;
    handshake.beginCapture();
    EXPECT_EQ(handshake.command(), 0x01);
    EXPECT_TRUE(handshake.waiting());

    handshake.onStatus(0x02);
    EXPECT_FALSE(handshake.captureFinished());

    handshake.onStatus(0x01);
    EXPECT_TRUE(handshake.sawCapturing());
    EXPECT_FALSE(handshake.captureFinished());

    handshake.onStatus(0x02);
    EXPECT_TRUE(handshake.captureFinished());
}

TEST(VisionCaptureHandshake, IgnoresStaleDoneFromPreviousStation)
{
    my_tunnel_guidance::VisionCaptureHandshake handshake;
    handshake.beginCapture();
    handshake.onStatus(0x01);
    handshake.onStatus(0x02);
    ASSERT_TRUE(handshake.captureFinished());
    handshake.finish();
    EXPECT_EQ(handshake.command(), 0x00);

    handshake.onStatus(0x02);
    handshake.beginCapture();
    EXPECT_FALSE(handshake.captureFinished());
    handshake.onStatus(0x02);
    EXPECT_FALSE(handshake.captureFinished());
    handshake.onStatus(0x01);
    handshake.onStatus(0x02);
    EXPECT_TRUE(handshake.captureFinished());
}

TEST(VisionCaptureHandshake, ResetReturnsToIdle)
{
    my_tunnel_guidance::VisionCaptureHandshake handshake;
    handshake.beginCapture();
    handshake.onStatus(0x01);
    handshake.reset();
    EXPECT_EQ(handshake.command(), 0x00);
    EXPECT_FALSE(handshake.waiting());
    EXPECT_FALSE(handshake.captureFinished());
}
