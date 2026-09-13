#ifndef MY_TUNNEL_GUIDANCE__VISION_CAPTURE_HANDSHAKE_HPP_
#define MY_TUNNEL_GUIDANCE__VISION_CAPTURE_HANDSHAKE_HPP_

#include <cstdint>

namespace my_tunnel_guidance {

// Bidirectional uint8 stream with the vision capture node.
// Host command: 0x00 idle, 0x01 open camera / capture.
// Vision status: 0x00 idle, 0x01 capturing, 0x02 capture finished.
class VisionCaptureHandshake {

public:

    static constexpr uint8_t kIdle = 0x00;
    static constexpr uint8_t kCapture = 0x01;
    static constexpr uint8_t kDone = 0x02;

    uint8_t command() const { return command_; }
    uint8_t status() const { return status_; }
    bool waiting() const { return waiting_; }
    bool sawCapturing() const { return saw_capturing_; }
    bool captureFinished() const {
        return waiting_ && saw_capturing_ && status_ == kDone;
    }

    void reset() {
        command_ = kIdle;
        status_ = kIdle;
        waiting_ = false;
        saw_capturing_ = false;
    }

    void beginCapture() {
        command_ = kCapture;
        waiting_ = true;
        saw_capturing_ = false;
        // Ignore a stale 0x02 from the previous station until vision
        // reports capturing (0x01) again.
        if (status_ == kDone) {
            status_ = kIdle;
        }
    }

    void onStatus(uint8_t status) {
        status_ = status;
        if (!waiting_) {
            return;
        }
        if (status_ == kCapture) {
            saw_capturing_ = true;
        }
    }

    void finish() {
        command_ = kIdle;
        waiting_ = false;
        saw_capturing_ = false;
    }

private:

    uint8_t command_ = kIdle;
    uint8_t status_ = kIdle;
    bool waiting_ = false;
    bool saw_capturing_ = false;
};

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__VISION_CAPTURE_HANDSHAKE_HPP_
