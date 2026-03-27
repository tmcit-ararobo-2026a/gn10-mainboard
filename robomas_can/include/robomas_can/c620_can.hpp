#include "robomas_can.hpp"
namespace robomas_can {
namespace c620 {
struct c620_feedback {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temp;
    uint8_t reserved;
} __attribute__((__packed__));

#define C620_CURRENT_CONVERSION 819.2f
}  // namespace c620

class C620CAN : public RobomasCAN
{
private:
    c620::c620_feedback feedback_[8];

public:
    // コンストラクタ
    C620CAN(gn10_can::CANBus& bus) : RobomasCAN(bus, C620_CURRENT_CONVERSION, 20.0f) {}

    // 純粋仮想関数の実装
    void receive_data(uint16_t can_id, uint8_t data[8]) override;
};

}  // namespace robomas_can
