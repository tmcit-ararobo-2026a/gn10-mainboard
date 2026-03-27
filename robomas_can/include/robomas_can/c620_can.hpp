#include "robomas_can.hpp"
namespace robomas_can {
namespace c620 {

// feedback data
struct c620_feedback {
    uint16_t angle;    // 値域:0~8191
    int16_t speed;     // 値域：-32768 ~ 32767 単位：rpm
    int16_t current;   // 値域：-16384~16384
    uint8_t temp;      // 値域：0~255 単位：℃
    uint8_t reserved;  // 空きデータ
} __attribute__((__packed__));

// c620電流変換用定数
#define C620_CURRENT_CONVERSION 819.2f
}  // namespace c620

class C620CAN : public RobomasCAN
{
private:
    c620::c620_feedback feedback_[8];

public:
    // コンストラクタ
    C620CAN(gn10_can::CANBus& bus) : RobomasCAN(bus, C620_CURRENT_CONVERSION) {}

    // 純粋仮想関数の実装
    void receive_data(uint16_t can_id, uint8_t data[8]) override;
};

}  // namespace robomas_can
