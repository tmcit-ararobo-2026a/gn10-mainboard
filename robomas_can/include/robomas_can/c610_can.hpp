#include "robomas_can.hpp"

namespace robomas_can {

namespace c610 {

// feedback data
struct c610_feedback {
    uint16_t angle;     // 値域：0~8191　
    int16_t speed;      // 値域 -10000 ~ 10000　単位：rpm
    int16_t current;    // 値域 -10000 ~ 10000
    uint16_t reserved;  // 空きデータ
};

// c610電流変換定数
#define C610_CURRENT_CONVERSION 100.0f
}  // namespace c610

class C610CAN : public RobomasCAN
{
private:
    // feedbackのデータ格納用配列
    c610::c610_feedback feedback_[8];

public:
    // コンストラクタ
    C610CAN(gn10_can::CANBus& bus) : RobomasCAN(bus, C610_CURRENT_CONVERSION) {}

    // 純粋仮想関数の実装
    void receive_data(uint16_t can_id, uint8_t data[8]) override;
};

}  // namespace robomas_can
