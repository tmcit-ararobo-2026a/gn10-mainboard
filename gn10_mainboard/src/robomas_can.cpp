#include "gn10_mainboard/robomas_can.hpp"

#include <cstring>

namespace robomaster {

// コンストラクタの引数にbusを設定することによって、他のバスとかぶらないようにする。
RobomasCAN::RobomasCAN(gn10_can::CANBus& bus) : bus_(bus) {}

void RobomasCAN::send_data(uint16_t can_id, uint8_t* data)
{
    frame.id  = can_id;
    frame.dlc = 8;  // robomasは8byte固定
    for (int i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    bus_.send_frame(frame);
}

void RobomasCAN::receive_data(uint16_t can_id, uint8_t data[8])
{
    uint8_t motor_num;
    motor_num = (can_id & 0x0F) - 1;
    memcpy(&feedback[motor_num], data, 8);
    process_data(&feedback[motor_num]);
}

void RobomasCAN::process_data(MotorFeedback* feedback)
{
    // C620はビックエンディアンなのでリトルエンディアンに変換
    // ここは変えないで！！
    feedback->angle   = __builtin_bswap16(feedback->angle);
    feedback->speed   = __builtin_bswap16(feedback->speed);
    feedback->current = __builtin_bswap16(feedback->current);
    // この下から処理
}

void RobomasCAN::send_current(
    uint8_t group_num,
    int16_t motor1_current,
    int16_t motor2_current,
    int16_t motor3_current,
    int16_t motor4_current
)
{
    // 型変換
    motor1_current = (static_cast<int16_t>(motor1_current * 819.2));
    motor2_current = (static_cast<int16_t>(motor2_current * 819.2));
    motor3_current = (static_cast<int16_t>(motor3_current * 819.2));
    motor4_current = (static_cast<int16_t>(motor4_current * 819.2));

    // stmはリトルエンディアンなのでビッグエンディアンに変換
    if (group_num < 2) {
        motor_current_[group_num].current[0] = __builtin_bswap16(motor1_current);
        motor_current_[group_num].current[1] = __builtin_bswap16(motor2_current);
        motor_current_[group_num].current[2] = __builtin_bswap16(motor3_current);
        motor_current_[group_num].current[3] = __builtin_bswap16(motor4_current);

        // packetしたデータを送信
        if (group_num == 0) {
            send_data(SEND_CANID_1_4, reinterpret_cast<uint8_t*>(&motor_current_[0]));
        } else {
            send_data(SEND_CANID_5_8, reinterpret_cast<uint8_t*>(&motor_current_[1]));
        }
    }
}
}  // namespace robomaster