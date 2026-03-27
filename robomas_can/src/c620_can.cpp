#include "robomas_can/c620_can.hpp"

#include <cstring>

namespace robomas_can {

void C620CAN::receive_data(uint16_t can_id, uint8_t data[8])
{
    uint8_t motor_num = (can_id & 0x0F) - 1;
    if (motor_num >= 8) return;  // ガード

    memcpy(&feedback_[motor_num], data, 8);

    // c620はビックエンディアンなのでリトルエンディアンに変換
    feedback_[motor_num].angle   = __builtin_bswap16(feedback_[motor_num].angle);
    feedback_[motor_num].speed   = __builtin_bswap16(feedback_[motor_num].speed);
    feedback_[motor_num].current = __builtin_bswap16(feedback_[motor_num].current);
}

}  // namespace robomas_can