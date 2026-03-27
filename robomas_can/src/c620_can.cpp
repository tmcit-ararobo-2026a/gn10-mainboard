#include "robomas_can/c620_can.hpp"

#include <cstring>

namespace robomas_can {

void C620CAN::receive_data(uint16_t can_id, uint8_t data[8])
{
    uint8_t motor_num = (can_id & 0x0F) - 1;
    memcpy(&feedback_[motor_num], data, 8);

    // c610はビックエンディアンなのでリトルエンディアンに変換
    feedback_->angle   = __builtin_bswap16(feedback_->angle);
    feedback_->speed   = __builtin_bswap16(feedback_->speed);
    feedback_->current = __builtin_bswap16(feedback_->current);
    feedback_->temp    = __builtin_bswap16(feedback_->temp);
}

}  // namespace robomas_can