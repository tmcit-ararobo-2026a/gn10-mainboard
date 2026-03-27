#include "robomas_can/robomas_can.hpp"

namespace robomas_can {
RobomasCAN::RobomasCAN(gn10_can::CANBus& bus) : bus_(bus) {}

void RobomasCAN::set_current(uint8_t motor_number, float current_value)
{
    /*    if (motor_number >= 1 && motor_number <= 8 && current_value >= -20 && current_value <= 20)
    {
        // 電流変換
        current_value = current_value * AMPS_CONVERSION;
        motor_number -= 1;
        uint8_t group                        = motor_number / 4;
        uint8_t index                        = motor_number % 4;
        motor_current_[group].current[index] = __builtin_bswap16(current_value);
    }*/
}

int16_t RobomasCAN::get_current(uint8_t motor_number) const
{
    if (motor_number < 1 || motor_number > 8) {
        return 0;
    }
    motor_number -= 1;
    uint8_t group = motor_number / 4;
    uint8_t index = motor_number % 4;
    return motor_current_[group].current[index];
}

void RobomasCAN::receive_data(uint16_t can_id, uint8_t data[8]) {}

void RobomasCAN::send_data(uint16_t can_id, uint8_t* data)
{
    frame_.id  = can_id;
    frame_.dlc = 8;  // robomasは8byte固定
    for (int i = 0; i < 8; i++) {
        frame_.data[i] = data[i];
    }
    bus_.send_frame(frame_);
}
}  // namespace robomas_can

/**
 *
 *     if (motor_number < 1 && motor_number > 8) {
        motor_number -= 1;
        uint8_t group = motor_number / 4;
        uint8_t index = motor_number % 4;
        return motor_current_[group].current[index];
    }
    return 0;
 */