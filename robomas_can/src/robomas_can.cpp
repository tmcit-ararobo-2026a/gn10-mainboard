#include "robomas_can/robomas_can.hpp"

namespace robomas_can {
RobomasCAN::RobomasCAN(gn10_can::CANBus& bus) : bus_(bus) {}

void RobomasCAN::set_current(uint8_t motor_number, int16_t current_value)
{
    if (motor_number >= 1 && motor_number <= 8) {
        motor_number -= 1;
        uint8_t group                        = motor_number / 4;
        uint8_t index                        = motor_number % 4;
        motor_current_[group].current[index] = current_value;
    }
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

void RobomasCAN::send_data(uint16_t can_id) {}
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