#include "gn10_mainboard/conversion_command.hpp"

ConversionCommand::ConversionCommand(
    uint8_t bucket_limit_h, uint8_t bucket_limit_l, uint8_t desk_limit_h, uint8_t desk_limit_l
)
    : bucket_limit_h_(bucket_limit_h),
      bucket_limit_l_(bucket_limit_l),
      desk_limit_h_(desk_limit_h),
      desk_limit_l_(desk_limit_l)
{
}

void ConversionCommand::set_belt_vel(const uint8_t belt_vel)
{
    belt_vel_ = belt_vel;
}

void ConversionCommand::set_bucket_distance_value(const uint8_t bucket_distance)
{
    bucket_distance_ = bucket_distance;
}

void ConversionCommand::set_desk_distance_value(const uint8_t desk_distance)
{
    desk_distance_ = desk_distance;
}

robot_config::command_t ConversionCommand::conversion(robot_config::teleop_t& teleop)
{
    // 足回り
    command_.x_vel       = teleop.analog.stick_left[0];
    command_.y_vel       = teleop.analog.stick_left[1];
    command_.angular_vel = teleop.analog.stick_right[0];

    /*バケツ回収 limitは受信側でかけること！*/
    if (teleop.buttons.up) {
        command_.bucket_arm_hight += 10;
    } else if (teleop.buttons.down) {
        command_.bucket_arm_hight -= 10;
    }
    if (teleop.buttons.right) {
        command_.bucket_arm_hold += 10;
    } else if (teleop.buttons.left) {
        command_.bucket_arm_hold -= 10;
    }

    /* 机上回収  limitは受信側でかけること!*/
    // 引き入れ
    if (teleop.buttons.triangle) {
        command_.desk_arm_pos += 10;
    } else if (teleop.buttons.cross) {
        command_.desk_arm_pos -= 10;
    }

    // アーム曲げ
    if (teleop.buttons.circle && teleop_last_.buttons.circle) {
        command_.desk_arm_hold = !command_.desk_arm_hold;
    }

    /*装填機構*/
    command_.loading_shift_cloth = teleop.buttons.cross;

    /*ベルト直動*/
    switch (teleop.buttons.lever_right) {
        case robot_config::LeverPosition::FRONT:
            break;

        case robot_config::LeverPosition::RIGHT:
            command_.belt_vel -= 0.05;
            command_.belt_init = false;
            break;

        case robot_config::LeverPosition::RIGHT_DEEP:
            command_.belt_vel -= 0.1;
            command_.belt_init = false;
            break;

        case robot_config::LeverPosition::LEFT:
            command_.belt_vel += 0.05;
            command_.belt_init = false;
            break;

        case robot_config::LeverPosition::LEFT_DEEP:
            command_.belt_vel += 0.1;
            command_.belt_init = false;
            break;

        case robot_config::LeverPosition::PUSH:
            command_.belt_throw = true;
            break;
        default:
            command_.belt_throw = false;
            break;
    }

    // init処理
    if (teleop.buttons.stick_push_left && teleop.buttons.stick_push_right) {
        command_.belt_init = true;
    } else {
        command_.belt_init = false;
    }

    // エアシリンダー
    switch (teleop.buttons.lever_left) {
        case robot_config::LeverPosition::RIGHT:
        case robot_config::LeverPosition::RIGHT_DEEP:
            if (count == 2) {
                count = 0;
            } else {
                count++;
            }
            break;

        case robot_config::LeverPosition::LEFT:
        case robot_config::LeverPosition::LEFT_DEEP:
            if (count == 0) {
                count = 2;
            } else {
                count--;
            }
            break;

        case robot_config::LeverPosition::PUSH:
            if (!count) {
                command_.air_rauncher_for_desk_l = true;
                command_.air_rauncher_for_desk_r = false;
                command_.air_rauncher_for_flag   = false;
            } else if (count == 2) {
                command_.air_rauncher_for_desk_l = false;
                command_.air_rauncher_for_desk_r = true;
                command_.air_rauncher_for_flag   = false;
            } else {
                command_.air_rauncher_for_desk_l = false;
                command_.air_rauncher_for_desk_r = false;
                command_.air_rauncher_for_flag   = true;
            }
            break;

        default:
            break;
    }

    teleop_last_ = teleop;
    return command_;
}
