#include "gn10_mainboard/conversion_command.hpp"

#include <algorithm>

ConversionCommand::ConversionCommand(
    uint8_t bucket_limit_h, uint8_t bucket_limit_l, uint8_t desk_limit_h, uint8_t desk_limit_l
)
    : bucket_limit_h_(bucket_limit_h),
      bucket_limit_l_(bucket_limit_l),
      desk_limit_h_(desk_limit_h),
      desk_limit_l_(desk_limit_l)
{
}

void ConversionCommand::set_init_belt_vel(const uint8_t belt_init_vel)
{
    belt_vel_      = belt_init_vel;
    belt_init_vel_ = belt_init_vel;
}

void ConversionCommand::set_bucket_move_value(const uint8_t bucket_move_value)
{
    bucket_move_value_ = bucket_move_value;
}

void ConversionCommand::set_desk_move_value(const uint8_t desk_distance)
{
    desk_move_value_ = desk_distance;
}

void ConversionCommand::set_belt_change_value(const uint8_t change_value)
{
    belt_change_value_ = change_value;
}

void ConversionCommand::set_belt_change_value_deep(const uint8_t change_value_d)
{
    belt_change_value_d_ = change_value_d;
}

robot_config::command_t ConversionCommand::conversion(robot_config::teleop_t& teleop)
{
    // 足回り

    command_.x_vel       = teleop.analog.stick_left[0];
    command_.y_vel       = teleop.analog.stick_left[1];
    command_.angular_vel = teleop.analog.stick_right[0];

    /*バケツ回収*/
    // 昇降機構
    if (teleop.buttons.up) {
        bucket_arm_hight_ += bucket_move_value_;
    } else if (teleop.buttons.down) {
        bucket_arm_hight_ -= bucket_move_value_;
    }
    bucket_arm_hight_         = std::clamp(bucket_arm_hight_, bucket_limit_l_, bucket_limit_h_);
    command_.bucket_arm_hight = bucket_arm_hight_;

    // ハンド機構
    if (teleop.buttons.circle && !teleop_last_.buttons.circle) {
        command_.bucket_arm_hold = !command_.bucket_arm_hold;
    }

    /* 机上回収 */
    // 引き入れ toggleni変更
    if (teleop.buttons.left && !teleop_last_.buttons.left) {
        command_.desk_arm_pos = !command_.desk_arm_pos;
    }

    // アーム曲げ
    if (teleop.buttons.cross && !teleop_last_.buttons.cross) {
        command_.desk_arm_hold = !command_.desk_arm_hold;
    }

    /*装填機構*/
    command_.loading_shift_cloth = teleop.buttons.right;

    /*ベルト直動*/
    switch (teleop.buttons.lever_right) {
        case robot_config::LeverPosition::FRONT:
            break;

        case robot_config::LeverPosition::RIGHT:
            belt_vel_ -= belt_change_value_;
            break;

        case robot_config::LeverPosition::RIGHT_DEEP:
            belt_vel_ -= belt_change_value_d_;
            break;

        case robot_config::LeverPosition::LEFT:
            belt_vel_ += belt_change_value_;
            break;

        case robot_config::LeverPosition::LEFT_DEEP:
            belt_vel_ += belt_change_value_d_;
            break;

        case robot_config::LeverPosition::PUSH:
            belt_vel_ = belt_init_vel_;
            break;
        default:
            break;
    }

    // commandにbelt_velを代入
    command_.belt_vel = belt_vel_;

    // belt_init処理
    if (teleop.buttons.stick_push_left && teleop.buttons.stick_push_right) {
        command_.belt_init = true;
    } else {
        command_.belt_init = false;
    }

    // belt_throw
    command_.belt_throw = teleop.buttons.triangle;

    // エアシリンダー
    switch (teleop.buttons.lever_left) {
        case robot_config::LeverPosition::RIGHT:
        case robot_config::LeverPosition::RIGHT_DEEP:
            if (air_rauncher_selector_ == 2) {
                air_rauncher_selector_ = 0;
            } else {
                air_rauncher_selector_++;
            }
            break;

        case robot_config::LeverPosition::LEFT:
        case robot_config::LeverPosition::LEFT_DEEP:
            if (air_rauncher_selector_ == 0) {
                air_rauncher_selector_ = 2;
            } else {
                air_rauncher_selector_--;
            }
            break;

        case robot_config::LeverPosition::PUSH:
            if (air_rauncher_selector_ == 0) {
                command_.air_rauncher_for_desk_l = true;
                command_.air_rauncher_for_desk_r = false;
                command_.air_rauncher_for_flag   = false;

            } else if (air_rauncher_selector_ == 1) {
                command_.air_rauncher_for_desk_l = false;
                command_.air_rauncher_for_desk_r = false;
                command_.air_rauncher_for_flag   = true;

            } else if (air_rauncher_selector_ == 2) {
                command_.air_rauncher_for_desk_l = false;
                command_.air_rauncher_for_desk_r = true;
                command_.air_rauncher_for_flag   = false;
            }
            break;

        default:
            break;
    }

    teleop_last_ = teleop;
    return command_;
}
