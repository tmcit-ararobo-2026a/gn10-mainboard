#include "gn10_mainboard/conversion_command.hpp"

#include <algorithm>

ConversionCommand::ConversionCommand(int8_t stick_max_value_high, int8_t stick_max_value_low)
    : stick_max_value_high_(stick_max_value_high), stick_max_value_low_(stick_max_value_low)
{
}

/* belt */
void ConversionCommand::set_belt_change_value(const uint8_t change_value)
{
    belt_change_value_ = change_value;
}

void ConversionCommand::set_belt_change_value_deep(const uint8_t change_value_d)
{
    belt_change_value_d_ = change_value_d;
}

void ConversionCommand::set_init_belt_vel(const uint8_t belt_init_vel)
{
    belt_vel_      = belt_init_vel;
    belt_init_vel_ = belt_init_vel;
}

/* bucket */
void ConversionCommand::set_bucket_move_value(const uint8_t bucket_move_value)
{
    bucket_high_value_ = bucket_move_value;
}

void ConversionCommand::set_bucket_limit_value(
    const int16_t bucket_limit_high, const int16_t bucket_limit_low
)
{
    bucket_limit_h_ = bucket_limit_high;
    bucket_limit_l_ = bucket_limit_low;
}

/* desk */
void ConversionCommand::set_desk_move_value(const uint8_t desk_distance)
{
    desk_move_value_ = desk_distance;
}

void ConversionCommand::set_desk_limit_value(
    const int16_t desk_limit_high, const int16_t desk_limit_low
)
{
    desk_limit_h_ = desk_limit_high;
    desk_limit_l_ = desk_limit_low;
}

/* wheel */
void ConversionCommand::set_wheel_max_vel(const float max_wheel_vel)
{
    max_wheel_vel_ = max_wheel_vel;
}

void ConversionCommand::set_max_angular_vel(const float max_angular_vel)
{
    max_angular_vel_ = max_angular_vel;
}

/* wheel */
robot_config::command_t ConversionCommand::conversion(robot_config::teleop_t& teleop)
{
    // 足回り
    int8_t x_vel;
    int8_t y_vel;
    int8_t angular_vel;

    // x velocity

    if (teleop.analog.stick_left[0] > 0) {
        x_vel = static_cast<int8_t>(
            static_cast<float>(teleop.analog.stick_left[0]) / stick_max_value_high_ * max_wheel_vel_
        );
    } else if (teleop.analog.stick_left[0] < 0) {
        x_vel = static_cast<int8_t>(
            static_cast<float>(teleop.analog.stick_left[0]) / stick_max_value_low_ * max_wheel_vel_
        );
    } else {
        x_vel = 0;
    }

    // y velocity
    if (teleop.analog.stick_left[1] > 0) {
        y_vel = static_cast<int8_t>(
            static_cast<float>(teleop.analog.stick_left[1]) / stick_max_value_high_ * max_wheel_vel_
        );
    } else if (teleop.analog.stick_left[1] < 0) {
        y_vel = static_cast<int8_t>(
            static_cast<float>(teleop.analog.stick_left[1]) / stick_max_value_low_ * max_wheel_vel_
        );
    } else {
        y_vel = 0;
    }

    // angular velocity
    if (teleop.analog.stick_right[0] > 0) {
        angular_vel = static_cast<int8_t>(
            static_cast<float>(teleop.analog.stick_right[0]) / stick_max_value_high_ *
            max_angular_vel_
        );
    } else if (teleop.analog.stick_right[0] < 0) {
        angular_vel = static_cast<int8_t>(
            static_cast<float>(teleop.analog.stick_right[0]) / stick_max_value_low_ *
            max_angular_vel_
        );
    } else {
        angular_vel = 0;
    }

    command_.x_vel       = x_vel;
    command_.y_vel       = y_vel;
    command_.angular_vel = angular_vel;

    /*バケツ回収*/
    // 昇降機構
    if (teleop.buttons.up) {
        bucket_arm_hight_ += bucket_high_value_;
    } else if (teleop.buttons.down) {
        bucket_arm_hight_ -= bucket_high_value_;
    } else {
        bucket_arm_hight_ += 0;
    }
    bucket_arm_hight_         = std::clamp(bucket_arm_hight_, bucket_limit_l_, bucket_limit_h_);
    command_.bucket_arm_hight = bucket_arm_hight_;

    // ハンド機構
    if (teleop.buttons.circle && !teleop_last_.buttons.circle) {
        command_.bucket_arm_hold = !command_.bucket_arm_hold;
    }

    /* 机上回収 */
    // 引き入れ
    if (teleop.buttons.left && !teleop_last_.buttons.left) {
        desk_flag = !desk_flag;
    }

    if (desk_flag) {
        desk_pos_ += desk_move_value_;
    } else {
        desk_pos_ -= desk_move_value_;
    }
    desk_pos_             = std::clamp(desk_pos_, desk_limit_l_, desk_limit_h_);
    command_.desk_arm_pos = desk_pos_;

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
