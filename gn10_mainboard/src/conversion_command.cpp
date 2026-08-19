#include "gn10_mainboard/conversion_command.hpp"

#include <algorithm>
#include <cstdint>

ConversionCommand::ConversionCommand() {}

/* belt */
void ConversionCommand::set_belt_vel_init(float belt_vel_init)
{
    belt_vel_      = belt_vel_init;
    belt_vel_init_ = belt_vel_init;
}

void ConversionCommand::set_belt_vel_adjust_value(float belt_vel_adjust_value)
{
    belt_vel_adjust_value_ = belt_vel_adjust_value;
}

void ConversionCommand::set_lever_degree_ofattenuation(float lever_degree_ofattenuation)
{
    lever_degree_ofattenuation_ = lever_degree_ofattenuation;
}

/* bucket */
void ConversionCommand::set_bucket_hight_value(uint8_t bucket_hight_value)
{
    bucket_hight_value_ = bucket_hight_value;
}

void ConversionCommand::set_bucket_limit_value(int16_t bucket_limit_high, int16_t bucket_limit_low)
{
    bucket_limit_high_ = bucket_limit_high;
    bucket_limit_low_  = bucket_limit_low;
}

/* desk */
void ConversionCommand::set_desk_move_value(uint8_t desk_distance)
{
    desk_draw_in_value_ = desk_distance;
}

void ConversionCommand::set_desk_limit_value(int16_t desk_limit_high, int16_t desk_limit_low)
{
    desk_limit_high_ = desk_limit_high;
    desk_limit_low_  = desk_limit_low;
}

/* wheel */
void ConversionCommand::set_wheel_max_vel(float wheel_max_vel)
{
    wheel_max_vel_ = wheel_max_vel;
}

void ConversionCommand::set_angular_max_vel(float angular_max_vel)
{
    angular_max_vel_ = angular_max_vel;
}

/* wheel */
robot_config::command_t ConversionCommand::conversion(robot_config::teleop_t& teleop)
{
    // 足回り
    float x_vel;
    float y_vel;
    float angular_vel;

    // x velocity
    x_vel = std::clamp(static_cast<float>(teleop.analog.stick_left[0]) / INT8_MAX, -1.0f, 1.0f) *
            wheel_max_vel_;
    // y velocity
    y_vel = std::clamp(static_cast<float>(teleop.analog.stick_left[1]) / INT8_MAX, -1.0f, 1.0f) *
            wheel_max_vel_;

    // angular velocity
    angular_vel =
        std::clamp(static_cast<float>(teleop.analog.stick_right[0]) / INT8_MAX, -1.0f, 1.0f) *
        angular_max_vel_;

    command_.x_vel       = x_vel;
    command_.y_vel       = y_vel;
    command_.angular_vel = angular_vel;

    /*バケツ回収*/
    // 昇降機構
    if (teleop.buttons.up) {
        bucket_arm_hight_ += bucket_hight_value_;
    } else if (teleop.buttons.down) {
        bucket_arm_hight_ -= bucket_hight_value_;
    } else {
        bucket_arm_hight_ += 0;
    }
    bucket_arm_hight_ = std::clamp(bucket_arm_hight_, bucket_limit_low_, bucket_limit_high_);
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
        desk_pos_ += desk_draw_in_value_;
    } else {
        desk_pos_ -= desk_draw_in_value_;
    }
    desk_pos_             = std::clamp(desk_pos_, desk_limit_low_, desk_limit_high_);
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
            belt_vel_ -= belt_vel_adjust_value_ * lever_degree_ofattenuation_;
            break;

        case robot_config::LeverPosition::RIGHT_DEEP:
            belt_vel_ -= belt_vel_adjust_value_;
            break;

        case robot_config::LeverPosition::LEFT:
            belt_vel_ += belt_vel_adjust_value_ * lever_degree_ofattenuation_;
            break;

        case robot_config::LeverPosition::LEFT_DEEP:
            belt_vel_ += belt_vel_adjust_value_;
            break;

        case robot_config::LeverPosition::PUSH:
            belt_vel_ = belt_vel_init_;
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
