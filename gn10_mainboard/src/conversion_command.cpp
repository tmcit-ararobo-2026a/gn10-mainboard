#include "gn10_mainboard/robot_config.hpp"

class ConversionCommand
{
public:
    robot_config::command_t conversion(robot_config::teleop_t teleop)
    {
        teleop_ = teleop;

        // 足回り
        command_.x_vel = teleop_.analog.stick_left[0];
        command_.y_vel = teleop_.analog.stick_left[1];

        // backet
        if (teleop_.buttons.up) {
            command_.bucket_arm_hight += 10;  // アホ
        } else if (teleop_.buttons.down) {
            command_.bucket_arm_hight -= 10;  // アホ
        }
        if (teleop_.buttons.right) {
            command_.bucket_arm_hold += 10;
        } else if (teleop_.buttons.left) {
            command_.bucket_arm_hold -= 10;
        }

        // 机上回収
        if (teleop_.buttons.triangle && !triangle_last) {
            if (!command_.desk_arm_open) {
                command_.desk_arm_open = true;
            } else {
                command_.desk_arm_open = false;
            }
        }

        triangle_last = teleop_.buttons.triangle;

        if (teleop_.buttons.circle && !circle_last) {
            if (!command_.desk_arm_push) {
                command_.desk_arm_push = true;
            } else {
                command_.desk_arm_push = false;
            }
        }
        circle_last = teleop_.buttons.circle;

        // 装填機構
        command_.loading_shift_cloth = teleop_.buttons.cross;

        // ベルト直動
        switch (teleop_.buttons.lever_right) {
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

        if (teleop_.buttons.lever_left == robot_config::LeverPosition::PUSH ||
            teleop_.buttons.lever_right == robot_config::LeverPosition::PUSH) {
            command_.belt_init = true;
        } else {
            command_.belt_init = false;
        }

        // エアシリンダー
        switch (teleop_.buttons.lever_left) {
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

        return command_;
    }

private:
    robot_config::teleop_t teleop_;
    robot_config::command_t command_;
    uint8_t count      = 1;
    bool triangle_last = false;  // edge
    bool circle_last   = false;
};