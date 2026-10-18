#include "app/bucket_arm_controller.hpp"

float BucketArmController::height_motor_output(bool up, bool down) const
{
    // 最大回転角度
    const float max_angle = (height_max_ - height_min_) / height_pulley_radius_;
    // 同時入力時は停止
    if (up == down) {
        return 0.0f;
    }
    // 上昇時はリミットスイッチが作動するため上限は設けない
    if (up) {
        return -height_adjustment_velocity_ratio_;
    }
    // 下限に達した際、その方向への移動のみを停止
    if (down && height_motor_angle_ < max_angle) {
        return height_adjustment_velocity_ratio_;
    }
    return 0.0f;
}

float BucketArmController::width_motor_output(bool input)
{
    // トグル動作
    if (!prev_input_ && input) {
        width_expand_ = !width_expand_;
    }
    prev_input_ = input;
    // 出力へ変換
    if (width_expand_) {
        return width_speed_ratio_;
    } else {
        return -width_speed_ratio_;
    }
}

float BucketArmController::hold_motor_output(bool input)
{
    // トグル動作
    if (!prev_hold_input_ && input) {
        hold_ = !hold_;
    }
    prev_hold_input_ = input;

    if (hold_) {
        return -hold_current_;
    } else {
        return release_current_;
    }
}