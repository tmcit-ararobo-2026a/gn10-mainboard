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

float BucketArmController::width_motor_output(bool left, bool right) const
{
    const float max_angle = (width_max_ - width_min_) / width_pulley_radius_;
    if (left == right) {
        return 0.0f;
    }
    if (left) {
        return -width_adjustment_velocity_ratio_;
    }
    if (right && width_motor_angle_ < max_angle) {
        return width_adjustment_velocity_ratio_;
    }
}

float BucketArmController::hold_motor_output(bool hold) const
{
    if (hold) {
        return -hold_current_;
    } else {
        return release_current_;
    }
}