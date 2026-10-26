/**
 * @brief Launcher device用変換関数たち
 */
#pragma once

class LauncherRevision
{
public:
    LauncherRevision(float motor_poles, float belt_pitch, float pully_teeth)
        : motor_poles_(motor_poles), belt_pitch_(belt_pitch), pully_teeth_(pully_teeth)
    {
    }

    float velocity_to_revision_erpm(float vel_target)
    {
        const float distance_per_rotation = belt_pitch_ * pully_teeth_;
        return (vel_target / distance_per_rotation) * 60.0f * (motor_poles_ / 2.0f);
    }

    /**
     * @brief ERPMから速度に変換
     */
    float erpm_to_velocity(float erpm)
    {
        const float distance_per_rotation = belt_pitch_ * pully_teeth_;
        const float motor_pair_poles      = motor_poles_ / 2.0f;
        return (erpm / motor_pair_poles) / 60.0f * distance_per_rotation;
    }

private:
    float motor_poles_{};
    float belt_pitch_{};
    float pully_teeth_{};
};
