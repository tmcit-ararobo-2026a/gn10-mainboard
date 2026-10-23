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
        const float v = vel_target;
        float erpm    = 0.0f;

        if (v <= 0.0f) {
            erpm = 0.0f;  // 停止指令
        } else if (v <= 2.000f) {
            erpm = 5286.0f;  // ノード0より下は一定
        } else if (v <= 4.460f) {
            // ノード0 -> 1
            erpm = 5286.0f + (v - 2.000f) * (13916.0f - 5286.0f) / (4.460f - 2.000f);
        } else if (v <= 4.470f) {
            // ノード1 -> 2 (実測の段差。ERPMが急に変わる)
            erpm = 13916.0f + (v - 4.460f) * (17816.0f - 13916.0f) / (4.470f - 4.460f);
        } else if (v <= 6.204f) {
            // ノード2 -> 3
            erpm = 17816.0f + (v - 4.470f) * (19350.0f - 17816.0f) / (6.204f - 4.470f);
        } else if (v <= 8.705f) {
            // ノード3 -> 4
            erpm = 19350.0f + (v - 6.204f) * (21109.0f - 19350.0f) / (8.705f - 6.204f);
        } else if (v <= 10.594f) {
            // ノード4 -> 5
            erpm = 21109.0f + (v - 8.705f) * (22868.0f - 21109.0f) / (10.594f - 8.705f);
        } else if (v <= 11.597f) {
            // ノード5 -> 6
            erpm = 22868.0f + (v - 10.594f) * (24627.0f - 22868.0f) / (11.597f - 10.594f);
        } else if (v <= 11.935f) {
            // ノード6 -> 7
            erpm = 24627.0f + (v - 11.597f) * (26386.0f - 24627.0f) / (11.935f - 11.597f);
        } else {
            erpm = 26386.0f;  // ノード7より上は一定(速度が頭打ち)
        }

        return erpm;
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
