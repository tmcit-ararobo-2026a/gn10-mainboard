#pragma once
/**
 * 欲しいリリース速度[m/s] -> 補正済み速度[m/s]
 *
 * 1. 実測データの対応表(速度 -> ERPM)を、区間ごとの線形補間の式で ERPM にする
 * 2. その ERPM を速度に戻して返す
 *
 * 線形補間の式:  erpm = erpm0 + (v - v0) * (erpm1 - erpm0) / (v1 - v0)
 *   (v0, erpm0) = 区間の始点ノード,  (v1, erpm1) = 区間の終点ノード
 */

namespace launcher_revision {

constexpr float BELT_PITCH       = 0.005f;  // 5 mm -> m
constexpr float PULLEY_TEETH     = 24.0f;
constexpr float MOTOR_POLE_PAIRS = 7.0f;

/**
 * @brief 欲しいリリース速度から、必要な ERPM を求める
 * @param v 欲しい速度 [m/s]
 */
inline float velocity_to_erpm_table(float v)
{
    float erpm = 0.0f;

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
 * @brief 欲しいリリース速度から、送信すべき補正済み速度を返す
 * @param v_target 欲しい速度 [m/s]
 * @return 補正済み速度 [m/s]
 */
inline float get_compensated_velocity(float v_target)
{
    const float erpm                  = velocity_to_erpm_table(v_target);
    const float distance_per_rotation = BELT_PITCH * PULLEY_TEETH;  // [m/rev]

    return (erpm / MOTOR_POLE_PAIRS / 60.0f) * distance_per_rotation;
}

}  // namespace launcher_revision