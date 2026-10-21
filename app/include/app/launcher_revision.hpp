#pragma once

#include <array>
#include <cstddef>

namespace launcher_revision {
/**
 * @brief 1次元線形補間のヘルパー
 */
constexpr float interp(float x, float x0, float x1, float y0, float y1)
{
    if (x1 == x0) return y0;
    const float t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
}

/**
 * @brief 欲しいリリース速度から、実測を一致させるための送信速度を計算する
 * @param v_target ユーザーが指定した欲しい速度 [m/s]
 * @return 送信すべき補正済み速度 [m/s]
 */
inline float get_compensated_velocity(float v_target)
{
    // ※ 2番目の値（送信側の値）は、受信側の `velocity_to_rpm` を経由したときに
    //    期待通りの ERPM になるような逆算値に書き換えてご活用ください。
    if (v_target <= 2.000f) return 2.000f;
    if (v_target <= 4.460f) return interp(v_target, 2.000f, 4.460f, 2.000f, 3.120f);     // 例
    if (v_target <= 4.470f) return interp(v_target, 4.460f, 4.470f, 3.120f, 4.470f);     // 例
    if (v_target <= 6.204f) return interp(v_target, 4.470f, 6.204f, 4.470f, 4.850f);     // 例
    if (v_target <= 8.705f) return interp(v_target, 6.204f, 8.705f, 4.850f, 5.290f);     // 例
    if (v_target <= 10.594f) return interp(v_target, 8.705f, 10.594f, 5.290f, 5.730f);   // 例
    if (v_target <= 11.597f) return interp(v_target, 10.594f, 11.597f, 5.730f, 6.170f);  // 例
    if (v_target <= 11.935f) return interp(v_target, 11.597f, 11.935f, 6.170f, 6.610f);  // 例
    return 6.610f;                                                                       // 頭打ち
}

}  // namespace launcher_revision