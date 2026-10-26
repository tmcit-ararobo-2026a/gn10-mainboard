#pragma once
#include <array>

struct LEDInformation {
    /*ベルト直動*/
    bool belt_initialization{};  // true:打てます！！ false:初期化未完了
    float belt_velocity{};
    /*エアー射出*/
    bool air_injection{};
    /*電源*/
    uint8_t
        logic_battery_level;  // 制御用電源の電圧レベル（10~25.5V以内で最も低い電圧のバッテリーを示す。スケールは1/10）
    uint8_t drive_battery_level;  // 駆動用電源の電圧レベル
    /*自己位置推定*/

    int8_t move_bucket_angle_yaw;  // 移動バケツの水平角[-127~127, not found:-128]
    int8_t bucket1_angle_yaw;      // バケツ1の水平角[-127~127, not found:-128]
    int8_t bucket2_angle_yaw;      // バケツ2の水平角[-127~127, not found:-128]
    int8_t bucket3_angle_yaw;      // バケツ3の水平角[-127~127, not found:-128]
    int8_t flag_angle_yaw;         // 旗の水平角[-127~127, not found:-128]
    int8_t desk_angle_yaw;         // 机の水平角[-127~127, not found:-128]

} __attribute__((__packed__));
