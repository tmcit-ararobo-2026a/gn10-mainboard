#pragma once
#include <array>

struct LEDInformation {
    /*ベルト直動*/
    bool belt_initialization{};  // true:打てます！！ false:初期化未完了
    float belt_velocity{};
    /*エアー射出*/
    bool air_injection{};
    /*電圧*/
    float battery_voltage[3]{};       // 0,1:logic 2:drive
    float bucket1_angle_yaw_rad;      // ロボット座標系におけるバケツ1の水平角[rad]
    float bucket2_angle_yaw_rad;      // ロボット座標系におけるバケツ2の水平角[rad]
    float bucket3_angle_yaw_rad;      // ロボット座標系におけるバケツ3の水平角[rad]
    float flag_angle_yaw_rad;         // ロボット座標系における旗の水平角[rad]
    float move_bucket_angle_yaw_rad;  // ロボット座標系における移動バケツの水平角[rad]

} __attribute__((__packed__));
