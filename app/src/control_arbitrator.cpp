#include "app/control_arbitrator.hpp"

ArbitratedControl ControlArbitrator::arbitrate(
    robot_config::NavigationStatus status,
    const robot_config::teleop_t& /*teleop*/,
    const robot_config::operation_t& op,
    float teleop_vx,
    float teleop_vy,
    float teleop_vyaw,
    bool teleop_fire_pressed,
    float teleop_arm_height,
    float teleop_arm_hold
)
{
    ArbitratedControl result{};

    // 全モード共通: ベルト射出トリガーは手動操作
    result.belt_fire_trigger = teleop_fire_pressed;

    switch (status) {
        case robot_config::NavigationStatus::Moving:
            // 水平・回転移動：自動 (Jetson座標系 -> オムニ機体座標系)
            result.chassis_vx   = -op.vel_y;
            result.chassis_vy   = op.vel_x;
            result.chassis_vyaw = op.vel_yaw;

            // ベルト直動威力：自動
            result.belt_auto_velocity          = true;
            result.belt_launcher_target_velocity = op.belt_launcher_speed;

            // ハンド操作：不可（安全ロック）
            result.arm_manual_allowed = false;
            result.arm_height_target  = 0.0f;
            result.arm_hold_target    = 0.0f;
            break;

        case robot_config::NavigationStatus::Tracking:
            // 水平移動：手動 / 回転移動：自動
            result.chassis_vx   = teleop_vx;
            result.chassis_vy   = teleop_vy;
            result.chassis_vyaw = op.vel_yaw;

            // ベルト直動威力：自動
            result.belt_auto_velocity          = true;
            result.belt_launcher_target_velocity = op.belt_launcher_speed;

            // ハンド操作：不可（安全ロック）
            result.arm_manual_allowed = false;
            result.arm_height_target  = 0.0f;
            result.arm_hold_target    = 0.0f;
            break;

        case robot_config::NavigationStatus::Ready:
        case robot_config::NavigationStatus::Goal:
            // 水平・回転移動：手動
            result.chassis_vx   = teleop_vx;
            result.chassis_vy   = teleop_vy;
            result.chassis_vyaw = teleop_vyaw;

            // ベルト直動威力：手動
            result.belt_auto_velocity = false;

            // ハンド操作：自動（Jetsonからの司令値）
            result.arm_manual_allowed = false;
            // op.bucket_arm_height は目標高さ[m]
            result.arm_height_target  = op.bucket_arm_height;
            result.arm_hold_target    = op.bucket_arm_hold ? 1.0f : 0.0f;
            break;

        case robot_config::NavigationStatus::Stanby:
        case robot_config::NavigationStatus::Fail:
        default:
            // 完全手動モード
            result.chassis_vx   = teleop_vx;
            result.chassis_vy   = teleop_vy;
            result.chassis_vyaw = teleop_vyaw;

            result.belt_auto_velocity = false;

            // ハンド操作：手動
            result.arm_manual_allowed = true;
            result.arm_height_target  = teleop_arm_height;
            result.arm_hold_target    = teleop_arm_hold;
            break;
    }

    return result;
}

