#pragma once

#include "app/robot_config.hpp"

/**
 * @brief 調停後の機体統合動作指令
 */
struct ArbitratedControl {
    // オムニ足回り指令 [m/s, rad/s]
    float chassis_vx = 0.0f;
    float chassis_vy = 0.0f;
    float chassis_vyaw = 0.0f;

    // ベルト直動速度 [m/s]
    float belt_launcher_target_velocity = 0.0f;
    bool belt_auto_velocity = false; // 自動計算の威力を適用するか

    // ベルト射出トリガー
    bool belt_fire_trigger = false;

    // アーム・ハンド操作
    bool arm_manual_allowed = false; // 手動操作を許可するか
    float arm_height_target = 0.0f;
    float arm_hold_target = 0.0f;
};

/**
 * @brief 状態機械のステータスに応じた操作権限（手動 vs 自動 vs 不可）の調停クラス
 *
 * 権限マッピング:
 *  Standby:  移動:手動 | 射出:手動 | 威力:手動 | ハンド:手動
 *  Ready:    移動:手動 | 射出:手動 | 威力:手動 | ハンド:自動
 *  Moving:   移動:自動 | 射出:手動 | 威力:自動 | ハンド:不可(ロック)
 *  Goal:     移動:手動 | 射出:手動 | 威力:手動 | ハンド:自動
 *  Tracking: 水平:手動 / 回転:自動 | 射出:手動 | 威力:自動 | ハンド:不可(ロック)
 *  Fail:     移動:手動 | 射出:手動 | 威力:手動 | ハンド:手動
 */
class ControlArbitrator
{
public:
    static ArbitratedControl arbitrate(
        robot_config::NavigationStatus status,
        const robot_config::teleop_t& teleop,
        const robot_config::operation_t& op,
        float teleop_vx,
        float teleop_vy,
        float teleop_vyaw,
        bool teleop_fire_pressed,
        float teleop_arm_height,
        float teleop_arm_hold
    );
};

