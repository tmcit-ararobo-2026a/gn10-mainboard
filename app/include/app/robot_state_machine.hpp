#pragma once

#include <cstdint>
#include "app/robot_config.hpp"

/**
 * @brief 機体の自動モード・ナビゲーション状態遷移マシン
 *
 * 状態遷移:
 *  Standby (手動) --(通信・推定成功)--> Ready
 *  Ready          --(自動移動命令)--> Moving
 *  Moving         --(エラー/危険)--> Fail
 *  Moving         --(誤差範囲内)--> Goal
 *  Moving         --(キャンセル)--> Ready
 *  Goal           --(誤差範囲外)--> Ready
 *  Ready          --(移動バケツ追従命令)--> Tracking
 *  Tracking       --(見失い/範囲外)--> Fail
 *  Tracking       --(追従終了)--> Ready
 *  Ready          --(自動移動不可)--> Fail
 *  Fail           --(復帰)--> Ready
 */
class RobotStateMachine
{
public:
    RobotStateMachine() = default;

    /**
     * @brief 周期更新処理（通信タイムアウト等の安全監視と状態の同期）
     * @param now_ms 現在時刻[ms]
     * @param operation_received 今回の周期でoperationを受信できたか
     * @param latest_op 受信した最新のoperationデータ
     */
    void update(uint32_t now_ms, bool operation_received, const robot_config::operation_t& latest_op);

    /**
     * @brief コントローラ入力によるコマンド更新
     * @param teleop 現在のコントローラ入力
     * @param last_teleop 前回のコントローラ入力
     */
    void handle_teleop_input(const robot_config::teleop_t& teleop, const robot_config::teleop_t& last_teleop);

    /**
     * @brief 送信用 command_t を取得
     */
    const robot_config::command_t& get_command() const { return command_; }

    /**
     * @brief 現在確定しているナビゲーションステータスを取得
     */
    robot_config::NavigationStatus get_status() const { return current_status_; }

    /**
     * @brief PC/Jetsonとの通信がタイムアウトしているか
     */
    bool is_operation_timeout() const { return operation_timeout_; }

    /**
     * @brief 手動モード要求をセット
     */
    void request_sleep()
    {
        command_.navigation_command = robot_config::NavigationCommand::Sleep;
        current_status_ = robot_config::NavigationStatus::Stanby;
    }

private:
    static constexpr uint32_t OPERATION_TIMEOUT_MS = 200;

    robot_config::command_t command_{
        .header = robot_config::header::command,
        .sequence = 0,
        .shutdown = false,
        .localization = false,
        .logging = false,
        .target_id = robot_config::TargetId::StartZone,
        .navigation_command = robot_config::NavigationCommand::Sleep,
        .cloth_collect = false,
    };

    robot_config::NavigationStatus current_status_ = robot_config::NavigationStatus::Stanby;
    uint32_t last_operation_received_ms_ = 0;
    bool operation_timeout_ = false;
};

