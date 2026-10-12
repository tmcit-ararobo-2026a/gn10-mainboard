#include "app/robot_state_machine.hpp"

void RobotStateMachine::update(
    uint32_t now_ms, bool operation_received, const robot_config::operation_t& latest_op
)
{
    if (operation_received) {
        last_operation_received_ms_ = now_ms;
        operation_timeout_          = false;

        // 手動モード(is_auto_mode_ == false)の場合は常にStanbyとして扱う
        if (!is_auto_mode_) {
            current_status_ = robot_config::NavigationStatus::Stanby;
        } else {
            // 自動モード時はJetson側の状態を同期
            current_status_ = latest_op.navigation_status;
        }
    } else {
        // 通信途絶時の安全フェールセーフ
        if ((now_ms - last_operation_received_ms_) > OPERATION_TIMEOUT_MS) {
            operation_timeout_ = true;
            // 通信途絶時は手動操作へ安全にフォールバック
            current_status_             = robot_config::NavigationStatus::Fail;
            command_.navigation_command = robot_config::NavigationCommand::Sleep;
        }
    }
}

void RobotStateMachine::handle_teleop_input(
    const robot_config::teleop_t& teleop, const robot_config::teleop_t& last_teleop
)
{
    // 1. トグルスイッチ状態によるモード切替
    // left_toggle_sw (1: 自動モード, 0: 手動モード)
    const bool prev_auto_mode = is_auto_mode_;
    is_auto_mode_ = (teleop.buttons.left_toggle_sw != 0);

    // 手動モードに切り替わった場合、即座にナビゲーションを停止
    if (prev_auto_mode && !is_auto_mode_) {
        command_.navigation_command = robot_config::NavigationCommand::Sleep;
        current_status_             = robot_config::NavigationStatus::Stanby;
        return;
    }

    // 手動モード中はTargetId変更や自動移動コマンドは発行しない
    if (!is_auto_mode_) {
        command_.navigation_command = robot_config::NavigationCommand::Sleep;
        return;
    }

    // 自動モードに入ったばかりの初期スタンバイ要求
    if (!prev_auto_mode && is_auto_mode_) {
        if (command_.navigation_command == robot_config::NavigationCommand::Sleep) {
            command_.navigation_command = robot_config::NavigationCommand::Stanby;
        }
    }

    // 2. 左スティック押し込み: ナビゲーションのキャンセル / 停止
    const bool stick_push_left_edge =
        teleop.buttons.stick_push_left && !last_teleop.buttons.stick_push_left;
    if (stick_push_left_edge) {
        command_.navigation_command = robot_config::NavigationCommand::Stanby;
        return;
    }

    // 3. 自動モード時の TargetId 指定
    // 仕様書:
    // 左十字キー: 上(FixedBucket1), 右(FixedBucket2), 下(FixedBucket3), 左(StartZone)
    // 右十字キー: 上(Desk1), 下(Desk2), 左(Flag), 右(MovingBucket: 長押し追従)

    // 移動バケツ追従 (右十字キー 右): ボタン長押し中のみ追従
    if (teleop.buttons.right_right) {
        command_.target_id          = robot_config::TargetId::MovingBucket;
        command_.navigation_command = robot_config::NavigationCommand::Move;
        return;
    } else if (last_teleop.buttons.right_right && !teleop.buttons.right_right) {
        // ボタンを離したら追従終了（Stanbyに戻す）
        if (command_.target_id == robot_config::TargetId::MovingBucket) {
            command_.navigation_command = robot_config::NavigationCommand::Stanby;
        }
    }

    // 固定ターゲット（ボタン押下エッジで目標地点へ自動移動開始）
    if (teleop.buttons.left_up && !last_teleop.buttons.left_up) {
        command_.target_id          = robot_config::TargetId::FixedBucket1;
        command_.navigation_command = robot_config::NavigationCommand::Move;
    } else if (teleop.buttons.left_right && !last_teleop.buttons.left_right) {
        command_.target_id          = robot_config::TargetId::FixedBucket2;
        command_.navigation_command = robot_config::NavigationCommand::Move;
    } else if (teleop.buttons.left_down && !last_teleop.buttons.left_down) {
        command_.target_id          = robot_config::TargetId::FixedBucket3;
        command_.navigation_command = robot_config::NavigationCommand::Move;
    } else if (teleop.buttons.left_left && !last_teleop.buttons.left_left) {
        command_.target_id          = robot_config::TargetId::StartZone;
        command_.navigation_command = robot_config::NavigationCommand::Move;
    } else if (teleop.buttons.right_up && !last_teleop.buttons.right_up) {
        command_.target_id          = robot_config::TargetId::Desk1;
        command_.navigation_command = robot_config::NavigationCommand::Move;
    } else if (teleop.buttons.right_down && !last_teleop.buttons.right_down) {
        command_.target_id          = robot_config::TargetId::Desk2;
        command_.navigation_command = robot_config::NavigationCommand::Move;
    } else if (teleop.buttons.right_left && !last_teleop.buttons.right_left) {
        command_.target_id          = robot_config::TargetId::Flag;
        command_.navigation_command = robot_config::NavigationCommand::Move;
    }
}
