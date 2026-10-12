#include "app/robot_state_machine.hpp"

void RobotStateMachine::update(
    uint32_t now_ms, bool operation_received, const robot_config::operation_t& latest_op
)
{
    if (operation_received) {
        last_operation_received_ms_ = now_ms;
        operation_timeout_          = false;

        // Jetson側のFSMが決定したnavigation_statusを同期
        current_status_ = latest_op.navigation_status;
    } else {
        // 通信途絶時の安全フェールセーフ
        if ((now_ms - last_operation_received_ms_) > OPERATION_TIMEOUT_MS) {
            operation_timeout_ = true;
            // 通信途絶時はフェールセーフとして手動モード（FailまたはStanby）へ強制遷移
            current_status_             = robot_config::NavigationStatus::Fail;
            command_.navigation_command = robot_config::NavigationCommand::Sleep;
        }
    }
}

void RobotStateMachine::handle_teleop_input(
    const robot_config::teleop_t& teleop, const robot_config::teleop_t& last_teleop
)
{
    // 左スティック押し込み: 自動モードの有効化(スタンバイ要求)/無効化(Sleep)トグル
    const bool stick_push_left_edge =
        teleop.buttons.stick_push_left && !last_teleop.buttons.stick_push_left;

    if (stick_push_left_edge) {
        if (command_.navigation_command == robot_config::NavigationCommand::Sleep) {
            command_.navigation_command = robot_config::NavigationCommand::Stanby;
        } else {
            // ナビゲーションのキャンセル・停止
            command_.navigation_command = robot_config::NavigationCommand::Sleep;
            current_status_             = robot_config::NavigationStatus::Stanby;
        }
    }
}

