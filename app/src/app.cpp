#include "app/app.hpp"
// std
#include <cmath>
// STM32 HAL
#include "fdcan.h"
// gn10-can
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/esc_hub_client.hpp"
#include "gn10_can/devices/launcher_client.hpp"
#include "gn10_can/devices/led_client.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_can/devices/power_manager_client.hpp"
#include "gn10_can/devices/robot_control_hub_server.hpp"
#include "gn10_can/devices/solenoid_driver_client.hpp"
// gn10-mainboard
#include "app/belt_launcher_controller.hpp"
#include "app/bucket_arm_controller.hpp"
#include "app/control_arbitrator.hpp"
#include "app/launcher_revision.hpp"
#include "app/led_information.hpp"
#include "app/robot_ethernet.hpp"
#include "app/robot_state_machine.hpp"
#include "app/serial_printf.hpp"
#include "app/three_wheel_omni.hpp"
// others
#include "gn10_stm32_fdcan_driver/can_callback_helper.hpp"
#include "gn10_stm32_fdcan_driver/can_driver.hpp"
#include "gn10_stm32_fdcan_driver/fdcan_driver.hpp"

namespace {
/* ----------------- 定数 ----------------------*/
// 足回り
constexpr float LINER_VELOCITY_MAX   = 4.0f;
constexpr float ANGULAR_VELOCITY_MAX = 4.5f;
constexpr float WHEEL_PID_GAINS[3]   = {0.05f, 0.0f, 0.0f};
// ベルト直動
constexpr float BELT_LAUNCHER_MAX_VELOCITY        = 11.5f;
constexpr float BELT_LAUNCHER_MIN_VELOCITY        = 2.0f;
constexpr float BELT_LAUNCHER_DEFAULT_VELOCITY    = 4.0f;
constexpr float BELT_LAUNCHER_ADJUSTMENT_VELOCITY = 0.5f;
constexpr float BELT_LAUNCHER_REFERENCE_VOLTAGE   = 19.8f;  // [V]
constexpr float BELT_LAUNCHER_COMPENSATION_RATIO  = 0.00f;  // [m/(s*V)]
// 装填機構
constexpr float RELOAD_ANGLE_DELTA  = -(float)M_PI * 2.0f / 3.0f;
constexpr uint32_t RELOAD_DELAY_MS  = 1200;
constexpr float RELOAD_PID_GAINS[3] = {-1.25f, 0.0f, -0.1f};
// バケツ用アーム
constexpr float BUCKET_ARM_HEIGHT_PULLEY_RADIUS = 0.04f;   // [m]
constexpr float BUCKET_ARM_HEIGHT_MAX           = 0.6f;    // [m]
constexpr float BUCKET_ARM_HEIGHT_MIN           = 0.075f;  // [m]
constexpr float BUCKET_ARM_WIDTH_SPEED_RATIO    = 2.0f;    // [A]
constexpr float BUCKET_ARM_HOLD_FORCE           = 2.0f;    // [A]
constexpr float BUCKET_ARM_RELEASE_FORCE        = 1.0f;    // [A]
// エアシリンダ
// 射出命令が切れてからエアシリンダを収納するまでの時間[ms]
constexpr uint32_t AIR_STORAGE_DELAY_MS = 800;
// エアシリンダを射出する際の開放時間[ms]
constexpr uint32_t AIR_FIRE_LENGTH_MS = 100;
// エアシリンダを収納する際の開放時間[ms]
constexpr uint32_t AIR_STORAGE_LENGTH_MS    = 400;
constexpr uint32_t AIR_LAUNCHER_RIGHT_INDEX = 0;
constexpr uint32_t AIR_LAUNCHER_LEFT_INDEX  = 1;
// 機械定数
constexpr float M3508_GEAR_RATIO = 19.0f;
// 処理定数
constexpr uint32_t HEARTBEAT_TOGGLE_INTERVAL_MS = 500;
constexpr uint32_t FEEDBACK_INTERVAL_MS         = 20;
constexpr uint32_t ETHER_INIT_DELAY_MS          = 1000;
constexpr uint32_t TELEOP_TIMEOUT_MS            = 100;
constexpr uint32_t OPERATION_TIMEOUT_MS         = 100;
/* ---------------------- gn10-can ---------------------- */
// Device Configuration
gn10_can::devices::MotorConfig motor_config_wheel;
gn10_can::devices::MotorConfig motor_config_hand;
gn10_can::devices::MotorConfig motor_config_belt;
gn10_can::devices::MotorConfig motor_config_loading;
gn10_can::devices::MotorConfig motor_config_arm_height;
gn10_can::devices::MotorConfig motor_config_arm_width;
gn10_can::devices::power_manager::Config drive_power_manager_config;
gn10_can::devices::power_manager::Config logic_power_manager_config;
// CAN Drivers
gn10_can::drivers::CANDriver can1_driver(&hfdcan1);
gn10_can::drivers::FDCANDriver fdcan2_driver(&hfdcan2);
gn10_can::drivers::FDCANDriver fdcan3_driver(&hfdcan3);
// CAN Bus
gn10_can::CANBus can1_bus(can1_driver);
gn10_can::FDCANBus fdcan2_bus(fdcan2_driver);
gn10_can::FDCANBus fdcan3_bus(fdcan3_driver);
// CAN Devices
gn10_can::devices::SolenoidDriverClient solenoid(can1_bus, 0);
gn10_can::devices::ESCHubClient esc_wheel(fdcan3_bus, 1);
gn10_can::devices::ESCHubClient esc_arm_hold_and_loading_and_width(fdcan3_bus, 2);
gn10_can::devices::MotorDriverClient dc_arm_height(can1_bus, 0);
gn10_can::devices::PowerManagerClient drive_power_manager(fdcan2_bus, 0);
gn10_can::devices::PowerManagerClient logic_power_manager(fdcan2_bus, 1);
gn10_can::devices::LauncherClient belt_launcher_client(fdcan3_bus, 0);
gn10_can::devices::LEDClient<LEDInformation> led_client(fdcan2_bus, 2);

/* ---------------------------- ethernet --------------------------*/
// Ethernet
RobotEthernet ether;

/* ---------------------------- 運動学 ------------------------- */
ThreeWheelOmni omni(0.4f, 0.13f / 2.0f);

/* ----------------------- robot control --------------------------*/
// 装填・アーム出力値
std::array<float, 4> arm_hold_and_loading_and_width_target{0.0f, 0.0f, 0.0f, 0.0f};

// ベルト直動
BeltLauncherController belt_launcher_controller(
    BELT_LAUNCHER_MAX_VELOCITY, BELT_LAUNCHER_MIN_VELOCITY
);

// バケツアーム
bool dc_arm_height_encoder_initialized = false;
BucketArmController bucket_arm(
    BUCKET_ARM_HEIGHT_PULLEY_RADIUS,
    BUCKET_ARM_HEIGHT_MAX,
    BUCKET_ARM_HEIGHT_MIN,
    BUCKET_ARM_WIDTH_SPEED_RATIO
);
// エアシリンダ
struct AirLauncherStorage {
    uint32_t last_fire_tick_ms;
    uint32_t start_storaging_tick_ms;
    bool fired;
    bool storaging;
};
AirLauncherStorage air_launcher[2];

/* --------------------- コントローラー（teleop）との通信 ---------------------*/
robot_config::teleop_t teleop{};
robot_config::teleop_t last_teleop{};
uint32_t last_teleop_received_ms     = 0;
bool teleop_timeout                  = false;
uint32_t selected_air_launcher_index = AIR_LAUNCHER_RIGHT_INDEX;

/* --------------------- 状態管理・PCとの通信 -----------------------------*/
RobotStateMachine state_machine;
robot_config::operation_t robot_operation{};
robot_config::feedback_t robot_feedback{};

/* ----------------------- LED --------------------------*/
LEDInformation led_info;

/* ------------------ Lチカ ----------------------- */
uint32_t heartbeat_last_toggle_time_ms = 0;
/**
 * @brief 一定周期のLEDトグル
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERVAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

uint32_t feedback_last_send_time_ms = 0;
/**
 * @brief 一定周期でフィードバックをEthernetで送信する
 *
 */
void periodic_feedback()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - feedback_last_send_time_ms) >= FEEDBACK_INTERVAL_MS) {
        feedback_last_send_time_ms = now_ms;
        if (ether.send_feedback_data(robot_feedback)) {
            robot_feedback.sequence++;
        }
        led_client.send_display_info(led_info);
    }
}

void packet_led_information_data()
{
    led_info.bucket1_angle_yaw_rad     = robot_operation.bucket1_angle_yaw;
    led_info.bucket2_angle_yaw_rad     = robot_operation.bucket2_angle_yaw;
    led_info.bucket3_angle_yaw_rad     = robot_operation.bucket3_angle_yaw;
    led_info.flag_angle_yaw_rad        = robot_operation.flag_angle_yaw;
    led_info.move_bucket_angle_yaw_rad = robot_operation.move_bucket_angle_yaw;
}

void stop_all_actuators()
{
    std::array<float, 4> esc_target_zero{};
    esc_wheel.set_targets(esc_target_zero.data());
    // esc_arm_hold_and_loading.set_targets(esc_target_zero.data());
    dc_arm_height.set_target(0.0f);
    std::array<bool, 8> solenoid_target_zero{};
    solenoid.set_target(solenoid_target_zero);
}

}  // namespace

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
    HAL_Delay(ETHER_INIT_DELAY_MS);

    // CAN initialization
    fdcan3_driver.set_tx_timeout(2);
    can1_driver.init();
    fdcan2_driver.init();
    fdcan3_driver.init();

    // Motor configuration
    motor_config_wheel.set_motor_type(gn10_can::devices::MotorType::C620);
    motor_config_wheel.set_encoder_type(gn10_can::devices::EncoderType::InternalIncremental);
    motor_config_wheel.set_max_duty_ratio(20.0f);
    motor_config_wheel.set_accel_ratio(1.0f);

    motor_config_belt.set_motor_type(gn10_can::devices::MotorType::VESC);

    // Other device configuration
    drive_power_manager_config.sensor_rate_ms            = 30;
    drive_power_manager_config.use_remote_emergency_stop = false;
    logic_power_manager_config.sensor_rate_ms            = 100;
    logic_power_manager_config.use_remote_emergency_stop = false;

    // Initialize devices on the network
    for (uint8_t i = 0; i < 4; i++) {
        esc_wheel.set_init(i, motor_config_wheel);
        esc_wheel.set_gains(i, WHEEL_PID_GAINS[0], WHEEL_PID_GAINS[1], WHEEL_PID_GAINS[2], 0.0f);
    }
    // 装填
    motor_config_loading.set_motor_type(gn10_can::devices::MotorType::C610);
    motor_config_loading.set_encoder_type(gn10_can::devices::EncoderType::IncrementalTotal);
    motor_config_loading.set_max_duty_ratio(10.0f);
    esc_arm_hold_and_loading_and_width.set_init(0, motor_config_loading);
    esc_arm_hold_and_loading_and_width.set_gains(
        0, RELOAD_PID_GAINS[0], RELOAD_PID_GAINS[1], RELOAD_PID_GAINS[2], 0.0f
    );

    // バケツの横軸
    motor_config_arm_width.set_max_duty_ratio(1.0f);
    motor_config_arm_width.set_motor_type(gn10_can::devices::MotorType::C610);
    motor_config_arm_width.set_encoder_type(gn10_can::devices::EncoderType::None);
    motor_config_arm_width.set_forward_limit_switch(true, 0);
    motor_config_arm_width.set_reverse_limit_switch(true, 1);
    esc_arm_hold_and_loading_and_width.set_init(1, motor_config_arm_width);
    // 雑巾保持
    motor_config_hand.set_max_duty_ratio(1.0f);
    motor_config_hand.set_motor_type(gn10_can::devices::MotorType::C610);
    motor_config_hand.set_encoder_type(gn10_can::devices::EncoderType::None);  // 電流制御
    esc_arm_hold_and_loading_and_width.set_init(2, motor_config_hand);
    // 高さ調整
    motor_config_arm_height.set_max_duty_ratio(0.75f);
    motor_config_arm_height.set_reverse_limit_switch(true, 0);
    motor_config_arm_height.set_motor_type(gn10_can::devices::MotorType::DC);
    motor_config_arm_height.set_encoder_type(gn10_can::devices::EncoderType::IncrementalTotal);
    motor_config_arm_height.set_feedback_cycle(10);
    dc_arm_height.set_init(motor_config_arm_height);

    solenoid.set_init();
    drive_power_manager.set_init(drive_power_manager_config);
    logic_power_manager.set_init(logic_power_manager_config);

    // Initialize Ethernet
    ether.init();

    bucket_arm.set_height_adjustment_velocity_ratio(1.0f);
    bucket_arm.set_hold_force_by_current(BUCKET_ARM_HOLD_FORCE);
    bucket_arm.set_release_force_by_current(BUCKET_ARM_RELEASE_FORCE);

    belt_launcher_controller.set_velocity(BELT_LAUNCHER_DEFAULT_VELOCITY);
    belt_launcher_controller.set_velocity_adjustment_amount(BELT_LAUNCHER_ADJUSTMENT_VELOCITY);
    belt_launcher_controller.set_reload_delay_ms(RELOAD_DELAY_MS);
    belt_launcher_controller.set_reload_angle_delta(RELOAD_ANGLE_DELTA);
    belt_launcher_controller.set_reference_voltage(BELT_LAUNCHER_REFERENCE_VOLTAGE);
    belt_launcher_controller.set_voltage_compensation_ratio(BELT_LAUNCHER_COMPENSATION_RATIO);
    // System setup
    const uint32_t now_ms         = HAL_GetTick();
    heartbeat_last_toggle_time_ms = now_ms;

    for (uint32_t i = 0; i < 2; i++) {
        air_launcher[i].last_fire_tick_ms       = now_ms;
        air_launcher[i].start_storaging_tick_ms = now_ms;
        air_launcher[i].storaging               = false;
        air_launcher[i].fired                   = false;
    }
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    const uint32_t now_ms = HAL_GetTick();

    // 1. コントローラ（Teleop）受信とアクチュエータ駆動
    if (ether.receive_teleop(teleop)) {
        teleop_timeout          = false;
        last_teleop_received_ms = now_ms;

        // コントローラ入力による状態遷移要求（左スティック押し込み等）
        state_machine.handle_teleop_input(teleop, last_teleop);

        const bool is_auto_mode = state_machine.is_auto_mode_enabled();

        // --- 1. 足回り手動入力（全モード共通: 左右アナログスティック） ---
        const float teleop_vx =
            std::clamp(static_cast<float>(teleop.analog.stick_left[0]) / INT8_MAX, -1.0f, 1.0f) *
            LINER_VELOCITY_MAX;
        const float teleop_vy =
            std::clamp(static_cast<float>(teleop.analog.stick_left[1]) / INT8_MAX, -1.0f, 1.0f) *
            LINER_VELOCITY_MAX;
        const float teleop_vyaw =
            std::clamp(static_cast<float>(teleop.analog.stick_right[0]) / INT8_MAX, -1.0f, 1.0f) *
            ANGULAR_VELOCITY_MAX;

        // --- 2. 射出トリガー（全モード共通: L1 / R1 スイッチ） ---
        // right_trigger: ベルト直動射出 (押下エッジ)
        const bool belt_fire_trigger =
            teleop.buttons.right_trigger && !last_teleop.buttons.right_trigger;
        // left_trigger: エアシリンダ射出 (押下エッジ)
        const bool air_fire_trigger =
            teleop.buttons.left_trigger && !last_teleop.buttons.left_trigger;

        // --- 3. 手動モード時のアクチュエータ入力処理 (teleop_key_config.md 第3.1項) ---
        float teleop_arm_height = 0.0f;
        float teleop_arm_hold   = 0.0f;
        float teleop_arm_width  = 0.0f;

        if (!is_auto_mode) {
            // 左十字キー 上/下: バケツ用アーム昇降
            teleop_arm_height =
                bucket_arm.height_motor_output(teleop.buttons.left_up, teleop.buttons.left_down);

            // 左十字キー 左: アーム展開 / 収納トグル
            teleop_arm_width = bucket_arm.width_motor_output(
                teleop.buttons.left_left && !last_teleop.buttons.left_left
            );

            // 左十字キー 右: ハンド把持 / 解放トグル
            teleop_arm_hold = bucket_arm.hold_motor_output(
                teleop.buttons.left_right && !last_teleop.buttons.left_right
            );

            // 右十字キー 上/下: ベルト速度微調整
            belt_launcher_controller.update_velocity(
                teleop.buttons.right_up && !last_teleop.buttons.right_up,
                teleop.buttons.right_down && !last_teleop.buttons.right_down
            );

            // 右十字キー 左/右: 作動させるエアシリンダを選択
            if (teleop.buttons.right_left && !last_teleop.buttons.right_left) {
                selected_air_launcher_index = AIR_LAUNCHER_LEFT_INDEX;
            } else if (teleop.buttons.right_right && !last_teleop.buttons.right_right) {
                selected_air_launcher_index = AIR_LAUNCHER_RIGHT_INDEX;
            }
        }

        // --- 4. 操作権限の調停 (Arbitration) ---
        const ArbitratedControl ctrl = ControlArbitrator::arbitrate(
            state_machine.get_status(),
            teleop,
            robot_operation,
            teleop_vx,
            teleop_vy,
            teleop_vyaw,
            belt_fire_trigger,
            air_fire_trigger,
            teleop_arm_height,
            teleop_arm_hold,
            teleop_arm_width
        );

        // --- 5. 足回り（オムニ）出力 ---
        constexpr float robot_yaw_angle = 0.0f;
        omni.convert(-ctrl.chassis_vx, ctrl.chassis_vy, ctrl.chassis_vyaw, robot_yaw_angle);
        float front = 0.0f, right = 0.0f, left = 0.0f;
        omni.getWheelAngularVelocity(&front, &left, &right);
        const std::array<float, 4> wheel_targets{
            front * M3508_GEAR_RATIO, left * M3508_GEAR_RATIO, right * M3508_GEAR_RATIO, 0.0f
        };
        esc_wheel.set_targets(wheel_targets.data());

        // --- 6. ベルト直動 ---
        // 右スティック押し込み: ベルト直動初期化 (全モード共通)
        if (teleop.buttons.stick_push_right && !last_teleop.buttons.stick_push_right) {
            belt_launcher_controller.set_deinit();
            belt_launcher_client.set_init();
        }

        if (ctrl.belt_auto_velocity) {
            belt_launcher_controller.set_velocity(ctrl.belt_launcher_target_velocity);
        }
        led_info.belt_velocity = belt_launcher_controller.get_target_velocity();

        // 射出実行
        if (ctrl.belt_fire_trigger) {
            float belt_launcher_target_vel = 0.0f;
            if (belt_launcher_controller.fire(
                    belt_launcher_target_vel, robot_feedback.drive_battery_voltages
                )) {
                belt_launcher_client.send_fire_command(
                    launcher_revision::get_compensated_velocity(belt_launcher_target_vel)
                );
                robot_feedback.belt_launcher_target_velocity =
                    launcher_revision::get_compensated_velocity(belt_launcher_target_vel);
            }
        }
        led_info.belt_initialization = belt_launcher_controller.get_fire_ready();

        // --- 7. エアシリンダ制御 ---
        const uint32_t now_ms = HAL_GetTick();
        std::array<bool, 8> solenoid_targets{};
        solenoid_targets[0] = 1;  // 空圧レギュレータ電源用

        // L1トリガーで選択中のエアシリンダを射出
        if (ctrl.air_fire_trigger) {
            const uint32_t solenoid_index    = selected_air_launcher_index + 1;
            solenoid_targets[solenoid_index] = true;
            air_launcher[selected_air_launcher_index].last_fire_tick_ms = now_ms;
            air_launcher[selected_air_launcher_index].fired             = true;
        }

        for (uint32_t i = 0; i < 2; i++) {
            if (air_launcher[i].fired) {
                uint32_t elapsed_time_ms = now_ms - air_launcher[i].last_fire_tick_ms;
                if (elapsed_time_ms > AIR_FIRE_LENGTH_MS + AIR_STORAGE_DELAY_MS) {
                    air_launcher[i].fired                   = false;
                    air_launcher[i].storaging               = true;
                    air_launcher[i].start_storaging_tick_ms = now_ms;
                }
            }
            if (air_launcher[i].storaging) {
                uint32_t elapsed_time_ms = now_ms - air_launcher[i].start_storaging_tick_ms;
                if (elapsed_time_ms > AIR_STORAGE_LENGTH_MS) {
                    air_launcher[i].fired     = false;
                    air_launcher[i].storaging = false;
                }
            }
            solenoid_targets[i + 1] = (uint8_t)air_launcher[i].fired;
        }
        solenoid_targets[5] = (uint8_t)air_launcher[0].storaging;
        solenoid_targets[6] = (uint8_t)air_launcher[1].storaging;

        led_info.air_injection =
            (solenoid_targets[0] || solenoid_targets[1] || solenoid_targets[2]);
        solenoid.set_target(solenoid_targets);

        // --- 8. バケツ用アーム ---
        float arm_height_target                  = ctrl.arm_height_target;
        arm_hold_and_loading_and_width_target[1] = ctrl.arm_width_target;
        arm_hold_and_loading_and_width_target[2] = ctrl.arm_hold_target;

        // 装填
        belt_launcher_controller.load_a_cloth(arm_hold_and_loading_and_width_target[0], now_ms);

        // CAN通信
        esc_arm_hold_and_loading_and_width.set_targets(
            arm_hold_and_loading_and_width_target.data()
        );
        dc_arm_height.set_target(arm_height_target);
    } else if ((now_ms - last_teleop_received_ms) > TELEOP_TIMEOUT_MS && !teleop_timeout) {
        teleop_timeout = true;
        state_machine.request_sleep();
        stop_all_actuators();
    }

    // 2. Jetsonからの operation 受信と状態機械の更新（タイムアウト監視含む）
    const bool operation_received = ether.receive_operation_data(robot_operation);
    state_machine.update(now_ms, operation_received, robot_operation);

    // 3. LED表示情報の更新
    packet_led_information_data();

    // 4. フィードバック処理と送信
    std::array<float, 4> wheel_feedbacks{};
    if (esc_wheel.get_feedbacks(wheel_feedbacks.data())) {
        robot_feedback.wheel_angular_velocity[0] = wheel_feedbacks[0];  // front
        robot_feedback.wheel_angular_velocity[1] = wheel_feedbacks[1];  // left
        robot_feedback.wheel_angular_velocity[2] = wheel_feedbacks[2];  // right
    }
    float belt_launcher_feedback_vel{};
    if (belt_launcher_client.get_velocity_feedback(belt_launcher_feedback_vel)) {
        robot_feedback.belt_launcher_velocity = belt_launcher_feedback_vel;
    }
    float belt_launcher_initial_angle{};
    if (belt_launcher_client.get_initial_point(belt_launcher_initial_angle)) {
        belt_launcher_controller.set_initial_point(HAL_GetTick());
    }
    gn10_can::devices::power_manager::Status drive_power_status{};
    if (drive_power_manager.get_new_status(drive_power_status)) {
        robot_feedback.emergency_stop_enabled = drive_power_status.emergency_stop_enabled;
    }

    std::array<float, 4> loading_feedback = {};
    if (esc_arm_hold_and_loading_and_width.get_feedbacks(loading_feedback.data())) {
        // 非常停止中は、高速更新されるCANフィードバック受信のたびに目標角度を更新する
        if (robot_feedback.emergency_stop_enabled) {
            belt_launcher_controller.set_reload_angle(loading_feedback[0]);
        }
    }

    float latest_arm_height_motor_angle = -dc_arm_height.feedback_value();  // 降下方向を+とする
    bucket_arm.set_height_motor_angle(latest_arm_height_motor_angle);
    // ゼロ点合わせが済んだ
    if (dc_arm_height_encoder_initialized) {
    } else {  // ゼロ点取りが済んでいない場合、リミットスイッチで最高点を設定する
        uint8_t dc_arm_height_limit_sw = dc_arm_height.limit_switches();
        if ((dc_arm_height_limit_sw & 0b1)) {
            dc_arm_height.set_init(motor_config_arm_height);
            dc_arm_height_encoder_initialized = true;
        }
    }

    gn10_can::devices::power_manager::Sensor drive_power_sensor{};
    if (drive_power_manager.get_new_sensor(drive_power_sensor)) {
        robot_feedback.drive_battery_voltages = drive_power_sensor.voltage;
        robot_feedback.drive_current          = drive_power_sensor.current;
    }
    std::array<float, 4> voltages;
    if (logic_power_manager.get_new_voltages(voltages)) {
        led_info.battery_voltage[0] = voltages[0];
        led_info.battery_voltage[1] = voltages[1];
    }
    led_info.battery_voltage[2] = robot_feedback.drive_battery_voltages;
    // 指令値を追加
    robot_feedback.command = state_machine.get_command();
    periodic_feedback();

    last_teleop = teleop;

    // 5. システム生存表示（青LED）
    update_heartbeat_led();
}

// ---------------------------- C language's functions override ----------------------------------
extern "C" {
/**
 * @brief Receive callback for FDCAN FIFO0.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, can1_bus, FDCAN_RX_FIFO0)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan2, fdcan2_bus, FDCAN_RX_FIFO0)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan3, fdcan3_bus, FDCAN_RX_FIFO0)) return;
}

/**
 * @brief Receive callback for FDCAN FIFO1.
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, can1_bus, FDCAN_RX_FIFO1)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan2, fdcan2_bus, FDCAN_RX_FIFO1)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan3, fdcan3_bus, FDCAN_RX_FIFO1)) return;
}
}
