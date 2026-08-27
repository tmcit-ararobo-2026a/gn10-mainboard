#include "gn10_mainboard/app.hpp"
// std
#include <cmath>
// STM32 HAL
#include "fdcan.h"
// gn10-can
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/esc_hub_client.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_can/devices/power_manager_client.hpp"
#include "gn10_can/devices/robot_control_hub_server.hpp"
#include "gn10_can/devices/servo_motor_client.hpp"
#include "gn10_can/devices/solenoid_driver_client.hpp"
// gn10-mainboard
#include "gn10_mainboard/can_driver.hpp"
#include "gn10_mainboard/conversion_command.hpp"
#include "gn10_mainboard/fdcan_driver.hpp"
#include "gn10_mainboard/robot_ethernet.hpp"
#include "gn10_mainboard/serial_printf.hpp"
#include "gn10_mainboard/three_wheel_omni.hpp"
// others

namespace {

constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;
constexpr float k_loading_angle_tolerance         = 1.2f * (2.0f * M_PI / 3.0f);

uint32_t heartbeat_last_toggle_time_ms = 0;
// Retained Data
float vesc_feedbacks[4];

float loading_feedbacks[4];

/**
 * @brief Toggle heartbeat LED at a fixed interval.
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= k_heartbeat_toggle_interval_ms) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

// Device Configuration
gn10_can::devices::power_manager::Config power_manager_config;
gn10_can::devices::MotorConfig motor_config_wheel;
gn10_can::devices::MotorConfig motor_config_arm;
gn10_can::devices::MotorConfig motor_config_belt;
gn10_can::devices::MotorConfig motor_config_loading;
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
gn10_can::devices::RobotControlHubServer<robot_config::command_t, robot_config::feedback_t>
    robot_control_hub(fdcan2_bus, 0);
gn10_can::devices::PowerManagerClient power_manager(fdcan2_bus, 0);
gn10_can::devices::ESCHubClient vesc_hub(fdcan3_bus, 0);
gn10_can::devices::ESCHubClient esc_wheel(fdcan3_bus, 1);
gn10_can::devices::ESCHubClient esc_arm(fdcan3_bus, 2);
gn10_can::devices::ESCHubClient esc_loading(fdcan3_bus, 2);

// Ethernet
RobotEthernet ether;
robot_config::debug_pc_t prev_debug_pc = {};
// belt
bool initilized_belt = false;
float vesc_vel       = 0.0f;

// loading
bool start_encoder = false;  // encoderの測定開始を合図
bool loading       = true;   // 装填していいのかを判別
bool loading_check = true;   // 自動で装填されたか確認

// Inverse Kinematics
ThreeWheelOmni omni(0.5f, 0.1f);
constexpr float M3508_GEAR_RATIO = 19.0f;
// Controller conversion
ConversionCommand conversion;

void command_robot_drivers(const robot_config::command_t& command)
{
    // Set speed to ESC Hub for wheels
    omni.convert(command.x_vel, command.y_vel, command.angular_vel, 0.0f);
    float front, right, left;
    omni.getWheelAngularVelocity(&front, &left, &right);

    float wheel_target[4];
    wheel_target[0] = front * M3508_GEAR_RATIO;
    wheel_target[1] = left * M3508_GEAR_RATIO;
    wheel_target[2] = right * M3508_GEAR_RATIO;
    wheel_target[3] = 0.0f;
    esc_wheel.set_targets(wheel_target);

    // Control the belt-type injection
    if (command.belt_init) {
        initilized_belt = true;
        vesc_hub.set_init(0, motor_config_belt);
    }

    if (command.belt_throw && initilized_belt) {
        vesc_vel = command.belt_vel;
    } else {
        vesc_vel = 0.0f;
    }

    float vesc_target[4] = {vesc_vel, 0.0f, 0.0f, 0.0f};
    vesc_hub.set_targets(vesc_target);
    // Control the air-type injection
    std::array<bool, 8> targets{};
    targets[0] = command.air_rauncher_for_flag;
    targets[1] = command.air_rauncher_for_desk_r;
    targets[2] = command.air_rauncher_for_desk_l;
    solenoid.set_target(targets);

    // Control the arm with C610
    float arm_and_loading_target[4] = {};
    arm_and_loading_target[0]       = command.bucket_arm_hight;        //[rad/s]
    arm_and_loading_target[1]       = (float)command.bucket_arm_hold;  // bool

    // Control the loadinf with C610
    float loading_target         = 0.0f;  // 装填機構の目標角度を入れる変数
    static uint8_t loading_count = 0;     // 回転回数を数える変数

    if (start_encoder && loading) {
        loading_count++;
        loading_target = (float)loading_count * (float)(2 * M_PI / 3);
        loading        = false;
    }

    static bool last_command_loading = false;  // 前回装填したか確認する変数

    if (!loading_check && command.loading && !last_command_loading) {
        loading_count++;
        loading_target = (float)loading_count * (float)(2 * M_PI / 3);
    }
    last_command_loading = command.loading;

    arm_and_loading_target[3] = loading_target;

    esc_arm.set_targets(arm_and_loading_target);
    serial_printf(
        "f:%3.1f, l:%3.1f, r:%3.1f, vesc:%.2f, air:%d, %d, %d, arm:%.2f, %.2f, %.2f, %.2f\n",
        wheel_target[0],
        wheel_target[1],
        wheel_target[2],
        vesc_target[0],
        targets[0],
        targets[1],
        targets[2],
        arm_and_loading_target[0],
        arm_and_loading_target[1],
        arm_and_loading_target[2],
        arm_and_loading_target[3]
    );
}
}  // namespace

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
    // CAN initialization
    can1_driver.init();
    fdcan2_driver.init();
    fdcan3_driver.init();

    // Motor configuration
    motor_config_wheel.set_max_duty_ratio(0.5f);
    motor_config_wheel.set_motor_type(gn10_can::devices::MotorType::C620);
    motor_config_wheel.set_encoder_type(gn10_can::devices::EncoderType::None);
    motor_config_arm.set_max_duty_ratio(0.5f);
    motor_config_arm.set_motor_type(gn10_can::devices::MotorType::C610);
    motor_config_arm.set_encoder_type(gn10_can::devices::EncoderType::None);

    motor_config_belt.set_motor_type(gn10_can::devices::MotorType::VESC);

    // Initialize devices on the network
    solenoid.set_init();
    power_manager.set_init(power_manager_config);
    HAL_Delay(1000);
    for (uint8_t i = 0; i < 4; i++) {
        esc_wheel.set_init(i, motor_config_wheel);
        esc_wheel.set_gains(i, 0.09f, 0.05f, 0.001f, 0.0f);
    }

    for (uint8_t i = 0; i < 3; i++) {
        esc_arm.set_init(i, motor_config_arm);
        esc_arm.set_gains(i, 0.05f, 0.0f, 0.0f, 0.0f);
    }

    // Initialize Ethernet
    ether.init();

    // controller command setup
    conversion.set_belt_vel_init(0.3f);
    conversion.set_belt_vel_adjust_value(0.1f);

    conversion.set_bucket_hight_value(10);
    conversion.set_bucket_limit_value(0, 110);

    conversion.set_wheel_max_vel(3.0f);
    conversion.set_angular_max_vel(3.0f);

    // System setup
    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    // Get latest teleop
    robot_config::teleop_t teleop;
    if (ether.receive_teleop(teleop)) {
        robot_config::command_t command;
        command = conversion.conversion(teleop);
        command_robot_drivers(command);
    }
    // Get latest belt angular velocity
    if (vesc_hub.get_feedbacks(vesc_feedbacks)) {
        start_encoder = true;
        loading       = true;
        serial_printf("1:%f\n", vesc_feedbacks[0]);
    }

    if (esc_loading.get_feedbacks(loading_feedbacks)) {
        serial_printf("loading_angle :%f\n", loading_feedbacks[3]);

        loading_check = true;

        if (loading_feedbacks[3] > k_loading_angle_tolerance) {
            loading_check = false;
        }
    }

    static bool last_start_encoder = false;  // 複数回初期化防止用

    if (start_encoder && !last_start_encoder) {
        // 装填機構init処理
        motor_config_loading.set_max_duty_ratio(0.5f);
        motor_config_loading.set_motor_type(gn10_can::devices::MotorType::C610);
        motor_config_loading.set_encoder_type(gn10_can::devices::EncoderType::IncrementalTotal);

        esc_loading.set_init(3, motor_config_loading);
        esc_loading.set_gains(3, 0.05f, 0.0f, 0.0f, 0.0f);
        loading_check      = false;
        last_start_encoder = start_encoder;
    }

    // ボタンが押されたら送る処理
    robot_config::debug_pc_t current_debug_pc = {};
    current_debug_pc.jetson_restart =
        (HAL_GPIO_ReadPin(operation_button1_GPIO_Port, operation_button1_Pin) == GPIO_PIN_SET);
    current_debug_pc.jetson_shutdown =
        (HAL_GPIO_ReadPin(operation_button2_GPIO_Port, operation_button2_Pin) == GPIO_PIN_SET);
    current_debug_pc.node_start =
        (HAL_GPIO_ReadPin(operation_button3_GPIO_Port, operation_button3_Pin == GPIO_PIN_SET));
    current_debug_pc.node_stop =
        (HAL_GPIO_ReadPin(operation_button4_GPIO_Port, operation_button4_Pin) == GPIO_PIN_SET);
    if (current_debug_pc.jetson_restart != prev_debug_pc.jetson_restart ||
        current_debug_pc.jetson_shutdown != prev_debug_pc.jetson_shutdown ||
        current_debug_pc.node_start != prev_debug_pc.node_start ||
        current_debug_pc.node_stop != prev_debug_pc.node_stop) {
        ether.send_pc_debug_data(current_debug_pc);
        prev_debug_pc = current_debug_pc;
    }

    // Basic System Process
    update_heartbeat_led();
    HAL_Delay(1);
}

// ---------------------------- C language's functions override ----------------------------------
extern "C" {
/**
 * @brief Receive callback for FDCAN FIFO0.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan1.Instance) {
        can1_bus.update();

    } else if (hfdcan->Instance == hfdcan2.Instance) {
        fdcan2_bus.update();

    } else if (hfdcan->Instance == hfdcan3.Instance) {
        fdcan3_bus.update();
        HAL_GPIO_TogglePin(LED_RAD_GPIO_Port, LED_RAD_Pin);
    }
}
}
