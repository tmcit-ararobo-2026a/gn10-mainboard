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

uint32_t heartbeat_last_toggle_time_ms = 0;
// Retained Data
robot_config::command_t operation;
float vesc_velocities_feedbacks[4];

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
gn10_can::devices::MotorConfig motor_config_collect;
gn10_can::devices::MotorConfig motor_config_wheel;
gn10_can::devices::MotorConfig motor_config_arm;
gn10_can::devices::MotorConfig motor_config_belt;
// CAN Drivers
gn10_can::drivers::CANDriver can1_driver(&hfdcan1);
gn10_can::drivers::FDCANDriver fdcan2_driver(&hfdcan2);
gn10_can::drivers::FDCANDriver fdcan3_driver(&hfdcan3);
// CAN Bus
gn10_can::CANBus can1_bus(can1_driver);
gn10_can::FDCANBus fdcan2_bus(fdcan2_driver);
gn10_can::FDCANBus fdcan3_bus(fdcan3_driver);
// CAN Devices
gn10_can::devices::MotorDriverClient motor_collect(can1_bus, 0);
gn10_can::devices::SolenoidDriverClient solenoid(can1_bus, 0);
gn10_can::devices::RobotControlHubServer<robot_config::command_t, robot_config::feedback_t>
    robot_control_hub(fdcan2_bus, 0);
gn10_can::devices::PowerManagerClient power_manager(fdcan2_bus, 0);
gn10_can::devices::ESCHubClient vesc_hub(fdcan3_bus, 0);
gn10_can::devices::ESCHubClient esc_wheel(fdcan3_bus, 1);
gn10_can::devices::ESCHubClient esc_arm(fdcan3_bus, 2);

// Ethernet
RobotEthernet ether;
robot_config::debug_pc_t prev_debug_pc = {};
// belt
bool initilized_belt = false;
float vesc_vel       = 0.0f;
// Inverse Kinematics
ThreeWheelOmni omni(0.5f, 0.1f);
constexpr float M3508_GEAR_RATIO = 19.0f;
// Controller conversion
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
    motor_config_collect.set_accel_ratio(1.0f);
    motor_config_collect.set_max_duty_ratio(1.0f);

    motor_config_wheel.set_max_duty_ratio(0.5f);
    motor_config_wheel.set_motor_type(gn10_can::devices::MotorType::C620);
    motor_config_arm.set_max_duty_ratio(0.5f);
    motor_config_arm.set_motor_type(gn10_can::devices::MotorType::C610);

    motor_config_belt.set_motor_type(gn10_can::devices::MotorType::VESC);

    // Initialize devices on the network
    motor_collect.set_init(motor_config_collect);
    solenoid.set_init();
    power_manager.set_init(power_manager_config);
    HAL_Delay(1000);
    for (uint8_t i = 0; i < 4; i++) {
        esc_wheel.set_init(i, motor_config_wheel);
        esc_wheel.set_gains(i, 0.05f, 0.0f, 0.0f, 0.0f);
        esc_arm.set_init(i, motor_config_arm);
        esc_arm.set_gains(i, 0.05f, 0.0f, 0.0f, 0.0f);
    }

    // Initialize Ethernet
    ether.init();

    // System setup
    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    // Get latest command from the Jetson
    if (robot_control_hub.get_command(operation)) {
        // ether.receive_operation_data(operation);
    }

    // Set speed to ESC Hub for wheels
    omni.convert(operation.x_vel, operation.y_vel, operation.angular_vel, 0.0f);
    float front, right, left;
    omni.getWheelAngularVelocity(&front, &left, &right);
    float wheel_angular_velocites[4];
    wheel_angular_velocites[0] = front * M3508_GEAR_RATIO;
    wheel_angular_velocites[1] = left * M3508_GEAR_RATIO;
    wheel_angular_velocites[2] = right * M3508_GEAR_RATIO;
    wheel_angular_velocites[3] = 0.0f;
    esc_wheel.set_angular_velocities(wheel_angular_velocites);

    // Control the belt-type injection
    if (operation.belt_init) {
        initilized_belt = true;
        vesc_hub.set_init(0, motor_config_belt);
    }

    if (operation.belt_throw && initilized_belt) {
        vesc_vel = (float)operation.belt_vel;
    } else {
        vesc_vel = 0.0f;
    }

    float vesc_velocities[4] = {vesc_vel, 0.0f, 0.0f, 0.0f};
    vesc_hub.set_angular_velocities(vesc_velocities);

    // Get latest belt angular velocity
    if (vesc_hub.get_angular_velocity_feedbacks(vesc_velocities_feedbacks)) {
        serial_printf("1:%f\n", vesc_velocities_feedbacks[0]);
        serial_printf("2:%f\n", vesc_velocities_feedbacks[1]);
        serial_printf("3:%f\n", vesc_velocities_feedbacks[2]);
        serial_printf("4:%f\n", vesc_velocities_feedbacks[3]);
        serial_printf("\n");
    }

    // Control the air-type injection
    std::array<bool, 8> targets{};
    targets[0] = operation.air_rauncher_for_flag;
    targets[1] = operation.air_rauncher_for_desk_r;
    targets[2] = operation.air_rauncher_for_desk_l;
    solenoid.set_target(targets);

    // Control the arm with C610
    float arm_velocities[4];
    esc_arm.set_angular_velocities(arm_velocities);

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
