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
#include "gn10_mainboard/fdcan_driver.hpp"
#include "gn10_mainboard/pid.hpp"
#include "gn10_mainboard/robot_data_config.hpp"
#include "gn10_mainboard/serial_printf.hpp"
// others

namespace {

constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;

uint32_t heartbeat_last_toggle_time_ms = 0;
// Retained Data
operation_data_t operation;
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
        /* serial_printf(
            "f:%f,bl:%f,br:%f\n",
            operation.wheel_front,
            operation.wheel_back_left,
            operation.wheel_back_right
        ); */
        // serial_printf("%f\n", vesc_velocities_feedbacks[0]);
    }
}

// Device Configuration
gn10_can::devices::power_manager::Config power_manager_config;
gn10_can::devices::MotorConfig motor_config_collect;
gn10_can::devices::MotorConfig motor_config_wheel;
gn10_can::devices::MotorConfig motor_config_arm;
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
gn10_can::devices::RobotControlHubServer<operation_data_t, feedback_data_t> robot_control_hub(
    fdcan2_bus, 0
);
gn10_can::devices::PowerManagerClient power_manager(fdcan2_bus, 0);
gn10_can::devices::ESCHubClient vesc_hub(fdcan3_bus, 0);
gn10_can::devices::ESCHubClient esc_wheel(fdcan3_bus, 1);
gn10_can::devices::ESCHubClient esc_arm(fdcan3_bus, 2);
gn10_can::devices::ESCHubClient desk_arm(fdcan3_bus, 3);

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
        desk_arm.set_init(i, motor_config_arm);
        desk_arm.set_gains(i, 0.05f, 0.0f, 0.0f, 0.0f);
    }

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
    }

    // Set speed to ESC Hub for wheels
    float wheel_angular_velocites[4];
    wheel_angular_velocites[0] = operation.wheel_front * 19.0f;
    wheel_angular_velocites[1] = operation.wheel_back_left * 19.0f;
    wheel_angular_velocites[2] = operation.wheel_back_right * 19.0f;
    wheel_angular_velocites[3] = 0.0f;
    esc_wheel.set_angular_velocities(wheel_angular_velocites);

    // Control the dust cloths collector
    if (operation.collect) {
        motor_collect.set_target(1.0f);
    } else {
        motor_collect.set_target(0.0f);
    }

    // Control the belt-type injection
    float vesc_vel = 0.0f;
    if (operation.belt_throw) {
        vesc_vel = (float)operation.belt_velocity;
    } else {
        vesc_vel = 0.0f;
    }
    float vesc_velocities[4] = {vesc_vel, 0.0f, 0.0f, 0.0f};
    vesc_hub.set_angular_velocities(vesc_velocities);

    // Get latest belt angular velocity
    if (esc_wheel.get_angular_velocity_feedbacks(vesc_velocities_feedbacks)) {
    }

    // Control the air-type injection
    std::array<bool, 8> targets{};
    for (size_t i = 0; i < 8; i++) {
        targets[i] = operation.air_throw;
    }
    solenoid.set_target(targets);

    // Control the arm with C610
    float arm_velocities[4];
    arm_velocities[0] = operation.arm_horizontal * 200.0f;
    arm_velocities[1] = operation.arm_vertical * 200.0f;
    if (operation.arm_hold) {
        arm_velocities[2] = 200.0f;
    } else {
        arm_velocities[2] = 0.0f;
    }
    arm_velocities[3] = 0.0f;
    esc_arm.set_angular_velocities(arm_velocities);

    float desk_arm_velocities[4];
    arm_velocities[0] = operation.desk_depth * 200.0f;
    arm_velocities[1] = operation.desk_lift * 200.0f;
    arm_velocities[2] = operation.desk_finger * 200.0f;
    arm_velocities[3] = 0.0f;
    desk_arm.set_angular_velocities(desk_arm_velocities);
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