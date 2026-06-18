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
gn10_can::devices::MotorConfig motor_config;
// CAN Drivers
gn10_can::drivers::CANDriver can1_driver(&hfdcan1);
gn10_can::drivers::FDCANDriver fdcan2_driver(&hfdcan2);
gn10_can::drivers::FDCANDriver fdcan3_driver(&hfdcan3);
// CAN Bus
gn10_can::CANBus can1_bus(can1_driver);
gn10_can::FDCANBus fdcan2_bus(fdcan2_driver);
gn10_can::FDCANBus fdcan3_bus(fdcan3_driver);
// CAN Devices
gn10_can::devices::MotorDriverClient motor(can1_bus, 0);
gn10_can::devices::SolenoidDriverClient solenoid(can1_bus, 0);
gn10_can::devices::RobotControlHubServer<operation_data_t, feedback_data_t> robot_control_hub(
    fdcan2_bus, 0
);
gn10_can::devices::PowerManagerClient power_manager(fdcan2_bus, 0);
gn10_can::devices::ESCHubClient esc_hub(fdcan3_bus, 0);
gn10_can::devices::ESCHubClient wheel_esc(fdcan3_bus, 1);

// Retained Data
operation_data_t operation;

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
    motor_config.set_accel_ratio(1.0f);
    motor_config.set_max_duty_ratio(1.0f);

    // Initialize devices on the network
    motor.set_init(motor_config);
    solenoid.set_init();
    power_manager.set_init(power_manager_config);

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
    wheel_angular_velocites[0] = operation.wheel_front;
    wheel_angular_velocites[1] = operation.wheel_back_left;
    wheel_angular_velocites[2] = operation.wheel_back_right;
    wheel_angular_velocites[3] = 0.0f;
    wheel_esc.set_angular_velocities(wheel_angular_velocites);

    // Control the dust cloths collector
    if (operation.collect) {
        motor.set_target(1.0f);
    } else {
        motor.set_target(0.0f);
    }

    // Control the belt-type injection
    float vesc_vel = 0.0f;
    if (operation.belt_throw) {
        vesc_vel = (float)operation.belt_velocity;
    } else {
        vesc_vel = 0.0f;
    }
    float vesc_velocities[4] = {vesc_vel, 0.0f, 0.0f, 0.0f};
    esc_hub.set_angular_velocities(vesc_velocities);

    // Get latest belt angular velocity
    float vesc_velocities_feedbacks[4];
    if (esc_hub.get_angular_velocity_feedbacks(vesc_velocities_feedbacks)) {
        serial_printf("%f\n", vesc_velocities_feedbacks[0]);
    }

    // Control the air-type injection
    std::array<bool, 8> targets{};
    for (size_t i = 0; i < 8; i++) {
        targets[i] = operation.air_throw;
    }
    solenoid.set_target(targets);

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