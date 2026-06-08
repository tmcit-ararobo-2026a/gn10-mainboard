#include "gn10_mainboard/app.hpp"

#include <cmath>

#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "fdcan.h"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_can/devices/power_manager_client.hpp"
#include "gn10_can/devices/robot_control_hub_server.hpp"
#include "gn10_can/devices/servo_motor_client.hpp"
#include "gn10_mainboard/fdcan_driver.hpp"
#include "gn10_mainboard/four_wheel_omni.hpp"
#include "gn10_mainboard/pid.hpp"
#include "gn10_mainboard/vesc_can.hpp"
#include "robomas_can/c610_can.hpp"
#include "robomas_can/c620_can.hpp"
#include "wiznet_ether/robot_ethernet.hpp"
#include "wiznet_ether/serial_printf.hpp"
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

}  // namespace

gn10_can::drivers::DriverSTM32FDCAN can1_driver(&hfdcan1);
gn10_can::drivers::FDCANDriver fdcan2_driver(&hfdcan2);
// gn10_can::drivers::DriverSTM32FDCAN can3_driver(&hfdcan3);

gn10_can::FDCANBus fdcan2_bus(fdcan2_driver);
// gn10_can::CANBus can3_bus(can3_driver);

robomas_can::C620CAN wheel_esc(can1_driver);
gn10_can::CANBus can1_bus(can1_driver);

gn10_can::devices::MotorDriverClient motor(can1_bus, 0);

// gn10_can::devices::ServoMotorClient servo_motor(can3_bus, 0);
gn10_can::devices::RobotControlHubServer<operation_data_t, feedback_data_t> robot_control_hub(
    fdcan2_bus, 0
);
gn10_can::devices::PowerManagerClient power_manager(fdcan2_bus, 0);
gn10_can::devices::power_manager::Config power_manager_config;
gn10_can::devices::MotorConfig motor_config;

FourWheelOmni omni(0.3f, 0.065f);
VescCAN vesc;

gn10_motor::PIDConfig<float> pid_config_wheel_f;
gn10_motor::PIDConfig<float> pid_config_wheel_bl;
gn10_motor::PIDConfig<float> pid_config_wheel_br;

gn10_motor::PID<float> pid_wheel_f(pid_config_wheel_f);
gn10_motor::PID<float> pid_wheel_bl(pid_config_wheel_bl);
gn10_motor::PID<float> pid_wheel_br(pid_config_wheel_br);

operation_data_t operation;
float wheel_angular_velocity_f  = 0.0f;
float wheel_angular_velocity_bl = 0.0f;
float wheel_angular_velocity_br = 0.0f;

float wheel_angular_velocity_f_feedback  = 0.0f;
float wheel_angular_velocity_bl_feedback = 0.0f;
float wheel_angular_velocity_br_feedback = 0.0f;

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
    can1_driver.init();
    fdcan2_driver.init();
    vesc.init();

    motor_config.set_accel_ratio(1.0f);
    motor_config.set_max_duty_ratio(1.0f);
    motor.set_init(motor_config);
    // can3_driver.init();

    pid_config_wheel_f.kp           = 0.5f;
    pid_config_wheel_f.ki           = 0.0f;
    pid_config_wheel_f.kd           = 0.0f;
    pid_config_wheel_f.output_limit = 20.0f;
    pid_wheel_f.update_config(pid_config_wheel_f);
    pid_config_wheel_bl.kp           = 0.5f;
    pid_config_wheel_bl.ki           = 0.0f;
    pid_config_wheel_bl.kd           = 0.0f;
    pid_config_wheel_bl.output_limit = 20.0f;
    pid_wheel_bl.update_config(pid_config_wheel_bl);
    pid_config_wheel_br.kp           = 0.5f;
    pid_config_wheel_br.ki           = 0.0f;
    pid_config_wheel_br.kd           = 0.0f;
    pid_config_wheel_br.output_limit = 20.0f;
    pid_wheel_br.update_config(pid_config_wheel_br);

    // servo_motor.set_init(1000, 1200);
    power_manager.set_init(power_manager_config);
    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    if (robot_control_hub.get_command(operation)) {
    }
    wheel_angular_velocity_f  = operation.wheel_front;
    wheel_angular_velocity_bl = operation.wheel_back_left;
    wheel_angular_velocity_br = operation.wheel_back_right;

    wheel_angular_velocity_f_feedback =
        2.0f * 3.1415f * (float)wheel_esc.get_feedback_speed(0) / 60.0f / 19.0f;
    wheel_angular_velocity_bl_feedback =
        2.0f * 3.1415f * (float)wheel_esc.get_feedback_speed(1) / 60.0f / 19.0f;
    wheel_angular_velocity_br_feedback =
        2.0f * 3.1415f * (float)wheel_esc.get_feedback_speed(2) / 60.0f / 19.0f;

    float wheel_currents[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    /*
    if (operation.arm_horizontal) {
        c610_current = 10000;
    } else {
        c610_current = 0.0f;
    }*/

    if (operation.belt_throw) {
        // servo_motor.set_angle_rad(M_PI * operation.belt_velocity);
        vesc.comm_can_set_current(43, -1.2f);
        vesc.comm_can_set_duty(43, -1.2f);

    } else {
        vesc.comm_can_set_current(43, 0.0f);
        vesc.comm_can_set_duty(43, 0.0f);

        //  servo_motor.set_angle_rad(0);
    }

    if (operation.arm_horizontal || operation.arm_vertical) {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }

    wheel_currents[0] =
        pid_wheel_f.update(wheel_angular_velocity_f, wheel_angular_velocity_f_feedback, 0.001f);
    wheel_currents[1] =
        pid_wheel_bl.update(wheel_angular_velocity_bl, wheel_angular_velocity_bl_feedback, 0.001f);
    wheel_currents[2] =
        pid_wheel_br.update(wheel_angular_velocity_br, wheel_angular_velocity_br_feedback, 0.001f);

    wheel_esc.set_current_can1(
        wheel_currents[0], wheel_currents[1], wheel_currents[2], wheel_currents[3]
    );

    if (operation.collect) {
        motor.set_target(1.0f);
    } else {
        motor.set_target(0.0f);
    }

    update_heartbeat_led();
    HAL_Delay(1);
}
extern "C" {
// C言語側の関数のオーバーライド
/**
 * @brief Receive callback for FDCAN FIFO0.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan1.Instance) {
        gn10_can::CANFrame rx_frame;
        can1_driver.receive(rx_frame);
        wheel_esc.receive_data(rx_frame.id, rx_frame.data.data());
    } else if (hfdcan->Instance == hfdcan2.Instance) {
        fdcan2_bus.update();
    } else if (hfdcan->Instance == hfdcan3.Instance) {
        // can3_bus.update();
    }
}
}