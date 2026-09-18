
#include "app/app.hpp"

#include "app/serial_printf.hpp"
// esc-hub
#include "gn10_can/devices/launcher_client.hpp"
// htmd
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_can/devices/motor_driver_types.hpp"
// trinity deivice
#include "gn10_can/devices/solenoid_driver_client.hpp"
// candriver
#include "gn10_stm32_fdcan_driver/can_callback_helper.hpp"
#include "gn10_stm32_fdcan_driver/can_driver.hpp"
#include "gn10_stm32_fdcan_driver/fdcan_driver.hpp"

gn10_can::drivers::FDCANDriver fdcan3_driver(&hfdcan3);
gn10_can::FDCANBus fdcan3_bus(fdcan3_driver);

gn10_can::drivers::CANDriver can1_driver(&hfdcan1);
gn10_can::CANBus can1_bus(can1_driver);

gn10_can::devices::SolenoidDriverClient trinity_device(can1_bus, 0);
// led
constexpr uint32_t HEARTBEAT_TOGGLE_INTERVAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

//
gn10_can::devices::LauncherClient belt_launcher_client(fdcan3_bus, 0);

// htmd
gn10_can::devices::MotorDriverClient motor_client(can1_bus, 0);
gn10_can::devices::MotorConfig motor_config;

void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERVAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

void htmd_setup()
{
    motor_config.set_max_duty_ratio(0.75f);
    motor_config.set_motor_type(gn10_can::devices::MotorType::DC);
    motor_config.set_encoder_type(gn10_can::devices::EncoderType::None);
    motor_config.set_feedback_cycle(10);
    motor_client.set_init(motor_config);
}

void setup()
{
    fdcan3_driver.set_tx_timeout(2);
    fdcan3_driver.init();
    can1_driver.set_tx_timeout(2);
    can1_driver.init();
    belt_launcher_client.set_init();  // ここで1回だけ;
    htmd_setup();
}

void loop()
{
    float encoder_feedback_data;
    if (belt_launcher_client.get_velocity_feedback(encoder_feedback_data)) {
        serial_printf("%f\n", encoder_feedback_data);
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }

    motor_client.set_target(2.0f);
    update_heartbeat_led();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan3, fdcan3_bus, FDCAN_RX_FIFO0)) return;
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan3, fdcan3_bus, FDCAN_RX_FIFO1)) return;
}
