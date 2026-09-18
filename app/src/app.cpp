
#include "app/app.hpp"

#include "app/serial_printf.hpp"
#include "gn10_can/devices/launcher_client.hpp"
#include "gn10_stm32_fdcan_driver/can_callback_helper.hpp"
#include "gn10_stm32_fdcan_driver/fdcan_driver.hpp"

gn10_can::drivers::FDCANDriver fdcan3_driver(&hfdcan3);
gn10_can::FDCANBus fdcan3_bus(fdcan3_driver);

// led
constexpr uint32_t HEARTBEAT_TOGGLE_INTERVAL_MS = 500;
uint32_t heartbeat_last_toggle_time_ms          = 0;

gn10_can::devices::LauncherClient belt_launcher_client(fdcan3_bus, 0);

void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERVAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

void setup()
{
    fdcan3_driver.set_tx_timeout(2);
    fdcan3_driver.init();
    belt_launcher_client.set_init();  // ここで1回だけ
}

void loop()
{
    float encoder_feedback_data;
    if (belt_launcher_client.get_velocity_feedback(encoder_feedback_data)) {
        serial_printf("%f\n", encoder_feedback_data);
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    }
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
