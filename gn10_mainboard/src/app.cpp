#include "gn10_mainboard/app.hpp"

#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "fdcan.h"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"

gn10_can::drivers::DriverSTM32FDCAN can1_driver(&hfdcan1);
gn10_can::CANBus can1_bus(can1_driver);
gn10_can::devices::MotorDriverClient motor1(can1_bus, 0);

gn10_can::devices::MotorConfig motor1_config;

float output = 0.0f;
float accel  = 0.001f;
bool sign    = true;

void setup()
{
    can1_driver.init();
    motor1_config.set_accel_ratio(1.0f);
    motor1_config.set_max_duty_ratio(1.0f);
    motor1.set_init(motor1_config);
}
void loop()
{
    if (sign) {
        output += accel;
    } else {
        output -= accel;
    }

    if (output > 1.0f) {
        sign = false;
    }
    if (output < -1.0f) {
        sign = true;
    }

    motor1.set_target(output);
    HAL_Delay(10);
}
extern "C" {
// C言語側の関数のオーバーライド
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    can1_bus.update();
}
}