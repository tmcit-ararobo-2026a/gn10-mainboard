#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/core/can_device.hpp"

class RobomasCAN : public gn10_can::CANDevice
{
public:
    /**
     * @brief ロボマスモーター用コンストラクタ
     *
     * @param bus CANBusクラスの参照
     * @param dev_id デバイスID
     */
    RobomasCAN(gn10_can::CANBus& bus, uint8_t dev_id);
};