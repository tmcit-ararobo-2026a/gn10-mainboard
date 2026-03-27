#pragma once
#include "gn10_can/core/can_bus.hpp"

namespace robomas_can {

// robomas共通の送信CANid
#define SEND_CANID 0x200

// M2006　M3508の制御フレーム（送信）
#define SEND_CANID_1_4 0x200
#define SEND_CANID_5_8 0x1FF

// motorに送る電流値を格納する配列
struct MotorCurrent {
    int16_t current[4];
} __attribute__((__packed__));

// c610とc620に対応するための基底クラス
class RobomasCAN
{
private:
    gn10_can::CANBus& bus_;
    gn10_can::CANFrame frame_;
    uint8_t cun_bus_data_[8];
    MotorCurrent motor_current_[2];

public:
    /**
     * @brief CAN通信用クラスのコンストラクタ
     */
    RobomasCAN(gn10_can::CANBus& bus, float current_conversion, float max_current);

    virtual ~RobomasCAN() = default;

    /**
     * @brief MotorCurrent用のget関数
     *
     * @param motor_number 何番目のロボマスモーターか判断する。
     * 値域は1-8。
     * @return 成功：設定された電流値
     * @return 失敗：０
     */
    int16_t get_current(uint8_t motor_number) const;

    /**
     * @brief MotorCurrent用のsetter関数　電流値の値を代入します
     *
     * @param　motor_number 何話目のロボマスモーターか
     * @param current_value 電流の値。
     */
    void set_current(uint8_t motor_number, float current_value);

    /**
     * @brief escからのfeedbackを受け取る関数
     * @param can_id feedback先のESCのid
     * @param data 受け取ったデータ
     */
    virtual void receive_data(uint16_t can_id, uint8_t data[8]) = 0;

    /**
     * @brief escにデータを送る関数
     * @details get_currentで取得した電流値を送信します。
     */
    void send_data(uint16_t can_id, uint8_t* data);
};
}  // namespace robomas_can
