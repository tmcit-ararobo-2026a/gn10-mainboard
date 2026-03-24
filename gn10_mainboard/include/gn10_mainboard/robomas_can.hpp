#pragma once
#include "gn10_can/core/can_bus.hpp"
// 定義
#define SEND_CANID 0x200

// M2006　M3508の制御フレーム（送信）
#define SEND_CANID_1_4 0x200
#define SEND_CANID_5_8 0x1FF

#define A_CONVERSION

// 受信したデータを格納する構造体
struct MotorFeedback {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temp;
    uint8_t spare;
} __attribute__((__packed__));

// motorに送る電流値を格納する配列
struct MotorCurrent {
    int16_t current[4];
} __attribute__((__packed__));

namespace robomaster {
/**
 * @brief C620/C610とのCAN通信用クラス
 */
class RobomasCAN
{
private:
    gn10_can::CANBus& bus_;
    gn10_can::CANFrame frame;
    MotorCurrent motor_current_[2];

public:
    MotorFeedback feedback[8];
    /**
     * @brief C620/C610とのCAN通信用クラスのコンストラクタ
     */
    RobomasCAN(gn10_can::CANBus& bus_);

    /**
     * @brief escにデータを送る関数
     * @details これはsend_current関数に組み込まれているので気にしなくていいです
     */
    void send_data(uint16_t can_id, uint8_t* data);

    /**
     * @brief escからのfeedbackを受け取る関数
     * @param can_id feedback先のESCのid
     * @param data うけっとたデータ
     */
    void receive_data(uint16_t can_id, uint8_t data[8]);

    /**
     * @brief 受け取ったデータを処理する関数です
     * @details これはreceive_dataに組み込まれています。受け取ったデータを処理したいなら
     * この関数に記載してください。
     *
     */
    void process_data(MotorFeedback* feedback);

    /**
     * @brief 目標の電流値を設定し、ESCに送信する
     * @param group_num 0 : (motor1-4) 1 : (motor5-8)
     * @param M3508_current M3508の場合　最大:20 最小20
     * @param M2006_current M2006の場合　最大:10 最小10
     */
    void send_current(
        uint8_t group_num,
        int16_t motor1_current,
        int16_t motor2_current,
        int16_t motor3_current,
        int16_t motor4_current
    );
};
}  // namespace robomaster
