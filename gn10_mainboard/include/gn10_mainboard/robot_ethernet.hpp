#pragma once
#include "gn10_mainboard/robot_data_config.hpp"

class RobotEthernet
{
private:
    uint8_t socket_operation_ = 0;  // 操作用ソケット
    uint8_t socket_feedback_  = 1;  // フィードバック用ソケット

    union operation_data_union_t {
        operation_data_t data;                   // 操作データ
        uint8_t code[sizeof(operation_data_t)];  // 送信バイト配列
    } __attribute__((__packed__)) operation_union_;

    union feedback_data_union_t {
        feedback_data_t data;
        uint8_t code[sizeof(feedback_data_t)];
    } __attribute__((__packed__)) feedback_union_;

public:
    bool init();
    void send_feedback_data(feedback_data_t data);
    bool receive_operation_data(operation_data_t& data);
};
