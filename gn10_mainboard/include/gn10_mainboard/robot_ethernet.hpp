#pragma once
#include "gn10_mainboard/robot_data_config.hpp"

class RobotEthernet
{
private:
    uint8_t socket_cmd_    = 0;
    uint8_t socket_teleop_ = 1;
    uint8_t socket_debug_  = 2;

    union operation_data_u {
        robot_network_config::operation_data_t data;                   // 操作データ
        uint8_t code[sizeof(robot_network_config::operation_data_t)];  // 送信バイト配列
    } __attribute__((__packed__)) operation_union_;

    union feedback_data_u {
        robot_network_config::feedback_data_t data;
        uint8_t code[sizeof(robot_network_config::feedback_data_t)];
    } __attribute__((__packed__)) feedback_union_;

    union teleop_u {
        robot_network_config::teleop_t input;
        uint8_t data[sizeof(robot_network_config::teleop_t)];
    };

    union pc_debug_data_u {
        robot_network_config::pc_debug_t data;
        uint8_t code[sizeof(robot_network_config::pc_debug_t)];
    } __attribute__((__packed__)) debug_union_;

    union main_debug_data_u {
        robot_network_config::main_debug_t data;
        uint8_t code[sizeof(robot_network_config::main_debug_t)];
    } __attribute__((__packed__)) debug_union_;

public:
    bool init();
    void send_feedback_data(robot_network_config::feedback_data_t data);
    bool receive_operation_data(robot_network_config::operation_data_t& data);
    bool receive_teleop(robot_network_config::teleop_t& data);
    void send_pc_debug_data(robot_network_config::pc_debug_t data);
    bool receive_main_debug(robot_network_config::main_debug_t& data);
};
