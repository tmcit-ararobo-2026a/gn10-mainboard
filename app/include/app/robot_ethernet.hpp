#pragma once
#include "app/robot_config.hpp"

class RobotEthernet
{
private:
    uint8_t socket_cmd_    = 0;
    uint8_t socket_teleop_ = 1;
    uint8_t socket_debug_  = 2;

public:
    bool init();
    bool receive_operation_data(robot_config::command_t& data);
    bool send_feedback_data(const robot_config::feedback_t& data);
    bool receive_teleop(robot_config::teleop_t& data);
    bool send_pc_debug_data(const robot_config::debug_pc_t& data);
    bool receive_main_debug(robot_config::debug_main_t& data);
};
