#pragma once
#include "app/robot_config.hpp"

class RobotEthernet
{
private:
    uint8_t socket_command_   = 0;
    uint8_t socket_operation_ = 1;
    uint8_t socket_teleop_    = 2;

public:
    bool init();
    bool send_command_data(const robot_config::command_t& data);
    bool receive_operation_data(robot_config::operation_t& data);
    bool send_feedback_data(const robot_config::feedback_t& data);
    bool receive_teleop(robot_config::teleop_t& data);
};
