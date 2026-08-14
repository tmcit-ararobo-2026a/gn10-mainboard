#include "gn10_mainboard/robot_config.hpp"

class ConversionCommand
{
public:
    robot_config::command_t conversion(robot_config::teleop_t teleop);

private:
    robot_config::teleop_t teleop_;
    robot_config::command_t command_;
    uint8_t count    = 1;  // エアシリンダー射出先: 0=desk_l, 1=flag, 2=desk_r
    bool circle_last = false;
};