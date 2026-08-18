#include "gn10_mainboard/robot_config.hpp"

class ConversionCommand
{
public:
    ConversionCommand(
        uint8_t bucket_limit_h, uint8_t bucket_limit_l, uint8_t desk_limit_h, uint8_t desk_limit_l
    );

    void set_init_belt_vel(const uint8_t belt_init_vel);

    void set_bucket_move_value(const uint8_t bucket_move_value);

    void set_desk_move_value(const uint8_t desk_move_value);

    void set_belt_change_value(const uint8_t change_value);

    void set_belt_change_value_deep(const uint8_t change_value_d);

    robot_config::command_t conversion(robot_config::teleop_t& teleop);

private:
    robot_config::teleop_t teleop_last_;
    robot_config::command_t command_;

    // エアシリンダー射出先: 0=desk_l, 1=flag, 2=desk_r
    uint8_t air_rauncher_selector_ = 1;

    // constructor
    uint8_t bucket_limit_h_;
    uint8_t bucket_limit_l_;
    uint8_t desk_limit_h_;
    uint8_t desk_limit_l_;

    // seter
    uint8_t belt_vel_;
    uint8_t belt_init_vel_;
    uint8_t bucket_move_value_;
    uint8_t desk_move_value_;
    uint8_t belt_change_value_;
    uint8_t belt_change_value_d_;

    bool desk_pos      = false;
    bool desk_init_pos = false;

    //
    uint8_t bucket_arm_hight_ = 0;
};