#include "gn10_mainboard/robot_config.hpp"

class ConversionCommand
{
public:
    // stickはwheel周りで使う
    ConversionCommand(int8_t stick_max_value_high, int8_t stick_max_value_low);

    /*belt関連*/
    void set_init_belt_vel(const uint8_t belt_init_vel);

    void set_belt_change_value(const uint8_t change_value);

    void set_belt_change_value_deep(const uint8_t change_value_d);

    /*bucket関連*/
    void set_bucket_move_value(const uint8_t bucket_move_value);

    void set_bucket_limit_value(const int16_t bucket_limit_high, const int16_t bucket_limit_low);

    /*desk関連*/
    void set_desk_move_value(const uint8_t desk_move_value);

    void set_desk_limit_value(const int16_t desk_limit_high, const int16_t desk_limit_low);

    /*wheel関連*/
    void set_wheel_max_vel(const float max_wheel_vel);

    void set_max_angular_vel(const float max_angular_vel);

    // 変換
    robot_config::command_t conversion(robot_config::teleop_t& teleop);

private:
    robot_config::teleop_t teleop_last_;
    robot_config::command_t command_;

    // エアシリンダー射出先: 0=desk_l, 1=flag, 2=desk_r
    uint8_t air_rauncher_selector_ = 1;

    /*const*/

    // belt
    uint8_t belt_init_vel_;
    uint8_t belt_change_value_;
    uint8_t belt_change_value_deep_;

    // bucket
    uint8_t bucket_high_value_;
    int16_t bucket_limit_high_;
    int16_t bucket_limit_low_;

    // desk
    int16_t desk_move_value_;
    int16_t desk_limit_high_;
    int16_t desk_limit_low_;

    // wheel
    float max_wheel_vel_;
    float max_angular_vel_;
    float stick_max_value_high_;
    float stick_max_value_low_;

    // not const
    uint8_t belt_vel_         = 0;
    int16_t bucket_arm_hight_ = 0;
    int16_t desk_pos_         = 0;
    bool desk_flag            = false;
};