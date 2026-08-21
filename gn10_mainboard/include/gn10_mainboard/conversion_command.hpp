#include "gn10_mainboard/robot_config.hpp"

class ConversionCommand
{
public:
    // stickはwheel周りで使う
    ConversionCommand();

    /*belt関連*/
    void set_belt_vel_init(float belt_vel_init);

    void set_belt_vel_adjust_value(float belt_vel_adjust_value);

    // 初期値0.5
    void set_lever_degree_ofattenuation(float lever_degree_ofattenuation);

    /*bucket関連*/
    void set_bucket_hight_value(uint8_t bucket_hight_value);

    void set_bucket_limit_value(int16_t bucket_limit_high, int16_t bucket_limit_low);

    /*desk関連*/
    void set_desk_move_value(uint8_t desk_move_value);

    void set_desk_limit_value(int16_t desk_limit_high, int16_t desk_limit_low);

    /*wheel関連*/
    void set_wheel_max_vel(float wheel_max_vel);

    void set_angular_max_vel(float angular_max_vel);

    // 変換
    robot_config::command_t conversion(robot_config::teleop_t& teleop);

private:
    robot_config::teleop_t teleop_last_;
    robot_config::command_t command_;

    // エアシリンダー射出先: 0=desk_l, 1=flag, 2=desk_r
    uint8_t air_rauncher_selector_ = 1;

    /* parameter */

    // belt
    float belt_vel_init_;
    float belt_vel_adjust_value_;
    float lever_degree_ofattenuation_ = 0.5;

    // bucket
    uint8_t bucket_hight_value_;
    int16_t bucket_limit_high_;
    int16_t bucket_limit_low_;

    // desk
    int16_t desk_draw_in_value_;
    int16_t desk_limit_high_;
    int16_t desk_limit_low_;

    // wheel
    float wheel_max_vel_;
    float angular_max_vel_;

    /* not parameter */
    uint8_t belt_vel_         = 0;
    int16_t bucket_arm_hight_ = 0;
    int16_t desk_pos_         = 0;
    bool desk_flag            = false;
};