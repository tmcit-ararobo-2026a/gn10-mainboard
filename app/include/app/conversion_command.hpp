#include "app/robot_config.hpp"

class ConversionCommand
{
public:
    // stickはwheel周りで使う
    ConversionCommand();

    /*belt関連*/
    void set_belt_vel_init(float belt_vel_init);

    void set_belt_vel_adjust_value(float belt_vel_adjust_value);

    /*bucket関連*/
    void set_bucket_hight_value(uint8_t bucket_hight_value);

    void set_bucket_limit_value(int16_t bucket_limit_high, int16_t bucket_limit_low);

    /*wheel関連*/
    void set_wheel_max_vel(float wheel_max_vel);

    void set_angular_max_vel(float angular_max_vel);

    // 変換
    robot_config::command_t conversion(robot_config::teleop_t& teleop);

private:
    robot_config::teleop_t teleop_last_;
    robot_config::command_t command_;

    /* parameter */

    // belt
    float belt_vel_init_;
    float belt_vel_adjust_value_;
    float lever_degree_ofattenuation_ = 0.5;

    // bucket
    uint8_t bucket_hight_value_;
    int16_t bucket_limit_high_;
    int16_t bucket_limit_low_;

    // wheel
    float wheel_max_vel_;
    float angular_max_vel_;

    /* not parameter */
    float belt_vel_           = 0.0f;
    int16_t bucket_arm_hight_ = 0;
};