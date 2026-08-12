/**
 * @file robot_data_config.hpp
 * @author tmcit-ararobo-2026a
 * @brief ロボットの通信データ構造体定義
 * @version 2.1
 * @date 2025-10-03
 *
 * @copyright Copyright (c) 2025
 *
 * socket_cmd (port:26574)
 *  |-  operation    pc          ->  main-board
 *  |-  feedback     main-board  ->  pc
 *
 * socket_teleop (port:10410)
 *  |-  teleop            robo-con    ->  pc ( main (in debug_mode))
 *
 * socket_debug (port:39244)
 *  |-  pc_debug          main-board  ->  pc
 *  |-  main_debug        pc          ->  main-board
 */
#pragma once
#include <stdint.h>

namespace robot_config {

namespace header {
constexpr uint8_t operation  = 0xAB;
constexpr uint8_t feedback   = 0x55;
constexpr uint8_t teleop     = 0xAA;
constexpr uint8_t pc_debug   = 0x38;
constexpr uint8_t main_debug = 0x2A;
}  // namespace header

namespace port {
constexpr uint16_t cmd    = 26574;
constexpr uint16_t teleop = 10410;
constexpr uint16_t debug  = 39244;
}  // namespace port

namespace ip {
constexpr uint8_t mainboard[] = {192, 168, 3, 2};
constexpr uint8_t pc_robot[]  = {192, 168, 3, 1};
constexpr uint8_t pc_wifi[]   = {192, 168, 2, 1};
constexpr uint8_t teleop[]    = {192, 168, 2, 2};
}  // namespace ip

/**
 * @brief ロボットの動作司令値 32byte
 *
 */
struct operation_t {
    // 識別ヘッダー 1byte
    uint8_t header;
    // 足回り 12byte
    float x_vel;        //[m/s]
    float y_vel;        //[m/s]
    float angular_vel;  //[rad/s]
    // バケツ用アーム 2byte
    uint8_t bucket_arm_hight;  //[cm]
    uint8_t bucket_arm_hold;
    // 装填機構 2byte
    uint8_t loading_hook_pos;  //[phase]
    bool loading_shift_cloth;
    // ベルト直動 6byte
    float belt_vel;  //[m/s]
    bool belt_throw;
    bool belt_init;
    // エアシリンダー射出 3byte
    bool air_rauncher_for_flag;
    bool air_rauncher_for_desk_r;
    bool air_rauncher_for_desk_l;
    // 机上雑巾回収 2byte
    uint8_t desk_arm_pos;  //[cm]
    bool desk_arm_tip_angle;
    // 状態表示 2byte
    int8_t target_bucket_angle_roll;
    int8_t target_bucket_angle_pitch;
    // 予備 2byte
    uint8_t reserved[2];
} __attribute__((__packed__));

union operation_u {
    robot_config::operation_t value;                    // 操作データ
    uint8_t binary[sizeof(robot_config::operation_t)];  // 送信バイト配列
} __attribute__((__packed__));

static_assert(sizeof(operation_t) == 32);

/**
 * @brief ロボットのセンサ値などのフィードバック
 *
 */
struct feedback_t {
    uint8_t header;  // ヘッダー
} __attribute__((__packed__));

union feedback_u {
    robot_config::feedback_t value;
    uint8_t binary[sizeof(robot_config::feedback_t)];
} __attribute__((__packed__));

static_assert(sizeof(feedback_t) == 1);

/**
 * @brief 操縦デバイスのレバーの傾きと押し込み
 *
 */
enum class LeverPosition : uint8_t {
    FRONT,
    RIGHT,
    RIGHT_DEEP,
    LEFT,
    LEFT_DEEP,
    PUSH,
};

/**
 * @brief ロボットの操縦信号値
 *
 */
struct teleop_t {
    uint8_t header;  // 認識番号

    struct {
        int8_t stick_right[2];             // 0:x, 1:y
        int8_t stick_left[2];              // 0:x, 1:y
    } __attribute__((__packed__)) analog;  // 4byte

    struct {
        LeverPosition lever_right : 3;
        LeverPosition lever_left  : 3;
        uint8_t stick_push_right  : 1;
        uint8_t stick_push_left   : 1;
        uint8_t up                : 1;
        uint8_t down              : 1;
        uint8_t right             : 1;
        uint8_t left              : 1;
        uint8_t circle            : 1;
        uint8_t cross             : 1;
        uint8_t triangle          : 1;
        uint8_t reserved          : 1;
    } __attribute__((__packed__)) buttons;  // 2byte

    /**
     * checksum以外を除いた7Byteの和の補数
     * ただし計算結果の8bitより大きい値は切り捨て
     */
    uint8_t data_checksum;
} __attribute__((__packed__));

union teleop_u {
    robot_config::teleop_t value;
    uint8_t binary[sizeof(robot_config::teleop_t)];
} __attribute__((__packed__));

static_assert(sizeof(teleop_t) == 8);

/**
 * @brief PCのデバッグ用通信（再起動やスクリプト開始など）
 *
 */
struct debug_pc_t {
    uint8_t header;  // ヘッダー
    bool jetson_restart;
} __attribute__((__packed__));

union debug_pc_u {
    robot_config::debug_pc_t value;
    uint8_t binary[sizeof(robot_config::debug_pc_t)];
} __attribute__((__packed__));

/**
 * @brief メイン基板(RobotControlHub)のデバッグ用通信（制御モード切り替えなど）
 *
 */
struct debug_main_t {
    uint8_t header;
} __attribute__((__packed__));

union debug_main_u {
    robot_config::debug_main_t value;
    uint8_t binary[sizeof(robot_config::debug_main_t)];
} __attribute__((__packed__));

}  // namespace robot_config