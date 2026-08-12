/**
 * @file robot_data_config.hpp
 * @author aiba-gento
 * @brief ロボットの通信データ構造体定義
 * @version 2.1
 * @date 2025-10-03
 *
 * @copyright Copyright (c) 2025
 *
 * socket_cmd (port:26574)
 *  |-  operation_data    pc          ->  main-board
 *  |-  feedback_data     main-board  ->  pc
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

namespace robot_network_config {

namespace header {
constexpr uint16_t operation_data = 0xAB36;
constexpr uint16_t feedback_data  = 0x554A;
constexpr uint8_t teleop_data     = 0x15A5;
constexpr uint16_t pc_debug       = 0x38F7;
constexpr uint16_t main_debug     = 0x2A84;
}  // namespace header

namespace port {
constexpr uint16_t cmd    = 26574;
constexpr uint16_t teleop = 10410;
constexpr uint16_t debug  = 39244;
}  // namespace port

namespace ip {
constexpr uint8_t mainboard[] = {192, 168, 1, 2};
constexpr uint8_t pc_robot[]  = {192, 168, 1, 1};
constexpr uint8_t pc_wifi[]   = {192, 168, 2, 1};
constexpr uint8_t teleop[]    = {192, 168, 2, 2};
}  // namespace ip

/**
 * @brief ロボットのセンサ値などのフィードバック
 *
 */
struct feedback_data_t {
    uint16_t header;  // ヘッダー
} __attribute__((__packed__));

/**
 * @brief ロボットの動作司令値
 *
 */
struct operation_data_t {
    uint16_t header;  // ヘッダー
    float wheel_front;
    float wheel_back_left;
    float wheel_back_right;
    float belt_velocity;
    float arm_horizontal;
    float arm_vertical;
    float desk_lift;
    float desk_depth;
    float desk_finger;
    bool arm_hold;
    bool belt_throw;
    bool collect;
    bool air_throw;
    bool belt_init;
    uint8_t reserved[21];
} __attribute__((__packed__));

/**
 * @brief 操縦デバイスのレバーの傾きと押し込み
 *
 */
enum lever_point_t {
    front,
    right,
    right_deep,
    left,
    left_deep,
    push,
};

/**
 * @brief ロボットの操縦信号値
 *
 */
struct teleop_t {
    uint8_t header;  // 認識番号
    struct {
        struct {
            int8_t x;
            int8_t y;
            uint8_t push : 1;
        } __attribute__((__packed__)) right;
        struct {
            int8_t x;
            int8_t y;
            uint8_t push : 1;
        } __attribute__((__packed__)) left;
    } __attribute__((__packed__)) stick;  // 34 bit = 4Byte + 2bit

    struct {
        lever_point_t right : 3;
        lever_point_t left  : 3;
    } __attribute__((__packed__)) lever;  // 6 bit

    struct {
        struct {
            uint8_t button_5 : 1;
            uint8_t button_6 : 1;
            uint8_t button_7 : 1;
        } __attribute__((__packed__)) right;
        struct {
            uint8_t button_1 : 1;
            uint8_t button_2 : 1;
            uint8_t button_3 : 1;
            uint8_t button_4 : 1;
        } __attribute__((__packed__)) left;
        uint8_t reserved : 1;
    } __attribute__((__packed__)) button;  // 1Byte

    uint8_t data_CRC;  // 1Byte
    /***
     * CRC以外を除いた6Byteの和の補数
     * ただし計算結果の8bitより大きい値は切り捨て
     */

} __attribute__((__packed__));

/**
 * @brief PCのデバッグ用通信（再起動やスクリプト開始など）
 *
 */
struct pc_debug_t {
    uint16_t header;  // ヘッダー
    bool jetson_restart;
} __attribute__((__packed__));

/**
 * @brief メイン基板(RobotControlHub)のデバッグ用通信（制御モード切り替えなど）
 *
 */
struct main_debug_t {
    uint16_t header;
} __attribute__((__packed__));

}  // namespace robot_network_config