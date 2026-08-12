/**
 * @file robot_data_config.hpp
 * @author aiba-gento
 * @brief ロボットの通信データ構造体定義
 * @version 2.1
 * @date 2025-10-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>

constexpr uint16_t operation_data_header  = 0xAB36;
constexpr uint16_t feedback_data_header   = 0x554A;
constexpr uint16_t controller_data_header = 0x15A5;
constexpr uint16_t pid_gain_data_header   = 0x5A5C;

struct feedback_data_t {
    uint16_t header;  // ヘッダー
} __attribute__((__packed__));

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

enum lever_point_t {
    front,
    right,
    right_deep,
    left,
    left_deep,
    push,
};

union controller_input_u {
    struct {
        struct {
            struct {
                uint8_t x;
                uint8_t y;
                bool push;
            } right;
            struct {
                uint8_t x;
                uint8_t y;
                bool push;
            } left;
        } stick;  // 34 bit = 4Byte + 2bit

        struct {
            lever_point_t right : 3;
            lever_point_t left  : 3;
        } lever;  // 6 bit

        struct {
            struct {
                bool button_5;
                bool button_6;
                bool button_7;
            } right;
            struct {
                bool button_1;
                bool button_2;
                bool button_3;
                bool button_4;
            } left;
            bool received;
        } button;  // 1Byte

        uint8_t data_CRC;  // 1Byte
        /***
         * CRC以外を除いた6Byteの和の補数
         * ただし計算結果の8bitより大きい値は切り捨て
         */

    } __attribute__((__packed__));

    uint8_t data[7];
};
