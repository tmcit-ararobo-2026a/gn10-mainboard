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
} __attribute__((__packed__));

struct operation_data_t {
    uint16_t header;  // ヘッダー
    float wheel_front;
    float wheel_back_left;
    float wheel_back_right;
    float belt_velocity;
    float arm_horizontal;
    float arm_vertical;
    bool arm_hold;
    bool belt_throw;
    bool collect;
    uint8_t reserved[3];
} __attribute__((__packed__));