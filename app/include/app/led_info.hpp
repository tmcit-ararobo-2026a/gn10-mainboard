#pragma once
#include <cstdint>

struct LedInfo {
    uint8_t r            = 0;
    uint8_t g            = 0;
    uint8_t b            = 0;
    uint16_t led_num_min = 0;
    uint16_t led_num_max = 65535;
    ShowType show_type   = ShowType::Normal;
} __attribute__((__packed__));

enum class ShowType : uint8_t {
    Normal    = 0,
    Spinning  = 1,
    Gradually = 2,
    Switching = 3,
};