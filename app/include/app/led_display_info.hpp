#pragma once
#include <cstdint>

struct LedDisplayInfo {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t led_num_min;
    uint16_t led_num_max;
    ShowType show_type;
} __attribute__((__packed__));

enum class ShowType : uint8_t {
    Normal    = 0,
    Spinning  = 1,
    Gradually = 2,
    Switching = 3,
};