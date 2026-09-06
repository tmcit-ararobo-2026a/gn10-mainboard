#include <cstdint>

enum class ShowType : uint8_t {
    Normal    = 0,
    Spinning  = 1,
    Gradually = 2,
    Switching = 3,
};
struct LedInfo {
    uint8_t r            = 0;
    uint8_t g            = 0;
    uint8_t b            = 0;
    uint16_t led_num_min = 0;
    uint16_t led_num_max = 65535;
    ShowType show_type   = ShowType::Normal;

    void reset_color_setting()
    {
        r = 0;
        g = 0;
        b = 0;
    }

    void set_color(uint8_t r, uint8_t g, uint8_t b)
    {
        this->r = r;
        this->g = g;
        this->b = b;
    }

    void set_led_scope(uint16_t led_num_min, uint16_t led_num_max)
    {
        this->led_num_min = led_num_min;
        this->led_num_max = led_num_max;
    }
} __attribute__((__packed__));
