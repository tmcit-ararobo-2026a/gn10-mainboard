#include "gn10_mainboard/serial_printf.hpp"

void serial_printf(const char* fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);

    int len = std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (len <= 0) {
        return;
    }
    if ((unsigned int)len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)&buffer[0], len, 0xFF);
}