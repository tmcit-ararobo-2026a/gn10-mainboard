#pragma once
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>

#include "usart.h"

#define LOG_INFO    "[INFO]  "
#define LOG_DEBUG   "[DEBUG] "
#define LOG_ERROR   "[ERROR] "
#define LOG_WARNING "[WARN]  "

#define DEBUG_MODE true

void serial_printf(const char* fmt, ...);