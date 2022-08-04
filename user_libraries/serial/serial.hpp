#pragma once

#include "mbed.h"
#include <stdarg.h>

namespace userLib
{
    class serial
    {
    private:
        const int baudRate = 115200;
        UnbufferedSerial obj;

    public:
        serial();
        serial(PinName TX, PinName RX, int baud);
        ~serial();
        uint32_t printf(const char *str, ...);
        uint32_t printf(std::string data);
    };

}