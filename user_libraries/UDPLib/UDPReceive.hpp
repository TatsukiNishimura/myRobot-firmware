#pragma once

#include "mbed.h"
#include "../serial/serial.hpp"
#include "../UDPLib/UDPBase.hpp"
#include "../param.hpp"

namespace userLib
{
    class UDPReceive : private UDPBase
    {
    private:
        userLib::serial serial;
        bool setBlocking = true;

    public:
        UDPReceive();
        UDPReceive(const int port);
        ~UDPReceive();
        nsapi_size_or_error_t receive();
        };
}