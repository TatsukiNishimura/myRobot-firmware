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
        SocketAddress me;
        userLib::serial serial;

    public:
        UDPReceive();
        UDPReceive(const char *fromIp, const int port);
        ~UDPReceive();
        nsapi_size_or_error_t receive();
    };
}