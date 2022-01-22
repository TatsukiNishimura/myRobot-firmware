#pragma once

#include "mbed.h"
#include "../UDPLib/UDPBase.hpp"
#include "../param.hpp"

namespace userLib
{
    class UDPSend : private UDPBase
    {
    private:
        SocketAddress destination;

    public:
        UDPSend(const char *destinationIp, const int port);
        ~UDPSend();
        nsapi_size_or_error_t send(std::string data);
    };
}