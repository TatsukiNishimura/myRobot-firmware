#pragma once

#include "mbed.h"
#include "EthernetInterface.h"
#include "../serial/serial.hpp"
#include "../param.hpp"

namespace userLib
{
    class UDPBase
    {
    private:
        static EthernetInterface eth;
        static bool initialized;
        serial userSerial;

    public:
        UDPBase();
        ~UDPBase();

        UDPSocket udp;
    };
}