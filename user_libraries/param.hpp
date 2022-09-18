#pragma once

#include "mbed.h"

namespace udpParam
{
    static const char *myIp = "192.168.0.100";
    static const char *myNetMask = "255.255.255.0";
    static const char *myGateWay = "192.168.0.255";

    static const char *pcIp = "192.168.0.28";
    static const int toPcPort = 2008;
    static const int fromPcPort = 6005;
}

namespace tcpParam
{
    static const char *myIp = "10.42.0.2";
    static const char *myNetMask = "255.255.255.0";
    static const char *myGateWay = "10.42.0.1";

    static const char *pcIp = "10.42.0.10";
    static const int pcPort = 6005;
    static const int myPort = 80;
}
