#include "UDPReceive.hpp"

namespace userLib
{
    UDPReceive::UDPReceive()
    {
        me.set_ip_address(udpParam::myIp);
        me.set_port(udpParam::fromPcPort);
        udp.set_blocking(true);
        udp.bind(udpParam::fromPcPort);
    }

    UDPReceive::UDPReceive(const char *fromIp, const int port)
    {
        me.set_ip_address(fromIp);
        me.set_port(port);
        udp.set_blocking(true);
        udp.bind(port);
    }

    UDPReceive::~UDPReceive()
    {
    }

    nsapi_size_or_error_t UDPReceive::receive()
    {
        char data[256] = {0};
        const nsapi_size_or_error_t status = udp.recvfrom(&me, data, sizeof(data));
        printf("data : %s\r\n", data);
        return status;
    }
}