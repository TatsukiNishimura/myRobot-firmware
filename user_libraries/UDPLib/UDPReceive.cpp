#include "UDPReceive.hpp"

namespace userLib
{
    UDPReceive::UDPReceive()
    {
        udp.set_blocking(setBlocking);
        udp.bind(udpParam::fromPcPort);
    }

    UDPReceive::UDPReceive(const int port)
    {
        udp.set_blocking(setBlocking);
        udp.bind(port);
    }

    UDPReceive::~UDPReceive()
    {
    }

    nsapi_size_or_error_t UDPReceive::receive()
    {
        unsigned char data[256] = {0};
        SocketAddress source;
        const nsapi_size_or_error_t status = udp.recvfrom(&source, data, sizeof(data));
        printf("data : %s from %s port %d\r\n", data, source.get_ip_address(), source.get_port());
        return status;
    }
}