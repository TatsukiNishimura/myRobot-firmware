#include "UDPSend.hpp"

namespace userLib
{
    UDPSend::UDPSend(const char *destinationIp, const int port)
    {
        destination.set_ip_address(destinationIp);
        destination.set_port(port);
    }

    UDPSend::~UDPSend()
    {
    }

    nsapi_size_or_error_t UDPSend::send(std::string data)
    {
        const nsapi_size_or_error_t status = UDPBase::udp.sendto(destination, data.c_str(), data.length());
        return status;
    }
}