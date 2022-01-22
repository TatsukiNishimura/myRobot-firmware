#include "UDPBase.hpp"

namespace userLib
{
    bool UDPBase::initialized = false;
    EthernetInterface UDPBase::eth;

    UDPBase::UDPBase()
    {
        if (!initialized)
        {
            eth.set_network(udpParam::myIp, udpParam::myNetMask, udpParam::myGateWay);
            const nsapi_error_t status = eth.connect();
            if (status != NSAPI_ERROR_OK)
            {
                userSerial.printf("net connect error\n");
            }
            else
            {
                SocketAddress ipAddr;
                eth.get_ip_address(&ipAddr);
                userSerial.printf("ip : %s\r\n", ipAddr.get_ip_address());
            }
            initialized = true;
        }
        userSerial.printf("UDPBase Instance made:  %d\r\n", &eth);

        udp.open(&eth);

        userSerial.printf("UDPSocket Instance made : %d\r\n", &udp);
    }

    UDPBase::~UDPBase()
    {
        udp.close();
        eth.disconnect();
    }

}