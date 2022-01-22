#include "mbed.h"
#include "rtos.h"
#include "user_libraries/UDPLib/UDPSend.hpp"
#include "user_libraries/UDPLib/UDPReceive.hpp"
#include "user_libraries/serial/serial.hpp"
#include "user_libraries/param.hpp"

void UDPReceive1();
void UDPReceive2();

int main(void)
{
    userLib::UDPSend sender("192.168.0.100", 2008);
    const std::string data = "hello";

    Thread udp1;
    Thread udp2;

    udp1.start(UDPReceive1);
    udp2.start(UDPReceive2);

    while (1)
    {
        sender.send(data);
        ThisThread::sleep_for(1s);
    }
    return 0;
}

void UDPReceive1()
{
    userLib::UDPReceive recv1(udpParam::myIp, 2008);
    while (1)
    {
        recv1.receive();
    }
}

void UDPReceive2()
{
    // userLib::UDPReceive recv2("192.168.0.100", 6005);
    // while (1)
    // {
    //     recv2.receive();
    // }
}