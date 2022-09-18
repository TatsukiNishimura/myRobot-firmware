#include "mbed.h"
#include "TCPlib/TCPbase.hpp"
#include "odometry/odometry.hpp"
#include "move/moveControl.hpp"

int main(void)
{
    TCPbase tcp;
    std::shared_ptr<odometry> odom_ptr = std::make_shared<odometry>();
    moveControl motor(odom_ptr);
    Thread t;
    t.start([&]
            {
                while (1)
                {
                    odom_ptr->update();
                } });

    tcp.configureSend();
    while (1)
    {
        std::array<float, 3> odom = odom_ptr->getOdom();
        std::array<float, 3> twist = odom_ptr->getTwist();
        uint8_t sendData[27] = {0};
        sendData[0] = 0xFF;
        sendData[1] = 0xFF;
        memcpy(&sendData[2], &odom[0], sizeof(float));
        memcpy(&sendData[6], &odom[1], sizeof(float));
        memcpy(&sendData[10], &odom[2], sizeof(float));
        memcpy(&sendData[14], &twist[0], sizeof(float));
        memcpy(&sendData[18], &twist[1], sizeof(float));
        memcpy(&sendData[22], &twist[2], sizeof(float));
        sendData[26] = 0xFE;
        // for (int i = 0; i < 27; i++)
        // {
        //     printf("%d ", sendData[i]);
        // }
        // printf("\r\n");
        tcp.send(sendData, sizeof(sendData));
        ThisThread::sleep_for(5ms);
    }
}
