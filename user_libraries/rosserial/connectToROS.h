#pragma once

#include "mbed.h"
#include "robotInterface.h"
#include <vector>

namespace robohan
{
    class connectToROS
    {
    public:
        connectToROS(UnbufferedSerial &serial);
        ~connectToROS();
        void sendToROS(float *data, uint8_t length, msgs msg);
        void sendToROS(int *data, uint8_t length, msgs msg);
        void sendToROS(uint8_t *data, uint8_t length, msgs msg);
        void readData();

        static float twistData[3];

    private:
        UnbufferedSerial &obj;

        const uint8_t endMsg = '\n';
        const uint8_t startMsg = 's';
        uint32_t recv_data_size = 0;
        uint8_t buffer[256] = {};
    };
}