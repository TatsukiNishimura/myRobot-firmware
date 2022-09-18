#pragma once

#include "TCPbase.hpp"
#include "move/moveControl.hpp"

class receiveTwist : protected TCPbase
{
public:
    receiveTwist(){

    };
    ~receiveTwist(){};

    void onReceive(uint8_t *rxBuf, int len)
    {
        if (len == 15 && data[0] == 0xFF && data[1] == 0xFF && data[14] == 0xFE)
        {
            const float vx = *reinterpret_cast<float *>(&data[2]);
            const float vy = *reinterpret_cast<float *>(&data[6]);
            const float omega = *reinterpret_cast<float *>(&data[10]);
        }
    }

private:
    moveControl move;
};