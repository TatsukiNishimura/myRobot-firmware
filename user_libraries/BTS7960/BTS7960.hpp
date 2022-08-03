#pragma once

#include "mbed.h"

class BTS7960
{
public:
    BTS7960(PinName LPWM, PinName RPWM);
    ~BTS7960();
    void rotate(float power);
    void setPeriod(float period);

private:
    PwmOut lpwm;
    PwmOut rpwm;
};