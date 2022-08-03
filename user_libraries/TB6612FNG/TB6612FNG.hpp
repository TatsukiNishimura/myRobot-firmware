#pragma once

#include "mbed.h"

class TB6612FNG
{
public:
    TB6612FNG(PinName pwm, PinName ain1, PinName ain2);
    ~TB6612FNG();
    void changeFrequencySecond(float second);
    void rotate(float power);
    void brake();

private:
    PwmOut pwm;
    DigitalOut ain1;
    DigitalOut ain2;
};