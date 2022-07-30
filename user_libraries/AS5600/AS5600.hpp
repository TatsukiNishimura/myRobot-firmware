#pragma once

#include "mbed.h"

class AS5600
{
public:
    AS5600(I2C &_i2c);
    ~AS5600();
    bool update();
    int getPulse();
    I2C &i2c;

private:
    bool isError;
    short rotate_count;
    short last_relative_angle;
    int angle;
    const char reg_raw_angle = 0x0C;
    const char address = 0x36;
};