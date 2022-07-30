#include "AS5600.hpp"

AS5600::AS5600(I2C &_i2c) : i2c(_i2c), isError(false), rotate_count(0), last_relative_angle(0)
{
    i2c.frequency(400000);
}

AS5600::~AS5600()
{
}

bool AS5600::update()
{
    char out[2] = {0};
    if (i2c.write(address << 1, &reg_raw_angle, 1) == 0 && i2c.read(address << 1, out, 2) == 0)
    {
        uint16_t relative_angle = (static_cast<uint16_t>(out[0]) << 8) & 0x0F00;
        relative_angle |= static_cast<uint16_t>(out[1]);
        if (std::abs(relative_angle - last_relative_angle) >= 2048)
        {
            if (relative_angle > last_relative_angle)
            {
                rotate_count--;
            }
            else
            {
                rotate_count++;
            }
        }
        angle = relative_angle + rotate_count * 4096;
        last_relative_angle = relative_angle;
        return true;
    }
    else
    {
        return false;
    }
}

int AS5600::getPulse()
{
    return angle;
}