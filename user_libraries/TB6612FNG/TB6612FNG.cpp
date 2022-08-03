#include "TB6612FNG.hpp"

TB6612FNG::TB6612FNG(PinName pwm, PinName ain1, PinName ain2)
    : pwm(pwm),
      ain1(ain1),
      ain2(ain2)
{
    changeFrequencySecond(0.001);
}

TB6612FNG::~TB6612FNG()
{
}

void TB6612FNG::changeFrequencySecond(float second)
{
    pwm.period(second);
}

void TB6612FNG::rotate(float power)
{
    if (power > 0)
    {
        ain1 = 1;
        ain2 = 0;
        pwm = power;
    }
    else if (power == 0)
    {
        brake();
    }
    else
    {
        ain1 = 0;
        ain2 = 1;
        pwm = -power;
    }
}

void TB6612FNG::brake()
{
    ain1 = 0;
    ain2 = 0;
    pwm = 0.f;
}