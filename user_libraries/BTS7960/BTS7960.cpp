#include "BTS7960.hpp"

BTS7960::BTS7960(PinName LPWM, PinName RPWM) : lpwm(LPWM), rpwm(RPWM)
{
}

BTS7960::~BTS7960()
{
}

void BTS7960::rotate(float power)
{
    if (power < 0.f)
    {
        rpwm = -power;
        lpwm = 0.f;
    }
    else
    {
        lpwm = power;
        rpwm = 0.f;
    }
}

void BTS7960::setPeriod(float period)
{
    lpwm.period(period);
    rpwm.period(period);
}
