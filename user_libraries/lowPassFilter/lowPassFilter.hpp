#pragma once
#include "mbed.h"

template <typename T>

class lowPassFilter
{
public:
    lowPassFilter(float _gain) : gain(_gain), last_value(static_cast<T>(0)), isFirst(false)
    {
        if (abs(gain) > 1)
        {
            setGain(0.f);
        }
    };
    ~lowPassFilter(){};

    void setGain(float newGain)
    {
        gain = newGain;
    }

    T filter(T value)
    {
        if (isFirst)
        {
            last_value = value;
            isFirst = false;
            return last_value;
        }
        const T updated_value = (1 - gain) * last_value + gain * value;
        last_value = updated_value;
        return updated_value;
    };

private:
    float gain;
    T last_value;
    bool isFirst;
};