/**
 * @file PID.hpp
 * @author T.nishimura (hbvcg00@gmail.com)
 * @brief インスタンスを共有する前提では作ってない
 * 同インスタンスで複数の関数を同時に使ってはいけない
 * @version 0.1
 * @date 2022-09-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "mbed.h"

class PID
{
public:
    /**
     * @brief Construct a new PID object
     *
     * @param _kp
     * @param _ki
     * @param _kd
     * @param _freq 単位はs
     */
    PID(float _freq) : freq(_freq)
    {
        if (_freq < 0.f)
        {
            error("PID.hpp: construnctor: frequency is negative.");
        }
        reset();
    };
    ~PID(){};
    void reset()
    {
        last_error = 0.f;
        I = 0.f;
    }
    float velocityPIDvalue(float target, float current_value, float kp, float ki = 0.f, float kd = 0.f)
    {
        const float error = target - current_value;
        const float P = error * kp;
        I += error * freq;
        const float D = (error - last_error) / freq;
        last_error = error;
        return P + I * ki + D * kd;
    };

private:
    float I = 0.f;
    float last_error = 0.f;
    const float freq;
};