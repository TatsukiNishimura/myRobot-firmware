#pragma once
#include "mbed.h"
#include "BMX055/BMX055.hpp"
#include "AS5600/AS5600.hpp"

class odometry
{
public:
    odometry() : l_encoder_i2c(PD_13, PD_12),
                 l_encoder(l_encoder_i2c),
                 r_encoder_i2c(PF_0, PF_1),
                 r_encoder(r_encoder_i2c),
                 gyro(PB_9, PB_8)
    {
        time.reset();
        time.start();

        if (l_encoder.update())
        {
            l_last_pulse = l_encoder.getPulse();
        }
        if (r_encoder.update())
        {
            r_last_pulse = r_encoder.getPulse();
        }
    };
    ~odometry(){

    };

    void update()
    {
        const float now = time.read();
        gyro.getGyro();
        if (l_encoder.update())
        {
            l_pulse = (l_ccw ? (-1) : 1) * l_encoder.getPulse();
        }

        if (r_encoder.update())
        {
            r_pulse = (r_ccw ? (-1) : 1) * r_encoder.getPulse();
        }
        const float dt = now - last_time;
        const int l_error_pulse = l_pulse - l_last_pulse;
        const int r_error_pulse = r_pulse - r_last_pulse;

        // 2×π÷4096 = 0.001533981...
        const float l_angular_velocity = l_error_pulse / dt * 0.001533981f / gear_rational;
        const float r_angular_velocity = r_error_pulse / dt * 0.001533981f / gear_rational;
        const float v = tire_radius * (l_angular_velocity + r_angular_velocity) * 0.5f;
        // const float omega = (v_r - v_l) * 0.5f / body_radius;
        omega = gyro.gyroscope[2];

        yaw += omega * dt;
        v_x = v * cos(yaw);
        v_y = v * sin(yaw);
        x += v_x * dt;
        y += v_y * dt;
        odom[0] = x;
        odom[1] = y;
        odom[2] = yaw;
        twist[0] = v_x;
        twist[1] = v_y;
        twist[2] = omega;
        // printf("v : %10.6f v_x : %10.6f v_y : %10.6f omega : %10.6f yaw : %10.6f\r\n", v, v_x, v_y, omega, yaw);
        // printf("x : %10.6f y :%10.6f yaw : %10.6f\r\n", x, y, yaw);
        // printf("x : %10.6f y : %10.6f yaw : %10.6f vx : %10.6f vy : %10.6f omega : %10.6f\r\n", x, y, yaw, v_x, v_y, omega);
        l_last_pulse = l_pulse;
        r_last_pulse = r_pulse;
        last_time = now;
    }

    const std::array<float, 3> &getOdom() const
    {
        return odom;
    }

    const std::array<float, 3> &getTwist() const
    {
        return twist;
    }

private:
    I2C l_encoder_i2c;
    AS5600 l_encoder;
    I2C r_encoder_i2c;
    AS5600 r_encoder;
    BMX055 gyro;
    Timer time;

    float yaw = 0.f;
    float v_x = 0.f;
    float v_y = 0.f;
    float x = 0.f;
    float y = 0.f;
    float omega = 0.f;

    int l_pulse = 0;
    int r_pulse = 0;
    int l_last_pulse = 0;
    int r_last_pulse = 0;
    float last_time = 0.f;

    std::array<float, 3> odom;
    std::array<float, 3> twist;

    const float tire_radius = 0.058f * 0.5f;
    const float body_radius = 0.1f * 0.5f;
    const bool l_ccw = true;
    const bool r_ccw = false;
    const int gear_rational = 30;
};