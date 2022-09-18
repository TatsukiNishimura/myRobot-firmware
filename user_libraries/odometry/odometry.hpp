#pragma once
#include "mbed.h"
#include "BMX055/BMX055.hpp"
#include "AS5600/AS5600.hpp"
#include "lowPassFilter/lowPassFilter.hpp"
class odometry
{
public:
    odometry() : l_encoder_i2c(PF_0, PF_1),
                 l_encoder(l_encoder_i2c),
                 r_encoder_i2c(PD_13, PD_12),
                 r_encoder(r_encoder_i2c),
                 gyro(PB_9, PB_8),
                 omegaWheelLPF(wheel_omega_lpf_gain),
                 omegaIMULPF(imu_omega_lpf_gain)
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
        now = time.read();
        gyro.getGyro();
        if (l_encoder.update())
        {
            l_pulse = (l_ccw ? (-1) : 1) * l_encoder.getPulse();
        }

        if (r_encoder.update())
        {
            r_pulse = (r_ccw ? (-1) : 1) * r_encoder.getPulse();
        }
        dt = now - last_time;
        const int l_error_pulse = l_pulse - l_last_pulse;
        const int r_error_pulse = r_pulse - r_last_pulse;

        // 2×π÷4096 = 0.001533981...
        l_angular_velocity = l_error_pulse / dt * 0.001533981f / gear_rational;
        r_angular_velocity = r_error_pulse / dt * 0.001533981f / gear_rational;
        const float v = tire_radius * (l_angular_velocity + r_angular_velocity) * 0.5f;
        const float imu_omega = omegaIMULPF.filter(gyro.gyroscope[2]);
        float omega_from_wheel = (r_angular_velocity - l_angular_velocity) * tire_radius * 0.5f / body_radius;

        if (abs(omega_from_wheel) > 1.f)
        {
            omega_from_wheel = 0.f;
        }
        // printf("%f %f\r\n", imu_omega, omega_from_wheel);

        omega_from_wheel = omegaWheelLPF.filter(omega_from_wheel);

        float k = 0.f;
        if (abs(imu_omega) > 0.45f)
        {
            k = 1.f;
        }
        else
        {
            k = 0.8f;
        }
        omega = k * imu_omega + (1.f - k) * omega_from_wheel;
        yaw += omega * dt;
        v_x = v * cos(yaw + 1.57f);
        v_y = v * sin(yaw + 1.57f);
        x += v_x * dt;
        y += v_y * dt;
        mutex.lock();
        odom[0] = x;
        odom[1] = y;
        odom[2] = yaw;
        twist[0] = v_x;
        twist[1] = v_y;
        twist[2] = omega;
        mutex.unlock();

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

    const std::array<float, 2> getAngularVelocity() const
    {
        return {l_angular_velocity, r_angular_velocity};
    }

private:
    I2C l_encoder_i2c;
    AS5600 l_encoder;
    I2C r_encoder_i2c;
    AS5600 r_encoder;
    BMX055 gyro;
    Timer time;
    lowPassFilter<float> omegaWheelLPF;
    lowPassFilter<float> omegaIMULPF;
    Mutex mutex;

    float v_x = 0.f;
    float v_y = 0.f;
    float omega = 0.f;
    float x = 0.f;
    float y = 0.f;
    float yaw = 0.f;

    int l_pulse = 0;
    int r_pulse = 0;
    int l_last_pulse = 0;
    int r_last_pulse = 0;
    float l_angular_velocity = 0.f;
    float r_angular_velocity = 0.f;
    float last_time = 0.f;
    float now = 0.f;
    float dt = 0.f;

    std::array<float, 3> odom;
    std::array<float, 3> twist;

    const float tire_radius = 0.058f * 0.5f;
    const float body_radius = 0.22f * 0.5f;
    const bool l_ccw = false;
    const bool r_ccw = true;
    const int gear_rational = 30;
    static constexpr float imu_omega_lpf_gain = 0.3f;
    static constexpr float wheel_omega_lpf_gain = 0.2f;
};