#pragma once

#include "mbed.h"
#include "odometry/odometry.hpp"
#include "BTS7960/BTS7960.hpp"
#include "utility/utility.hpp"
#include "move/PID.hpp"

class moveControl
{
public:
    moveControl(std::shared_ptr<odometry> _odom_ptr) : odom_ptr(_odom_ptr), motor1(D5, D6), motor2(PE_12, PE_14), pid(0.005)
    {
        motor1.setPeriod(0.0001);
        motor2.setPeriod(0.0001);
    };
    moveControl() : odom_ptr(nullptr), motor1(D5, D6), motor2(PE_12, PE_14), pid(0.005){};
    ~moveControl(){};
    void setPosition(float x, float y)
    {
        if (odom_ptr == nullptr)
        {
            error("move_control: setPosition(): odom_ptr is null");
        }
        while (1)
        {
            mutex.lock();
            std::array<float, 3> position = odom_ptr->getOdom();
            mutex.unlock();
            if (abs(position[0] - x) < 0.01f && abs(position[1] - y) < 0.01f)
            {
                motor1.rotate(0.f);
                motor2.rotate(0.f);
                ThisThread::sleep_for(10ms);
                break;
            }
            else
            {
                // something
            }
        }
    };
    void setPositionX(float x)
    {
        if (odom_ptr == nullptr)
        {
            error("move_control: setPositionX(): odom_ptr is null");
        }
        while (1)
        {
            mutex.lock();
            std::array<float, 3> position = odom_ptr->getOdom();
            mutex.unlock();
            if (abs(position[0] - x) < 0.01f)
            {
                tank(0.f, 0.f);
                ThisThread::sleep_for(10ms);
                break;
            }
            else
            {
                tank(0.13, 0.13);
            }
        }
    };
    void setPositionY(float y)
    {
        if (odom_ptr == nullptr)
        {
            error("move_control: setPositionY(): odom_ptr is null");
        }
        pid.reset();
        while (1)
        {
            mutex.lock();
            std::array<float, 3> position = odom_ptr->getOdom();
            mutex.unlock();
            if (abs(y - position[1]) < 0.01f)
            {
                tank(0.f, 0.f);
                printf("end\r\n");
                ThisThread::sleep_for(10ms);
                break;
            }
            else
            {
                const float raw_value = pid.velocityPIDvalue(y, position[1], 1.4f, 0.05f, 0.1f);
                const float value = abs_limiter(raw_value, 0.15f);
                printf("%10.6f %10.6f %10.6f\n", raw_value, value, position[1]);
                tank(value, value);
            }
        }
    };

    /**
     * @brief Set the Velocity object
     *
     * @param v m/s
     * @param omega rad/s
     */
    void setVelocity(float v, float omega)
    {
        const float l_omega = (v - d * omega) / r;
        const float r_omega = (v + d * omega) / r;
        float l_vol = 0.f;
        float r_vol = 0.f;
        if (abs(l_omega) < abs(l_b))
        {
            l_vol = 0.f;
        }
        else
        {
            if (l_omega > 0)
            {
                l_vol = (l_omega + l_b) / l_a;
            }
            else
            {
                l_vol = (l_omega - l_b) / l_a;
            }
        }
        if (abs(r_omega) < abs(r_b))
        {
            r_vol = 0.f;
        }
        else
        {
            if (r_omega > 0)
            {
                r_vol = (r_omega + r_b) / r_a;
            }
            else
            {
                r_vol = (r_omega - r_b) / r_a;
            }
        }
        // printf("%f %f\r\n", abs_limiter(l_vol / voltage, max_pwm), abs_limiter(r_vol / voltage, max_pwm));
        tank(abs_limiter(l_vol / voltage, max_pwm), abs_limiter(r_vol / voltage, max_pwm));
    }

    void tank(float left, float right)
    {
        motor1.rotate(left);
        motor2.rotate(right);
    }

private:
    std::shared_ptr<odometry> odom_ptr;
    Mutex mutex;
    BTS7960 motor1;
    BTS7960 motor2;
    PID pid;
    const float d = 0.22f * 0.5f;
    const float r = 0.058f * 0.5f;
    const float max_pwm = 0.2f;
    const float voltage = 11.3f;
    const float l_a = 2.3022f;
    const float l_b = -2.53110f;
    const float r_a = 2.2829f;
    const float r_b = -2.5004f;
};