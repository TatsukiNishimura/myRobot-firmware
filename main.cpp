#include "mbed.h"
#include "user_libraries/AS5600/AS5600.hpp"
#include "user_libraries/BTS7960/BTS7960.hpp"
#include "user_libraries/BMX055/BMX055.hpp"
// ギア比 30:1

float yaw = 0.f;
float v_x = 0.f;
float v_y = 0.f;
float x = 0.f;
float y = 0.f;
float omega = 0.f;

int main(void)
{
    I2C i2c1(PD_13, PD_12);
    AS5600 encoder(i2c1);
    I2C i2c2(PF_0, PF_1);
    AS5600 encoder2(i2c2);
    BTS7960 motor1(D5, D6);
    BTS7960 motor2(PE_12, PE_14);
    BMX055 gyro(PB_9, PB_8);
    motor1.setPeriod(0.0001);
    motor2.setPeriod(0.0001);

    int pulse1 = 0;
    int pulse2 = 0;
    int last_pulse1 = 0;
    int last_pulse2 = 0;
    float last_time = 0.f;
    const float tire_radius = 0.058f * 0.5f;
    const float body_radius = 0.1f * 0.5f;
    float yaw_from_odometry = 0.f;
    Timer time;
    time.reset();
    time.start();

    if (encoder.update())
    {
        last_pulse1 = encoder.getPulse();
    }
    if (encoder2.update())
    {
        last_pulse2 = encoder2.getPulse();
    }
    while (1)
    {
        motor1.rotate(0.15);
        motor2.rotate(0.15);
        const float now = time.read();
        gyro.getGyro();
        if (encoder.update())
        {
            pulse1 = encoder.getPulse();
        }

        if (encoder2.update())
        {
            pulse2 = encoder2.getPulse();
        }
        const float dt = now - last_time;
        const int d_rad1 = pulse1 - last_pulse1;
        const int d_rad2 = pulse2 - last_pulse2;

        const float rads1 = static_cast<float>(d_rad1) / 4096.f * 3.141592f * 2.f / dt / 5.f;
        const float rads2 = static_cast<float>(d_rad2) / 4096.f * 3.141592f * 2.f / dt / 5.f;
        const float v_l = -1.f * tire_radius * rads1;
        const float v_r = tire_radius * rads2;

        // printf("%f %f\r\n", v_l, v_r);
        const float v = (v_l + v_r) * 0.5f;
        const float omega_from_odometry = (v_r - v_l) * 0.5f / body_radius;

        omega = gyro.gyroscope[2];
        yaw_from_odometry += omega_from_odometry * dt;
        yaw += omega * dt;
        v_x = v * cos(yaw);
        v_y = v * sin(yaw);
        x += v_x * dt;
        y += v_y * dt;
        // printf("v : %10.6f v_x : %10.6f v_y : %10.6f omega : %10.6f yaw : %10.6f\r\n", v, v_x, v_y, omega, yaw);
        // printf("x : %10.6f y :%10.6f yaw : %10.6f\r\n", x, y, yaw);
        printf("x : %10.6f y : %10.6f yaw : %10.6f vx : %10.6f vy : %10.6f omega : %10.6f yaw2 : %10.6f omega : %10.6f\r\n",
               x, y, yaw, v_x, v_y, omega,
               yaw_from_odometry, omega_from_odometry);
        last_pulse1 = pulse1;
        last_pulse2 = pulse2;
        last_time = now;
    }

    return 0;
}
