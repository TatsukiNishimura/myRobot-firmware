#include "mbed.h"
#include "user_libraries/AS5600/AS5600.hpp"
#include "user_libraries/BMX055/BMX055.hpp"
int main(void)
{
    I2C i2c1(PD_13, PD_12);
    AS5600 encoder(i2c1);
    I2C i2c2(PF_0, PF_1);
    AS5600 encoder2(i2c2);
    BMX055 gyro(PB_9, PB_8);
    Timer time;
    time.reset();
    time.start();

    float last_time = 0.f;
    float yaw = 0.f;
    float vx = 0.f;
    float vy = 0.f;
    float x = 0.f;
    float y = 0.f;
    bool isFirst = true;
    float initial_acc_x = 0.f;
    float initial_acc_y = 0.f;
    while (1)
    {
        const float now = time.read();
        const float dt = now - last_time;
        gyro.getAcc();
        gyro.getGyro();
        if (isFirst)
        {
            initial_acc_x = gyro.accel[0];
            initial_acc_y = gyro.accel[1];
            isFirst = false;
        }
        vx += (gyro.accel[0] - initial_acc_x) * dt;
        vy += (gyro.accel[1] - initial_acc_y) * dt;
        x += vx * dt;
        y += vy * dt;
        yaw += gyro.gyroscope[2] * dt;
        // if (encoder.update())
        // {
        //     printf("pulse1 : %d ", encoder.getPulse());
        // }

        // if (encoder2.update())
        // {
        //     printf("pulse2 : %d ", encoder2.getPulse());
        // }
        // printf("%.6f, %.6f, %.6f ", gyro.gyroscope[0], gyro.gyroscope[1], gyro.gyroscope[2]);
        printf("x : %f y : %f yaw : %f", x, y, yaw);
        printf("\r\n");
        last_time = now;
    }

    return 0;
}