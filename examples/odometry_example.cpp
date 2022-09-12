#include "mbed.h"
#include "user_libraries/AS5600/AS5600.hpp"
#include "user_libraries/BMX055/BMX055.hpp"
void send();
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
    BMX055 gyro(PB_9, PB_8);

    int pulse1 = 0;
    int pulse2 = 0;
    int last_pulse1 = 0;
    int last_pulse2 = 0;
    float last_time = 0.f;
    const float tire_radius = 0.058f * 0.5f;
    const float body_radius = 0.1f * 0.5f;

    Thread t;
    t.start(send);

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

        const float rads1 = static_cast<float>(d_rad1) / 4096.f * 3.141592f * 2.f / dt;
        const float rads2 = static_cast<float>(d_rad2) / 4096.f * 3.141592f * 2.f / dt;
        const float v_l = (-1.f) * tire_radius * rads1;
        const float v_r = tire_radius * rads2;

        const float v = (v_l + v_r) * 0.5f;
        // const float omega = (v_r - v_l) * 0.5f / body_radius;

        omega = gyro.gyroscope[2];
        yaw += omega * dt;
        v_x = v * cos(yaw);
        v_y = v * sin(yaw);
        x += v_x * dt;
        y += v_y * dt;
        // printf("v : %10.6f v_x : %10.6f v_y : %10.6f omega : %10.6f yaw : %10.6f\r\n", v, v_x, v_y, omega, yaw);
        // printf("x : %10.6f y :%10.6f yaw : %10.6f\r\n", x, y, yaw);
        // printf("x : %10.6f y : %10.6f yaw : %10.6f vx : %10.6f vy : %10.6f omega : %10.6f\r\n", x, y, yaw, v_x, v_y, omega);
        last_pulse1 = pulse1;
        last_pulse2 = pulse2;
        last_time = now;
    }

    return 0;
}

void send()
{
    const uint8_t start_byte = 0xFF;
    const uint8_t start_byte2 = 0xFE;
    const uint8_t end_byte = 0xFD;
    UnbufferedSerial serial(USBTX, USBRX, 1000000);

    while (1)
    {
        uint8_t sendByte[sizeof(float) * 6 + 3] = {0};
        sendByte[0] = start_byte;
        sendByte[1] = start_byte2;
        memcpy(&sendByte[2], &x, sizeof(float));
        memcpy(&sendByte[6], &y, sizeof(float));
        memcpy(&sendByte[10], &yaw, sizeof(float));
        memcpy(&sendByte[14], &v_x, sizeof(float));
        memcpy(&sendByte[18], &v_y, sizeof(float));
        memcpy(&sendByte[22], &omega, sizeof(float));
        sendByte[26] = end_byte;
        serial.write(sendByte, sizeof(sendByte) / sizeof(uint8_t));
        // ThisThread::sleep_for(10ms);
    }
}