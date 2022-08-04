#include "mbed.h"
#include "user_libraries/AS5600/AS5600.hpp"
#include "user_libraries/TB6612FNG/TB6612FNG.hpp"
#include "user_libraries/BTS7960/BTS7960.hpp"
// ギア比 1 : 60

void getEncoder();
float rads = 0;

int main(void)
{
    Thread t;
    t.start(getEncoder);
    BTS7960 motor(A0, PA_7);
    motor.setPeriod(0.001f);
    while (1)
    {
        printf("%f\r\n", rads);
        // motor.rotate(0.15f);
        ThisThread::sleep_for(10ms);
    }

    return 0;
}

void getEncoder()
{
    I2C i2c(D14, D15);
    AS5600 encoder(i2c);
    Timer time;
    time.reset();
    time.start();
    int last_pulse = 0;
    float last_time = 0;
    while (1)
    {
        if (encoder.update(true))
        {
            const int pulse = encoder.getPulse();
            const float now = time.read();
            rads = static_cast<float>((pulse - last_pulse)) / (now - last_time) * 0.000025566f;
            last_pulse = pulse;
            last_time = now;
        }
    }
}