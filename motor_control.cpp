#include "mbed.h"
#include "user_libraries/AS5600/AS5600.hpp"
#include "user_libraries/BTS7960/BTS7960.hpp"
#include "user_libraries/rosserial/EasyConnectToROS.hpp"

// Memo 角速度を読める限界はduty比50％ そのときの値は8.7rad/s

int main(void)
{

    BTS7960 LMotor(D12, D13);
    BTS7960 RMotor(D10, D11);
    I2C i2c1(D14, D15);
    I2C i2c2(PF_0, PF_1);
    AS5600 Rencoder(i2c2);
    AS5600 Lencoder(i2c1);
    Thread t;

    LMotor.setPeriod(0.001);
    RMotor.setPeriod(0.001);

    int L_last_pulse = 0;
    int R_last_pulse = 0;
    float last_time = 0;
    Timer time;
    time.reset();
    time.start();

    t.start([&]()
            {   
             
                while(1)
                {
                    Lencoder.update(true);
                    Rencoder.update(true);
                } });

    const float wheel_diameter_m = 0.035f;
    const float rotation_circle_radius = 0.112f;
    while (1)
    {
        LMotor.rotate(0.25f);
        RMotor.rotate(0.25f);
        const float now = time.read();
        const int L_pulse = Lencoder.getPulse();
        const int R_pulse = Rencoder.getPulse();
        const float Lrad = static_cast<float>(L_pulse - L_last_pulse) / (now - last_time) * 0.000025566f;
        const float Rrad = static_cast<float>(R_pulse - R_last_pulse) / (now - last_time) * 0.000025566f;

        const float v_l = wheel_diameter_m * Lrad;
        const float v_r = wheel_diameter_m * Rrad;

        const float v = (v_l + v_r) * 0.5f;

        const float omega = (v_l - v_r) * 0.5f / rotation_circle_radius;

        printf("v : %f omega : %f\r\n", v, omega);
        last_time = now;
        L_last_pulse = L_pulse;
        R_last_pulse = R_pulse;
        ThisThread::sleep_for(20ms);
    }
    return 0;
}