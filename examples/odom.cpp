#include "mbed.h"
#include "user_libraries/AS5600/AS5600.hpp"
#include "user_libraries/BTS7960/BTS7960.hpp"
#include "user_libraries/BMX055/BMX055.hpp"
#include "user_libraries/odometry/odometry.hpp"
#include "user_libraries/lowPassFilter/lowPassFilter.hpp"
// ギア比 30:1

int main(void)
{

    BTS7960 motor1(D5, D6);
    BTS7960 motor2(PE_12, PE_14);
    motor1.setPeriod(0.0001);
    motor2.setPeriod(0.0001);

    odometry odom;
    lowPassFilter<float> omegaWheelLPF(0.2f);
    lowPassFilter<float> omegaIMULPF(0.3f);
    Timer time;
    time.reset();
    time.start();
    float last_time = 0.f;
    float k = 0.f;
    float wheel_yaw = 0.f;
    float imu_yaw = 0.f;
    float fusioned_yaw = 0.f;
    while (1)
    {
        motor1.rotate(0.13);
        motor2.rotate(-0.13);
        odom.update();
        const float dt = odom.getDt();
        const float wheelomega = omegaWheelLPF.filter(odom.getOmegaFromWheel());
        const float IMUomega = omegaIMULPF.filter(odom.getTwist()[2]);
        if (abs(IMUomega) > 0.45f)
        {
            k = 1.f;
        }
        else
        {
            k = 0.8f;
        }
        const float fusioned_omega = k * IMUomega + (1.f - k) * wheelomega;
        // wheel_yaw = odom.getYawFromWheel();
        // imu_yaw = odom.getOdom()[2];
        fusioned_yaw += dt * fusioned_omega;
        printf("%f,%f,%f,%f,%f,%f,",
               odom.getOdom()[0], odom.getOdom()[1], fusioned_yaw,
               odom.getTwist()[0], odom.getTwist()[1], fusioned_omega);
        printf("\n");
    }
    return 0;
}
