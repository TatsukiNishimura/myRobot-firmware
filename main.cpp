#include "mbed.h"
#include "user_libraries/AS5600/AS5600.hpp"
#include "user_libraries/BTS7960/BTS7960.hpp"
#include "user_libraries/BMX055/BMX055.hpp"
#include "user_libraries/odometry/odometry.hpp"
// ギア比 30:1

int main(void)
{

    BTS7960 motor1(D5, D6);
    BTS7960 motor2(PE_12, PE_14);
    motor1.setPeriod(0.0001);
    motor2.setPeriod(0.0001);

    odometry odom;
    while (1)
    {
        motor1.rotate(0.13);
        motor2.rotate(0.13);
        odom.update();
        for (int i = 0; i < 3; i++)
        {
            printf("%10.6f ", odom.getOdom()[i]);
        }
        printf("\r\n");
    }
    return 0;
}
