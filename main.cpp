#include "mbed.h"

#include "user_libraries/odometry/odometry.hpp"
#include "user_libraries/move/moveControl.hpp"
// ギア比 30:1

int main(void)
{

    std::shared_ptr<odometry> odom_ptr = std::make_shared<odometry>();
    Thread t;
    t.start([&]
            {
                while (1)
                {
                    odom_ptr->update();
                } });

    moveControl move(odom_ptr);
    ThisThread::sleep_for(1s);
    // for (float i = -1.f; i < 1.f; i = i + 0.05f)
    // {
    //     printf("%f\n", abs_limiter(i, 0.7f));
    //     ThisThread::sleep_for(500ms);
    // }
    move.setPositionY(0.15f);
    printf("end\n");

    return 0;
}
