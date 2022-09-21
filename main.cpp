#include "TCPlib/tcpConnection.hpp"
#include "mbed.h"
#include "move/PID.hpp"
#include "move/moveControl.hpp"

int main(void)
{
  std::shared_ptr<odometry> odom_ptr = std::make_shared<odometry>();
  std::shared_ptr<moveControl> motor_ptr = std::make_shared<moveControl>(odom_ptr);

  Thread t;
  t.start([&] {
    while (1)
    {
      odom_ptr->update();
      ThisThread::sleep_for(5ms);
    }
  });
  ThisThread::sleep_for(1s);
  omegaPID(odom_ptr, motor_ptr, 0.4, 0.3, 0.1);

  // straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  // yawPID(odom_ptr, motor_ptr, -M_PI / 2.f, 0.6, 0.3, 0.5, 0.015);
  // straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  // yawPID(odom_ptr, motor_ptr, -M_PI, 0.6, 0.3, 0.5, 0.015);
  // straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  // yawPID(odom_ptr, motor_ptr, M_PI / 2.f, 0.6, 0.3, 0.5, 0.015);
  // straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  // yawPID(odom_ptr, motor_ptr, 0.f, 0.6, 0.3, 0.5, 0.015);
  // printf("end\r\n");
}
