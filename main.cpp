#include "TCPlib/tcpConnection.hpp"
#include "mbed.h"
#include "move/PID.hpp"
#include "move/moveControl.hpp"
int main(void)
{
  std::shared_ptr<odometry> odom_ptr = std::make_shared<odometry>();
  std::shared_ptr<moveControl> motor_ptr = std::make_shared<moveControl>(odom_ptr);
  std::shared_ptr<tcpConnection> tcp_ptr = std::make_shared<tcpConnection>(odom_ptr);

  Thread t;
  t.start([&] {
    while (1)
    {
      odom_ptr->update();
      ThisThread::sleep_for(10ms);
    }
  });

  Thread t2;
  t2.start([&] { tcp_ptr->poll(); });

  ThisThread::sleep_for(1s);

  cmdVelPID(odom_ptr, motor_ptr, tcp_ptr, 0.3, 0.f, 0.05, 0.f);
}
