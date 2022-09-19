#include "TCPlib/tcpConnection.hpp"
#include "mbed.h"
#include "move/moveControl.hpp"

int main(void)
{
  std::shared_ptr<odometry> odom_ptr = std::make_shared<odometry>();
  moveControl motor(odom_ptr);
  std::shared_ptr<tcpConnection> tcp_ptr = std::make_shared<tcpConnection>();
  Thread t;
  t.start([&] {
    while (1) { odom_ptr->update(); }
  });

  Thread t2;
  t2.start([&] { tcp_ptr->poll(); });
  tcp_ptr->configureSend();
  while (1)
  {
    tcp_ptr->send(odom_ptr);
    const float v = sqrt(pow(odom_ptr->getTwist()[0], 2) + pow(odom_ptr->getTwist()[1], 2)) *
                    (odom_ptr->getTwist()[1] / abs(odom_ptr->getTwist()[1]));
    printf("v : %10.6f omega: %10.6f\r\n", v, odom_ptr->getTwist()[2]);
    // printf("vx : %10.6f vy : %10.6f omega: %10.6f\r\n", odom_ptr->getTwist()[0], odom_ptr->getTwist()[1],
    // odom_ptr->getTwist()[2]);
    ThisThread::sleep_for(5ms);
  }
}
