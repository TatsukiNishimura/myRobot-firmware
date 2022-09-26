#pragma once

#include "TCPbase.hpp"
#include "move/moveControl.hpp"
#include "odometry/odometry.hpp"
class tcpConnection : public TCPbase {
 public:
  tcpConnection(std::shared_ptr<odometry> &odom_ptr)
  {
    configureReceive();
    configureSend();
    setOnReceiveHandler([&](uint8_t *buf, int len) {
      onReceive(buf, len);
      send(odom_ptr);
      // motor.setVelocity(v, omega);
    });
  };
  ~tcpConnection(){};

  void onReceive(uint8_t *rxBuf, int len)
  {
    if (len == 15 && rxBuf[0] == 0xFF && rxBuf[1] == 0xFF && rxBuf[14] == 0xFE)
    {
      mutex.lock();
      const float vx = *reinterpret_cast<float *>(&rxBuf[2]);
      const float vy = *reinterpret_cast<float *>(&rxBuf[6]);
      omega = *reinterpret_cast<float *>(&rxBuf[10]);
      v = vy;
      mutex.unlock();
      // printf("%f %f %f %f\r\n", vx, vy, v, omega);
    }
  }

  void send(std::shared_ptr<odometry> &odom_ptr)
  {
    uint8_t data[27] = {0};
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[26] = 0xFE;
    memcpy(&data[2], &odom_ptr->getOdom()[0], sizeof(float));
    memcpy(&data[6], &odom_ptr->getOdom()[1], sizeof(float));
    memcpy(&data[10], &odom_ptr->getOdom()[2], sizeof(float));
    memcpy(&data[14], &odom_ptr->getTwist()[0], sizeof(float));
    memcpy(&data[18], &odom_ptr->getTwist()[1], sizeof(float));
    memcpy(&data[22], &odom_ptr->getTwist()[2], sizeof(float));
    TCPbase::send(data, sizeof(data));
  }

  std::array<float, 2> getCmdVel() const
  {
    return {v, omega};
  }

 private:
  moveControl motor;
  Mutex mutex;
  float v = 0.f;
  float omega = 0.f;
};