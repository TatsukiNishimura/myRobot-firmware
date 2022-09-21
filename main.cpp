#include "TCPlib/tcpConnection.hpp"
#include "mbed.h"
#include "move/moveControl.hpp"

// kp = 0.6 ki = 0.3 kd = 0.5 error_range = 0.015
// 絶対角度で指定(-π < θ <= π)
void yawPID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float target, float kp, float ki,
    float kd, float error_range);

// 相対角度で指定(-π < θ <= π)
void rotatePID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float target, float kp, float ki,
    float kd, float error_range);

// kp = 0.2
void straightTimePID(
    shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float second, float straight_power, float kp);

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

  straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  yawPID(odom_ptr, motor_ptr, -M_PI / 2.f, 0.6, 0.3, 0.5, 0.015);
  straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  yawPID(odom_ptr, motor_ptr, -M_PI, 0.6, 0.3, 0.5, 0.015);
  straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  yawPID(odom_ptr, motor_ptr, M_PI / 2.f, 0.6, 0.3, 0.5, 0.015);
  straightTimePID(odom_ptr, motor_ptr, 2.5f, 0.13f, 0.2f);
  yawPID(odom_ptr, motor_ptr, 0.f, 0.6, 0.3, 0.5, 0.015);
  printf("end\r\n");
}

void yawPID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float target, float kp, float ki,
    float kd, float error_range)
{
  Mutex mutex;
  float I = 0.f;
  float wait = 0.01;
  float last_omega = 0.f;
  while (1)
  {
    mutex.lock();
    const float error = odom_ptr->getOdom()[2] - target;
    const float omega = odom_ptr->getTwist()[2];
    mutex.unlock();
    if (abs(error) < error_range)
    {
      motor_ptr->tank(0.f, 0.f);
      ThisThread::sleep_for(10ms);
      break;
    }
    I += ki * error * wait;
    const float power = error * kp + (omega - last_omega) * kd + I;
    const float left = abs_limiter<float>(power, 0.16f);
    const float right = abs_limiter<float>(-power, 0.16f);
    motor_ptr->tank(left, right);
    printf(
        "error: %10.6f omega: %10.6f power: %10.6f left: %10.6f right: %10.6f\r\n", error, omega, power, left, right);
    last_omega = omega;
    ThisThread::sleep_for(10ms);
  }
}

void rotatePID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float target, float kp, float ki,
    float kd, float error_range)
{
  Mutex mutex;
  float I = 0.f;
  float wait = 0.01;
  float last_omega = 0.f;
  mutex.lock();
  const float initial_yaw = odom_ptr->getOdom()[2];
  mutex.unlock();
  while (1)
  {
    mutex.lock();
    const float error = (odom_ptr->getOdom()[2] - initial_yaw) - target;
    const float omega = odom_ptr->getTwist()[2];
    mutex.unlock();
    if (abs(error) < error_range)
    {
      motor_ptr->tank(0.f, 0.f);
      ThisThread::sleep_for(10ms);
      break;
    }
    I += ki * error * wait;
    const float power = error * kp + (omega - last_omega) * kd + I;
    const float left = abs_limiter<float>(power, 0.2f);
    const float right = abs_limiter<float>(-power, 0.2f);
    motor_ptr->tank(left, right);
    printf(
        "error: %10.6f omega: %10.6f power: %10.6f left: %10.6f right: %10.6f\r\n", error, omega, power, left, right);
    last_omega = omega;
    ThisThread::sleep_for(10ms);
  }
}

void straightTimePID(
    shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float second, float straight_power, float kp)
{
  Mutex mutex;
  Timer time;
  time.reset();
  time.start();
  while (time.read() < second)
  {
    mutex.lock();
    const float omega = odom_ptr->getTwist()[2];
    mutex.unlock();
    const float power = omega * kp;
    const float left = abs_limiter(straight_power + power, 0.2f);
    const float right = abs_limiter(straight_power - power, 0.2f);
    printf("omega: %10.6f error: %10.6f left: %10.6f right: %10.6f\r\n", omega, error, left, right);
    motor_ptr->tank(left, right);
    ThisThread::sleep_for(10ms);
  }
  motor_ptr->tank(0.f, 0.f);
  ThisThread::sleep_for(10ms);
}