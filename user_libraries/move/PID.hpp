#pragma once
#include "mbed.h"
#include "move/moveControl.hpp"
#include "odometry/odometry.hpp"

// kp = 0.6 ki = 0.3 kd = 0.5 error_range = 0.015
// 絶対角度で指定(-π < θ <= π)
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

// 相対角度で指定(-π < θ <= π)
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

// kp = 0.2
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

// kp=0.3, ki=0.1
void omegaPID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float target, float kp, float ki)
{
  // max 0.93 rad/s
  // 0.25 -> 0.93 rad/s
  // 0.2 -> 0.77 rad/s
  // 0.2 -> 0.7 rad/s
  // 0.18 -> 0.59 rad/s
  // 0.15 -> 0.45 rad/s
  // 0.13 -> 0.34 rad/s
  // 0.11 -> 0.18 rad/s
  // min 0.18 rad/s
  auto pwmToRad = [](float pwm) { return 6.01823 * pwm - 0.500; };
  auto radToPwm = [](float rad) { return (rad + 0.500) / 6.01823; };
  float I = 0.f;
  const float loop_rate = 0.01f;
  while (1)
  {
    const float error = odom_ptr->getTwist()[2] - target;
    // 偏差が小さいときだけ誤差を蓄積
    if (abs(error) < abs(target * 0.5f))
    {
      I += error * loop_rate;
    }
    const float pid = error * kp + ki * I;
    const float power = abs_limiter(sign(target) * radToPwm(abs(target)) - pid, 0.25);
    motor_ptr->tank(-power, power);
    printf("%10.6f %10.6f %10.6f %10.6f\r\n", error, I, power, odom_ptr->getTwist()[2]);
    ThisThread::sleep_for(10ms);
  }
}