#pragma once
#include "TCPlib/tcpConnection.hpp"
#include "mbed.h"
#include "move/moveControl.hpp"
#include "odometry/odometry.hpp"

// kp = 0.6 ki = 0.3 kd = 0.5 error_range = 0.015
// 絶対角度で指定(-π < θ <= π)
void yawPID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float target, float kp, float ki,
    float kd, float error_range)
{
  if (odom_ptr == nullptr || motor_ptr == nullptr)
  {
    error("pointer is null");
  }
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
  if (odom_ptr == nullptr || motor_ptr == nullptr)
  {
    error("pointer is null");
  }
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
  if (odom_ptr == nullptr || motor_ptr == nullptr)
  {
    error("pointer is null");
  }
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
    // printf("omega: %10.6f error: %10.6f left: %10.6f right: %10.6f\r\n", omega, error, left, right);
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
  if (odom_ptr == nullptr || motor_ptr == nullptr)
  {
    error("pointer is null");
  }
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
  motor_ptr->tank(0.f, 0.f);
  ThisThread::sleep_for(10ms);
}
// kp = 0.05, ki = 0.1f
void velocityPID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float target, float kp, float ki)
{
  // 0.2 -> v=0.075m/s
  // 0.15-> v=0.05m/s
  // 0.13-> v=0.038m/s
  // 0.1 -> v=0.013m/s
  auto pwmToVelocity = [](float pwm) { return pwm * 0.8100 - 0.073; };
  auto velocityToPwm = [](float velocity) { return (velocity + 0.073) / 0.81; };
  lowPassFilter<float> lpf(0.3);
  float I = 0.f;
  const float loop_rate = 0.01f;
  if (odom_ptr == nullptr || motor_ptr == nullptr)
  {
    error("pointer is null");
  }
  while (1)
  {
    const float error = lpf.filter(odom_ptr->getVecocity()) - target;
    I += error * loop_rate;
    const float pid = error * kp + ki * I;
    const float power = abs_limiter(sign(target) * velocityToPwm(abs(target)) - pid, 0.25);
    motor_ptr->tank(power, power);
    ThisThread::sleep_for(10ms);
  }
  motor_ptr->tank(0.f, 0.f);
  ThisThread::sleep_for(10ms);
}
// kp_o = 0.3, ki_o = 0.1 kp_v = 0.05, ki_v = 0.1f
void velocityAndOmegaPID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, float targetVelocity,
    float targetOmega, float kp_o, float ki_o, float kp_v, float ki_v)
{
  auto radToPwm = [](float rad) {
    if ((rad + 0.500f) / 6.01823f < 0.1f)
    {
      return 0.f;
    }
    else
    {
      (rad + 0.500f) / 6.01823f;
    }
  };

  auto velocityToPwm = [](float velocity) {
    if ((velocity + 0.073f) / 0.81f < 0.1f)
    {
      return 0.f;
    }
    else
    {
      return (velocity + 0.073f) / 0.81f;
    }
  };

  float I_o = 0.f;
  float I_v = 0.f;
  const float loop_rate = 0.01f;
  lowPassFilter<float> lpf(0.3);

  if (odom_ptr == nullptr || motor_ptr == nullptr)
  {
    error("pointer is null");
  }
  while (1)
  {
    const float error_omega = odom_ptr->getTwist()[2] - targetOmega;
    // 偏差が小さいときだけ誤差を蓄積
    if (abs(error_omega) < abs(targetOmega * 0.5f))
    {
      I_o += error_omega * loop_rate;
    }
    const float pid_omega = error_omega * kp_o + ki_o * I_o;
    const float omega_power = abs_limiter<float>(sign(targetOmega) * radToPwm(abs(targetOmega)) - pid_omega, 0.2f);

    const float error_velocity = lpf.filter(odom_ptr->getVecocity()) - targetVelocity;
    // 偏差が小さいときだけ誤差を蓄積
    if (abs(error_velocity) < abs(targetVelocity * 0.5f))
    {
      I_v += error_velocity * loop_rate;
    }

    const float pid_velocity = error_velocity * kp_v + ki_v * I_v;
    const float velocity_power =
        abs_limiter<float>(sign(targetVelocity) * velocityToPwm(abs(targetVelocity)) - pid_velocity, 0.2f);

    const float power_l = abs_limiter<float>(velocity_power - omega_power, 0.25f);
    const float power_r = abs_limiter<float>(velocity_power + omega_power, 0.25f);
    printf(
        "omega_power: %10.6f velocity_power: %10.6f power_l: %10.6f power_r: %10.6f  v: %10.6f omega: "
        "%10.6f\r\n",
        omega_power, velocity_power, power_l, power_r, lpf.filter(odom_ptr->getVecocity()), odom_ptr->getTwist()[2]);
    motor_ptr->tank(power_l, power_r);
    // printf("v: %10.6f omega: %10.6f\r\n", lpf.filter(odom_ptr->getVecocity()), odom_ptr->getTwist()[2]);
    ThisThread::sleep_for(10ms);
  }
  motor_ptr->tank(0.f, 0.f);
  ThisThread::sleep_for(10ms);
}

// kp_o = 0.3, ki_o = 0.1 kp_v = 0.05, ki_v = 0.1f
void cmdVelPID(shared_ptr<odometry>& odom_ptr, shared_ptr<moveControl>& motor_ptr, shared_ptr<tcpConnection>& tcp_ptr,
    float kp_o, float ki_o, float kp_v, float ki_v)
{
  auto radToPwm = [](float rad) {
    if ((rad + 0.500f) / 6.01823f < 0.1f)
    {
      return 0.f;
    }
    else
    {
      (rad + 0.500f) / 6.01823f;
    }
  };

  auto velocityToPwm = [](float velocity) {
    if ((velocity + 0.073f) / 0.81f < 0.1f)
    {
      return 0.f;
    }
    else
    {
      return (velocity + 0.073f) / 0.81f;
    }
  };

  float I_o = 0.f;
  float I_v = 0.f;
  const float loop_rate = 0.01f;
  lowPassFilter<float> lpf(0.3);
  Mutex mutex;

  if (odom_ptr == nullptr || motor_ptr == nullptr || tcp_ptr == nullptr)
  {
    error("pointer is null");
  }
  while (1)
  {
    mutex.lock();
    const float targetVelocity = tcp_ptr->getCmdVel()[0];
    const float targetOmega = tcp_ptr->getCmdVel()[1];
    const float error_omega = odom_ptr->getTwist()[2] - targetOmega;
    const float error_velocity = lpf.filter(odom_ptr->getVecocity()) - targetVelocity;
    mutex.unlock();
    // 偏差が小さいときだけ誤差を蓄積
    if (abs(error_omega) < abs(targetOmega * 0.5f) && targetOmega != 0.f)
    {
      I_o += error_omega * loop_rate;
    }
    const float pid_omega = error_omega * kp_o + ki_o * I_o;
    const float omega_power =
        targetOmega != 0.f ? abs_limiter<float>(sign(targetOmega) * radToPwm(abs(targetOmega)) - pid_omega, 0.2f) : 0.f;

    // 偏差が小さいときだけ誤差を蓄積
    if (abs(error_velocity) < abs(targetVelocity * 0.5f) && targetVelocity != 0.f)
    {
      I_v += error_velocity * loop_rate;
    }

    const float pid_velocity = error_velocity * kp_v + ki_v * I_v;
    const float velocity_power =
        targetVelocity != 0.f
            ? abs_limiter<float>(sign(targetVelocity) * velocityToPwm(abs(targetVelocity)) - pid_velocity, 0.2f)
            : 0.f;

    const float power_l = abs_limiter<float>(velocity_power - omega_power, 0.25f);
    const float power_r = abs_limiter<float>(velocity_power + omega_power, 0.25f);
    // printf(
    //     "targetOmega: %10.6f targetVelocity: %10.6f power_l: %10.6f power_r: %10.6f  v: %10.6f omega: "
    //     "%10.6f\r\n",
    //     omega_power, velocity_power, power_l, power_r, lpf.filter(odom_ptr->getVecocity()), odom_ptr->getTwist()[2]);
    motor_ptr->tank(power_l, power_r);
    // printf("v: %10.6f omega: %10.6f\r\n", lpf.filter(odom_ptr->getVecocity()), odom_ptr->getTwist()[2]);
    ThisThread::sleep_for(10ms);
  }
  motor_ptr->tank(0.f, 0.f);
  ThisThread::sleep_for(10ms);
}