#include "mbed.h"
#include "odometry/odometry.hpp"
#include "BTS7960/BTS7960.hpp"
#include "utility/utility.hpp"
class moveControl
{
public:
    moveControl(std::shared_ptr<odometry> _odom_ptr) : odom_ptr(nullptr), motor1(D5, D6), motor2(PE_12, PE_14)
    {
        odom_ptr = _odom_ptr;
        motor1.setPeriod(0.0001);
        motor2.setPeriod(0.0001);
    };
    ~moveControl(){};
    void setPosition(float x, float y)
    {
        while (1)
        {
            mutex.lock();
            std::array<float, 3> position = odom_ptr->getOdom();
            mutex.unlock();
            if (abs(position[0] - x) < 0.01f && abs(position[1] - y) < 0.01f)
            {
                motor1.rotate(0.f);
                motor2.rotate(0.f);
                ThisThread::sleep_for(10ms);
                break;
            }
            else
            {
                // something
            }
        }
    };
    void setPositionX(float x)
    {
        while (1)
        {
            mutex.lock();
            std::array<float, 3> position = odom_ptr->getOdom();
            mutex.unlock();
            if (abs(position[0] - x) < 0.01f)
            {
                tank(0.f, 0.f);
                ThisThread::sleep_for(10ms);
                break;
            }
            else
            {
                tank(0.13, 0.13);
            }
        }
    };
    void setPositionY(float y)
    {
        while (1)
        {
            mutex.lock();
            std::array<float, 3> position = odom_ptr->getOdom();
            mutex.unlock();
            if (abs(y - position[1]) < 0.01f)
            {
                tank(0.f, 0.f);
                ThisThread::sleep_for(10ms);
                break;
            }
            else
            {
                tank(abs_limiter(y - position[1], 0.13f), abs_limiter(y - position[1], 0.13f));
            }
        }
    };

    void tank(float left, float right)
    {
        motor1.rotate(left);
        motor2.rotate(right);
    }

private:
    std::shared_ptr<odometry> odom_ptr;
    Mutex mutex;
    BTS7960 motor1;
    BTS7960 motor2;
};