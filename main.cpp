#include "mbed.h"
#include "user_libraries/rosserial/connectToROS.hpp"

int main(void)
{
    connectToROS rosserial;
    Timer time;
    time.reset();
    time.start();
    DigitalOut led(LED2);
    rosserial.startReceive();
    while (1)
    {
        const float now = time.read();
        led = (rosserial.writeSerializedData(0, now) > 0) ? 1 : 0;
        ThisThread::sleep_for(50ms);
    }
}