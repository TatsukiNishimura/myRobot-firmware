#include "BMX055.hpp"

BMX055::BMX055(PinName SDA, PinName SCL) : bmx055(SDA, SCL)
{
    bmx055.frequency(100000);
    bmx_init();
}

void BMX055::bmx_init(void)
{
    char buf[2] = {0};
    printf("accel setting\r\n");
    buf[0] = 0x14;
    buf[1] = 0xB6;
    bmx055.write(ACC, buf, 2);
    ThisThread::sleep_for(2ms);
    buf[0] = 0x0F;
    buf[1] = 0x05;
    buf[0] = 0x10;
    buf[1] = 0x08;
    bmx055.write(ACC, buf, 2);
    buf[0] = 0x11;
    buf[1] = 0x00;
    bmx055.write(ACC, buf, 2);
    ThisThread::sleep_for(2ms);

    printf("gyroscope setting\r\n");
    buf[0] = 0x0F;
    buf[1] = 0x04;
    bmx055.write(GYRO, buf, 2);
    ThisThread::sleep_for(10ms);
    buf[0] = 0x10;
    buf[1] = 0x07;
    bmx055.write(GYRO, buf, 2);
    ThisThread::sleep_for(10ms);
    buf[0] = 0x11;
    buf[1] = 0x00;
    bmx055.write(GYRO, buf, 2);
    ThisThread::sleep_for(10ms);

    printf("magnet setting\r\n");
    buf[0] = 0x4B;
    buf[1] = 0x82;
    bmx055.write(MAG, buf, 2);
    ThisThread::sleep_for(10ms);
    buf[0] = 0x4B;
    buf[1] = 0x01;
    bmx055.write(MAG, buf, 2);
    ThisThread::sleep_for(10ms);
    buf[0] = 0x4C;
    buf[1] = 0x00;
    bmx055.write(MAG, buf, 2);
    buf[0] = 0x4E;
    buf[1] = 0x84;
    bmx055.write(MAG, buf, 2);
    buf[0] = 0x51;
    buf[1] = 0x04;
    bmx055.write(MAG, buf, 2);
    buf[0] = 0x52;
    buf[1] = 0x16;
    bmx055.write(MAG, buf, 2);
    ThisThread::sleep_for(10ms);

    buf[0] = 0x00;
    bmx055.write(MAG, buf, 1, 1);
    bmx055.read(MAG, buf, 1);
    printf("read:0x%02x\r\n", buf[0]);
}

void BMX055::getAcc(void)
{
    uint8_t data[6] = {0};
    char send[1], get[1];
    char temp;

    send[0] = (char)(2);
    bmx055.write(ACC, send, 1, true);
    bmx055.read(ACC, (char *)data, 6);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (int16_t)(((int16_t)data[i * 2 + 1] << 8) | data[i * 2]) >> 4;
        if (accel[i] > 2047)
            accel[i] -= 4096;
        // +/- 2g
        accel[i] = accel[i] * 0.0098f;
    }
}

void BMX055::getGyro(void)
{
    int data[6] = {0};
    char send[1], get[1];
    char temp;

    for (int i = 0; i < 6; i++)
    {
        send[0] = (char)(2 + i);
        bmx055.write(GYRO, send, 1);
        bmx055.read(GYRO, get, 1);
        temp = get[0];
        data[i] = temp;
    }

    for (int i = 0; i < 3; i++)
    {
        gyroscope[i] = (int16_t)(((int16_t)data[i * 2 + 1] << 8) | data[i * 2]) >> 4;
        if (gyroscope[i] > 32767)
            gyroscope[i] -= 65536;
        // rad/s
        gyroscope[i] = gyroscope[i] * 0.001065f;
    }
}

void BMX055::getMag(void)
{
    int data[8] = {0};
    char send[1], get[1];
    char temp;

    for (int i = 0; i < 8; i++)
    {
        send[0] = (char)(0x42 + i);
        bmx055.write(MAG, send, 1);
        bmx055.read(MAG, get, 1);
        //        printf("%02X ",get[0]);
        temp = get[0];
        data[i] = temp;
    }

    for (int i = 0; i < 3; i++)
    {
        if (i != 2)
            magnet[i] = (int16_t)(((int16_t)data[i * 2 + 1] << 8) | data[i * 2]) >> 3;
        else
            magnet[i] = (int16_t)(((int16_t)data[i * 2 + 1] << 8) | data[i * 2]) >> 1;
        if (i == 2 && magnet[i] > 16383)
            magnet[i] -= 32768;
        else if (i != 2 && magnet[i] > 4095)
            magnet[i] -= 8092;
    }
}