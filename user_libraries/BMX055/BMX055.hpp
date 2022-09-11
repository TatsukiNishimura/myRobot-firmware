/*
宣言方法
BMX055 bmx(SDA_pin,SCL_pin);

bmx.getAcc();
bmx.getGyro();
bmx.getMag();    で値をゲット！

値は
bmx.accel[3];
bmx.gyroscope[3];
bmx.magnet[3];      に入っている！



用例-----------------------------------------------------------
#include "mbed.h"
#include "BMX055.h"


BMX055 bmx(D14,D15);  //SDA SCL
DigitalOut myled(LED1);
DigitalIn sw(PC_13);
Timer t;

int main() {
    float   preAngle=0, AngGyr=0, gy=0, acc[2]={0}, angleAccel=0;
    int angle=0, use_angle=0;
    const double dt = 0.01;

    printf("\r\nrun\r\n");

    while(sw==0){}
    while(sw==1){}
    while(1) {

//        getAcc(accel);
//        printf("%2.4f, %2.4f, %2.4f\r\n",(accel[0]/512)*9.8,(accel[1]/512)*9.8,(accel[2]/512)*9.8);
//        getGyro(gyroscope);
//        printf("%2.4f, %2.4f, %2.4f\r\n",gyroscope[0]*125/2048,gyroscope[1]*125/2048,gyroscope[2]*125/2048);
//        getMag(magnet);
//        printf("%2.4f, %2.4f, %2.4f\r\n",magnet[0],magnet[1],magnet[2]);

        bmx.getGyro();

        while(sw==0){
            angle=0;
            use_angle=0;
            printf("Setting now!\r\n");
        }

        gy=bmx.gyroscope[2]*125/2048;
        angleAccel=bmx.gyroscope[2]*125/2560;
        AngGyr = preAngle+gy*dt;
        angle = (int)(0.905*AngGyr+0.09*angleAccel);   //回転の検知幅小さくすると遅いのができて、大きくすると早いのができる
                                                    //回転の検知制度を調整:上と反比例
        use_angle += angle;

        printf("%3d\r\n", use_angle/48);
        preAngle=angle;
        acc[1]=acc[0];

        wait(dt);
    }
}
----------------------------------------------------------------------
回転角がわかるよV
*/

#ifndef BMX055_H_
#define BMX055_H_

#define ACC 0x19 << 1
#define GYRO 0x69 << 1
#define MAG 0x13 << 1

#include "mbed.h"
class BMX055
{
public:
    BMX055(PinName SDA, PinName SCL);
    float accel[3];
    float gyroscope[3];
    float magnet[3];
    void getAcc(void);
    void getGyro(void);
    void getMag(void);

private:
    I2C bmx055;
    void bmx_init(void);
};

#endif /* BMX055_H_ */