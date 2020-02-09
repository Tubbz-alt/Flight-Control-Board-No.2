#ifndef _IMU_H_
#define	_IMU_H_

#include "mymath.h"

//��λת��:gyro[-16383,16384];deg[-90,90];rad[-PI/2,PI/2];pwm[1000,2000];pwmAdd[-1000,1000]
#define GyroToDeg(x)          ((float)(x)*0.0076293945)  //(x*250/2^15)
#define GyroToRad(x)          ((float)(x)*1.331580545e-4)
#define DegToRad(x)           ((x)*0.0174532925)  //(x/57.3)
#define DegToGyro(x)          ((short)((x)*131.072))  //(x*2^15/250)
#define RadToDeg(x)           ((x)*57.2957795131)   //(x*57.3)
#define RadToGyro(x)          ((short)((x)*7509.8724))  //(x*2^15/250*57.3)
#define DegToPwmAdd(x)        ((x)/9.0*50.0+1500)
#define DegToPwm(x)           ((x)/9.0*50.0)
#define PwmToDegAdd(x)        ((((float)(x))-1500.0)*0.18)
#define PwmToDeg(x)           ((float)(x)*0.18)
#define PwmToRadAdd(x)        (((float)(x)-1500.0)*0.00314159265)

typedef struct
{
	float x,y,z;
}Axis;
typedef struct
{
	short x,y,z;
}AxisInt;

float IIR_LowPassFilter(float DataIn,float *delay);
void Acc_Calibrate(AxisInt *acc);
void IMUupdate(AxisInt acc,AxisInt *gyro,Quaternion *Q);

#endif
