#include "task.h"
/**************文件说明**********************
与task.c共享头文件task.h。
--------------飞行器安装与控制---------------
********************************************/
short PwmOut[8]={1000,1000,1000,1000,1000,1000,1000,1000};  //把值赋给定时器，输出PWM
AxisInt PosOut;
float errx,erry;
float RollKin=0.1,PitchKin=0.1;  //角速度控制比例增益
float RollKout=0.1,PitchKout=0.1;  //姿态控制比例增益

Quaternion Quaternion_Error(Quaternion E,Quaternion P)
{
	Quaternion ans;
	ans.q0=E.q0*P.q0+E.q1*P.q1+E.q2*P.q2+E.q3*P.q3;
	ans.q1=P.q0*E.q1-E.q0*P.q1+E.q3*P.q2-E.q2*P.q3;
	ans.q2=P.q0*E.q2-E.q0*P.q2+E.q1*P.q3-E.q3*P.q1;
	ans.q3=P.q0*E.q3-E.q0*P.q3+E.q2*P.q1-E.q1*P.q2;
	return ans;
}

/***********************
角速度控制
*@period:20ms
**********************/
void Speed_Control(void)
{
	OUTPUT1=VAL_LIMIT(DegToPwmAdd(RollKin*(errx-GyroToDeg(gyro.x))),1000,2000);
	OUTPUT2=VAL_LIMIT(DegToPwmAdd(PitchKin*(erry-GyroToDeg(gyro.y))),1000,2000);
}

/***********************
姿态控制
*@period:50ms
**********************/
void Position_Control(void)
{
	Quaternion Qerr=Quaternion_Error(Qexp,Qpos);
	errx=RollKout*RadToDeg(Masin(Qerr.q1));
	erry=PitchKout*RadToDeg(Masin(Qerr.q2));
}
