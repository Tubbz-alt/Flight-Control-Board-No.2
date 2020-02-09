#include "imu.h"
/**************文件说明**********************
传感器数据校准,滤波,互补滤波解算四元数
********************************************/

#define T 0.02  //采样周期,0.02s
#define hT 0.01  //采样周期/2
#define qT 0.005  //采样周期/4
#define Kp 0.02f
#define Ki 0.001f
const float accA[3][3]={
	{0.998710289816700,-0.002469338979608,-0.042756977266890},
	{0.021500654929696,0.994526164619401,-0.053733369334623},
	{0.006340026957332,0.014960289002374,0.985614299769622}};
const float accB[3]={-573,-255,3283};
float gyroB[3]={0,0,0};

/***********************
二阶IIR低通滤波，直接II型结构
*@delay:需要暂存3个状态变量的存储空间
*@DataIn:每次新增的数据
输出滤波后的新增数据
**********************/
float IIR_LowPassFilter(float DataIn,float *delay)
{
	delay[0] = DataIn + 0.76295*delay[1] - 0.283438*delay[2];
	float DataOut = 0.129*delay[0] + 0.258*delay[1] + 0.129*delay[2];
	delay[2] = delay[1];
	delay[1] = delay[0];
	return DataOut;
}

/***********************
用事先确定的校准参数校正加速度计原始数据
**********************/
void Acc_Calibrate(AxisInt *acc)
{
	float ax=acc->x,ay=acc->y,az=acc->z;
	acc->x=accA[0][0]*ax+accA[0][1]*ay+accA[0][2]*az+accB[0];
	acc->y=accA[1][0]*ax+accA[1][1]*ay+accA[1][2]*az+accB[1];
	acc->z=accA[2][0]*ax+accA[2][1]*ay+accA[2][2]*az+accB[2];
}

void IMUupdate(AxisInt acc,AxisInt *gyro,Quaternion *Q)
{
	float ax=acc.x,ay=acc.y,az=acc.z;  //归一化加速度计数据暂存
	if(ax==0 && ay==0 && az==0)return;
	float q0=Q->q0,q1=Q->q1,q2=Q->q2,q3=Q->q3;  //四元数暂存
	if(q0==0 && q1==0 && q2==0 && q3==0)return;
	static float ogx=0,ogy=0,ogz=0;  //上一时刻的角速度
	static float oq0=1,oq1=0,oq2=0,oq3=0;  //上一时刻的四元数
	static float exInt=0,eyInt=0,ezInt=0;
	//重力加速度归一化
	float norm=Q_rsqrt(ax*ax+ay*ay+az*az);
	ax*=norm;ay*=norm;az*=norm;
	//提取四元数的等效余弦矩阵中的重力分量
	float vx=2*(q1*q3-q0*q2);
	float vy=2*(q0*q1+q2*q3);
	float vz=1-2*(q1*q1+q2*q2);
	//向量叉积得出姿态误差
	float ex=ay*vz-az*vy; 
	float ey=az*vx-ax*vz;
	float ez=ax*vy-ay*vx;
	//对误差进行积分
	exInt+=ex*Ki*T;
	eyInt+=ey*Ki*T;
	ezInt+=ez*Ki*T;
	//姿态误差补偿到角速度上,修正角速度积分漂移
	float gx=GyroToRad(gyro->x)+Kp*ex+exInt;
	float gy=GyroToRad(gyro->y)+Kp*ey+eyInt;
	float gz=GyroToRad(gyro->z)+Kp*ez+ezInt;
	//改进欧拉法数值求解四元数微分方程
	float K0=-oq1*ogx-oq2*ogy-oq3*ogz;
	float K1=oq0*ogx-oq3*ogy+oq2*ogz;
	float K2=oq3*ogx+oq0*ogy-oq1*ogz;
	float K3=-oq2*ogx+oq1*ogy+oq0*ogz;
	K0+=-q1*gx-q2*gy-q3*gz;
	K1+=q0*gx-q3*gy+q2*gz;
	K2+=q3*gx+q0*gy-q1*gz;
	K3+=-q2*gx+q1*gy+q0*gz;
	q0+=qT*K0;
	q1+=qT*K1;
	q2+=qT*K2;
	q3+=qT*K3;
	//四元数归一化与输出,新值保存
	norm=Q_rsqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	Q->q0=q0*norm;Q->q1=q1*norm;Q->q2=q2*norm;Q->q3=q3*norm;
	oq0=Q->q0;oq1=Q->q1;oq2=Q->q2;oq3=Q->q3;
	ogx=gx;ogy=gy;ogz=gz;
	gyro->x=RadToGyro(gx);
	gyro->y=RadToGyro(gy);
	gyro->z=RadToGyro(gz);
}
