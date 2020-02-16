#include "imu.h"
/**************文件说明**********************
传感器数据校准,滤波,互补滤波解算四元数
********************************************/

#define T 0.02  //采样周期,0.02s
#define hT 0.01  //采样周期/2
#define qT 0.005  //采样周期/4
float gyroB[3]={-70,-140,-90};

/***********************
二阶IIR低通滤波，直接II型结构
*@delay:需要暂存3个状态变量的存储空间
*@DataIn:每次新增的数据
输出滤波后的新增数
**********************/
float IIR_LowPassFilter(float DataIn,float *delay)
{
	delay[0] = DataIn + 0.76295*delay[1] - 0.283438*delay[2];
	float DataOut = 0.129*delay[0] + 0.258*delay[1] + 0.129*delay[2];
	delay[2] = delay[1];
	delay[1] = delay[0];
	return DataOut;
}

void Gyro_Calibrate(AxisInt *gyro)
{
	gyro->x-=gyroB[0];
	gyro->y-=gyroB[1];
	gyro->z-=gyroB[2];
}

/***********************
惯导数据处理
**********************/
void IMUupdate(AxisInt gyro,Quaternion *Q)
{
	static float q0[3]={1,1,1},q1[3],q2[3],q3[3];
	static float gx[3],gy[3],gz[3];
	q0[0]=q0[1];q0[1]=q0[2];q0[2]=Q->q0;
	q1[0]=q1[1];q1[1]=q1[2];q1[2]=Q->q1;
	q2[0]=q2[1];q2[1]=q2[2];q2[2]=Q->q2;
	q3[0]=q3[1];q3[1]=q3[2];q3[2]=Q->q3;
	if((q0[2]==0)&&(q1[2]==0)&&(q2[2]==0)&&(q3[2]==0))return;
	gx[0]=gx[1];gx[1]=gx[2];gx[2]=GyroToRad(gyro.x);
	gy[0]=gy[1];gy[1]=gy[2];gy[2]=GyroToRad(gyro.y);
	gz[0]=gz[1];gz[1]=gz[2];gz[2]=GyroToRad(gyro.z);
	//陀螺仪原始数据转换为弧度
	gx[2]=GyroToRad(gyro.x);
	gy[2]=GyroToRad(gyro.y);
	gz[2]=GyroToRad(gyro.z);
	//四阶龙格库塔法数值求解四元数微分方程
	float K00=-q1[0]*gx[0]-q2[0]*gy[0]-q3[0]*gz[0];
	float K10=q0[0]*gx[0]-q3[0]*gy[0]+q2[0]*gz[0];
	float K20=q3[0]*gx[0]+q0[0]*gy[0]-q1[0]*gz[0];
	float K30=-q2[0]*gx[0]+q1[0]*gy[0]+q0[0]*gz[0];
	float K01=-(q1[0]+K10*hT)*gx[1]-(q2[0]+K20*hT)*gy[1]-(q3[0]+K30*hT)*gz[1];
	float K11=(q0[0]+K00*hT)*gx[1]-(q3[0]+K30*hT)*gy[1]+(q2[0]+K20*hT)*gz[1];
	float K21=(q3[0]+K30*hT)*gx[1]+(q0[0]+K00*hT)*gy[1]-(q1[0]+K10*hT)*gz[1];
	float K31=-(q2[0]+K20*hT)*gx[1]+(q1[0]+K10*hT)*gy[1]+(q0[0]+K00*hT)*gz[1];
	float K02=-(q1[0]+K11*hT)*gx[1]-(q2[0]+K21*hT)*gy[1]-(q3[0]+K31*hT)*gz[1];
	float K12=(q0[0]+K01*hT)*gx[1]-(q3[0]+K31*hT)*gy[1]+(q2[0]+K21*hT)*gz[1];
	float K22=(q3[0]+K31*hT)*gx[1]+(q0[0]+K01*hT)*gy[1]-(q1[0]+K11*hT)*gz[1];
	float K32=-(q2[0]+K21*hT)*gx[1]+(q1[0]+K11*hT)*gy[1]+(q0[0]+K01*hT)*gz[1];
	float K03=-(q1[0]+K12*T)*gx[2]-(q2[0]+K22*T)*gy[2]-(q3[0]+K32*T)*gz[2];
	float K13=(q0[0]+K02*T)*gx[2]-(q3[0]+K32*T)*gy[2]+(q2[0]+K22*T)*gz[2];
	float K23=(q3[0]+K32*T)*gx[2]+(q0[0]+K02*T)*gy[2]-(q1[0]+K12*T)*gz[2];
	float K33=-(q2[0]+K22*T)*gx[2]+(q1[0]+K12*T)*gy[2]+(q0[0]+K02*T)*gz[2];
	q0[2]=q0[0]+(K00+K01+K01+K02+K02+K03)*T/6.0;
	q1[2]=q1[0]+(K10+K11+K11+K12+K12+K13)*T/6.0;
	q2[2]=q2[0]+(K20+K21+K21+K22+K22+K23)*T/6.0;
	q3[2]=q3[0]+(K30+K31+K31+K32+K32+K33)*T/6.0;
	//四元数归一化与输出
	float norm=Q_rsqrt(q0[2]*q0[2]+q1[2]*q1[2]+q2[2]*q2[2]+q3[2]*q3[2]);
	Q->q0=q0[2]*norm;Q->q1=q1[2]*norm;Q->q2=q2[2]*norm;Q->q3=q3[2]*norm;
}
