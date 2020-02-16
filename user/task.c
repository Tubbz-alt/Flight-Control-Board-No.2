#include "task.h"
/**************文件说明**********************
除了飞行器控制函数之外的定时函数，分别为：
Lock_And_Unlock();             锁定，解锁
RC_Prepare();                  对接收机的信号进行预处理
IMU_Processing();              姿态解算更新，MPU6050数据校准
Send_Data_To_DMA_20ms();       往DMA缓存中填入数据等待发给地面站
Send_Data_To_DMA_50ms();       往DMA缓存中填入数据等待发给地面站
Send_Data_To_DMA_200ms();      往DMA缓存中填入数据等待发给地面站
PID_Set_Parameter();           PID等参数设置
********************************************/

Quaternion Qpos={1,0,0,0},Qexp={1,0,0,0};  //姿态四元数和期望四元数
AxisInt gyro;  //三轴角速度

/***********************
*@function:姿态解算更新,MPU6050数据校准
*@period:20ms
**********************/
void IMU_Processing(void)
{
	static float IIRgyrox[3],IIRgyroy[3],IIRgyroz[3];
	MPU_Get_Gyroscope(&gyro.x,&gyro.y,&gyro.z);
	Gyro_Calibrate(&gyro);
	gyro.x=IIR_LowPassFilter(gyro.x,IIRgyrox);
	gyro.y=IIR_LowPassFilter(gyro.y,IIRgyroy);
	gyro.z=IIR_LowPassFilter(gyro.z,IIRgyroz);
	IMUupdate(gyro,&Qpos);
}

/***********************
*@function:往DMA缓存中填入数据等待发给地面站
*@period:20ms
**********************/
void Send_Data_To_DMA_20ms(void)
{
	s16 mydata[6];
	float roll=Matan2(2*(Qpos.q0*Qpos.q1+Qpos.q2*Qpos.q3),1-2*(Qpos.q1*Qpos.q1+Qpos.q2*Qpos.q2))*57.3f;
	float pitch=Masin(2*(Qpos.q0*Qpos.q2-Qpos.q1*Qpos.q3))*57.3f;
	float yaw=Matan2(2*(Qpos.q1*Qpos.q2+Qpos.q0*Qpos.q3),1-2*(Qpos.q2*Qpos.q2+Qpos.q3*Qpos.q3))*57.3f;
	mydata[0]=OUTPUT1;
	mydata[1]=OUTPUT2;
	mydata[2]=Qpos.q0;
	mydata[3]=Qpos.q1;
	mydata[4]=Qpos.q2;
	mydata[5]=Qpos.q3;
	ANO_Send_User_Data(mydata,6,0xF1);
	ANO_DT_Send_Senser(0,0,0,gyro.x,gyro.y,gyro.z,0,0,0);
	ANO_DT_Send_Status(roll,pitch,yaw,0,0,0);
}

/***********************
*@function:往DMA缓存中填入数据等待发给地面站
*@period:50ms
**********************/
void Send_Data_To_DMA_50ms(void)
{
}

/***********************
*@function:往DMA缓存中填入数据等待发给地面站
*@period:200ms
**********************/
void Send_Data_To_DMA_200ms(void)
{
//	ANO_DT_Send_RCData(PwmInTemp[2],PwmInTemp[3],PwmInTemp[0],PwmInTemp[1],PwmIn[4],PwmIn[5],PwmIn[6],1000,1000,1000);
}

/***********************
*@function:参数设置
*@period:100ms
**********************/
void PID_Set_Parameter(void)
{
}
