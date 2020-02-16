#include "task.h"
/**************�ļ�˵��**********************
���˷��������ƺ���֮��Ķ�ʱ�������ֱ�Ϊ��
Lock_And_Unlock();             ����������
RC_Prepare();                  �Խ��ջ����źŽ���Ԥ����
IMU_Processing();              ��̬������£�MPU6050����У׼
Send_Data_To_DMA_20ms();       ��DMA�������������ݵȴ���������վ
Send_Data_To_DMA_50ms();       ��DMA�������������ݵȴ���������վ
Send_Data_To_DMA_200ms();      ��DMA�������������ݵȴ���������վ
PID_Set_Parameter();           PID�Ȳ�������
********************************************/

Quaternion Qpos={1,0,0,0},Qexp={1,0,0,0};  //��̬��Ԫ����������Ԫ��
AxisInt gyro;  //������ٶ�

/***********************
*@function:��̬�������,MPU6050����У׼
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
*@function:��DMA�������������ݵȴ���������վ
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
*@function:��DMA�������������ݵȴ���������վ
*@period:50ms
**********************/
void Send_Data_To_DMA_50ms(void)
{
}

/***********************
*@function:��DMA�������������ݵȴ���������վ
*@period:200ms
**********************/
void Send_Data_To_DMA_200ms(void)
{
//	ANO_DT_Send_RCData(PwmInTemp[2],PwmInTemp[3],PwmInTemp[0],PwmInTemp[1],PwmIn[4],PwmIn[5],PwmIn[6],1000,1000,1000);
}

/***********************
*@function:��������
*@period:100ms
**********************/
void PID_Set_Parameter(void)
{
}
