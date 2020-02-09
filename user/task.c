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

Quaternion Qpos={1,0,0,0},Qexp;  //��̬��Ԫ����������Ԫ��
AxisInt acc;  //������ٶ�
AxisInt gyro;  //������ٶ�
AxisInt oacc;  //������ٶȼ�ԭʼ����
AxisInt ogyro;  //����������ԭʼ����
AxisInt Bias;  //�ֶ�ģʽ��ʼ����,���ڹ̶�����ֶ�ģʽ����������,�ɴ���ң������΢��
//AxisInt AutoBias;  //�Զ�ģʽ��ʼ���������ڹ̶���İ��Զ���ȫ�Զ�ģʽ
u8 Armed=0;  //Ĭ������
short PwmInTemp[7]={1000,1000,1000,1000,1000,1000,1000};  //��ֹ����������б��ж�������(�ٽ���뱣��)
ADRC_Param RollParam,PitchParam;

/***********************
*@function:����������
*@period:100ms
*@note:����ݸ��Ƶ����·�����;����ݸ��Ƶ����·�����
**********************/
void Lock_And_Unlock(void)
{
	static u8 LockMode=LOCKED;//����/��������״̬��
	static u16 t=0;//����������Ҫ��ʱ��
	if((acc.z<15000)&&(LockMode==LOCKED)){Armed=0;return;}//�������쳣
	if(AircraftMode&0x80){LockMode=UNLOCKED;Armed=1;return;}//�̶������������
	switch(LockMode)
	{
		case LOCKED:
			if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//�Լ���ɲ����н�������
			{LockMode=TOUNLOCK;t++;}
			break;
		case TOUNLOCK:
			if(t>TIME)//�ﵽ����ʱ������
			{LockMode=UNLOCKED;t=0;Armed=1;}
			else if((PwmInTemp[0]>LOCK_HIGH)&&(PwmInTemp[1]<LOCK_LOW)&&(PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]<LOCK_LOW))//��Ȼ�ڳ��Խ���
				t++;
			else//��������
			{LockMode=LOCKED;t=0;}
			break;
		case UNLOCKED:
			if((PwmInTemp[2]<LOCK_LOW)&&(PwmInTemp[3]>LOCK_HIGH))//����������
			{LockMode=LOCKED;Armed=0;}//�����������ͬ��������������������
			break;
		default:break;
	}
}

/***********************
*@function:��̬�������,MPU6050����У׼
*@period:20ms
**********************/
void IMU_Processing(void)
{
	static float IIRgyrox[3]={0,0,0};
	static float IIRgyroy[3]={0,0,0};
	static float IIRgyroz[3]={0,0,0};
	MPU_Get_Accelerometer(&acc.x,&acc.y,&acc.z);
	MPU_Get_Gyroscope(&gyro.x,&gyro.y,&gyro.z);
	oacc=acc;ogyro=gyro;
	Acc_Calibrate(&acc);
	IMUupdate(acc,&gyro,&Qpos);
	gyro.x=IIR_LowPassFilter(gyro.x,IIRgyrox);
	gyro.y=IIR_LowPassFilter(gyro.y,IIRgyroy);
	gyro.z=IIR_LowPassFilter(gyro.z,IIRgyroz);
}

/***********************
*@function:�Խ��ջ����źŽ���Ԥ����ת��Ϊ������Ԫ��
*@period:20ms
**********************/
void RC_Prepare(void)
{
	PwmInTemp[0]=PwmIn[0]-20+Bias.x;
	PwmInTemp[1]=PwmIn[1]-20+Bias.y;
	PwmInTemp[2]=PwmIn[2]-20;
	PwmInTemp[3]=PwmIn[3]-20+Bias.z;
	PwmInTemp[4]=PwmIn[4]-20;
	PwmInTemp[5]=PwmIn[5]-20;
	PwmInTemp[6]=PwmIn[6]-20;
	float Hroll=PwmToRadAdd(PwmInTemp[0])/2;
	float Hpitch=PwmToRadAdd(PwmInTemp[1])/2;
	float Hyaw=PwmToRadAdd(PwmInTemp[3])/2;
	Qexp.q0=Mcos(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Msin(Hpitch)*Msin(Hyaw);
	Qexp.q1=Msin(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)-Mcos(Hroll)*Msin(Hpitch)*Msin(Hyaw);
	Qexp.q2=Mcos(Hroll)*Msin(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Mcos(Hpitch)*Msin(Hyaw);
	Qexp.q3=Mcos(Hroll)*Mcos(Hpitch)*Msin(Hyaw)-Msin(Hroll)*Msin(Hpitch)*Mcos(Hyaw);
}

/***********************
*@function:��DMA�������������ݵȴ���������վ
*@period:20ms
**********************/
void Send_Data_To_DMA_20ms(void)
{
	static u8 count=0;
	s16 mydata1[7];
	mydata1[0]=OUTPUT1;
	mydata1[1]=OUTPUT2;
	mydata1[2]=OUTPUT3;
	mydata1[3]=OUTPUT4;
	mydata1[4]=count;
	if(PwmInTemp[4]>=HIGH_THRESHOLD)
	{
		ANO_Send_User_Data(mydata1,5,0xF1);
		ANO_DT_Send_Senser(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,0,0,0);
	}
	else if((PwmInTemp[4]<HIGH_THRESHOLD)&&(PwmInTemp[4]>LOW_THRESHOLD))
		ANO_DT_Send_Senser(oacc.x,oacc.y,oacc.z,ogyro.x,ogyro.y,ogyro.z,0,0,0);
	count++;
}

/***********************
*@function:��DMA�������������ݵȴ���������վ
*@period:50ms
**********************/
void Send_Data_To_DMA_50ms(void)
{
	float roll=Matan2(2*(Qpos.q0*Qpos.q1+Qpos.q2*Qpos.q3),1-2*(Qpos.q1*Qpos.q1+Qpos.q2*Qpos.q2))*57.3f;
	float pitch=Masin(2*(Qpos.q0*Qpos.q2-Qpos.q1*Qpos.q3))*57.3f;
	float yaw=Matan2(2*(Qpos.q1*Qpos.q2+Qpos.q0*Qpos.q3),1-2*(Qpos.q2*Qpos.q2+Qpos.q3*Qpos.q3))*57.3f;
	if(PwmInTemp[4]>=HIGH_THRESHOLD)
		ANO_DT_Send_Status(roll,pitch,yaw,0,0,Armed);
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
	if(ANO_CMD&PID_REQUIRE)//����վҪ���ȡPID
	{
		ANO_DT_Send_PID(1,RollParam.KpOut,RollParam.KpIn,RollParam.b,PitchParam.KpOut,PitchParam.KpIn,PitchParam.b/100,0,0,0);
		ANO_CMD&=~PID_REQUIRE;
	}
	if(ANO_CMD&PID_SENDBACK)//����վҪ��д��PID
	{
		RollParam.KpOut=(PIDReceiveTemp[0][0]*256.0+PIDReceiveTemp[0][1])/1000.0;
		RollParam.KpIn=(PIDReceiveTemp[0][2]*256.0+PIDReceiveTemp[0][3])/1000.0;
		RollParam.b=(PIDReceiveTemp[0][4]*256.0+PIDReceiveTemp[0][5])/10.0;
		PitchParam.KpOut=(PIDReceiveTemp[0][6]*256.0+PIDReceiveTemp[0][7])/1000.0;
		PitchParam.KpIn=(PIDReceiveTemp[0][8]*256.0+PIDReceiveTemp[0][9])/1000.0;
		PitchParam.b=(PIDReceiveTemp[0][10]*256.0+PIDReceiveTemp[0][11])/10.0;

		ANO_CMD&=~PID_SENDBACK;
	}
}
