#ifndef __TASK_H
#define __TASK_H

#include "niming.h"
#include "imu.h"
#include "mpu6050.h"
#include "adrc.h"
#include "pwm_in.h"
#include "gpio.h"

#define OUTPUT1  (TIM3->CCR1) 
#define OUTPUT2  (TIM3->CCR2)
#define OUTPUT3  (TIM3->CCR3)
#define OUTPUT4  (TIM3->CCR4)
#define OUTPUT5  (TIM4->CCR1)
#define OUTPUT6  (TIM4->CCR2)
#define OUTPUT7  (TIM4->CCR3)
#define OUTPUT8  (TIM4->CCR4)

#define LOWSPEED          1100	 //����
#define LOW_THRESHOLD     1250  //5��7ͨ����1,2����ֵ
#define HIGH_THRESHOLD    1750  //5��7ͨ����2,3����ֵ
#define LOCK_HIGH         1800  //�����ͽ���ҡ������ֵ
#define LOCK_LOW          1200  //�����ͽ���ҡ������ֵ
#define LOW_SERVO         500  //����г��¼���
#define HIGH_SERVO        2500  //����г��ϼ���
//LockMode
#define LOCKED    0	//����״̬���޲���
#define TOUNLOCK  1	//����״̬�ҳ��Խ���
#define UNLOCKED  2	//����״̬
#define TIME      20	//����ʱ��,2��
//WholeCommand
#define NORMAL_WORK     0x01  //�ߵ�ƽ��ʾ�Լ���ϣ�������������״̬
#define PREPARE         0x02  //�ߵ�ƽ���崥��У׼
#define CALIBRATING     0x04  //�ߵ�ƽ��ʾ����У׼
#define CALIBRATED      0x08  //�ߵ�ƽ��ʾУ׼���
//AircraftMode
#define AircraftMode  SIZHOU
#define SIZHOU        1  //������
#define YUYING        2  //��ӥ

typedef struct
{
	float KpOut;  //�⻷��������
	float KpIn;  //�ڻ���������
	float b;  //�Ŷ���������
}ADRC_Param;

extern u8 Armed;
extern short PwmInTemp[7];
extern AxisInt acc;
extern AxisInt gyro;
extern ADRC_Param RollParam,PitchParam;
extern Quaternion Qpos,Qexp;

//��task.c��
void Para_Init(void);
void Lock_And_Unlock(void);
void IMU_Processing(void);
void RC_Prepare(void);
void Send_Data_To_DMA_20ms(void);
void Send_Data_To_DMA_50ms(void);
void Send_Data_To_DMA_200ms(void);
void PID_Set_Parameter(void);
void Self_Test(void);
//�ڷ��������Ƴ�����
void Para_Init(void);
void Motor_Iner_loop(void);
void Motor_Outer_loop(void);

#endif
