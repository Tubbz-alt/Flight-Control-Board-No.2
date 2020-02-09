#include "mpu6050.h"
/*MPU6050�÷�
������pitch�����row�����yaw
MPU6050ģ��ˮƽ���ã����볯�³��ң�������roll��gyroxΪ��,accyΪ����̧ͷpitch��gyroyΪ����accxΪ��������ƫ��yaw��gyrozΪ��
*/

//������ʱ
void Delay_ms(unsigned short time_ms)
{
	unsigned short i, j;
	for (i = 0; i < time_ms; i++)
		for (j = 0; j < 10309; j++);
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//����������ַ+д����	
	IIC_Wait_Ack();	//�ȴ�Ӧ��
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	for (i = 0; i < len; i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		IIC_Wait_Ack();	//�ȴ�Ӧ��
	}
	IIC_Stop();
	return 0;
}
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//����������ַ+д����	
	IIC_Wait_Ack();	//�ȴ�Ӧ��
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 1);//����������ַ+������	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while (len)
	{
		if (len == 1)*buf = IIC_Read_Byte(0);//������,����nACK 
		else *buf = IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;
	}
	IIC_Stop();	//����һ��ֹͣ���� 
	return 0;
}

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
void MPU_Set_Gyro_Fsr(u8 fsr)
{
	IIC_Write_Reg(MPU_ADDR, MPU_GYRO_CFG_REG, fsr << 3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
void MPU_Set_Accel_Fsr(u8 fsr)
{
	IIC_Write_Reg(MPU_ADDR, MPU_ACCEL_CFG_REG, fsr << 3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
void MPU_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)data = 1;
	else if (lpf >= 98)data = 2;
	else if (lpf >= 42)data = 3;
	else if (lpf >= 20)data = 4;
	else if (lpf >= 10)data = 5;
	else data = 6;
	IIC_Write_Reg(MPU_ADDR, MPU_CFG_REG, data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
void MPU_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)rate = 1000;
	if (rate < 4)rate = 4;
	data = 1000 / rate - 1;
	IIC_Write_Reg(MPU_ADDR, MPU_SAMPLE_RATE_REG, data);	//�������ֵ�ͨ�˲���
	MPU_Set_LPF(rate / 2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//��ʼ��MPU6050
void MPU_Init(void)
{
	IIC_Init();//��ʼ��IIC����
	IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X80);	//��λMPU6050
	Delay_ms(100);
	IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(0);                        //�����Ǵ�����,��250dps
	MPU_Set_Accel_Fsr(0);                       //���ٶȴ�����,��4g
	MPU_Set_Rate(50);                           //���ò�����50Hz
	IIC_Write_Reg(MPU_ADDR, MPU_INT_EN_REG, 0X00);       //�ر������ж�
	IIC_Write_Reg(MPU_ADDR, MPU_USER_CTRL_REG, 0X00);    //I2C��ģʽ�ر�
	IIC_Write_Reg(MPU_ADDR, MPU_FIFO_EN_REG, 0X00);      //�ر�FIFO
	IIC_Write_Reg(MPU_ADDR, MPU_INTBP_CFG_REG, 0X02);	//������·ģʽ
	IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X01);    //����CLKSEL,PLL X��Ϊ�ο�
	IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT2_REG, 0X00);    //���ٶ��������Ƕ�����
	MPU_Set_Rate(50);                           //���ò�����Ϊ50Hz
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
	u8 buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((u16)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
void MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6];
	MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	*gx = ((u16)buf[0] << 8) | buf[1];
	*gy = ((u16)buf[2] << 8) | buf[3];
	*gz = ((u16)buf[4] << 8) | buf[5];
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
void MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6];
	MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	*ax = ((u16)buf[0] << 8) | buf[1];
	*ay = ((u16)buf[2] << 8) | buf[3];
	*az = ((u16)buf[4] << 8) | buf[5];
}

