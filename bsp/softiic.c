#include "softiic.h"

//��ʼ��IIC
void IIC_Init(void)
{					     
  GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//��ʹ������IO PORTBʱ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 

  GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);						 //PB8,PB9 �����	
}
 
//IIC��ʱ
void IIC_Delay(void)
{
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	IIC_Delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	IIC_Delay();
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_Delay();
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
void IIC_Wait_Ack(void)
{
	MPU_SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;IIC_Delay();	   
	IIC_SCL=1;IIC_Delay();	 
	while(MPU_READ_SDA);
	IIC_SCL=0;//ʱ�����0 	   
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	MPU_SDA_OUT();
	IIC_SDA=0;
	IIC_Delay();
	IIC_SCL=1;
	IIC_Delay();
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	MPU_SDA_OUT();
	IIC_SDA=1;
	IIC_Delay();
	IIC_SCL=1;
	IIC_Delay();
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MPU_SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		    IIC_SCL=1;
		    IIC_Delay(); 
		    IIC_SCL=0;	
		    IIC_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        IIC_Delay();
		IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		IIC_Delay(); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

//IICдһ���ֽ�
//addr:IIR���豸7λ��ַ
//reg:�Ĵ�����ַ
//data:����
void IIC_Write_Reg(u8 addr, u8 reg, u8 data)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(data);//��������
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Stop();
}
//IIC��һ���ֽ�
//addr:IIR���豸7λ��ַ
//reg:�Ĵ�����ַ
//����ֵ:����������
u8 IIC_Read_Reg(u8 addr, u8 reg)
{
	u8 res;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);//����������ַ+д����
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 1);//����������ַ+������
	IIC_Wait_Ack();		//�ȴ�Ӧ��
	res = IIC_Read_Byte(0);//��ȡ����,����nACK
	IIC_Stop();
	return res;
}
