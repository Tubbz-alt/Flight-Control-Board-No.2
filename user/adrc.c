#include "adrc.h"
/**************�ļ�˵��**********************
ǰ�벿��Ϊ�Կ��ſ�������غ���,��벿��Ϊ���Ʒ�������4������
��task.c����ͷ�ļ�task.h
--------------��������װ�����---------------
F450���ᣬX�ͣ���ǰ�׺󣬴���ǰ��������Ϊ1��ʼ��ʱ����
********************************************/

/**********************
��ɢϵͳ���ٿ����ۺϺ���
**********************/
float ADRC_fhan(float x1,float x2)
{
	float d=0.0004;
	float a0=0.02*x2;
	float y=x1+a0;
	float a1=Msqrt(d*(d+8*ABS(y)));
	float a2=a0+SIGN(y)*(a1-d)/2;
	float sy=(SIGN(y+d)-SIGN(y-d))/2;
	float a=(a0+y-a2)*sy+a2;
	float sa=(SIGN(a+d)-SIGN(a-d))/2;
	y=-5*(a/d-SIGN(a))*sa-5*SIGN(a);
	return y;
}

/**********************
����΢����
**********************/
void ADRC_TD(float x,float *track,float *derivative)
{
	float u=ADRC_fhan(*track-x,*derivative);
	*derivative+=u;
	*track+=*derivative;
}

/**********************
����������
**********************/
float ADRC_fal(float x)
{
	float y;
	if(ABS(x)<=0.02)
		y=x*0.1414213562;
	else
		y=Msqrt(ABS(x))*SIGN(x);
	return y;
}

/**********************
����״̬�۲���
**********************/
float ADRC_ESO(float u,float y,float b)
{
	static float z1=0,z2=0;
	float e=z1-y;
	z1+=b*u+z2-50*e;
	z2-=220*ADRC_fal(e);
	float w=z2/b;
	return w;
}

/**********************
�����̬��Ԫ��pos��������Ԫ��exp֮��������Ԫ��
**********************/
Quaternion Quaternion_Error(Quaternion E,Quaternion P)
{
	Quaternion ans;
	ans.q0=E.q0*P.q0+E.q1*P.q1+E.q2*P.q2+E.q3*P.q3;
	ans.q1=P.q0*E.q1-E.q0*P.q1+E.q3*P.q2-E.q2*P.q3;
	ans.q2=P.q0*E.q2-E.q0*P.q2+E.q1*P.q3-E.q3*P.q1;
	ans.q3=P.q0*E.q3-E.q0*P.q3+E.q2*P.q1-E.q1*P.q2;
	return ans;
}
