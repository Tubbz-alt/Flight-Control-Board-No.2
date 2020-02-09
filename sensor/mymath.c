#include "mymath.h"
/**************�ļ�˵��**********************
���ó��Ⱥ����Ŀ����㷨
��⺯����ͻ�ĺ�����ǰ׺"M"
����Ŀ¼(������Ϊ�������ֵ��):
Msin(x)     ([-pi/2,pi/2],[-1,1]);
Mcos(x)     ([-pi/2,pi/2],[-1,1]);
Matan(x)    ((-��,��),(-pi/2,pi/2));
Matan2(y,x) ((-��,��),(-pi/2,pi/2));
Masin(x)    ([-1,1],[-pi/2,pi/2]);
Msqrt,Q_rsqrt,Mpow,ABS,SIGN,MIN,MAX,VAL_LIMIT
********************************************/

//�����к���.������(-��,��),ֵ��(-pi/2,pi/2)
float Matan(float rad)
{
	if (rad < 0)
		return -Matan(-rad);
	if (rad <= 0.25)
	{
		float ans = rad*(0.47831170583472860f * rad - 1.1679191357523431f);
		return ans*(0.29922976727874773f * ans - 0.85630501107017942f);
	}
	if (rad <= 0.75)
		return 0.463647609f + Matan((rad - 0.5) / (1 + rad*0.5));
	if (rad <= 2)
		return 0.8760580506f + Matan((rad - 1.2) / (1 + rad*1.2));
	return 1.3258186637f + Matan((rad - 4) / (1 + rad * 4));
}

//360�㷴����.������(-��,��),ֵ��(-pi,pi]
float Matan2(float y,float x)
{
	if (x > 0)
		return Matan(y / x);
	if (x < 0)
		return (y >= 0) ? (Matan(y / x) + PI) : (Matan(y / x) - PI);
	return (y > 0) ? (PI / 2) : ((y < 0) ? (-PI / 2) : 0);
}

//����ƽ�����㷨
float Msqrt(float number)
{
	long i;
	float x, y;
	x = number * 0.5F;
	y = number;
	i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (x * y * y));
	y = y * (1.5f - (x * y * y));
	return number * y;
}

//����ƽ���������㷨
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;
	i  = 0x5f3759df - ( i >> 1 );
	y  = * ( float * ) &i;
	y  = y * ( 1.5f - ( x2 * y * y ) );
	y  = y * ( 1.5f - ( x2 * y * y ) );
	return y;
}

//�����Һ���.������[-1,1],ֵ��[-pi/2,pi/2]
float Masin(float x)
{
	if(x<0)
		return -Masin(-x);
	else if(x<1)
		return Matan(x*Q_rsqrt(1-x*x));
	else
	{
		x=PI/2;
		return x;
	}
}

//���Һ���.������[-pi,pi],ֵ��[-1,1]
float Msin(float rad)
{
	float ans;
	ans = rad*(1.27323954f - 0.405284735f * ABS(rad));
	ans = ans*(0.775 + 0.225*ABS(ans));
	return ans;
}

//ָ������
double pow_i(double num, int n)//����num��n���ݣ�����nΪ������
{
	double powint = 1;
	int i;
	for (i = 1; i <= n; i++) powint *= num;
	return powint;
}
/*
����ʵ�����̩��չ����ָ��
(1+x)^m=1+mx+m(m-1)/2!+...
x��������(-1,1)����ԭ��ԽԶ��ָ��mԽ�������Խ��
*/
double pow_f(double num, double m)//����num��m���ݣ�num��m��Ϊ˫���ȣ�num��������(0,2)��
{
	double powf = 1, tmpm = 1;//̩�ռ�����1����1
	double x = num - 1;
	for (int i = 1; ABS(tmpm) > 1e-12; i++)//̩�ռ������һ��ǳ��ӽ���0ʱ��������
	{
		tmpm *= (m - i + 1)*x / i;//̩�ռ�����i��
		powf += tmpm;
	}
	return powf;
}
float Mpow(float num, float m)//����pow_f()��pow_i(),����num��m����,�Ǽ����ݵ����
{
	if (num > 0 && num < 2 && m>0 && (m - (int)m) != 0)
		return pow_f(num, (m - (int)m))*pow_i(num, (int)m);//�ֽ�Ϊ�������ݺ�С���������㣬��С�����������
	if (num > 2)//������֮��
		return Mpow(1 / num, -m);
	if (num == 2)//������߽磬�����ٶȷǳ�������˲������
		return Mpow(0.4, -m)*Mpow(0.8, m);
	if (m == 0)//�κε�����0����Ϊ1������0
		return 1;
	if (num == 0)
		return 0;
	if (m < 0)
		return 1 / Mpow(num, -m);
	//�������Ϊ�Ǹ�����С�����ݣ��ɿ�����һ������
	if (num < 0 && (m - (int)m) != 0)//������С�����ݣ����Ϊ����������0��ʾ�����������
		return 0;
	return pow_i(num, (int)m);//����ʵ�����������ݣ�����������
}
