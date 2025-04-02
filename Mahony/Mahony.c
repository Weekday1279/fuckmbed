#include "Mahony.h"
#include "math.h"

#define IMU_Update_Freq 200	 // Ƶ�ʣ�Hz����������IMU_Update()���õ�Ƶ�ʡ�
#define twoKpDef (2.0f * 2.50f)  // 2 * proportional gain
#define twoKiDef (2.0f * 0.08f) // 2 * integral gain

volatile float twoKp = twoKpDef; // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef; // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;				         // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // ��KiΪϵ���Ļ���������

static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


// IMU�㷨����
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;			  
	float halfvx, halfvy, halfvz; 
	float halfex, halfey, halfez; 
	float qa, qb, qc;


  //ֻ���ڼ��ٶȼƲ�����Чʱ�ż��㷴�����������ٶȼ�����ʱ����NaN��
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;


	halfvx = q1 * q3 - q0 * q2;
	halfvy = q0 * q1 + q2 * q3;
	halfvz = q0 * q0 - 0.5f + q3 * q3;


	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);


  // ��KiΪ�߶ȶ���������
	integralFBx += twoKi * halfex * (1.0f / IMU_Update_Freq); 
	integralFBy += twoKi * halfey * (1.0f / IMU_Update_Freq);
	integralFBz += twoKi * halfez * (1.0f / IMU_Update_Freq);
	
	// Ӧ�û��ַ���
	gx += integralFBx; 
	gy += integralFBy;
	gz += integralFBz;


	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;

  // Ԥ�ȳ��Թ������� IMU_Update_Freq
	gx *= (0.5f * (1.0f / IMU_Update_Freq)); 
	gy *= (0.5f * (1.0f / IMU_Update_Freq));
	gz *= (0.5f * (1.0f / IMU_Update_Freq));
	
	qa = q0;
	qb = q1;
	qc = q2;

	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += ( qa * gx + qc * gz - q3 * gy);
	q2 += ( qa * gy - qb * gz + q3 * gx);
	q3 += ( qa * gz + qb * gy - qc * gx);

	// ��һ����Ԫ��
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
