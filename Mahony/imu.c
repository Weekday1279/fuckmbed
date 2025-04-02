#include "imu.h"
#include <math.h>

#define OFFSET_COUNT 100
#define FIFO_SIZE 10

signed short     int IMU_FIFO[6][FIFO_SIZE];
static unsigned char Wr_Index = 0; // FIFO���м�����

static float Pitch_offset;
static float Roll_offset;
static float Yaw_offset;
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ���� 
short gyrox,gyroy,gyroz;	//������ԭʼ���� 
float IMU_Temperature = 0 ; //IMU���õ��¶�	


typedef short int16_t;
typedef char u8;
typedef int int32_t;

IMU_Info imu;



/**-------------------------------------------------------------------------------------------------------------------
  * @brief    �����µ����ݷ���FIFO����
  *
  * @param    IMU_FIFO:   
	*           IMU_FIFO[0][]:GyroX��FIFO����
  *           IMU_FIFO[1][]:GyroY��FIFO����
  *           IMU_FIFO[2][]:GyroZ��FIFO����
  *           IMU_FIFO[3][]:AcceX��FIFO����
  *           IMU_FIFO[4][]:AcceY��FIFO����
  *           IMU_FIFO[5][]:AcceZ��FIFO����
	*
  * @param 	  IMU_val :
	*           IMU���õ�һ������ֵ
  *
  * @return   NULL
  *
  * @date     2022/6/19 17:35
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
// void IMU_NewVal(int16_t *IMU_FIFO, int16_t IMU_val)
// {
// 	IMU_FIFO[Wr_Index] = IMU_val;
// }




// /**-------------------------------------------------------------------------------------------------------------------
//   * @brief    ��FIFO�����е�����ȡƽ��ֵ
//   *
//   * @param    IMU_FIFO[0][]:GyroX��FIFO����
//   *           IMU_FIFO[1][]:GyroY��FIFO����
//   *           IMU_FIFO[2][]:GyroZ��FIFO����
//   *           IMU_FIFO[3][]:AcceX��FIFO����
//   *           IMU_FIFO[4][]:AcceY��FIFO����
//   *           IMU_FIFO[5][]:AcceZ��FIFO����
//   *
//   * @return   ������ֵ�˲���6������
//   *
//   * @date     2022/6/20 14:28
//   *
//   * @author   WxxGreat
//   *-----------------------------------------------------------------------------------------------------------------*/
// int16_t IMU_GetAvg(int16_t *IMU_FIFO)
// {
// 	u8 i;
// 	int32_t sum = 0;
// 	for (i = 0; i < FIFO_SIZE; i++)
// 		sum += IMU_FIFO[i];
// 	sum = sum / FIFO_SIZE;
// 	return (int16_t)sum;
// }




// /**-------------------------------------------------------------------------------------------------------------------
//   * @brief    ���þ�����ֵ�˲�����IMU 6��ԭʼֵ
//   *
//   * @param    IMU_values[0]:GyroX
//   *           IMU_values[1]:GyroY
//   *           IMU_values[2]:GyroZ
//   *           IMU_values[3]:AcceX
//   *           IMU_values[4]:AcceY
//   *           IMU_values[5]:AcceZ
//   *
//   * @return   6������ԭʼֵ
//   *
//   * @date     2022/6/23 16:37
//   *
//   * @author   WxxGreat
//   *-----------------------------------------------------------------------------------------------------------------*/
//  void Get_IMU_Values(float *IMU_values)
// {
// 	short i = 0;
// 	int16_t gyro[3], acc[3];

// 	IMU_readGyro_Acc(&gyro[0], &acc[0]);
	
// 	for (; i < 3; i++)
// 	{
// 		IMU_values[i] = ((float)gyro[i]) / 16.4f; // gyro range: +-2000; adc accuracy 16 bits: 2^16=65536; 65536/4000=16.4; so  1^-> 16.4
// 		IMU_values[3 + i] = (float)acc[i];
// 	}
// }



// /**-------------------------------------------------------------------------------------------------------------------
//   * @brief    ��ȡ6��ԭʼֵ�����о�ֵ�˲�
//   *
//   * @param    gyro[]: ������ֵ�˲��Ľ��ٶ�ֵ��ַ
//   * @param    acce[]: ������ֵ�˲��ļ��ٶ�ֵ��ַ
//   *
//   * @return   ������ֵ�˲���6������
//   *
//   * @date     2022/6/22 15:46
//   *
//   * @author   WxxGreat
//   *-----------------------------------------------------------------------------------------------------------------*/
// void IMU_readGyro_Acc(int16_t *gyro, int16_t *acce)
// {
// 	static short buf[6];
// 	static int16_t gx, gy, gz;
// 	static int16_t ax, ay, az;

// #if USE_ICM42605 == 1
// 	Get_ICM42605_gyro(&buf[0], &buf[1], &buf[2]);
// 	Get_ICM42605_accdata(&buf[3], &buf[4], &buf[5]);
// #else
// 	MPU_Get_Gyroscope(&buf[0], &buf[1], &buf[2]);
// 	MPU_Get_Accelerometer(&buf[3], &buf[4], &buf[5]);
// #endif
	
// 	IMU_NewVal(&IMU_FIFO[0][0], buf[0]);
// 	IMU_NewVal(&IMU_FIFO[1][0], buf[1]);
// 	IMU_NewVal(&IMU_FIFO[2][0], buf[2]);

// 	IMU_NewVal(&IMU_FIFO[3][0], buf[3]);
// 	IMU_NewVal(&IMU_FIFO[4][0], buf[4]);
// 	IMU_NewVal(&IMU_FIFO[5][0], buf[5]);

// 	Wr_Index = (Wr_Index + 1) % FIFO_SIZE;

// 	gx = IMU_GetAvg(&IMU_FIFO[0][0]);
// 	gy = IMU_GetAvg(&IMU_FIFO[1][0]);
// 	gz = IMU_GetAvg(&IMU_FIFO[2][0]);

// 	gyro[0] = gx - Roll_offset;
// 	gyro[1] = gy - Pitch_offset;
// 	gyro[2] = gz - Yaw_offset;

// 	ax = IMU_GetAvg(&IMU_FIFO[3][0]);
// 	ay = IMU_GetAvg(&IMU_FIFO[4][0]);
// 	az = IMU_GetAvg(&IMU_FIFO[5][0]);

// 	acce[0] = ax;
// 	acce[1] = ay;
// 	acce[2] = az;
	
// }


// /**-------------------------------------------------------------------------------------------------------------------
//   * @brief    �����ڶ�ʱ���е��õĺ���
//   *
//   * @param    NULL
//   *
//   * @return   Roll��Pitch
//   *
//   * @date     2022/6/23 17:02
//   *
//   * @author   WxxGreat
//   *-----------------------------------------------------------------------------------------------------------------*/
// void IMU_Update(void)
// {
// 	static float q[4];
// 	float Values[6];
// 	Get_IMU_Values(Values);
// #if USE_ICM42605 == 1
// 	    Get_ICM42605_Tempdata(&IMU_Temperature);
// #else
//       //  MPU_Get_Temperature(&IMU_Temperature);
// #endif
	
	
// 	//���Ƕȸ���Ϊ���ȣ�ʹ��Mahony����
// 	MahonyAHRSupdateIMU(Values[0] * 3.14159265358979f / 180, Values[1] * 3.14159265358979f / 180, Values[2] * 3.14159265358979f / 180,Values[3], Values[4], Values[5]);

// 	//������Ԫ��
// 	q[0] = q0;
// 	q[1] = q1;
// 	q[2] = q2;
// 	q[3] = q3;

// 	imu.ax = Values[3];
// 	imu.ay = Values[4];
// 	imu.az = Values[5];

// 	imu.Pitch_v = Values[0];
// 	imu.Roll_v = Values[1];
// 	imu.Yaw_v = Values[2];

//   //��Ԫ��������ŷ����
// 	imu.Roll = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / 3.14159265358979f;
// //imu.Pitch = -safe_asin(2.0f * (q[0] * q[2] - q[1] * q[3])) * 180 / 3.14159265358979f;
// //  imu.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/3.14159265358979f;
// //  if(Yaw >=  360  ) Yaw = 0;
// //	if(Yaw <= -360  ) Yaw = 0;			
// //  imu.Yaw += ((double)(-gyroz + Yaw_offset) * 3.1415926f / 360 / 32.8f * 0.1 * 1.5 );//1.5��ϵ��

// }



// /**-------------------------------------------------------------------------------------------------------------------
//   * @brief    ����IMU������ƫ��
//   *
//   * @note     ʹ��Mahony�㷨ȥ��IMU����ƫ�Ի������õĹ���ֵ
//   *
//   * @return   Roll_offset ��Pitch_offset  ��Yaw_offset
//   *
//   * @date     2022/6/19 17:00
//   *
//   * @author   WxxGreat
//   *-----------------------------------------------------------------------------------------------------------------*/
// void IMU_Init_Offset(void)
// {
// 	short i;
// 	int tempgx = 0, tempgy = 0, tempgz = 0;
// 	int tempax = 0, tempay = 0, tempaz = 0;
// 	Pitch_offset = 0;
// 	Roll_offset = 0;
// 	Yaw_offset = 0;
//   delay_ms(10);
	
// 	// read the mpu data for calculate the offset
// 	for (i = 0; i < OFFSET_COUNT; i++)
// 	{
// 		delay_ms(5);
// #if USE_ICM42605 == 1
// 		Get_ICM42605_accdata(&aacx,&aacy,&aacz);  //�õ����ٶȴ���������
//     Get_ICM42605_gyro(&gyrox,&gyroy,&gyroz);  //�õ�����������
// 	  Get_ICM42605_Tempdata(&IMU_Temperature);
// #else
// 		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//     MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
// 		MPU_Get_Temperature(&IMU_Temperature); 
// #endif
// 		tempgx += gyrox;
// 		tempgy += gyroy;
// 		tempgz += gyroz;

// 		tempax += aacx;
// 		tempay += aacy;
// 		tempaz += aacz;
// 	}
		
// 	Roll_offset  = tempgx / OFFSET_COUNT;
// 	Pitch_offset = tempgy / OFFSET_COUNT;
// 	Yaw_offset   = tempgz / OFFSET_COUNT;

// 	//printf("Pitch_offset = %f,Roll_offset = %f,Yaw_offset = %f\r\n", Pitch_offset, Roll_offset, Yaw_offset);
// }



// /**-------------------------------------------------------------------------------------------------------------------
//   * @brief    IMU��ʼ������
//   *
//   * @note     Ҫ��ICM42605����Keil5:  "ħ���� -> C/C++ -> Define"   ʹ��USE_ICM42605 = 1
//   *
//   * @return   NULL
//   *
//   * @date     2022/6/19 17:00
//   *
//   * @author   WxxGreat
//   *-----------------------------------------------------------------------------------------------------------------*/
// void IMU_Init(void)
// {
//  	char Err;
// 	MPU_RE_INIT:
// #if  USE_ICM42605 == 1
// 	Err = ICM42605_init();
// #else
// 	Err = MPU_Init();
// #endif
// 	if (Err != 0)
// 	{
// 		LCD_printf(0,90,"IMU error:%d",Err);
// 		goto MPU_RE_INIT;
// 	}
	
// 	delay_ms(10);
	
// 	LCD_ShowChinese(0,90,"������У׼��",0xFFFF,0x0000,32,0);
// 	LCD_ShowChar(187,90,':',0xFFFF,0x0000,32,0);//��ʾһ���ַ�
	
// 	IMU_Init_Offset();
// }


// //DeBug����
// IMU_Info *IMU_GetInfo(void) { return &imu; }

//arcsin����

