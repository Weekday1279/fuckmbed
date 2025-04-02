#define _IMU_H

#include "Mahony.h"
#include <stdio.h>
#include <math.h>

extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����  angular acceleration
extern short gyrox,gyroy,gyroz;	//������ԭʼ����  gyroscope
extern float Roll,Pitch,Yaw;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;

	float Pitch_v;
	float Roll_v;
	float Yaw_v;

	float ax;
	float ay;
	float az;

} IMU_Info;
#include "Mahony.h"
#include <stdio.h>
#include <math.h>
extern IMU_Info imu;
extern float IMU_Temperature;


// void IMU_Init(void);
// void IMU_Update(void);
// void IMU_readGyro_Acc(signed short int *gyro, signed short int *acc);
// void IMU_Init_Offset(void);


