#include "PID.h"

/********************************����ʽPID����********************************
����ʽPID�ٶȻ�����
1st������Ki��������㣬����ᵼ�¹���
2nd������Kp�������񵴣�����ᵼ�´���ȵ���
3rd������Kd����������������ᵼ�µ�Ƶ��
******************************************************************************/
float Increment_PID_Cal(PID *s_PID, float now_point)
{
  s_PID->LastResult = s_PID->Result; //���ϴθú��������Result��ֵ����LastResult

  s_PID->Error = s_PID->SetPoint - now_point; //������

  // PID����
  s_PID->Result =   s_PID->LastResult                                                       //�ϴν��
                  + s_PID->Kp * (s_PID->Error - s_PID->LastError)                           // ������
                  + s_PID->Ki * s_PID->Error                                                // ������
                  + s_PID->Kd * (s_PID->Error - 2 * (s_PID->LastError) + s_PID->PrevError); // ΢����

  s_PID->PrevError = s_PID->LastError; // ���ϴ����ֵ(LastError)����ǰһ�����(PrevError)��
  s_PID->LastError = s_PID->Error;     // �����κ�����������(Error)�����ϴ������(LastError)

  PID_Output_limit(s_PID, &s_PID->Result); //����޷�

  return s_PID->Result;
}



/*******************************λ��ʽPID����************************************/
float Position_PID_Cal(PID *s_PID, float now_point)
{
	float IOutValue ;
  s_PID->LastResult = s_PID->Result; //���ϴθú��������Result��ֵ����LastResult

  s_PID->Error = s_PID->SetPoint - now_point; //������
	
	//s_PID->Error = s_PID->Error * 0.9f + s_PID->LastError * 0.2f;	//һ�׵�ͨ�˲�

  s_PID->SumError += s_PID->Error; //��������ۼ�
	
	
	
  IOutValue = s_PID->SumError * s_PID->Ki; //���ּ���
  PID_Integral_limit(s_PID, &IOutValue); //�����޷�

  // PID����
  s_PID->Result = s_PID->Kp * s_PID->Error                         // ������
                  + IOutValue                                      // SumError * Ki
                  + s_PID->Kd * (s_PID->Error - s_PID->LastError); // ΢����

  s_PID->PrevError = s_PID->LastError; // ���ϴ����ֵ(LastError)����ǰһ�����(PrevError)��
  s_PID->LastError = s_PID->Error;     // �����κ�����������(Error)�����ϴ������(LastError)

  PID_Output_limit(s_PID, &s_PID->Result); //����޷�

  return s_PID->Result;
}



/**-------------------------------------------------------------------------------------------------------------------
  * @brief    ƽ�⳵�ٶȻ�ר��PID
  *
  * @note     ƽ�⳵�ٶȻ�ר��PID  !!!!!!!!!
  *
  * @param    State_Flag  
  *           0: ֱ��ƽ��״̬
  *           1: ǰ��ģʽ
  *          -1: ����ģʽ
  *
  * @date     2022/7/2 22:38
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
float Balance_Car_Velocity_ONLY_Position_PID_Cal(PID *s_PID, float now_point , unsigned char State_Flag)
{
	float IOutValue ;
  s_PID->LastResult = s_PID->Result; //���ϴθú��������Result��ֵ����LastResult

  s_PID->Error = s_PID->SetPoint - now_point; //������
	
//	s_PID->Error = s_PID->Error * 0.8f + s_PID->LastError * 0.2f;	//һ�׵�ͨ�˲�
	
  s_PID->SumError += (s_PID->Error + State_Flag * 90 ); //��������ۼ�
	
  IOutValue = s_PID->SumError * s_PID->Ki; //���ּ���
  PID_Integral_limit(s_PID, &IOutValue); //�����޷�

  // PID����
  s_PID->Result = s_PID->Kp * s_PID->Error                         // ������
                  + IOutValue                                      // SumError * Ki
                  + s_PID->Kd * (s_PID->Error - s_PID->LastError); // ΢����

  s_PID->PrevError = s_PID->LastError; // ���ϴ����ֵ(LastError)����ǰһ�����(PrevError)��
  s_PID->LastError = s_PID->Error;     // �����κ�����������(Error)�����ϴ������(LastError)

  PID_Output_limit(s_PID, &s_PID->Result); //����޷�

  return s_PID->Result;
}


/*****************************��������ʽPID����**********************************/
float PID_Cal(PID *s_PID, float now_point) 
{
	float IOutValue;
  s_PID->LastResult = s_PID->Result; // �򵥸�ֵ����

  s_PID->Error = s_PID->SetPoint - now_point; //������

  s_PID->SumError += s_PID->Error; //��������ۼ�

  IOutValue = s_PID->SumError * s_PID->Ki; //���������

  PID_Integral_limit(s_PID, &IOutValue); //�����޷�

  // PID����
  s_PID->Result = s_PID->Kp * (s_PID->Error + IOutValue + s_PID->Kd * (s_PID->Error - s_PID->LastError));

  s_PID->PrevError = s_PID->LastError; // �򵥸�ֵ����
  s_PID->LastError = s_PID->Error;     // �򵥸�ֵ����
  //����޷�
  PID_Output_limit(s_PID, &s_PID->Result); //����޷�

  return s_PID->Result;
}



void PID_Init(PID *s_PID, float target, float PID_Kp, float PID_Ki, float PID_Kd) //��ʼ��PID�ṹ�����
{
  s_PID->SetPoint = target;
  s_PID->Kp = PID_Kp;
  s_PID->Ki = PID_Ki;
  s_PID->Kd = PID_Kd;
  s_PID->Error = 0;
  s_PID->LastError = 0;
  s_PID->PrevError = 0;
  s_PID->SumError = 0;
  s_PID->LastResult = 0;
  s_PID->Result = 0;
  s_PID->OutMax = DEFAULT_PID_OUT_MAX;
  s_PID->OutMin = DEFAULT_PID_OUT_MIN;
  s_PID->IntegralMax = DEFAULT_PID_INTEGRAL_OUT_MAX;
  s_PID->IntegralMin = DEFAULT_PID_INTEGRAL_OUT_MIN;
}

void PID_SetPoint(PID *s_PID, float target) //����Ŀ��ֵ
{
  s_PID->SetPoint = target;
}

void PID_Set_out_Range(PID *s_PID, float outMax, float outMin) //����PID�����Χ
{
  s_PID->OutMax = outMax;
  s_PID->OutMin = outMin;
}

void PID_Set_Integral_out_Range(PID *s_PID, float outMax, float outMin) //����PID���ַ�Χ
{
  s_PID->IntegralMax = outMax;
  s_PID->IntegralMin = outMin;
}

void PID_Integral_limit(PID *s_PID, float *IOutValue) //�����޷�
{
  if (*IOutValue > s_PID->IntegralMax)
  {
    *IOutValue = s_PID->IntegralMax;
  }
  else if (*IOutValue < s_PID->IntegralMin)
  {
    *IOutValue = s_PID->IntegralMin;
  }
}

void PID_Output_limit(PID *s_PID, float *Result) //����޷�
{
  if (*Result > s_PID->OutMax)
  {
    *Result = s_PID->OutMax;
  }
  else if (*Result < s_PID->OutMin)
  {
    *Result = s_PID->OutMin;
  }
}


float Yaw_PID(PID *s_PID, float now_point) //
{
  // PID����
  s_PID->Result = s_PID->Kp * now_point +  s_PID->Kd * (now_point - s_PID->LastResult);
	
  s_PID->LastResult = now_point; // �򵥸�ֵ����
	
  PID_Output_limit(s_PID, &s_PID->Result); //����޷�

  return s_PID->Result;
}

