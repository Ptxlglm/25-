#include "stm32f10x.h"
#include "GRAY_Sensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "PWM.h"
#include "TIM2.h"

// ���ز����趨��
#define BASE_SPEED      70      // ���ٶȻ�׼�����ݳ�����С�����ܵ���
#define DEVIATION_MAX   5.0f    // �������Ĺ켣���

extern PID_Controller pid_track;
 

void TIM2_IRQHandler(void)
{
	static float Right_Speed;
	static float Left_Speed;
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Right_Speed = Encoder_GetRightCount();
		Left_Speed = Encoder_GetLeftCount();
		
		
		// �ɼ�λ��ƫ��
		float deviation = Get_Track_Deviation();    // -3~3 ��ʧ��ʱ��5

		// ����ѭ��PID���
		float correction = PID_Compute(&pid_track, 0.0f, deviation, 0.01f); // Ŀ��0, ��ǰdeviation, ����10ms=0.01s

		// �������ۺ��ٶ����
		int left_output  = BASE_SPEED + correction;
		int right_output = BASE_SPEED - correction;

		// ����������
		if(left_output  > 100) left_output = 100;
		if(left_output  < -100) left_output = -100;
		if(right_output > 100) right_output = 100;
		if(right_output < -100) right_output = -100;

		// ���õ���ٶ��뷽��
		LMotor_SetSpeed((int8_t)left_output);
		RMotor_SetSpeed((int8_t)right_output);

		// ���Լ���ʧ���źŵĴ��������ƫ��ֱ��ɲ
		if(deviation >= DEVIATION_MAX || deviation <= -DEVIATION_MAX)
		{
				LMotor_SetSpeed(0);
				RMotor_SetSpeed(0);
		}

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
