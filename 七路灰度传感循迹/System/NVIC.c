#include "stm32f10x.h"
#include "GRAY_Sensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "PWM.h"
#include "TIM2.h"

// ���ز����趨��
#define BASE_SPEED      78      // ���ٶȻ�׼�����ݳ�����С�����ܵ���
#define DEVIATION_MAX   5.0f    // �������Ĺ켣���

extern PID_Controller pid_track;
extern PID_Controller pid_left_speed;
extern PID_Controller pid_right_speed;
 

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
		float left_target  = BASE_SPEED + correction;
		float right_target = BASE_SPEED - correction;
		
		//�ٶ�PID�ջ�����
		float left_pwm = PID_Compute(&pid_left_speed, left_target,  Left_Speed,  0.01f);   // Left_Speedʵ��ֵ
    float right_pwm= PID_Compute(&pid_right_speed, right_target, Right_Speed, 0.01f);  // Right_Speedʵ��ֵ


		// ����������
		if(left_pwm  > 88) left_pwm = 88;
		if(left_pwm  < 68) left_pwm = 68;
		if(right_pwm > 88) right_pwm = 88;
		if(right_pwm < 68) right_pwm = 88;

		// ���õ���ٶ��뷽��
		LMotor_SetSpeed((int8_t)left_pwm);
		RMotor_SetSpeed((int8_t)right_pwm);

		// ���Լ���ʧ���źŵĴ��������ƫ��ֱ��ɲ
		if(deviation >= DEVIATION_MAX || deviation <= -DEVIATION_MAX)
		{
				LMotor_SetSpeed(0);
				RMotor_SetSpeed(0);
		}

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
