#include "stm32f10x.h"
#include "GRAY_Sensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "PWM.h"
#include "TIM2.h"

// 主控参数设定区
#define BASE_SPEED      70      // 线速度基准，跟据场地与小车性能调节
#define DEVIATION_MAX   5.0f    // 最大允许的轨迹误差

extern PID_Controller pid_track;
 

void TIM2_IRQHandler(void)
{
	static float Right_Speed;
	static float Left_Speed;
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Right_Speed = Encoder_GetRightCount();
		Left_Speed = Encoder_GetLeftCount();
		
		
		// 采集位置偏差
		float deviation = Get_Track_Deviation();    // -3~3 或失踪时±5

		// 计算循迹PID输出
		float correction = PID_Compute(&pid_track, 0.0f, deviation, 0.01f); // 目标0, 当前deviation, 周期10ms=0.01s

		// 左右轮综合速度输出
		int left_output  = BASE_SPEED + correction;
		int right_output = BASE_SPEED - correction;

		// 限制最大输出
		if(left_output  > 100) left_output = 100;
		if(left_output  < -100) left_output = -100;
		if(right_output > 100) right_output = 100;
		if(right_output < -100) right_output = -100;

		// 设置电机速度与方向
		LMotor_SetSpeed((int8_t)left_output);
		RMotor_SetSpeed((int8_t)right_output);

		// 可以加上失踪信号的处理，例如大偏差直接刹
		if(deviation >= DEVIATION_MAX || deviation <= -DEVIATION_MAX)
		{
				LMotor_SetSpeed(0);
				RMotor_SetSpeed(0);
		}

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
