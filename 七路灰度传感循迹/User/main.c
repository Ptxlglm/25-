#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "GRAY_Sensor.h"
#include "Encoder.h"
#include "PID.h"
#include "PWM.h"
#include "TIM2.h"

// PID������ѭ���ã��ʵ����Σ�
#define Kp_track    25.0f
#define Ki_track    0.7f
#define Kd_track    7.0f
PID_Controller pid_track;

// �ٶȻ�PID������Ҫ���е��ԣ�
#define Kp_speed    0.7f      
#define Ki_speed    0.12f
#define Kd_speed    0.10f
PID_Controller pid_right_speed;
PID_Controller pid_left_speed;

// ������
int main(void)
{
    Motor_Init();            // �����塢PWM��
    GRAY_Sensor_Init();      // �Ҷȴ������ӿ�
    Encoders_Init();         // �������ӿ�
    Timer_Init();            // 10ms��ʱѭ��
    PID_Init(&pid_track, Kp_track, Ki_track, Kd_track, 68.0f, 88.0f);    // ��������Χ��ȷ�����������ת
		PID_Init(&pid_right_speed, Kp_speed, Ki_speed, Kd_speed, 68.0f, 88.0f);
		PID_Init(&pid_left_speed, Kp_speed, Ki_speed, Kd_speed, 68.0f, 88.0f);
    // ����̬
    LMotor_SetSpeed(0);
    RMotor_SetSpeed(0);

    while (1)
    {
        // ��ѭ��Ϊ��ת/�����������չ
        // ʵ����һ��ʲô��������ȫ�����ж���ɵ���ѭ��
        // ����ɼӵ���LED�����������
    }
}
