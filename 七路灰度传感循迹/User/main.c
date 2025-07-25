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


// ������
int main(void)
{
    Motor_Init();            // �����塢PWM��
    GRAY_Sensor_Init();      // �Ҷȴ������ӿ�
    Encoders_Init();         // �������ӿ�
    Timer_Init();            // 10ms��ʱѭ��
    PID_Init(&pid_track, Kp_track, Ki_track, Kd_track, 68.0f, 88.0f);    // �����Χ��ֹ�����±���

    // ȫ������̬
    LMotor_SetSpeed(78);
    RMotor_SetSpeed(78);

    while (1)
    {
        // ��ѭ��Ϊ��ת/�����������չ
        // ʵ����һ��ʲô��������ȫ�����ж���ɵ���ѭ��
        // ����ɼӵ���LED�����������
    }
}
