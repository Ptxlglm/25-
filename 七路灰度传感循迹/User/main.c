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

// PID参数（循迹用，适当调参）
#define Kp_track    25.0f
#define Ki_track    0.7f
#define Kd_track    7.0f

PID_Controller pid_track;


// 主函数
int main(void)
{
    Motor_Init();            // 驱动板、PWM等
    GRAY_Sensor_Init();      // 灰度传感器接口
    Encoders_Init();         // 编码器接口
    Timer_Init();            // 10ms定时循环
    PID_Init(&pid_track, Kp_track, Ki_track, Kd_track, 68.0f, 88.0f);    // 输出范围防止过调致饱和

    // 全部先稳态
    LMotor_SetSpeed(78);
    RMotor_SetSpeed(78);

    while (1)
    {
        // 主循环为裸转/随用情况补扩展
        // 实测中一般什么都不做，全部靠中断完成调速循迹
        // 这里可加调试LED、串口输出等
    }
}
