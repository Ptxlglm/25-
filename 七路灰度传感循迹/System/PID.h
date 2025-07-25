#ifndef __PID_H__
#define __PID_H__

// PID控制器结构体定义
typedef struct {
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数
    float integral;      // 积分项
    float prev_error;    // 上一次误差
    float out_min;       // 输出最小值（限幅）
    float out_max;       // 输出最大值（限幅）
} PID_Controller;

// PID控制器初始化函数
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float min, float max);

// PID计算函数
float PID_Compute(PID_Controller *pid, float setpoint, float input, float dt);

#endif
