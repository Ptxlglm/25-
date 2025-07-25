#include "stm32f10x.h"

// PID控制器结构体
typedef struct {
    float Kp, Ki, Kd;       // PID参数
    float integral;          // 积分项
    float prev_error;        // 上一次误差
    float out_min, out_max;  // 输出限制
} PID_Controller;

// PID初始化
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float min, float max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->out_min = min;
    pid->out_max = max;
}

// PID计算函数
float PID_Compute(PID_Controller *pid, float setpoint, float input, float dt) {
    // 计算误差
    float error = setpoint - input;
    
    // 比例项
    float proportional = pid->Kp * error;
    
    // 积分项（带抗饱和）
    pid->integral += error * dt;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    else if (pid->integral < pid->out_min) pid->integral = pid->out_min;
    float integral = pid->Ki * pid->integral;
    
    // 微分项
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // 计算总输出
    float output = proportional + integral + derivative;
    
    // 限制输出范围
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;
    
    return output;
}
