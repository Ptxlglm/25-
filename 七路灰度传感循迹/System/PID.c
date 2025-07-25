#include "stm32f10x.h"

// PID�������ṹ��
typedef struct {
    float Kp, Ki, Kd;       // PID����
    float integral;          // ������
    float prev_error;        // ��һ�����
    float out_min, out_max;  // �������
} PID_Controller;

// PID��ʼ��
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float min, float max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->out_min = min;
    pid->out_max = max;
}

// PID���㺯��
float PID_Compute(PID_Controller *pid, float setpoint, float input, float dt) {
    // �������
    float error = setpoint - input;
    
    // ������
    float proportional = pid->Kp * error;
    
    // ������������ͣ�
    pid->integral += error * dt;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    else if (pid->integral < pid->out_min) pid->integral = pid->out_min;
    float integral = pid->Ki * pid->integral;
    
    // ΢����
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // ���������
    float output = proportional + integral + derivative;
    
    // ���������Χ
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;
    
    return output;
}
