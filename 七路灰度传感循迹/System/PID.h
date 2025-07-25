#ifndef __PID_H__
#define __PID_H__

// PID�������ṹ�嶨��
typedef struct {
    float Kp;            // ����ϵ��
    float Ki;            // ����ϵ��
    float Kd;            // ΢��ϵ��
    float integral;      // ������
    float prev_error;    // ��һ�����
    float out_min;       // �����Сֵ���޷���
    float out_max;       // ������ֵ���޷���
} PID_Controller;

// PID��������ʼ������
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float min, float max);

// PID���㺯��
float PID_Compute(PID_Controller *pid, float setpoint, float input, float dt);

#endif
