#include "stm32f10x.h"                  // Device header

void GRAY_Sensor_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_1 | GPIO_Pin_2 |
                                  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | 
                                  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

float Get_Track_Deviation(void) {      
    const float weights[7] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};		// ������λ��Ȩ�أ������ң�
    
    float deviation = 0.0f;
    uint8_t active_count = 0;
    
    // ��ȡ���д�����
    for (int i = 0; i < 7; i++) {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 << i) == 0) {
            deviation += weights[i];
            active_count++;
        }
    }
    
    // ����ƽ��ƫ��޼��ʱά��ǰֵ��
    if (active_count > 0) {
        return deviation / active_count;
    } 
		// ��ȫƫ����ʱ���⴦��
		else { 
        return (deviation > 0) ? 5.0f : -5.0f;
    }
}
