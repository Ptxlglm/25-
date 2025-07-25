#include "stm32f10x.h"                  // Device header

void GRAY_Sensor_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_1 | GPIO_Pin_2 |
                                  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | 
                                  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

float Get_Track_Deviation(void) {      
    const float weights[7] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};		// 传感器位置权重（从左到右）
    
    float deviation = 0.0f;
    uint8_t active_count = 0;
    
    // 读取所有传感器
    for (int i = 0; i < 7; i++) {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0 << i) == 0) {
            deviation += weights[i];
            active_count++;
        }
    }
    
    // 计算平均偏差（无检测时维持前值）
    if (active_count > 0) {
        return deviation / active_count;
    } 
		// 完全偏离轨道时特殊处理
		else { 
        return (deviation > 0) ? 5.0f : -5.0f;
    }
}
