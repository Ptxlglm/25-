#include "stm32f10x.h"

// ��������ʼ�������Ҹ�һ��
void Encoders_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    // 1. ����ʱ��ʹ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB1Periph_TIM3, ENABLE);

    // 2. ����GPIOΪ�������루������A/B�ࣩ
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    // ����������TIM1_CH3/CH4 -> PA10, PA11��
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // �Ҳ��������TIM3_CH1/CH2 -> PA6, PA7��
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. ����TIM1Ϊ�������ӿڣ���
    TIM_TimeBaseStruct.TIM_Prescaler = 0;  // ����Ƶ
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF; // ������ֵ
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM1, ENABLE); // ������ʱ��

    // 4. ����TIM3Ϊ�������ӿڣ��ң�
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM3, ENABLE); // ������ʱ��
}

// ��ȡ����������������TIM1��
int16_t Encoder_GetLeftCount(void) {
	int16_t Temp;
	Temp = TIM_GetCounter(TIM1);
	TIM_SetCounter(TIM1, 0);
	return Temp;
}

// ��ȡ�Ҳ��������������TIM3��
int16_t Encoder_GetRightCount(void) {
	int16_t Temp;
	Temp = TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	return Temp;
}
