#include "stm32f10x.h"

// 编码器初始化：左右各一个
void Encoders_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    // 1. 外设时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB1Periph_TIM3, ENABLE);

    // 2. 配置GPIO为浮空输入（编码器A/B相）
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    // 左侧编码器（TIM1_CH3/CH4 -> PA10, PA11）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 右侧编码器（TIM3_CH1/CH2 -> PA6, PA7）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. 配置TIM1为编码器接口（左）
    TIM_TimeBaseStruct.TIM_Prescaler = 0;  // 不分频
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF; // 最大计数值
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM1, ENABLE); // 启动定时器

    // 4. 配置TIM3为编码器接口（右）
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM3, ENABLE); // 启动定时器
}

// 获取左侧编码器脉冲数（TIM1）
int16_t Encoder_GetLeftCount(void) {
	int16_t Temp;
	Temp = TIM_GetCounter(TIM1);
	TIM_SetCounter(TIM1, 0);
	return Temp;
}

// 获取右侧编码器脉冲数（TIM3）
int16_t Encoder_GetRightCount(void) {
	int16_t Temp;
	Temp = TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	return Temp;
}
