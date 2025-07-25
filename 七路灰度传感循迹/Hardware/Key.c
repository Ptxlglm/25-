#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t OpenKey_Get(void)
{
    static uint8_t keyState = 0;  // 静态变量，保存当前状态（0或1）
    uint8_t KeyNum = 0;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0)  // 检测按键按下
    {
        Delay_ms(20);  // 消抖
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0);  // 等待按键释放
        Delay_ms(20);  // 消抖

        keyState = !keyState;  // 切换状态（0变1，1变0）
        KeyNum = keyState;     // 返回当前状态
    }

    return KeyNum;
}

