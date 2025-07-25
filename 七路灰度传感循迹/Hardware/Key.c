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
    static uint8_t keyState = 0;  // ��̬���������浱ǰ״̬��0��1��
    uint8_t KeyNum = 0;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0)  // ��ⰴ������
    {
        Delay_ms(20);  // ����
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0);  // �ȴ������ͷ�
        Delay_ms(20);  // ����

        keyState = !keyState;  // �л�״̬��0��1��1��0��
        KeyNum = keyState;     // ���ص�ǰ״̬
    }

    return KeyNum;
}

