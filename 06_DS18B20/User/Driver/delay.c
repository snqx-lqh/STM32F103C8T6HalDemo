#include "delay.h"

static uint8_t  fac_us=0;							//us��ʱ������

void delay_init()
{
    fac_us = HAL_RCC_GetHCLKFreq() / 1000000;
}

void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD=nus*fac_us; 					//ʱ�����
    SysTick->VAL=0x00;        					//��ռ�����
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����
    do
    {
        temp=SysTick->CTRL;
    } while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
    SysTick->VAL =0X00;      					 //��ռ�����
}

void delay_ms(uint16_t nms)
{
    for(int i=0; i<nms; i++)
        delay_us(1000);
}

/*
�� STM32F103��Cortex-M3 �ںˣ��У�DWT��Data Watchpoint and Trace��ģ�� 
��������ʵ�ָ߾��ȵ�΢�뼶��ʱ����Ȼ HAL �Ȿ��û�з�װ DWT ��ʱ������
�����ǿ����Լ����� DWT �����ڼ�������CYCCNT����ʵ�֡�
*/

void DWT_Delay_Init(void)
{
    // ����DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    // ����Ŀ��������
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - clk_cycle_start) < ticks);
}

void DWT_Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        DWT_Delay_us(1000);
    }
}