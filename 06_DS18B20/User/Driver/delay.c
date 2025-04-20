#include "delay.h"

static uint8_t  fac_us=0;							//us延时倍乘数

void delay_init()
{
    fac_us = HAL_RCC_GetHCLKFreq() / 1000000;
}

void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD=nus*fac_us; 					//时间加载
    SysTick->VAL=0x00;        					//清空计数器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数
    do
    {
        temp=SysTick->CTRL;
    } while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
    SysTick->VAL =0X00;      					 //清空计数器
}

void delay_ms(uint16_t nms)
{
    for(int i=0; i<nms; i++)
        delay_us(1000);
}

/*
在 STM32F103（Cortex-M3 内核）中，DWT（Data Watchpoint and Trace）模块 
可以用来实现高精度的微秒级延时。虽然 HAL 库本身并没有封装 DWT 延时函数，
但我们可以自己启用 DWT 的周期计数器（CYCCNT）来实现。
*/

void DWT_Delay_Init(void)
{
    // 启用DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    // 计算目标周期数
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