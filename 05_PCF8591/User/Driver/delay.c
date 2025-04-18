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
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 //清空计数器	 
}

void delay_ms(uint16_t nms)
{	 		  	  
	for(int i=0;i<nms;i++)
		delay_us(1000);
} 

