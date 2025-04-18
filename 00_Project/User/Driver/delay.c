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
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      					 //��ռ�����	 
}

void delay_ms(uint16_t nms)
{	 		  	  
	for(int i=0;i<nms;i++)
		delay_us(1000);
} 

