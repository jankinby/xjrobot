#include "led.h" 



void LED_Init(void)
{  	
	RCC->AHB1ENR|=1<<7;	//使能PORTH时钟 
	GPIO_Set(GPIOH,PIN9|PIN12,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); 
	
	LED0(0);			//关闭
	LED1(0);			
}







