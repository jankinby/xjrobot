#include "led.h" 



void LED_Init(void)
{  	
	RCC->AHB1ENR|=1<<7;	//ʹ��PORTHʱ�� 
	GPIO_Set(GPIOH,PIN9|PIN12,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); 
	
	LED0(0);			//�ر�
	LED1(0);			
}







