#include "timer.h"
#include "task.h"
#include "usart.h"
#include "engine.h"
//********************************************************************************
//�޸�˵��
//V1.1 20160711
//����TIM3_PWM_Init����,����PWM���	
//V1.2 20160711
//����TIM5_CH1_Cap_Init����,�������벶��	
//V1.3 20160713
//����TIM9_CH2_PWM_Init����,����PWM DACʵ��					  
////////////////////////////////////////////////////////////////////////////////// 

u16 ledCount=0;
u16 ps2RecCount=0;
u16 Usart1DataResCount=0;
u16 positionRequest=0;
u16 OdomCount=0;
u16 underplantimeCnt=0;
u32 carsport_state=0;
u16 motor_state=0;
u16 time_test=0;

//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{ 		  
	if(TIM3->SR&0X0001)//����ж�
	{
		ledCount++;
		ps2RecCount++;
		USART1_Rx_Time_count++;
		positionRequest++;
	//	OdomCount++;
		engineStarcnt++;
		engineRecCount++;
		enginekeepRuntimcnt++;
//		underplantimeCnt++;
//		enginekeeptime++;
//		carsport_state++;
//		motor_state++;
	}			   
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ54M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3ʱ��ʹ��    
 	TIM3->ARR=arr;  	//�趨�������Զ���װֵ 
	TIM3->PSC=psc;  	//Ԥ��Ƶ��	  
	TIM3->DIER|=1<<0;   //��������ж�	  
	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
  MY_NVIC_Init(0,0,TIM3_IRQn,2);								 
}

//��ʱ��5�жϷ������	 
void TIM5_IRQHandler(void)
{ 		  
	if(TIM5->SR&0X0001)//����ж�
	{
//		ledCount++;
//		ps2RecCount++;
//		USART1_Rx_Time_count++;
//		positionRequest++;
  		OdomCount++;
//		engineStarcnt++;
//		engineRecCount++;
//		enginekeepRuntimcnt++;
//		underplantimeCnt++;
//		enginekeeptime++;
//		carsport_state++;
//		motor_state++;
//		time_test++;
	}			   
	TIM5->SR&=~(1<<0);//����жϱ�־λ 	    
}
void TIM5_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<3;	//TIM5ʱ��ʹ��    
 	TIM5->ARR=arr;  	//�趨�������Զ���װֵ 
	TIM5->PSC=psc;  	//Ԥ��Ƶ��	  
	TIM5->DIER|=1<<0;   //��������ж�	  
	TIM5->CR1|=0x01;    //ʹ�ܶ�ʱ��5
  MY_NVIC_Init(0,1,TIM5_IRQn,2);								 
}







