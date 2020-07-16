#include "timer.h"
#include "task.h"
#include "usart.h"
#include "engine.h"
//********************************************************************************
//修改说明
//V1.1 20160711
//新增TIM3_PWM_Init函数,用于PWM输出	
//V1.2 20160711
//新增TIM5_CH1_Cap_Init函数,用于输入捕获	
//V1.3 20160713
//新增TIM9_CH2_PWM_Init函数,用于PWM DAC实验					  
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

//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{ 		  
	if(TIM3->SR&0X0001)//溢出中断
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
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为54M
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值 
	TIM3->PSC=psc;  	//预分频器	  
	TIM3->DIER|=1<<0;   //允许更新中断	  
	TIM3->CR1|=0x01;    //使能定时器3
  MY_NVIC_Init(0,0,TIM3_IRQn,2);								 
}

//定时器5中断服务程序	 
void TIM5_IRQHandler(void)
{ 		  
	if(TIM5->SR&0X0001)//溢出中断
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
	TIM5->SR&=~(1<<0);//清除中断标志位 	    
}
void TIM5_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<3;	//TIM5时钟使能    
 	TIM5->ARR=arr;  	//设定计数器自动重装值 
	TIM5->PSC=psc;  	//预分频器	  
	TIM5->DIER|=1<<0;   //允许更新中断	  
	TIM5->CR1|=0x01;    //使能定时器5
  MY_NVIC_Init(0,1,TIM5_IRQn,2);								 
}







