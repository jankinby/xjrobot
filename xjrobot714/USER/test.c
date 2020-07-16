#include "sys.h"
#include "delay.h" 
#include "led.h"  
#include "mpu.h" 
#include "usart.h"  
#include "sdram.h"  
#include "usmart.h"
#include "can.h"
#include "can_drive.h"
#include "task.h"
#include "timer.h"
#include "engine.h"
#include "battery.h"


 
int main(void)
{  
	u8 mode=0;	//CAN工作模式;0,普通模式;1,环回模式
	Stm32_Clock_Init(432,25,2,9);//设置时钟,216Mhz
  delay_init(216);			//延时初始化	
	LED_Init();	
	CAN1_Mode_Init(1,7,10,6,mode);	//CAN初始化,波特率500Kbps 
	MOTOR_init();	
	//串口 2-5 7-8 为PLCK1 
	//串口  1 和 6 为PCLK2
	USART1_Init(108,115200);		//初始化串口1波特率为115200  
 	USART3_Init(54,115200);//遥控器串口初始化
	UART4_Init(54,115200);	
	// 波特率可能会有问题
	UART8_Init(54,38400);       // 初始化串口8波特率为38400    与发动机控制板通信	
	UART5_Init(54,9600);      // 初始化 为9600   电池信息采集
	TIM3_Init(540-1,2-1);  //开启定时器3  10us
	TIM5_Init(540-1,2-1);  //开启定时器5  10us
	data_init(0x01);//01框架车 02 G33 03 G60
	while(1)
	{
		creatTask();
		// EngineSettings(0x00,0x5A);
		//underPlantUp();
	}
	
}

















