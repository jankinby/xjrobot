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
	u8 mode=0;	//CAN����ģʽ;0,��ͨģʽ;1,����ģʽ
	Stm32_Clock_Init(432,25,2,9);//����ʱ��,216Mhz
  delay_init(216);			//��ʱ��ʼ��	
	LED_Init();	
	CAN1_Mode_Init(1,7,10,6,mode);	//CAN��ʼ��,������500Kbps 
	MOTOR_init();	
	//���� 2-5 7-8 ΪPLCK1 
	//����  1 �� 6 ΪPCLK2
	USART1_Init(108,115200);		//��ʼ������1������Ϊ115200  
 	USART3_Init(54,115200);//ң�������ڳ�ʼ��
	UART4_Init(54,115200);	
	// �����ʿ��ܻ�������
	UART8_Init(54,38400);       // ��ʼ������8������Ϊ38400    �뷢�������ư�ͨ��	
	UART5_Init(54,9600);      // ��ʼ�� Ϊ9600   �����Ϣ�ɼ�
	TIM3_Init(540-1,2-1);  //������ʱ��3  10us
	TIM5_Init(540-1,2-1);  //������ʱ��5  10us
	data_init(0x01);//01��ܳ� 02 G33 03 G60
	while(1)
	{
		creatTask();
		// EngineSettings(0x00,0x5A);
		//underPlantUp();
	}
	
}

















