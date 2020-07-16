#include "sys.h"
#include "usart.h"	  
//#include "imu.h"      //�ߵ�����


#if 1
#pragma import(__use_no_semihosting)  
//���HAL��ʹ��ʱ,ĳЩ������ܱ����bug
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->ISR&0X40)==0);//ѭ������,ֱ���������   
	USART1->TDR = (u8) ch;      
	return ch;
}
#endif 
//end


//////////////////////////////////////////////////////////////////
//����1�жϷ������ 	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_STA=0;                 //����״̬���	

uint8_t USART1_Rx_Time_count=0;    
uint8_t USART1_RX_BUF[USART_RX_ARRAY_NUM][USART_REC_LEN]; //���ջ���
uint8_t Usart1RxLength[USART_RX_ARRAY_NUM];//���ջ��峤��
uint8_t Usart1RxHead = 0;//����ͷ
uint8_t Usart1Rxend = 0;//����β
uint8_t Usart1RxNum = 0;//�Ѵ�������
uint8_t Rx_count_UART1=0;


int t=0;
uint8_t USART4_RX_BUF[USART_RX_ARRAY_NUM][USART_REC_LEN]; //���ջ���
uint8_t Usart4RxLength[USART_RX_ARRAY_NUM];//���ջ��峤��
uint8_t Usart4RxHead = 0;//����ͷ
uint8_t Usart4Rxend = 0;//����β
uint8_t Usart4RxNum = 0;//�Ѵ�������


void USART1_IRQHandler(void)
{
	u8 res;	
	if(USART1->ISR&(1<<5))//���յ�����
	{	 
		  res=USART1->RDR; 
	    USART1_Rx_Time_count=0;                         //�����ʱ
		if( Rx_count_UART1 < USART_REC_LEN)
		{
			USART1_RX_BUF[Usart1Rxend][Rx_count_UART1] = res;    //���ܵ����ݷ���USART1���ݻ���
			Rx_count_UART1++;                               //���ݻ���ָ��������
		}      
	}
	USART1->ICR |=0x0008;
} 										 
//��ʼ��IO ����1 ��λ��
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void USART1_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	
	temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
	RCC->AHB1ENR|=1<<1;   	//ʹ��PORTB��ʱ��  
	RCC->APB2ENR|=1<<4;  	//ʹ�ܴ���1ʱ�� 
	GPIO_Set(GPIOB,PIN14|PIN15,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//PB14,PB15,���ù���,�������
 	GPIO_AF_Set(GPIOB,14,4);	//PB14,AF4
	GPIO_AF_Set(GPIOB,15,4);//PB15,AF4
	//����������
 	USART1->BRR=temp; 		//����������@OVER8=0 	
	USART1->CR1=0;		 	//����CR1�Ĵ���
	USART1->CR1|=0<<28;	 	//����M1=0
	USART1->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
	USART1->CR1|=0<<15; 	//����OVER8=0,16�������� 
	USART1->CR1|=1<<3;  	//���ڷ���ʹ�� 

	//ʹ�ܽ����ж� 
	USART1->CR1|=1<<2;  	//���ڽ���ʹ��
	USART1->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,2,USART1_IRQn,2);//��2��������ȼ� 

	USART1->CR1|=1<<0;  	//����ʹ��
}







/*
void USART2_IRQHandler(void)      
{
	if(USART2->ISR&(1<<5))//���յ�����
	{	 
	}
	USART2->ICR |=0x0008;
	
}



//����2��ʼ��
void USART2_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
	
	
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<17;  	//ʹ�ܴ���2ʱ�� 
	
	
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOA,2,7);	//PB10,AF7����
	GPIO_AF_Set(GPIOA,3,7);  //PB11,AF7����
	//����������
 	USART2->BRR=temp; 		//����������@OVER8=0 	
	USART2->CR1=0;		 	//����CR1�Ĵ���
	USART2->CR1|=0<<28;	 	//����M1=0
	USART2->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
	USART2->CR1|=0<<10;	 
	USART2->CR1|=0<<15; 	//����OVER8=0,16�������� 
	USART2->CR1|=1<<3;  	//���ڷ���ʹ�� 
	//ʹ�ܽ����ж� 
	USART2->CR1|=1<<2;  	//���ڽ���ʹ��
	USART2->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1,1,USART2_IRQn,2);//��2��������ȼ� 
	USART2->CR1|=1<<0;  	//����ʹ��
}*/






u8 Ps2Receive[10];
u8 Ps2ReceiveCount=0;
void USART3_IRQHandler(void)
{
	u8 data;	
	if(USART3->ISR&(1<<5))//���յ�����
	{	 
		data=USART3->RDR; 
 		Ps2Receive[Ps2ReceiveCount]=data;
		Ps2ReceiveCount++;
	}
	USART3->ICR |=0x0008;
	
}

//����3��ʼ�� ң��
void USART3_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
	RCC->AHB1ENR|=1<<1;   	//ʹ��PORTB��ʱ��  
	RCC->APB1ENR|=1<<18;  	//ʹ�ܴ���3ʱ�� 
	GPIO_Set(GPIOB,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOB,10,7);	//PB10,AF7
	GPIO_AF_Set(GPIOB,11,7);  //PB11,AF7
	//����������
 	USART3->BRR=temp; 		//����������@OVER8=0 	
	USART3->CR1=0;		 	//����CR1�Ĵ���
	USART3->CR1|=0<<28;	 	//����M1=0
	USART3->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
	USART3->CR1|=0<<10;	 
	USART3->CR1|=0<<15; 	//����OVER8=0,16�������� 
	
	USART3->CR1|=1<<3;  	//���ڷ���ʹ�� 
	USART3->CR1|=1<<2;  	//���ڽ���ʹ��
	USART3->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	 
	
	MY_NVIC_Init(1,0,USART3_IRQn,2);//��2��������ȼ� 
	USART3->CR1|=1<<0;  	//����ʹ��
}


//  ����4    IMU���ݽ���
u8 IMUDatareceive[54];        //IMU���ݽ������� 
u8 IMUDatareceiveCount=0;     //IMU���ݽ��ռ���
void UART4_IRQHandler(void)      	
{
	u8 data;	
	if(UART4->ISR&(1<<5))//���յ�����
	{
		data=UART4->RDR; 
		IMUDatareceive[IMUDatareceiveCount]=data;
		IMUDatareceiveCount++;			
	}
	UART4->ICR |=0x0008;
	
}
//����4��ʼ��
void UART4_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
	
	
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<19;  	//ʹ��UART����4ʱ�� 
	
	
	GPIO_Set(GPIOA,PIN11|PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOA,12,6);	//PA12, AF6  ,UART4_Tx
	GPIO_AF_Set(GPIOA,11,6);  //PA11,  AF6,U4_Rx
	//����������
 	UART4->BRR=temp; 		//����������@OVER8=0 	
	UART4->CR1=0;		 	//����CR1�Ĵ���
	UART4->CR1|=0<<28;	 	//����M1=0
	UART4->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
	UART4->CR1|=0<<10;	 
	UART4->CR1|=0<<15; 	//����OVER8=0,16�������� 
	UART4->CR1|=1<<3;  	//���ڷ���ʹ�� 
	//ʹ�ܽ����ж� 
	UART4->CR1|=1<<2;  	//���ڽ���ʹ��
	UART4->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1,2,UART4_IRQn,2);//��2��������ȼ� 
	UART4->CR1|=1<<0;  	//����ʹ��
}




//������
u8 engineReceive[10];
u8 engineReceiveCount=0;
void UART8_IRQHandler(void)
{
	u8 data;	
	if(UART8->ISR&(1<<5))//���յ�����
	{	 
		data=UART8->RDR; 
 		engineReceive[engineReceiveCount]=data;
		engineReceiveCount++;
	}
	UART8->ICR |=0x0008;
	
}

//����8��ʼ��
void UART8_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
	RCC->AHB1ENR|=1<<4;   	//ʹ��PORTE��ʱ��  
	RCC->APB1ENR|=((unsigned int)1<<31);  	//ʹ�ܴ���8ʱ�� 
	GPIO_Set(GPIOE,PIN0|PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOE,0,8);	//PE0,AF8
	GPIO_AF_Set(GPIOE,1,8);  //PE1,AF8
	//����������
 	UART8->BRR=temp; 		//����������@OVER8=0 	
	UART8->CR1=0;		 	//����CR1�Ĵ���
	UART8->CR1|=0<<28;	 	//����M1=0
	UART8->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
	UART8->CR1|=0<<10;	 
	UART8->CR1|=0<<15; 	//����OVER8=0,16�������� 
	UART8->CR1|=1<<3;  	//���ڷ���ʹ�� 
	//ʹ�ܽ����ж� 
	UART8->CR1|=1<<2;  	//���ڽ���ʹ��
	UART8->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,3,UART8_IRQn,2);//��2��������ȼ� 
	UART8->CR1|=1<<0;  	//����ʹ��
}






//  ����5   ���
 u8 BatteryDataReceive[28];        //������ݽ������� 
 u8 batteryDataReceiveCnt=0;       //����������ݽ��ռ���

void UART5_IRQHandler(void)
{
	u8 data;	
	if(UART5->ISR&(1<<5))//���յ�����
	{	 
		data=UART5->RDR; 
 		BatteryDataReceive[batteryDataReceiveCnt]=data;
		batteryDataReceiveCnt++;
	}
	UART5->ICR |=0x0008;
	
}


//����5��ʼ��
void UART5_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//�õ�USARTDIV@OVER8=0,���������������
	RCC->AHB1ENR|=1<<2;   	//ʹ��PORTC��ʱ��  
	RCC->AHB1ENR|=1<<3;   	//ʹ��PORTE��ʱ��  
	RCC->APB1ENR|=1<<20;  	//ʹ��UART����5ʱ�� 
	GPIO_Set(GPIOC,PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	GPIO_Set(GPIOD,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOC,12,8);	//PC 12
	GPIO_AF_Set(GPIOD,2,8);  //PD2
	//����������
 	UART5->BRR=temp; 		//����������@OVER8=0 	
	UART5->CR1=0;		 	//����CR1�Ĵ���
	UART5->CR1|=0<<28;	 	//����M1=0
	UART5->CR1|=0<<12;	 	//����M0=0&M1=0,ѡ��8λ�ֳ� 
	UART5->CR1|=0<<10;	 
	UART5->CR1|=0<<15; 	//����OVER8=0,16�������� 
	UART5->CR1|=1<<3;  	//���ڷ���ʹ�� 
	//ʹ�ܽ����ж� 
	UART5->CR1|=1<<2;  	//���ڽ���ʹ��
	UART5->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1,2,UART5_IRQn,2);//��2��������ȼ� 
	UART5->CR1|=1<<0;  	//����ʹ��
}


void USART1_Transmi_Data(uint8_t *DATA,uint8_t len)
{
			for(t=0;t<len;t++)
			{
				USART1->TDR=DATA[t];
				while((USART1->ISR&0X40)==0);//�ȴ����ͽ���
			}

}

void UART8_Transmi_Data(uint8_t *DATA,uint8_t len)
{
			for(t=0;t<len;t++)
			{
				UART8->TDR=DATA[t];
				while((UART8->ISR&0X40)==0);//�ȴ����ͽ���
			}

}


void UART5_Transmi_Data(uint8_t *DATA,uint8_t len)
{
			for(t=0;t<len;t++)
			{
				UART5->TDR=DATA[t];
				while((UART5->ISR&0X40)==0);//�ȴ����ͽ���
			}

}






