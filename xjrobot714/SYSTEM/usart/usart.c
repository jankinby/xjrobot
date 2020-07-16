#include "sys.h"
#include "usart.h"	  
//#include "imu.h"      //惯导数据


#if 1
#pragma import(__use_no_semihosting)  
//解决HAL库使用时,某些情况可能报错的bug
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART1->TDR = (u8) ch;      
	return ch;
}
#endif 
//end


//////////////////////////////////////////////////////////////////
//串口1中断服务程序 	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_STA=0;                 //接收状态标记	

uint8_t USART1_Rx_Time_count=0;    
uint8_t USART1_RX_BUF[USART_RX_ARRAY_NUM][USART_REC_LEN]; //接收缓冲
uint8_t Usart1RxLength[USART_RX_ARRAY_NUM];//接收缓冲长度
uint8_t Usart1RxHead = 0;//缓存头
uint8_t Usart1Rxend = 0;//缓存尾
uint8_t Usart1RxNum = 0;//已存在数量
uint8_t Rx_count_UART1=0;


int t=0;
uint8_t USART4_RX_BUF[USART_RX_ARRAY_NUM][USART_REC_LEN]; //接收缓冲
uint8_t Usart4RxLength[USART_RX_ARRAY_NUM];//接收缓冲长度
uint8_t Usart4RxHead = 0;//缓存头
uint8_t Usart4Rxend = 0;//缓存尾
uint8_t Usart4RxNum = 0;//已存在数量


void USART1_IRQHandler(void)
{
	u8 res;	
	if(USART1->ISR&(1<<5))//接收到数据
	{	 
		  res=USART1->RDR; 
	    USART1_Rx_Time_count=0;                         //清零计时
		if( Rx_count_UART1 < USART_REC_LEN)
		{
			USART1_RX_BUF[Usart1Rxend][Rx_count_UART1] = res;    //接受的数据放入USART1数据缓存
			Rx_count_UART1++;                               //数据缓存指针向下移
		}      
	}
	USART1->ICR |=0x0008;
} 										 
//初始化IO 串口1 上位机
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void USART1_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	
	temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
	RCC->AHB1ENR|=1<<1;   	//使能PORTB口时钟  
	RCC->APB2ENR|=1<<4;  	//使能串口1时钟 
	GPIO_Set(GPIOB,PIN14|PIN15,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//PB14,PB15,复用功能,上拉输出
 	GPIO_AF_Set(GPIOB,14,4);	//PB14,AF4
	GPIO_AF_Set(GPIOB,15,4);//PB15,AF4
	//波特率设置
 	USART1->BRR=temp; 		//波特率设置@OVER8=0 	
	USART1->CR1=0;		 	//清零CR1寄存器
	USART1->CR1|=0<<28;	 	//设置M1=0
	USART1->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
	USART1->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
	USART1->CR1|=1<<3;  	//串口发送使能 

	//使能接收中断 
	USART1->CR1|=1<<2;  	//串口接收使能
	USART1->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,2,USART1_IRQn,2);//组2，最低优先级 

	USART1->CR1|=1<<0;  	//串口使能
}







/*
void USART2_IRQHandler(void)      
{
	if(USART2->ISR&(1<<5))//接收到数据
	{	 
	}
	USART2->ICR |=0x0008;
	
}



//串口2初始化
void USART2_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
	
	
	RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  	//使能串口2时钟 
	
	
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOA,2,7);	//PB10,AF7复用
	GPIO_AF_Set(GPIOA,3,7);  //PB11,AF7复用
	//波特率设置
 	USART2->BRR=temp; 		//波特率设置@OVER8=0 	
	USART2->CR1=0;		 	//清零CR1寄存器
	USART2->CR1|=0<<28;	 	//设置M1=0
	USART2->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
	USART2->CR1|=0<<10;	 
	USART2->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
	USART2->CR1|=1<<3;  	//串口发送使能 
	//使能接收中断 
	USART2->CR1|=1<<2;  	//串口接收使能
	USART2->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1,1,USART2_IRQn,2);//组2，最低优先级 
	USART2->CR1|=1<<0;  	//串口使能
}*/






u8 Ps2Receive[10];
u8 Ps2ReceiveCount=0;
void USART3_IRQHandler(void)
{
	u8 data;	
	if(USART3->ISR&(1<<5))//接收到数据
	{	 
		data=USART3->RDR; 
 		Ps2Receive[Ps2ReceiveCount]=data;
		Ps2ReceiveCount++;
	}
	USART3->ICR |=0x0008;
	
}

//串口3初始化 遥控
void USART3_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
	RCC->AHB1ENR|=1<<1;   	//使能PORTB口时钟  
	RCC->APB1ENR|=1<<18;  	//使能串口3时钟 
	GPIO_Set(GPIOB,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOB,10,7);	//PB10,AF7
	GPIO_AF_Set(GPIOB,11,7);  //PB11,AF7
	//波特率设置
 	USART3->BRR=temp; 		//波特率设置@OVER8=0 	
	USART3->CR1=0;		 	//清零CR1寄存器
	USART3->CR1|=0<<28;	 	//设置M1=0
	USART3->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
	USART3->CR1|=0<<10;	 
	USART3->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
	
	USART3->CR1|=1<<3;  	//串口发送使能 
	USART3->CR1|=1<<2;  	//串口接收使能
	USART3->CR1|=1<<5;    //接收缓冲区非空中断使能	 
	
	MY_NVIC_Init(1,0,USART3_IRQn,2);//组2，最低优先级 
	USART3->CR1|=1<<0;  	//串口使能
}


//  串口4    IMU数据接收
u8 IMUDatareceive[54];        //IMU数据接收数组 
u8 IMUDatareceiveCount=0;     //IMU数据接收计数
void UART4_IRQHandler(void)      	
{
	u8 data;	
	if(UART4->ISR&(1<<5))//接收到数据
	{
		data=UART4->RDR; 
		IMUDatareceive[IMUDatareceiveCount]=data;
		IMUDatareceiveCount++;			
	}
	UART4->ICR |=0x0008;
	
}
//串口4初始化
void UART4_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
	
	
	RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
	RCC->APB1ENR|=1<<19;  	//使能UART串口4时钟 
	
	
	GPIO_Set(GPIOA,PIN11|PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOA,12,6);	//PA12, AF6  ,UART4_Tx
	GPIO_AF_Set(GPIOA,11,6);  //PA11,  AF6,U4_Rx
	//波特率设置
 	UART4->BRR=temp; 		//波特率设置@OVER8=0 	
	UART4->CR1=0;		 	//清零CR1寄存器
	UART4->CR1|=0<<28;	 	//设置M1=0
	UART4->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
	UART4->CR1|=0<<10;	 
	UART4->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
	UART4->CR1|=1<<3;  	//串口发送使能 
	//使能接收中断 
	UART4->CR1|=1<<2;  	//串口接收使能
	UART4->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1,2,UART4_IRQn,2);//组2，最低优先级 
	UART4->CR1|=1<<0;  	//串口使能
}




//发动机
u8 engineReceive[10];
u8 engineReceiveCount=0;
void UART8_IRQHandler(void)
{
	u8 data;	
	if(UART8->ISR&(1<<5))//接收到数据
	{	 
		data=UART8->RDR; 
 		engineReceive[engineReceiveCount]=data;
		engineReceiveCount++;
	}
	UART8->ICR |=0x0008;
	
}

//串口8初始化
void UART8_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
	RCC->AHB1ENR|=1<<4;   	//使能PORTE口时钟  
	RCC->APB1ENR|=((unsigned int)1<<31);  	//使能串口8时钟 
	GPIO_Set(GPIOE,PIN0|PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOE,0,8);	//PE0,AF8
	GPIO_AF_Set(GPIOE,1,8);  //PE1,AF8
	//波特率设置
 	UART8->BRR=temp; 		//波特率设置@OVER8=0 	
	UART8->CR1=0;		 	//清零CR1寄存器
	UART8->CR1|=0<<28;	 	//设置M1=0
	UART8->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
	UART8->CR1|=0<<10;	 
	UART8->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
	UART8->CR1|=1<<3;  	//串口发送使能 
	//使能接收中断 
	UART8->CR1|=1<<2;  	//串口接收使能
	UART8->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,3,UART8_IRQn,2);//组2，最低优先级 
	UART8->CR1|=1<<0;  	//串口使能
}






//  串口5   电池
 u8 BatteryDataReceive[28];        //电池数据接收数组 
 u8 batteryDataReceiveCnt=0;       //电池数据数据接收计数

void UART5_IRQHandler(void)
{
	u8 data;	
	if(UART5->ISR&(1<<5))//接收到数据
	{	 
		data=UART5->RDR; 
 		BatteryDataReceive[batteryDataReceiveCnt]=data;
		batteryDataReceiveCnt++;
	}
	UART5->ICR |=0x0008;
	
}


//串口5初始化
void UART5_Init(u32 pclk2,u32 bound)
{  	 
	u32	temp;	   
	temp=(pclk2*1000000+bound/2)/bound;	//得到USARTDIV@OVER8=0,采用四舍五入计算
	RCC->AHB1ENR|=1<<2;   	//使能PORTC口时钟  
	RCC->AHB1ENR|=1<<3;   	//使能PORTE口时钟  
	RCC->APB1ENR|=1<<20;  	//使能UART串口5时钟 
	GPIO_Set(GPIOC,PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	GPIO_Set(GPIOD,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
 	GPIO_AF_Set(GPIOC,12,8);	//PC 12
	GPIO_AF_Set(GPIOD,2,8);  //PD2
	//波特率设置
 	UART5->BRR=temp; 		//波特率设置@OVER8=0 	
	UART5->CR1=0;		 	//清零CR1寄存器
	UART5->CR1|=0<<28;	 	//设置M1=0
	UART5->CR1|=0<<12;	 	//设置M0=0&M1=0,选择8位字长 
	UART5->CR1|=0<<10;	 
	UART5->CR1|=0<<15; 	//设置OVER8=0,16倍过采样 
	UART5->CR1|=1<<3;  	//串口发送使能 
	//使能接收中断 
	UART5->CR1|=1<<2;  	//串口接收使能
	UART5->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(1,2,UART5_IRQn,2);//组2，最低优先级 
	UART5->CR1|=1<<0;  	//串口使能
}


void USART1_Transmi_Data(uint8_t *DATA,uint8_t len)
{
			for(t=0;t<len;t++)
			{
				USART1->TDR=DATA[t];
				while((USART1->ISR&0X40)==0);//等待发送结束
			}

}

void UART8_Transmi_Data(uint8_t *DATA,uint8_t len)
{
			for(t=0;t<len;t++)
			{
				UART8->TDR=DATA[t];
				while((UART8->ISR&0X40)==0);//等待发送结束
			}

}


void UART5_Transmi_Data(uint8_t *DATA,uint8_t len)
{
			for(t=0;t<len;t++)
			{
				UART5->TDR=DATA[t];
				while((UART5->ISR&0X40)==0);//等待发送结束
			}

}






