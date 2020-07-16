#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"	 
//#include "imu.h"      //惯性导航传感器


#define USART_REC_LEN  			30 	//定义最大接收字节数 30
#define USART_RX_ARRAY_NUM	5//接收缓存数组数量
#define Max_Rx_Time  50   //10us
	


///////////没用
extern u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART_RX_STA;       //接收状态标记
/////////没用结束


extern uint8_t USART1_Rx_Time_count;    
extern uint8_t USART1_RX_BUF[USART_RX_ARRAY_NUM][USART_REC_LEN]; //接收缓冲
extern uint8_t Usart1RxLength[USART_RX_ARRAY_NUM];//接收缓冲长度
extern uint8_t Usart1RxHead ;//缓存头
extern uint8_t Usart1Rxend ;//缓存尾
extern uint8_t Usart1RxNum ;//已存在数量
extern uint8_t Rx_count_UART1;


extern u8 Ps2Receive[10];                //遥控器接收字节
extern u8 Ps2ReceiveCount;               //遥控器接收数据计数

extern u8 IMUDatareceive[54];           //IMU数据接收
extern u8 IMUDatareceiveCount;          //IMU数据接收计数



extern u8 engineReceive[10];            // 发动机接收
extern u8 engineReceiveCount;                

extern u8 BatteryDataReceive[28];        
extern u8 batteryDataReceiveCnt;        // 电池信息接收

void USART1_Init(u32 pclk2,u32 bound); 
void USART2_Init(u32 pclk2,u32 bound);
void USART3_Init(u32 pclk2,u32 bound);
void UART4_Init(u32 pclk2,u32 bound);
void UART8_Init(u32 pclk2,u32 bound);

void UART5_Init(u32 pclk2,u32 bound);
void USART1_Transmi_Data(uint8_t *DATA,uint8_t len);
void UART8_Transmi_Data(uint8_t *DATA,uint8_t len);
void UART5_Transmi_Data(uint8_t *DATA,uint8_t len);

#endif	   
















