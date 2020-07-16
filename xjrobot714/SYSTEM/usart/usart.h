#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"	 
//#include "imu.h"      //���Ե���������


#define USART_REC_LEN  			30 	//�����������ֽ��� 30
#define USART_RX_ARRAY_NUM	5//���ջ�����������
#define Max_Rx_Time  50   //10us
	


///////////û��
extern u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
extern u16 USART_RX_STA;       //����״̬���
/////////û�ý���


extern uint8_t USART1_Rx_Time_count;    
extern uint8_t USART1_RX_BUF[USART_RX_ARRAY_NUM][USART_REC_LEN]; //���ջ���
extern uint8_t Usart1RxLength[USART_RX_ARRAY_NUM];//���ջ��峤��
extern uint8_t Usart1RxHead ;//����ͷ
extern uint8_t Usart1Rxend ;//����β
extern uint8_t Usart1RxNum ;//�Ѵ�������
extern uint8_t Rx_count_UART1;


extern u8 Ps2Receive[10];                //ң���������ֽ�
extern u8 Ps2ReceiveCount;               //ң�����������ݼ���

extern u8 IMUDatareceive[54];           //IMU���ݽ���
extern u8 IMUDatareceiveCount;          //IMU���ݽ��ռ���



extern u8 engineReceive[10];            // ����������
extern u8 engineReceiveCount;                

extern u8 BatteryDataReceive[28];        
extern u8 batteryDataReceiveCnt;        // �����Ϣ����

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
















