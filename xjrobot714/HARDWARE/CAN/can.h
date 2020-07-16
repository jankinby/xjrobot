#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/7/19
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE			1		 			//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);	//��������

u8 CAN1_Msg_Pend(u8 fifox);								//��ѯ���䱨��
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);//��������
u8 CAN1_Tx_Staus(u8 mbox);  							//���ط���״̬
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������
u8 CAN1_Receive_Msg(u8 *buf);							//��������
#endif



extern u32 Motor1EncodeCount;
extern u32 Motor2EncodeCount;
extern u32 Motor3EncodeCount;
extern u32 Motor4EncodeCount;

extern volatile u8 Motor1Error;
extern volatile u8 Motor2Error;
extern volatile u8 Motor3Error;
extern volatile u8 Motor4Error;

extern volatile float Motor1Current;
extern volatile float Motor2Current;
extern volatile float Motor3Current;
extern volatile float Motor4Current;

typedef union{
     float v;
	   uint8_t bytes[4];
}byte_to_float;















