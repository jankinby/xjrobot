#include "can_drive.h"
#include "stdint.h"
#include "can.h"
#include "delay.h"

unsigned char CAN1_Send(Message *m)
{
	unsigned char i;
	uint8_t mbox;
	uint16_t j=0;
	uint8_t Data[8];
	for(i=0;i<m->len;i++)
	{
		Data[i] = m->data[i];
	}			  	 						       
  mbox=CAN1_Tx_Msg((uint32_t)(m->cob_id),0,0,m->len	,Data);
	while((CAN1_Tx_Staus(mbox)!=0X07)&&(j<0XFFF))j++;//�ȴ����ͽ���
	if(j>=0XFFF)return 1;							//����ʧ��
	return 0;										//���ͳɹ�;
}




u8 res=1;
unsigned char canbuf[8];
u8 MOTOR_init(void)       //������ʼ��
{

	Message Motor_Init = Message_Initializer;              //�����ʼ���ṹ��
	Message Motor_Mode_Velocity = Message_Initializer;     //����ٶ�ģʽ�ṹ��
	Message Motor_Enable = Message_Initializer;	           //���ʹ�ܽṹ��
	
	delay_ms(2000);
	//CANģʽ ID��0000Ϊ�㲥ģʽ���ɿ���ȫ�����   ����ȫ���������ģʽѡ��
	Motor_Init.cob_id = 0x0000;    
	Motor_Init.rtr=0;
	Motor_Init.len=2;
	Motor_Init.data[0]=0x01;
	Motor_Init.data[1]=0x00;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Init);
		delay_ms(20);
	}
	
	//�ٶ�ģʽ   ���ݣ�2F 60 60 00 03 00 00 00  		 ���������ѡ���ٶȻ�����ģʽ
	Motor_Mode_Velocity.rtr=0;
	Motor_Mode_Velocity.len=8;
	Motor_Mode_Velocity.data[0]= 0x2F;
	Motor_Mode_Velocity.data[1]= 0x60;
	Motor_Mode_Velocity.data[2]= 0x60;
	Motor_Mode_Velocity.data[3]= 0x00;
	Motor_Mode_Velocity.data[4]= 0x03;
	Motor_Mode_Velocity.data[5]= 0x00;
	Motor_Mode_Velocity.data[6]= 0x00;
	Motor_Mode_Velocity.data[7]= 0x00;
	
	Motor_Mode_Velocity.cob_id=Motor1_Drive_ID;          //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Mode_Velocity);
		delay_ms(20);
	}
	
	Motor_Mode_Velocity.cob_id=Motor2_Drive_ID;          //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Mode_Velocity);
		delay_ms(20);
	}
	
	Motor_Mode_Velocity.cob_id=Motor3_Drive_ID;          //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Mode_Velocity);
		delay_ms(20);
	}
	
	Motor_Mode_Velocity.cob_id=Motor4_Drive_ID;          //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Mode_Velocity);
		delay_ms(20);
	}
//���ʹ��    ���ݣ�2B 40 60 00 0F 00 00 00          ����������ֶ�ʹ��
	
	Motor_Enable.rtr=0;
	Motor_Enable.len=8;
	Motor_Enable.data[0] = 0x2B;
	Motor_Enable.data[1] = 0x40;
	Motor_Enable.data[2] = 0x60;
	Motor_Enable.data[3] = 0x00;
	Motor_Enable.data[4] = 0x0F;
	Motor_Enable.data[5] = 0x00;
	Motor_Enable.data[6] = 0x00;
	Motor_Enable.data[7] = 0x00;
	
	Motor_Enable.cob_id=Motor1_Drive_ID;           //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Enable);
		delay_ms(20);
	}
	
	Motor_Enable.cob_id=Motor2_Drive_ID;           //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Enable);
		delay_ms(20);
	}
	
	Motor_Enable.cob_id=Motor3_Drive_ID;           //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Enable);
		delay_ms(20);
	}
	
	Motor_Enable.cob_id=Motor4_Drive_ID;           //���ID
	res=1;
	while(res>0)
	{
		res=CAN1_Send(&Motor_Enable);
		delay_ms(20);
	}
	return 1;
}


void Motor_Velocity(uint16_t COD_ID, uint32_t speed)
{
	uint32_t temp=speed;
	
	if(1)//��1��Ϊ�ĸ�����Ĺ���
	{
	Message RxMSG_Vel=Message_Initializer;
	RxMSG_Vel.cob_id=COD_ID;
	RxMSG_Vel.rtr=0;
	RxMSG_Vel.len=8;
	RxMSG_Vel.data[0]=0x23;
	RxMSG_Vel.data[1]=0xFF;
	RxMSG_Vel.data[2]=0x60;
	RxMSG_Vel.data[3]=0x00;
	RxMSG_Vel.data[4]=(uint8_t)speed;
	RxMSG_Vel.data[5]=(uint8_t)(speed>>8);
	RxMSG_Vel.data[6]=(uint8_t)(speed>>16);
	RxMSG_Vel.data[7]=(uint8_t)(speed>>24);
	CAN1_Send(&RxMSG_Vel);
	//delay_ms(10);
	delay_us(2000);
	}
}

//��������������  ��������0x6063
void Motor_Position_Request(uint16_t COD_ID)
{
	Message Pos_TxMessage=Message_Initializer;
	Pos_TxMessage.cob_id=COD_ID;
	Pos_TxMessage.rtr=0;
	Pos_TxMessage.len=8;
	Pos_TxMessage.data[0]=0x40;
	Pos_TxMessage.data[1]=0x64;
	Pos_TxMessage.data[2]=0x60;
	Pos_TxMessage.data[3]=0x00;
	Pos_TxMessage.data[4]=0x00;
	Pos_TxMessage.data[5]=0x00;
	Pos_TxMessage.data[6]=0x00;
	Pos_TxMessage.data[7]=0x00;
	CAN1_Send(&Pos_TxMessage);
	delay_us(1000);
	//delay_ms(10);
}
//�������  ��������0x221C
void Motor_Current_Request(uint16_t COD_ID)
{
	Message Cur_TxMessage=Message_Initializer;
	Cur_TxMessage.cob_id=COD_ID;
	Cur_TxMessage.rtr=0;
	Cur_TxMessage.len=8;
	Cur_TxMessage.data[0]=0x4B;
	Cur_TxMessage.data[1]=0x1C;
	Cur_TxMessage.data[2]=0x22;
	Cur_TxMessage.data[3]=0x00;
	Cur_TxMessage.data[4]=0x00;
	Cur_TxMessage.data[5]=0x00;
	Cur_TxMessage.data[6]=0x00;
	Cur_TxMessage.data[7]=0x00;
	CAN1_Send(&Cur_TxMessage);
	delay_ms(10);
}
//������ϲ�ѯ ��������0x1001 
void Motor_Error_query(uint16_t COD_ID)
{
	Message Err_TxMessage=Message_Initializer;
	Err_TxMessage.cob_id=COD_ID;
	Err_TxMessage.rtr=0;
	Err_TxMessage.len=8;
	Err_TxMessage.data[0]=0x4F;
	Err_TxMessage.data[1]=0x01;
	Err_TxMessage.data[2]=0x10;
	Err_TxMessage.data[3]=0x00;
	Err_TxMessage.data[4]=0x00;
	Err_TxMessage.data[5]=0x00;
	Err_TxMessage.data[6]=0x00;
	Err_TxMessage.data[7]=0x00;
	CAN1_Send(&Err_TxMessage);
	delay_ms(10);
}
//������ϸ�λ ��������0x6040 2BΪд16 0f��8f
void Motor_ErrorReset_Request(uint16_t COD_ID)
{
	Message ErrRes_TxMessage=Message_Initializer;
	ErrRes_TxMessage.cob_id=COD_ID;
	ErrRes_TxMessage.rtr=0;
	ErrRes_TxMessage.len=8;
	ErrRes_TxMessage.data[0]=0x2B;
	ErrRes_TxMessage.data[1]=0x40;
	ErrRes_TxMessage.data[2]=0x60;
	ErrRes_TxMessage.data[3]=0x00;
	ErrRes_TxMessage.data[4]=0x0f;
	ErrRes_TxMessage.data[5]=0x00;
	ErrRes_TxMessage.data[6]=0x00;
	ErrRes_TxMessage.data[7]=0x00;
	CAN1_Send(&ErrRes_TxMessage);
	delay_ms(10);
	ErrRes_TxMessage.data[4]=0x8f;
	CAN1_Send(&ErrRes_TxMessage);
	delay_ms(10);
}











