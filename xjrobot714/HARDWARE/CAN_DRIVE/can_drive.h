#ifndef __CAN_DRIVE_H
#define __CAN_DRIVE_H
#include "stdint.h"
#include "sys.h"

#define Motor1_Drive_ID 0x601
#define Motor2_Drive_ID 0x602
#define Motor3_Drive_ID 0x603
#define Motor4_Drive_ID 0x604

typedef struct
{
	unsigned short cob_id;
	unsigned char rtr;
	unsigned char len;
	unsigned char data[8];
}Message;
#define Message_Initializer {0,0,0,{0,0,0,0,0,0,0,0}}


unsigned char CAN1_Send(Message *m);
u8 MOTOR_init(void);   
void Motor_Velocity(uint16_t COD_ID, uint32_t speed);
void Motor_Position_Request(uint16_t COD_ID);
void Motor_Current_Request(uint16_t COD_ID);
void Motor_Error_query(uint16_t COD_ID);
void Motor_ErrorReset_Request(uint16_t COD_ID);	


#endif




