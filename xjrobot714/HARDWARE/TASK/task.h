#ifndef __TASK_H
#define __TASK_H

#include "sys.h" 
#include "dataprocess.h"

typedef struct
{
	__IO  uint8_t  id1;
	__IO  uint8_t  id2;
	__IO 	float wheel_interval;// �������
	__IO float multiplier;           //��Ƶ��
	__IO float deceleration_ratio;  //���ٱ�
	__IO float wheel_diameter;     //����ֱ������λmm 13Ӣ�� 33.02cm
	__IO float line_number;       //��������
}CAR_id;
extern CAR_id car_message;

static const u16 LEDRATE=50000;
static const u16 PS2RECTIME=1000;
static const u16 POSITIONREQUEST=2000;
static const u16 OdomTime=10000;
static const u32 time_5s=500000;//5s
static const u32 time_10s=1000000;//10s
static const u16 time_100ms=10000;//100ms
static const u16 time_300ms=30000;//300ms

extern u8 PS2KEY;
extern u8 ps2DataFlg;
extern int32_t Pre_Motor1EncodeCount;
extern int32_t Pre_Motor4EncodeCount;
extern int32_t Pre_Motor2EncodeCount;
extern int32_t Pre_Motor3EncodeCount;
extern float Pitch,Yaw,Roll,Yaw_Angle;
extern volatile float X,Y,Distance_check;
extern Position *SendP_Data_Fusion,*WriteP_Data_Fusion;

extern u8 emginestarstopstate;
extern u8  underplatestate;

extern float const_frame,const_angle,const_speed;
extern float wheel_interval;// �������
extern float multiplier;           //��Ƶ��
extern float deceleration_ratio;  //���ٱ�
extern float wheel_diameter;     //����ֱ������λmm 13Ӣ�� 33.02cm
extern float pi_1_2;          //��/2
extern float pi;              //��
extern float pi_3_2;          //��*3/2
extern float pi_2_1;          //��*2
extern float line_number;       //�������� 
extern float yaw_number;       //ԭ��ת��ϵ��
extern u8 state_yksp,state_pcsp;

void creatTask(void);

void LEDTASK(void);
void PS2DataRecrive(void);
void MotorPositionRequest(void);
void Odeometer(void);
void USART1Data_Process(void);
void USART1_ReceiveData(void);
void PS2ControlRun(u8 ps2data);
void PS2ControlUnderPlate(u8 ps2data);
void CarsportR(float rad,float ver);
void Carsport_state(void);
void motor_State(void);
void data_init(u8 id);
#endif



