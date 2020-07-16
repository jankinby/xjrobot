#ifndef __DATAPROCESS_H
#define __DATAPROCESS_H


#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f7xx.h"
	 
	 
//错误码定义
#define Checksum_Error 0x01             //校验核错误
#define Length_information_Error 0x02   //长度错误
#define Undefined_Order_Error 0x03      //无定义错误
#define Header_Error 0x04               
#define Brake_instructions_Error 0x05	  
	 
	 
extern uint8_t DataFrequency;
extern uint8_t IMU_DataFrequency;
extern uint32_t Car_Motion_status_Fre;
extern uint32_t Data_Fusion_Fre;
extern uint32_t IMU_Data_Fre;
extern uint8_t UnderPlantUpdownConState;
extern uint32_t underplantttime;

	 

static const uint8_t USART1DATARETIMER=200;  //   Tim3定时器10us访问一次    计数10次=20us；	
	 

	 
	 
//*************转发小车上传信息*********************//	 
typedef union
{
	struct
	{
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Lenghth;
		__IO  uint8_t  Order;
		__IO  uint8_t  Error;
		__IO  uint8_t  SumCheck;
	}detail;
	uint8_t bytes[6];
}Error_Massage;	 

typedef union
{
	struct
	{
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Lenghth;
		__IO  uint8_t  Order;
		__IO  uint8_t  Return_Data;
		__IO  uint8_t  SumCheck;
	}detail;
	uint8_t bytes[6];
}Return;

typedef union
{
	struct
	{
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Lenghth;
		__IO  uint8_t  Order;
		__IO  uint8_t  Return_Data;
		__IO  uint8_t  SumCheck;
	}detail;
	uint8_t bytes[6];
}Return_set;

typedef union
{
	struct
	{
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Lenghth;
		__IO  uint8_t  Order;
		__IO  uint8_t  Return_Data;
		__IO  uint8_t  Distance;
		__IO  uint8_t  SumCheck;
	}detail;
	uint8_t bytes[6];
}Return_Odom;
typedef union
{
	struct
	{	
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Lenghth;
		__IO  uint8_t  Order;
		__IO  uint8_t  Ultrasonic1_H;
		__IO  uint8_t  Ultrasonic1_L;
		__IO  uint8_t  Ultrasonic2_H;
		__IO  uint8_t  Ultrasonic2_L;
		__IO  uint8_t  Ultrasonic3_H;
		__IO  uint8_t  Ultrasonic3_L;
		__IO  uint8_t  Ultrasonic4_H;
		__IO  uint8_t  Ultrasonic4_L;
		__IO  uint8_t  Ultrasonic5_H;
		__IO  uint8_t  Ultrasonic5_L;
		__IO  uint8_t  Ultrasonic6_H;
		__IO  uint8_t  Ultrasonic6_L;
		__IO  uint8_t  SumCheck;
	}detail;
	uint8_t bytes[17];
}Ultrasonic_Massage;

typedef union
{
	struct
	{	
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Lenghth;
		__IO  uint8_t  Order;
		__IO  uint8_t  LED_1;
		__IO  uint8_t  LED_2;
		__IO  uint8_t  LED_3;
		__IO  uint8_t  LED_4;
		__IO  uint8_t  LED_5;
		__IO  uint8_t  Charge_status;
		__IO  uint8_t  Power;
		__IO  uint8_t  Error;
		__IO  uint8_t  SumCheck;
	}detail;
	uint8_t bytes[13];
}Motion_status_Massage;
//=====================End==================================//

//=======================数据融合上传=========================//

typedef struct {
	
	__IO  uint8_t  Head1;
	__IO  uint8_t  Head2;
	__IO  uint8_t  Lenghth;
	__IO  uint8_t  Order;
  __IO  uint32_t  DistanceX;
	__IO  uint32_t   DistanceY;
	__IO  uint32_t   Angular;
	__IO  uint32_t   SpeedV;
	__IO  uint32_t  SpeedAng;
	__IO  uint8_t  SumCheck;
}Position_str;

typedef union{
	Position_str Detial;
	uint8_t bytes[25];
}Position;

typedef struct
{
	__IO  uint8_t  Head1;
	__IO  uint8_t  Head2;
	__IO  uint8_t  Lenghth;
	__IO  uint8_t  Order;
  __IO  float  Pitch;
	__IO  float  Yaw;
	__IO  float  Roll;
	__IO  uint8_t  SumCheck;
}IMU_Data_str;

typedef union{
	IMU_Data_str Detial;
	uint8_t bytes[17];
}IMU_Data_inf;
//=====================End==================================//


//===================上位机信息重新封装=======================//
struct Control_Str{
	
	__IO  uint8_t  Header;
	__IO  uint8_t  Control;
	__IO  uint8_t  Lin_Speed_H;
	__IO  uint8_t  Lin_Speed_L;
  __IO  uint8_t  Angular_Speed_H;
	__IO  uint8_t  Angular_Speed_L;
	__IO  uint8_t  Stop;
	__IO  uint8_t  CRC8;
	__IO  uint8_t  Tail;
};

typedef union Control_Un
{
	struct Control_Str Detail;
	uint8_t bytes[9];
}ControlDef;

struct Configuration_Str{
	
	__IO  uint8_t  Header;
	__IO  uint8_t  Control;
	__IO  uint8_t  LED_1;
	__IO  uint8_t  LED_2;
  __IO  uint8_t  LED_3;
	__IO  uint8_t  LED_4;
	__IO  uint8_t  LED_5;
	__IO  uint8_t  FRQ_1;
	__IO  uint8_t  FRQ_2;
	__IO  uint8_t  CRC8;
	__IO  uint8_t  Tail;
};

typedef union Configuration_Un
{
	struct Configuration_Str Detail;
	uint8_t bytes[11];
}ConfigDef;

typedef union
{
	struct
	{
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Lenghth;
		__IO  uint8_t  Order;
		__IO  uint8_t  Order1;
		__IO  uint8_t  Return_Data;
		__IO  uint8_t  SumCheck;
	}detail;
	uint8_t bytes[7];
}uint8_t7_union;

typedef union{
	struct 
		{	
			__IO  uint8_t  Head1;
			__IO  uint8_t  Head2;
			__IO  uint8_t  Lenghth;
			__IO  uint8_t  Order;
			__IO  float  Return_Data1;
			__IO  float  Return_Data2;
			__IO  float  Return_Data3;
			__IO  float  Return_Data4;
			__IO  uint8_t  SumCheck;
     }Detial1;
	   uint8_t bytes[21];
}Current_union;
//========================End===================================//


extern IMU_Data_inf *SendP_Data_IMU,*WriteP_Data_IMU;

void Usart1_Data_Analyse(uint8_t *Point, uint8_t num);
void Massage_Timing_To_Pc(void);
 


#endif

















