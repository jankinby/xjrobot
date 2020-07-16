#ifndef __ENGINE_H
#define __ENGINE_H

#include "stm32f7xx.h"
#include "sys.h"
#include "time.h" 

#define ENGINESTRCNT 10000   // 100 ms
#define ENGINRECNT   10000   // 100 ms  读一次串口返回数据
#define ENGINEKEEPRUNTIMECNT  10000  // 100ms 喂一次发动机控制器
#define ENGINEKEEPTIME  10800000  //108s 油量3h=10800s，每隔108s减少1

#define UNDERPLANTSENDFRN 1000    

extern int  timedurcnt;

#define RobotRise 0x04
#define RobotDrop 0x08
#define EngineStartStop 0x80

extern uint32_t engineStarcnt;
extern u16 engineRecCount;
extern u16 enginekeepRuntimcnt;
extern u16 underplantsendState;
extern u32 enginekeeptime;
extern u8 oil;

typedef union
{
	struct
	{
		__IO  uint8_t  Head1;
		__IO  uint8_t  Head2;
		__IO  uint8_t  Reserved;
		__IO  uint8_t  Function;
		__IO  uint8_t  MotorHReserved;
		__IO  uint8_t  MotorLReserved;
		__IO  uint8_t  MotorReserved;
		__IO  uint8_t  Accelerator;
		__IO  uint8_t  SumCheckH;
		__IO  uint8_t  SumCheckL;
	}detail;
	uint8_t bytes[10];
}EngineControl;	



void EngineSettings(uint8_t  Function,uint8_t  Accelerator);

void EngineStar(void);
void EngineClose(void);
void engineDataRecrive(void);
void EnginekeepRun(void); 

void underPlantUp(void);
void underPlantDown(void);
void UnderPlantUpdownCon(uint8_t underplatestate);
void engineStartStop(u8 state);

void underPlantDownTime(uint32_t times);
void underPlantUpTime(uint32_t times);

void UnderPlantControl(void);



#endif



