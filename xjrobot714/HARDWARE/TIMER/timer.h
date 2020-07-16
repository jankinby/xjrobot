#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern u16 ledCount;
extern u16 ps2RecCount;
extern u16 Usart1DataResCount;
extern u16 positionRequest;
extern u16 OdomCount;
extern u32 carsport_state;
extern u16 motor_state;
extern u16 time_test;

extern u16 underplantimeCnt;


void TIM3_Init(u16 arr,u16 psc);
void TIM5_Init(u16 arr,u16 psc);
#endif























