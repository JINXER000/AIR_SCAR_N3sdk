#ifndef _TIME_H_
#define _TIME_H_

#include "sys.h"

#define NVIC_TIME_P       2		
#define NVIC_TIME_S       0

#define GET_TIME_NUM 	(5)		
void TIM_INIT(void);
void sys_time(void);

u16 Get_Time(u8,u16,u16);

u32 Get_Cycle_T(u8 );

void Cycle_Time_Init(void);

extern volatile uint32_t sysTickUptime;
extern int time_1h,time_1m,time_1s,time_1ms;

void Delay_us(uint32_t);
void Delay_ms(uint32_t);
void SysTick_Configuration(void);
uint32_t GetSysTime_us(void);
#endif
