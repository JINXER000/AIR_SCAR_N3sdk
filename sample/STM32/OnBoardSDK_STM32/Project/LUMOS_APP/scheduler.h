#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "sys.h"
typedef enum
{
	MANUAL_STATE,
	TASK1_STATE,
	TASK2_STATE,
}Workstate_e;
typedef enum
{
	STAYSTILL,
	FORWARD,
	HIGHER,
	LOWER,
	SCAN1,
	SCAN2,

}Motionlist_e;
typedef enum
{
	ZUO,
	ZHONG,
	YOU,
	SHANG,
	XIA,
}radarpoints_e;
typedef struct
{
	u8 check_flag;
	u8 err_flag;
	s16 cnt_1ms;
	s16 cnt_2ms;
	s16 cnt_5ms;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	u16 time;
}loop_t;

void Loop_check(void);

void Duty_Loop(void);

void Inner_Loop(float);

void Outer_Loop(float);

void keephight(void);
void avoidobstacle(void);
int getworkstate(void);
void controltask(void);

//#define WORKINGMODE




#endif

