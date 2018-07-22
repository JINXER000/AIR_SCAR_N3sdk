#include "scheduler.h"
#include "time.h"
#include "ANO-Tech.h"
#include "Driver_vision.h"
#include "RemoteTask.h"
#include "FlightControlSample.h"

#include "pwm.h"
#include "ospid.h"
#include "kdbase.h"

s16 loop_cnt;


loop_t loop;
extern  float pitchgoal,rollgoal, yawgoal,pitchnow,rollnow,yawnow;
extern int autoflag,	pwmwatch[4];
extern  PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID,AnglePID ,YawIPID,YawOPID ;
extern float angle_inc,angle_cmd;
AngleF_Struct TargetAngle;
int i;

float pitchSpeed = 0.0,rollSpeed = 0.0;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	
//		LED_1ms_DRV( );								//20级led渐变显示
}

void Duty_1ms()
{
//	Get_Cycle_T(1)/1000000.0f;

//	ANO_DT_Data_Exchange();												//数传通信定时调用
}

float test[5],magrms;
int tempcnt=0,framecnt;
extern int framecounter;

void Duty_2ms()
{
	 ANO_DT_Send_Senser(Enemy_now.X,Enemy_now.Y,PWMC1, PWMC2, PitchOPID.CurrentError,YawOPID.CurrentError,Enemy_now.ID*6666,framecnt,0);
//	usart1_report_imu(Enemy_now.X,Enemy_now.Y,PWMC1, PWMC2, PitchOPID.CurrentError,YawOPID.CurrentError,0,0,0);
}

void Duty_5ms()
{
	 
}
void Duty_10ms()
{

		if(tempcnt==100)
		{
			framecnt=framecounter;
			tempcnt=0;
			framecounter=0;
		}
		else
		{
			tempcnt++;
		}
}

void Duty_20ms()
{
//		if(	(KDBaseinfo.forwardspeed!=0)|| (KDBaseinfo.rotatespeed!=0))				//execute only once
//		{
//			 moveByPositionOffset(KDBaseinfo.forwardspeed/100, 0, 0, KDBaseinfo.rotatespeed);
//		KDBaseinfo.forwardspeed=0;
//		KDBaseinfo.rotatespeed=0;

//		}
//		
	keepvx_of();

}
int gun_cnt;
void Duty_50ms()
{
//			GPIO_SetBits(GPIOB,GPIO_Pin_15);

//		SetShootState(NOSHOOTING);//TEST

		if(GetShootState() == SHOOTING)
		{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
			PWMC3=TRIGGER_PULL;
			gun_cnt++;
			if(gun_cnt>20)
			{
				gun_cnt=0;
				SetShootState(NOSHOOTING);

			}
		}
		else
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_15);
			PWMC3=TRIGGER_RELEASE;

		}
		pwmwatch[2]=PWMC1;
		pwmwatch[3]=PWMC2;

if(autoflag)	
{
//		TargetAngle=RecToPolar(EnemyDataBuffer[EnemyDataBufferPoint].X, EnemyDataBuffer[EnemyDataBufferPoint].Y, EnemyDataBuffer[EnemyDataBufferPoint].Z, 0, 0, 0);
		pitchgoal=0;
		pitchnow=Enemy_now.Y;
		PWMC1+=Control_PitchPID();

		yawgoal=0;
		yawnow=Enemy_now.X;
		PWMC2-=Control_YawPID();
	
}	
				VAL_LIMIT(PWMC1,700,850);
			VAL_LIMIT(PWMC2,560,1140);

// limit yaw angle within (580,880)
	
}


void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}




	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

