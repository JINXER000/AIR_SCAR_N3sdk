#include "kdbase.h"
#include "ano_of.h"
#include "FlightControlSample.h"
#include "mypid.h"
#include "dji_control.hpp"
#include "dji_vehicle.hpp"

#include "FlightControlSample.h"
#include "dji_ack.hpp"
extern Vehicle  vehicle;
extern Vehicle* v;

//output from ano of: velocity , cm/s
unsigned char kdBuffer[20];
unsigned char kdcnt=0;
KDBaseinfo_t KDBaseinfo;

int byte2int(unsigned char order)
{
	if(order>128)
	{
	return (int)(128-order);
	}else
	{
	return order;
	}
}
void KDBase_process(unsigned char kddata)
{
	kdBuffer[kdcnt]=kddata;
	kdcnt++;
	if (kdBuffer[0]!=0x01)
	{
		kdcnt=0;
		return;
	}
//	if (kdBuffer[9]!=0x0D)
//	{
//		kdcnt=0;
//		return;
//	}
//	if (kdBuffer[10]!=0x0A)
//	{
//		kdcnt=0;
//		return;
//	}		
//	
	if(kdcnt<11) {return;}
	else
	{
		KDBaseinfo.forwardspeed=byte2int(kdBuffer[1]);
		KDBaseinfo.rotatespeed=byte2int(kdBuffer[3]);
		moveByPositionOffset(KDBaseinfo.forwardspeed/100, 0, 0, KDBaseinfo.rotatespeed);

		kdcnt=0;
	}
	
	
}
float vycmd,vxcmd;
void keepvx_of()
{
		//if we head x, we should keep of_y=0 and set vx as a constant value
	vycmd=myPIDcontrol(&VYPID,0,OF_DY2FIX);
	v->control->angularRateAndVertPosCtrl(vycmd,0,0,0);
	//pidy(0-of_y);
	

}

void keepvy_of()
{
		//if we head x, we should keep of_x=0
	//pidx(0-of_x);
	//pid(vy_set-of_y)

}

