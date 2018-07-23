
#include "math.h"
#include "mypid.h"
PID_Type VXPID,VYPID,VHPID;

void myPIDinit()
{
	VXPID.P=1;
	VXPID.I=0;
	VXPID.D=0;
	VXPID.CurrentError=0;
	VXPID.LastError=0;
	VXPID.IMax=0;
	VXPID.PIDMax=10;
	VXPID.deadbond=0;

	VYPID.P=1;
	VYPID.I=0;
	VYPID.D=0;
	VYPID.CurrentError=0;
	VYPID.LastError=0;
	VYPID.IMax=0;
	VYPID.PIDMax=10;
	VYPID.deadbond=0;

	VHPID.P=5;
	VHPID.I=0;
	VHPID.D=0;
	VHPID.CurrentError=0;
	VHPID.LastError=0;
	VHPID.IMax=0;
	VHPID.PIDMax=100;
	VHPID.deadbond=5;
	

}

float myPIDcontrol(PID_Type *mypid,float ref,float fdb)
{
	mypid->CurrentError=ref-fdb;
	mypid->Pout=mypid->P*mypid->CurrentError;

	if(mypid->CurrentError>mypid->deadbond)
	{
		mypid->index=0;
	}
	else
	{
		mypid->index=1;
		mypid->Iout+=mypid->I *  mypid->CurrentError;
		mypid->Iout=mypid->Iout > mypid->IMax ? mypid->IMax:mypid->Iout;
		mypid->Iout=mypid->Iout < -mypid->IMax ? -mypid->IMax:mypid->Iout;
	}

	mypid->Dout=mypid->D*(mypid->CurrentError-mypid->LastError);

	mypid->PIDout=mypid->Pout+mypid->Iout+mypid->Dout;
	mypid->PIDout=mypid->PIDout > mypid->PIDMax ? mypid->PIDMax : mypid->PIDout;
	mypid->PIDout=mypid->PIDout < -mypid->PIDMax ? -mypid->PIDMax : mypid->PIDout;

	mypid->LastError=mypid->CurrentError;

	return (short)mypid->PIDout;
}