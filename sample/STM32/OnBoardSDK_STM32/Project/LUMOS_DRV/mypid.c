
#include "math.h"
#include "mypid.h"
PID_Type VXPID,VYPID;

void myPIDinit(PID_Type *mypid)
{
	mypid->P=10;
	mypid->I=0;
	mypid->D=0;
	mypid->CurrentError=0;
	mypid->LastError=0;
	mypid->IMax=0;
	mypid->PIDMax=0;
	mypid->deadbond=0;
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