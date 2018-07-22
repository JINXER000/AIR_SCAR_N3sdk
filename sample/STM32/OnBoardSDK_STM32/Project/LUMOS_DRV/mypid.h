#ifndef __MYPID_
#define __MYPID_
#include "ospid.h"

extern PID_Type VXPID,VYPID;
void myPIDinit(PID_Type *mypid);
float myPIDcontrol(PID_Type *mypid,float ref,float fdb);


#endif