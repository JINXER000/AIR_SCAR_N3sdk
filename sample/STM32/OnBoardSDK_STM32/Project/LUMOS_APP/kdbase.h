#ifndef __KDBASE_H_
#define __KDBASE_H_
extern int testcmd;
typedef struct KDBaseinfo_t
{
	int forwardspeed,rotatespeed;
	unsigned char fdb[29];
}KDBaseinfo_t;
extern KDBaseinfo_t KDBaseinfo;
void KDBase_process(unsigned char kddata);
void keepvx_of();
void keepvy_of();
void keepalt(float tgtalt);
void keepstation();

#endif
