
#ifndef __PCM_H__
#define __PCM_H__


#include "defs.h"

#define SNDRATE 44100

struct pcm
{
	int hz, len;
	int stereo;
	short *buf;
	int pos;
};
struct cputstr
{
    unsigned int t0,t1,t2,t3,t4,t5,t6;
};

extern struct pcm pcm;
extern struct cputstr cput;


#endif


