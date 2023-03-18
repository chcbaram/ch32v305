#ifndef BSP_H_
#define BSP_H_


#include "def.h"

#include "ch32v30x_conf.h"



void logPrintf(const char *fmt, ...);



bool bspInit(void);

void delay(uint32_t time_ms);
uint32_t millis(void);



#endif