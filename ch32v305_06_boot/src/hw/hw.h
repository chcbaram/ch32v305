#ifndef HW_H_
#define HW_H_


#include "hw_def.h"

#include "qbuffer.h"
#include "led.h"
#include "uart.h"
#include "cli.h"
#include "log.h"
#include "usb.h"
#include "vcp.h"
#include "flash.h"
#include "cmd.h"
#include "util.h"
#include "reset.h"

bool hwInit(void);
void jumpToFw(void);

#endif