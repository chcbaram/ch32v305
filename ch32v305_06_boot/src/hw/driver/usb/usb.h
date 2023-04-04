#ifndef USB_H_
#define USB_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "hw_def.h"

#include "usbhs_cdc.h"


bool usbInit(void);
void usbDeInit(void);
bool usbUpdate(void);
bool usbIsConnected(void);

#ifdef __cplusplus
}
#endif

#endif 