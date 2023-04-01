#ifndef VCP_H_
#define VCP_H_



#ifdef __cplusplus
 extern "C" {
#endif


#include "hw_def.h"


bool     vcpInit(void);
bool     vcpFlush(void);
uint32_t vcpAvailable(void);
uint32_t vcpWrite(uint8_t *p_data, uint32_t length);
uint8_t  vcpRead(void);
bool     vcpIsConnected(void);
uint32_t vcpGetBaud(void);

uint32_t vcpPrintf( const char *fmt, ...);



#ifdef __cplusplus
}
#endif

#endif