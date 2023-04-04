#include "hw.h"



volatile const firm_ver_t firm_ver __attribute__((section(".version"))) = 
{
  .magic_number = VERSION_MAGIC_NUMBER,
  .version_str  = _DEF_FIRMWATRE_VERSION,
  .name_str     = _DEF_BOARD_NAME,
};



bool hwInit(void)
{
  bspInit();

  logInit();
  resetInit();
  ledInit();
  flashInit();
  uartInit();
  logOpen(HW_LOG_CH, 115200);

  logPrintf("\r\n[ Bootloader Begin... ]\r\n");
  logPrintf("Booting..Name \t\t: %s\r\n", _DEF_BOARD_NAME);
  logPrintf("Booting..Ver  \t\t: %s\r\n", _DEF_FIRMWATRE_VERSION);  
  logPrintf("\n");  
  logPrintf("Reset Mode    \t\t: %s\r\n", resetGetBootModeMsg());
  logPrintf("Reset Count   \t\t: %d\r\n", resetGetCount());  
  logPrintf("\n");

  vcpInit();  
  return true;
}

void jumpToFw(void)
{
  usbDeInit();
  delay(50);
  bspDeInit();

  NVIC_EnableIRQ(Software_IRQn);
  NVIC_SetPendingIRQ(Software_IRQn);

  // __asm("li  a6, 0x8400");
  // __asm("jr  a6");
  // while(1);  
  // (*jump_func)();
}

void SW_Handler(void) __attribute__((interrupt("machine")));
void SW_Handler(void) 
{
  __asm("li  a6, 0x8400");
  __asm("jr  a6");
  while(1);
}