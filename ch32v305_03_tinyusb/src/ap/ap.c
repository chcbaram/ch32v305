#include "ap.h"






void apInit(void)
{
  cliOpen(_DEF_UART1, 115200);
}

void apMain(void)
{
  uint32_t pre_time;


  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);

      logPrintf("vcp : %d, usb : %d\n", vcpIsConnected(), usbIsConnected());
      vcpPrintf("vcp printf %d\n", millis());
    }    

    if (vcpAvailable() > 0)
    {
      logPrintf("vcp rx : 0x%X\n", vcpRead());
    }
    usbUpdate();
    cliMain();
  }
}
