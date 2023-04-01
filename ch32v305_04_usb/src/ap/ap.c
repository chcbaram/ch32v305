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
    if (millis()-pre_time >= 1000)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);

      logPrintf("cdc conn : %d, open : %d\n", usbhsCdc()->isConnected(), usbhsCdc()->isOpen());      
    }    

    cliMain();

    uint32_t rx_len;

    rx_len = usbhsCdc()->available();
    if (rx_len > 0)
    {
      uint8_t rx_buf[256];

      if (rx_len > 256)
        rx_len = 256;

      for (int i=0; i<rx_len; i++)
        rx_buf[i] = usbhsCdc()->read();
      
      uartWrite(_DEF_UART1, rx_buf, rx_len);       
    }
    
  }
}
