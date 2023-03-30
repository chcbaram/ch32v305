#include "ap.h"




extern uint32_t sof_cnt;
extern qbuffer_t q_rx;


void apInit(void)
{
  cliOpen(_DEF_UART1, 115200);
}

void apMain(void)
{
  uint32_t pre_time;
  uint32_t sof_pre;


  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 1000)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
      // logPrintf("sof %d\n", sof_cnt-sof_pre);
      // sof_pre = sof_cnt;
    }    

    cliMain();

    uint32_t rx_len;

    rx_len = qbufferAvailable(&q_rx);
    if (rx_len > 0)
    {
      uint8_t rx_buf[256];

      if (rx_len > 256)
        rx_len = 256;

      qbufferRead(&q_rx, rx_buf, rx_len);
      uartWrite(_DEF_UART1, rx_buf, rx_len);       
    }
    
  }
}
