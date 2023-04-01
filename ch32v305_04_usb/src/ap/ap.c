#include "ap.h"




static void cliInfo(cli_args_t *args);






void apInit(void)
{
  cliOpen(_DEF_UART1, 115200);
  cliAdd("info", cliInfo);

  uartOpen(_DEF_UART2, 115200);
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
    }    

    cliMain();

    uint32_t rx_len;

    rx_len = uartAvailable(_DEF_UART2);
    if (rx_len > 0)
    {
      uint8_t rx_buf[256];

      if (rx_len > 256)
        rx_len = 256;

      for (int i=0; i<rx_len; i++)
        rx_buf[i] = uartRead(_DEF_UART2);
      
      uartWrite(_DEF_UART2, rx_buf, rx_len);   
    } 
  }
}

void cliInfo(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "usb"))
  {
    while(cliKeepLoop())
    {
      cliPrintf("cdc conn : %d, open : %d\n", usbhsCdcDriver()->isConnected(), usbhsCdcDriver()->isOpen());    
      delay(100);
    }
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "usb_tx"))
  {
    uint32_t pre_cnt;
    uint32_t pre_time;

    pre_cnt = uartGetTxCnt(_DEF_UART2);
    pre_time = millis();
    while(cliKeepLoop())
    {
      uartPrintf(_DEF_UART2, "01234567890012345678900123456789001234567890012345678900123456789001234567890\n");

      if (millis()-pre_time >= 1000)
      {
        pre_time = millis();
        
        uint32_t tx_cnt;
        
        tx_cnt = uartGetTxCnt(_DEF_UART2) - pre_cnt;
        cliPrintf("tx cnt : %d, %d KB/sec\n", tx_cnt, tx_cnt/1024);
        pre_cnt = uartGetTxCnt(_DEF_UART2);
      }
    }
    ret = true;
  }


  if (ret == false)
  {
    cliPrintf("info usb\n");
    cliPrintf("info usb_tx\n");
  }
}