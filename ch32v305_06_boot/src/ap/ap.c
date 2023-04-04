#include "ap.h"
#include "boot/boot.h"


bool is_boot_mode = true;
cmd_t cmd_boot;


firm_ver_t *p_firm_ver = (firm_ver_t *)FLASH_ADDR_FW_VER;




void apInit(void)
{
  uartOpen(_DEF_UART2, 115200);

  if (resetGetBootMode() == RESET_MODE_FW)
  {
    if (resetGetCount() != 2)
    {
      logPrintf("FIRM Mode .. ");
      if (bootVerifyFw() == true)
      {
        logPrintf("OK\n");
        delay(10);
      }
      else
      {
        logPrintf("Fail\n");
      }
    }
  }

  cmdInit(&cmd_boot);
  cmdOpen(&cmd_boot, _DEF_UART2, 115200);
  logPrintf("BOOT Mode ..\n");

  usbInit();
  logBoot(false);
}

void apMain(void)
{
  uint32_t pre_time;
  bool is_retry_update_fw = false;
  uint32_t retry_pre_time;


  pre_time = millis();
  retry_pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 100)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);     
    }    

    if (cmdReceivePacket(&cmd_boot) == true)
    {
      bootProcessCmd(&cmd_boot);
      retry_pre_time = millis();
    }    

    // Timeout
    //
    if (millis()-retry_pre_time >= (15*1000))
    {
      if (is_retry_update_fw == false)
      {
        is_retry_update_fw = true;
        logPrintf("jump fw retry.. ");
        bootJumpToFw();        
        logPrintf("fail\n");
      }
    }    
  }
}
