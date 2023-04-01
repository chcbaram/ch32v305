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

  cliInit();
  logInit();
  ledInit();
  flashInit();
  uartInit();
  logOpen(HW_LOG_CH, 115200);

  logPrintf("\r\n[ Firmware Begin... ]\r\n");
  logPrintf("Booting..Name \t\t: %s\r\n", _DEF_BOARD_NAME);
  logPrintf("Booting..Ver  \t\t: %s\r\n", _DEF_FIRMWATRE_VERSION);  
  logPrintf("\n");

  vcpInit();  
  usbInit();

  return true;
}