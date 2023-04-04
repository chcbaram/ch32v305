

#include "usb.h"
#include "usbhs_cdc.h"
#include "cli.h"





static bool is_connected = false;




bool usbInit(void)
{
  usbhsCdcInit();

  return true;
}

void usbDeInit(void)
{
  usbhsCdcDeInit();
  NVIC_DisableIRQ(USBHS_IRQn);
  
  USBHSD->HOST_CTRL   = 0x00;
  USBHSD->CONTROL     = 0x00;
  USBHSD->INT_EN      = 0x00;
  USBHSD->ENDP_CONFIG = 0xFFffffff;
  USBHSD->CONTROL    &= ~USBHS_UC_DEV_PU_EN;
  USBHSD->CONTROL    |= USBHS_UC_CLR_ALL|USBHS_UC_RESET_SIE;
  USBHSD->CONTROL     = 0x00;  
}

bool usbUpdate(void)
{
  return true;
}

bool usbIsConnected(void)
{
  return is_connected;
}

