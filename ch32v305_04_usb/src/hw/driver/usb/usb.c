

#include "usb.h"
#include "usbhs_cdc.h"





static bool is_connected = false;




bool usbInit(void)
{
  usbhsCdcInit();

  return true;
}

void usbDeInit(void)
{
}

bool usbUpdate(void)
{
  return true;
}

bool usbIsConnected(void)
{
  return is_connected;
}

