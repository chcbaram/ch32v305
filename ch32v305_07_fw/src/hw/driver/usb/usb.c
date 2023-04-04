

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
}

bool usbUpdate(void)
{
  return true;
}

bool usbIsConnected(void)
{
  return is_connected;
}

