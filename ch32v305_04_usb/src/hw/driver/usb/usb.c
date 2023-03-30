

#include "usb.h"
#include "usbhs_device.h"





static bool is_connected = false;




bool usbInit(void)
{
  usbhsDeviceInit();

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

