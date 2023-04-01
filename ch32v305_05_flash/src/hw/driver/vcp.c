#include "vcp.h"
#include "usb.h"



#ifdef _USE_HW_VCP


static bool is_init = false;





bool vcpInit(void)
{
  is_init = true;
  return true;
}

bool vcpFlush(void)
{

  return true;
}

uint32_t vcpGetBaud(void)
{
  return usbhsCdcDriver()->getBaud();
}

uint32_t vcpAvailable(void)
{
  if (!vcpIsConnected())
    return 0;

  return usbhsCdcDriver()->available();
}

uint32_t vcpWrite(uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;

  if (vcpIsConnected() != true)
    return 0;

  ret = usbhsCdcDriver()->write(p_data, length);
  
  return ret;
}

uint8_t  vcpRead(void)
{
  uint8_t ret = 0;

  usbhsCdcDriver()->read(&ret, 1);

  return ret;
}

bool vcpIsConnected(void)
{
  bool ret;

  ret = usbhsCdcDriver()->isConnected() & usbhsCdcDriver()->isOpen();
  return ret;
}

uint32_t vcpPrintf( const char *fmt, ...)
{
  char buf[256];
  va_list args;
  int len;
  uint32_t ret;


  va_start(args, fmt);
  len = vsnprintf(buf, 256, fmt, args);

  ret = vcpWrite((uint8_t *)buf, len);

  va_end(args);
  return ret;
}


#endif