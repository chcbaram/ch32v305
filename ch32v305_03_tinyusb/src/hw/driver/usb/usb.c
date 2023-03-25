

#include "usb.h"



void USBHS_IRQHandler (void) __attribute__((naked));
void USBHS_IRQHandler (void)
{
  tud_int_handler(0);
  __asm volatile ("mret");
}


static bool is_connected = false;




bool usbInit(void)
{
  __disable_irq();

  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
  RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
  RCC_USBHSConfig(RCC_USBPLL_Div2);
  RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
  RCC_USBHSPHYPLLALIVEcmd(ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);

  __enable_irq();

  tud_init(BOARD_TUD_RHPORT);
  return true;
}

void usbDeInit(void)
{
}

bool usbUpdate(void)
{
  tud_task(); 
  return true;
}

bool usbIsConnected(void)
{
  return is_connected;
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
  // logPrintf("tud_mount_cb()\n");
  is_connected = true;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  // logPrintf("tud_umount_cb()\n");
  is_connected = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  // logPrintf("tud_suspend_cb() %d\n", remote_wakeup_en);
  is_connected = false;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  // logPrintf("tud_resume_cb()\n");
  is_connected = true;
}
