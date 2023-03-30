#include "bsp.h"





static volatile uint32_t systick_counter = 0;




bool bspInit(void)
{
  SystemCoreClockUpdate();
  
  NVIC_SetPriority(SysTicK_IRQn, 0xF0);


  SysTick->SR   = 0;
  SysTick->CNT  = 0;
  SysTick->CMP  = (uint32_t)((SystemCoreClock/1000) - 1UL);
  SysTick->CTLR = 0x0F;

  NVIC_EnableIRQ(SysTicK_IRQn);

  return true;
}

void delay(uint32_t time_ms)
{
  uint32_t pre_time = systick_counter;

  while(systick_counter-pre_time < time_ms);
}

uint32_t millis(void)
{
  return systick_counter;
}

extern void usbhsDeviceUpdate(void);

void SysTick_Handler(void) __attribute__((naked));
void SysTick_Handler(void)
{
  systick_counter++;
  SysTick->SR = 0;

  usbhsDeviceUpdate();

  __asm volatile ("mret");
}

