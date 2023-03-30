#include "led.h"


#ifdef _USE_HW_LED



const typedef struct 
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  BitAction     on_state;
  BitAction     off_state;
} led_tbl_t;


static led_tbl_t led_tbl[LED_MAX_CH] = 
{
  {GPIOA, GPIO_Pin_3,  Bit_RESET, Bit_SET},
};



bool ledInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};



  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;


  for (int i=0; i<LED_MAX_CH; i++)
  {
    GPIO_InitStructure.GPIO_Pin = led_tbl[i].pin;

    GPIO_Init(led_tbl[i].port, &GPIO_InitStructure);
    ledOff(i);
  }

  return true;
}

void ledOn(uint8_t ch)
{
  if (ch >= LED_MAX_CH) return;

  GPIO_WriteBit(led_tbl[ch].port, led_tbl[ch].pin, led_tbl[ch].on_state);
}

void ledOff(uint8_t ch)
{
  if (ch >= LED_MAX_CH) return;

  GPIO_WriteBit(led_tbl[ch].port, led_tbl[ch].pin, led_tbl[ch].off_state);
}

void ledToggle(uint8_t ch)
{
  if (ch >= LED_MAX_CH) return;

  uint8_t out_bit;

  out_bit = !GPIO_ReadOutputDataBit(led_tbl[ch].port, led_tbl[ch].pin);
  GPIO_WriteBit(led_tbl[ch].port, led_tbl[ch].pin, out_bit);
}
#endif