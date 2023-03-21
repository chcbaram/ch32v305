#include "uart.h"
#include "qbuffer.h"

#ifdef _USE_HW_UART


#define UART_RX_BUF_LENGTH        1024


typedef struct
{
  bool is_open;
  uint32_t baud;

  uint8_t  rx_buf[UART_RX_BUF_LENGTH];
  qbuffer_t qbuffer;
  USART_TypeDef       *huart;
  USART_InitTypeDef    huart_init;
  DMA_Channel_TypeDef *hdma_rx;
  DMA_InitTypeDef      hdma_rx_init;
} uart_tbl_t;


static bool is_init = false;
static uart_tbl_t uart_tbl[UART_MAX_CH];




bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    uart_tbl[i].is_open = false;
    uart_tbl[i].baud = 57600;
  }

  is_init = true;

  return true;
}

bool uartDeInit(void)
{
  return true;
}

bool uartIsInit(void)
{
  return is_init;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;
  GPIO_InitTypeDef  GPIO_InitStructure = {0};
  USART_InitTypeDef *p_uart_init;
  DMA_InitTypeDef *p_dma_init;


  if (ch >= UART_MAX_CH) return false;

  if (uart_tbl[ch].is_open == true && uart_tbl[ch].baud == baud)
  {
    return true;
  }


  switch(ch)
  {
    case _DEF_UART1:
      uart_tbl[ch].baud      = baud;

      uart_tbl[ch].huart   = USART1;
      uart_tbl[ch].hdma_rx = DMA1_Channel5;
      
      p_uart_init = &uart_tbl[ch].huart_init; 
      p_dma_init = &uart_tbl[ch].hdma_rx_init;

      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

      GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
      GPIO_PinRemapConfig(GPIO_Remap_USART1_HighBit, ENABLE);


      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
      GPIO_Init(GPIOA, &GPIO_InitStructure);


      qbufferCreate(&uart_tbl[ch].qbuffer, &uart_tbl[ch].rx_buf[0], UART_RX_BUF_LENGTH);

      p_uart_init->USART_BaudRate    = baud;
      p_uart_init->USART_WordLength  = USART_WordLength_8b;
      p_uart_init->USART_StopBits    = USART_StopBits_1;
      p_uart_init->USART_Parity      = USART_Parity_No;
      p_uart_init->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      p_uart_init->USART_Mode        = USART_Mode_Tx | USART_Mode_Rx;


      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
      DMA_DeInit(uart_tbl[ch].hdma_rx);
      p_dma_init->DMA_PeripheralBaseAddr  = (uint32_t)(&uart_tbl[ch].huart->DATAR); 
      p_dma_init->DMA_MemoryBaseAddr      = (uint32_t)&uart_tbl[ch].rx_buf[0];
      p_dma_init->DMA_DIR                 = DMA_DIR_PeripheralSRC;
      p_dma_init->DMA_BufferSize          = UART_RX_BUF_LENGTH;
      p_dma_init->DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
      p_dma_init->DMA_MemoryInc           = DMA_MemoryInc_Enable;
      p_dma_init->DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
      p_dma_init->DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
      p_dma_init->DMA_Mode                = DMA_Mode_Circular;
      p_dma_init->DMA_Priority            = DMA_Priority_VeryHigh;
      p_dma_init->DMA_M2M                 = DMA_M2M_Disable;
      DMA_Init(uart_tbl[ch].hdma_rx, p_dma_init);
      DMA_Cmd(uart_tbl[ch].hdma_rx, ENABLE);  

      USART_Init(uart_tbl[ch].huart, p_uart_init);
      USART_Cmd(uart_tbl[ch].huart, ENABLE);

      USART_DMACmd(uart_tbl[ch].huart, USART_DMAReq_Rx, ENABLE);

      uart_tbl[ch].qbuffer.in  = uart_tbl[ch].qbuffer.len - uart_tbl[ch].hdma_rx->CNTR;
      uart_tbl[ch].qbuffer.out = uart_tbl[ch].qbuffer.in;
      uart_tbl[ch].is_open = true;
      ret = true;
      break;
  }

  return ret;
}

bool uartClose(uint8_t ch)
{
  if (ch >= UART_MAX_CH) return false;

  uart_tbl[ch].is_open = false;

  return true;
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case _DEF_UART1:
      uart_tbl[ch].qbuffer.in = (uart_tbl[ch].qbuffer.len - uart_tbl[ch].hdma_rx->CNTR);
      ret = qbufferAvailable(&uart_tbl[ch].qbuffer);      
      break;
  }

  return ret;
}

bool uartFlush(uint8_t ch)
{
  uint32_t pre_time;


  pre_time = millis();
  while(uartAvailable(ch))
  {
    if (millis()-pre_time >= 10)
    {
      break;
    }
    uartRead(ch);
  }

  return true;
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;


  switch(ch)
  {
    case _DEF_UART1:
      qbufferRead(&uart_tbl[ch].qbuffer, &ret, 1);
      break;
  }

  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  uint32_t pre_time;
  uint32_t count = 0;
  bool req_send = true;

  switch(ch)
  {
    case _DEF_UART1:
      pre_time = millis();
      while(millis()-pre_time < 100)
      {
        if (req_send == true)
        {
          USART_SendData(uart_tbl[ch].huart, p_data[count]);
          req_send = false;
        }
        else
        {
          if (USART_GetFlagStatus(uart_tbl[ch].huart, USART_FLAG_TXE) == SET)
          {
            count++;
            req_send = true;
            if (count == length)
            {
              break;
            }
          } 
        }
      }
      break;
  }

  ret = count;

  return ret;
}

uint32_t uartPrintf(uint8_t ch, const char *fmt, ...)
{
  char buf[256];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, 256, fmt, args);

  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;


  if (ch >= UART_MAX_CH) return 0;

  ret = uart_tbl[ch].baud;
  
  return ret;
}

#endif
