/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbhs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/08/20
* Description        : This file provides all the USBHS firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "usbhs_cdc.h"
#include "qbuffer.h"

#if 0
#define logUsb(fmt, ...)  logPrintf(fmt, ##__VA_ARGS__)
#else
#define logUsb(fmt, ...)  
#endif


typedef struct
{
  uint32_t baud;
  uint8_t  stop;
  uint8_t  parity;
  uint8_t  data;
  uint8_t  dtr;
  uint8_t  rts;
  uint8_t  buf[8];
} line_coding_t;



// void USBHS_IRQHandler(void) __attribute__((naked));
void USBHS_IRQHandler(void) __attribute__((interrupt("machine")));


void usbhsCdcRccInit(void);
void usbhsCdcEnable(FunctionalState sta);


/* Variable Definition */
const uint8_t    *pUSBHS_Descr;

/* Setup Request */
volatile uint8_t  USBHS_SetupReqCode;
volatile uint8_t  USBHS_SetupReqType;
volatile uint16_t USBHS_SetupReqValue;
volatile uint16_t USBHS_SetupReqIndex;
volatile uint16_t USBHS_SetupReqLen;

/* USB Device Status */
volatile uint8_t  USBHS_DevConfig;
volatile uint8_t  USBHS_DevAddr;
volatile uint16_t USBHS_DevMaxPackLen;
volatile uint8_t  USBHS_DevSpeed;
volatile uint8_t  USBHS_DevSleepStatus;
volatile uint8_t  USBHS_DevEnumStatus;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBHS_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP2_Tx_Buf[ DEF_USB_EP2_HS_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP2_Rx_Buf[ DEF_USB_EP2_HS_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP3_Tx_Buf[ DEF_USB_EP3_HS_SIZE ];

/* Endpoint tx busy flag */
volatile uint8_t  USBHS_Endp_Busy[ DEF_UEP_NUM ];
static line_coding_t line_coding;

qbuffer_t q_rx;
qbuffer_t q_tx;
volatile bool q_rx_full = false;
volatile bool q_tx_busy = false;
static uint8_t q_tx_buf[2048];
static uint8_t q_rx_buf[2048];


volatile bool is_connected = false;
volatile bool is_open = false;




void usbhsCdcInit(void)
{
  qbufferCreate(&q_rx, q_rx_buf, 2048);
  qbufferCreate(&q_tx, q_tx_buf, 2048);

  memset(&line_coding, 0, sizeof(line_coding));

  usbhsCdcRccInit();
  usbhsCdcEnable(ENABLE);
}

void usbhsCdcDeInit(void)
{
  usbhsCdcEnable(DISABLE);
}

void usbhsCdcRccInit(void)
{
  RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
  RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
  RCC_USBHSConfig( RCC_USBPLL_Div2 );
  RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
  RCC_USBHSPHYPLLALIVEcmd( ENABLE );
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );
}

void usbhsCdcEndpInit(void)
{
  uint8_t i;

  USBHSD->ENDP_CONFIG = USBHS_UEP3_T_EN | USBHS_UEP3_R_EN |
                        USBHS_UEP2_T_EN | USBHS_UEP2_R_EN;

  USBHSD->UEP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
  USBHSD->UEP2_MAX_LEN  = DEF_USB_EP3_HS_SIZE;
  USBHSD->UEP3_MAX_LEN  = DEF_USB_EP3_HS_SIZE;

  USBHSD->UEP0_DMA    = (uint32_t)(uint8_t *)USBHS_EP0_Buf;

  USBHSD->UEP3_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP3_Tx_Buf;
  USBHSD->UEP2_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP2_Rx_Buf;
  USBHSD->UEP2_TX_DMA = (uint32_t)(uint8_t *)USBHS_EP2_Tx_Buf;

  USBHSD->UEP0_TX_LEN  = 0;
  USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
  USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

  USBHSD->UEP2_TX_LEN  = 0;
  USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
  USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

  USBHSD->UEP3_TX_LEN  = 0;
  USBHSD->UEP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
  USBHSD->UEP3_RX_CTRL = USBHS_UEP_R_RES_ACK;

  /* Clear End-points Busy Status */
  for( i=0; i<DEF_UEP_NUM; i++ )
  {
    USBHS_Endp_Busy[ i ] = 0;
  }
}

void usbhsCdcEnable(FunctionalState sta)
{
  if( sta )
  {
    USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
    
    for (volatile int i=0; i<100000; i++);

    USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
    USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
    USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
    USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER | USBHS_UIE_DETECT | USBHS_UIE_SUSPEND;
    usbhsCdcEndpInit( );
    USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
    NVIC_EnableIRQ( USBHS_IRQn );
  }
  else
  {
    USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
    
    for (volatile int i=0; i<100000; i++);

    USBHSD->CONTROL = 0;
    NVIC_DisableIRQ( USBHS_IRQn );
  }
}

void usbhsCdcSendResume(void)
{

}

void usbhsCdcUpdate(void)
{
  if (q_rx_full == true)
  {
    if ((q_rx.len - qbufferAvailable(&q_rx)) > (DEF_USB_EP2_HS_SIZE + 1))
    {
      USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
      USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;
      q_rx_full = false;
    }
  }

  uint32_t tx_len;
  
  tx_len = qbufferAvailable(&q_tx);
  if (q_tx_busy == false && tx_len > 0)
  {
    if (tx_len > DEF_USB_EP2_HS_SIZE)
      tx_len = DEF_USB_EP2_HS_SIZE;
    
    qbufferRead(&q_tx, USBHS_EP2_Tx_Buf, tx_len); 
    USBHSD->UEP2_TX_LEN  = tx_len;
    USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
    q_tx_busy = true;
  }
}

bool usbhsCdcFlush(void)
{
  qbufferFlush(&q_rx);
  return true;
}

uint32_t usbhsCdcAvailable(void)
{
  return qbufferAvailable(&q_rx);
}

uint32_t usbhsCdcWrite(uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  uint32_t pre_time;
  uint32_t tx_len;
  uint32_t buf_len;
  uint32_t sent_len;


  if (usbhsIsConnected() != true) return 0;
  if (usbhsIsOpen() != true) return 0;


  sent_len = 0;

  pre_time = millis();
  while(sent_len < length)
  {
    buf_len = (q_tx.len - qbufferAvailable(&q_tx)) - 1;
    tx_len = length - sent_len;

    if (tx_len > buf_len)
    {
      tx_len = buf_len;
    }

    if (tx_len > 0)
    {
      qbufferWrite(&q_tx, p_data, tx_len);
      p_data += tx_len;
      sent_len += tx_len;
    }
    // else
    // {
    //   delay(1);
    // }
    
    if (usbhsIsConnected() != true) break;
    if (usbhsIsOpen() != true) break;

    if (millis()-pre_time >= 100)
    {
      break;
    }
  }
  ret = sent_len;

  return ret;
}

uint32_t usbhsRead(uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;

  if (qbufferRead(&q_rx, p_data, length) == false)
  {
    ret = 0;
  }

  return ret;
}

bool usbhsIsConnected(void)
{
  return is_connected;
}

bool usbhsIsOpen(void)
{
  return is_open;
}

uint32_t usbhsCdcGetBaud(void)
{
  return line_coding.baud;
}

usbhs_cdc_driver_t *usbhsCdcDriver(void)
{
  static usbhs_cdc_driver_t cdc = 
  {
    .flush = usbhsCdcFlush,
    .available = usbhsCdcAvailable,
    .write = usbhsCdcWrite,
    .read = usbhsRead,
    .isConnected = usbhsIsConnected,
    .isOpen = usbhsIsOpen,
    .getBaud = usbhsCdcGetBaud
  };

  return &cdc;
}

void USBHS_IRQHandler(void)
{
  uint8_t  intflag, intst, errflag;
  uint16_t len;
  uint32_t tx_len;

  intflag = USBHSD->INT_FG;
  intst = USBHSD->INT_ST;



  if (intflag & USBHS_UIF_TRANSFER)
  {
    switch (intst & USBHS_UIS_TOKEN_MASK)
    {
      /* data-in stage processing */
      case USBHS_UIS_TOKEN_IN:

        switch (intst & ( USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK ))
        {
          /* end-point 0 data in interrupt */
          case USBHS_UIS_TOKEN_IN | DEF_UEP0:
            logUsb("USBHS_UIS_TOKEN_IN | DEF_UEP0 0x%X 0x%X\n", USBHS_SetupReqType, USBHS_SetupReqCode);
            
            if( USBHS_SetupReqLen == 0 )
            {
              USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
            }
            if (( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD)
            {
              /* Non-standard request endpoint 0 Data upload */

              if (USBHS_SetupReqCode == CDC_SET_LINE_CTLSTE)
              {
                // USB_SETUP_REQ *p_requst = (USB_SETUP_REQ *)USBHS_EP0_Buf;

                line_coding.dtr = (USBHS_SetupReqValue & (1<<0)) ? 1 : 0;
                line_coding.rts = (USBHS_SetupReqValue & (1<<1)) ? 1 : 0;

                is_open = line_coding.dtr;

                logUsb("  CDC_SET_LINE_CTLSTE : dtr %d, rts %d\n", line_coding.dtr, line_coding.rts);
              }
            }
            else
            {
              /* Standard request endpoint 0 Data upload */
              switch( USBHS_SetupReqCode )
              {
                case USB_GET_DESCRIPTOR:
                  logUsb("  USB_GET_DESCRIPTOR\n");
                  len = USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                  memcpy(USBHS_EP0_Buf, pUSBHS_Descr, len);
                  USBHS_SetupReqLen -= len;
                  pUSBHS_Descr += len;
                  USBHSD->UEP0_TX_LEN = len;
                  USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                  break;

                case USB_SET_ADDRESS:
                  logUsb("  USB_SET_ADDRESS\n");
                  USBHSD->DEV_AD = USBHS_DevAddr;
                  break;
                
                default:
                  USBHSD->UEP0_TX_LEN = 0;
                  break;
              }
            }
            break;

          /* end-point 1 data in interrupt */
          case USBHS_UIS_TOKEN_IN | DEF_UEP2:
            logUsb("USBHS_UIS_TOKEN_IN | DEF_UEP2\n");

            USBHSD->UEP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;

            tx_len = qbufferAvailable(&q_tx);
            if (tx_len > 0)
            {
              if (tx_len > DEF_USB_EP2_HS_SIZE)
                tx_len = DEF_USB_EP2_HS_SIZE;
              qbufferRead(&q_tx, USBHS_EP2_Tx_Buf, tx_len); 
              USBHSD->UEP2_TX_LEN  = tx_len;
              USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
              q_tx_busy = true;
            }
            else
            {
              if (USBHSD->UEP2_TX_LEN == DEF_USB_EP2_HS_SIZE)
              {
                USBHSD->UEP2_TX_LEN = 0;
                USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
                q_tx_busy = true;
              }
              else
              {
                USBHSD->UEP2_TX_LEN = 0;
                USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                q_tx_busy = false;
              }
            }
            break;

          /* end-point 4 data in interrupt */
          case USBHS_UIS_TOKEN_IN | DEF_UEP3:
            USBHSD->UEP3_TX_CTRL = (USBHSD->UEP3_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
            USBHSD->UEP3_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
            USBHS_Endp_Busy[ DEF_UEP3 ] &= ~DEF_UEP_BUSY;
            logUsb("USBHS_UIS_TOKEN_IN | DEF_UEP3\n");
            break;

          default :
            break;   
        }
        break;

      /* data-out stage processing */
      case USBHS_UIS_TOKEN_OUT:
        switch( intst & ( USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK ) )
        {
          /* end-point 0 data out interrupt */
          case USBHS_UIS_TOKEN_OUT | DEF_UEP0:
            logUsb("USBHS_UIS_TOKEN_OUT | DEF_UEP0 0x%X 0x%X\n", USBHS_SetupReqType, USBHS_SetupReqCode);
            len = USBHSH->RX_LEN;
            if ( intst & USBHS_UIS_TOG_OK )
            {
              /* if any processing about rx, set it here */
              if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
              {                
                USBHS_SetupReqLen = 0;
                /* Non-standard request end-point 0 Data download */
                if (USBHS_SetupReqCode == CDC_SET_LINE_CODING)
                {
                  logUsb("  CDC_SET_LINE_CODING\n");
                  /* Save relevant parameters such as serial port baud rate */
                  /* The downlinked data is processed in the endpoint 0 OUT packet, the 7 bytes of the downlink are, in order
                    4 bytes: baud rate value: lowest baud rate byte, next lowest baud rate byte, next highest baud rate byte, highest baud rate byte.
                    1 byte: number of stop bits (0: 1 stop bit; 1: 1.5 stop bit; 2: 2 stop bits).
                    1 byte: number of parity bits (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space).
                    1 byte: number of data bits (5,6,7,8,16); */

                  for (int i=0; i<7; i++)
                    line_coding.buf[i] = USBHS_EP0_Buf[i];

                  line_coding.stop   = USBHS_EP0_Buf[4];
                  line_coding.parity = USBHS_EP0_Buf[5];
                  line_coding.baud   = USBHS_EP0_Buf[0] << 0;
                  line_coding.baud  += USBHS_EP0_Buf[1] << 8;
                  line_coding.baud  += USBHS_EP0_Buf[2] << 16;
                  line_coding.baud  += USBHS_EP0_Buf[3] << 24;
                }     
              } 
              else
              {
                /* Standard request end-point 0 Data download */
              }

              if (USBHS_SetupReqLen == 0)
              {
                USBHSD->UEP0_TX_LEN  = 0;
                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
              }          
            }
            break;

          /* end-point 1 data out interrupt */
          case USBHS_UIS_TOKEN_OUT | DEF_UEP2:

            qbufferWrite(&q_rx, USBHS_EP2_Rx_Buf, USBHSD->RX_LEN);
            if ((q_rx.len - qbufferAvailable(&q_rx)) < (DEF_USB_EP2_HS_SIZE + 1))
            {
                USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
                USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_NAK;
                q_rx_full = true;
            }
            USBHSD->UEP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
            break;

          default:  
            errflag = 0xFF;
            break;
        }
        break;

      /* Sof pack processing */
      case USBHS_UIS_TOKEN_SOF:
        break;

      default :
          break;
    }
    USBHSD->INT_FG = USBHS_UIF_TRANSFER;
  }
  else if (intflag & USBHS_UIF_SETUP_ACT)
  {
    logUsb("USBHS_UIF_SETUP_ACT\n");
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_NAK;

    /* Store All Setup Values */
    USBHS_SetupReqType  = pUSBHS_SetupReqPak->bRequestType;
    USBHS_SetupReqCode  = pUSBHS_SetupReqPak->bRequest;
    USBHS_SetupReqLen   = pUSBHS_SetupReqPak->wLength;
    USBHS_SetupReqValue = pUSBHS_SetupReqPak->wValue;
    USBHS_SetupReqIndex = pUSBHS_SetupReqPak->wIndex;

    len = 0;
    errflag = 0;
    if (( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD)
    {
      /* usb non-standard request processing */
      if( USBHS_SetupReqType & USB_REQ_TYP_CLASS )
      {
        logUsb("  USB_REQ_TYP_CLASS\n");
        /* Class requests */
        switch( USBHS_SetupReqCode )
        {
          case CDC_GET_LINE_CODING:
            // pUSBHS_Descr = (uint8_t *)&Uart.Com_Cfg[ 0 ];
            pUSBHS_Descr = line_coding.buf;
            len = 7;
            logUsb("  CDC_GET_LINE_CODING\n");
            break;

          case CDC_SET_LINE_CODING:
            logUsb("  CDC_SET_LINE_CODING\n");
            break;

          case CDC_SET_LINE_CTLSTE:
            logUsb("  CDC_SET_LINE_CTLSTE\n");
            is_open = true;
            break;

          case CDC_SEND_BREAK:
            logUsb("  CDC_SEND_BREAK\n");
            break;

          default:
            errflag = 0xff;
            logUsb("  errflag\n");
            break;
        }
      }
      else if( USBHS_SetupReqType & USB_REQ_TYP_VENDOR )
      {
        /* Manufacturer request */
        logUsb("  USB_REQ_TYP_VENDOR\n");
      }
      else
      {
        errflag = 0xFF;
      }

      /* Copy Descriptors to Endp0 DMA buffer */
      len = (USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
      memcpy( USBHS_EP0_Buf, pUSBHS_Descr, len );
      pUSBHS_Descr += len;
    }
    else
    {
      /* usb standard request processing */
      switch (USBHS_SetupReqCode)
      {
        /* get device/configuration/string/report/... descriptors */
        case USB_GET_DESCRIPTOR:
          switch ((uint8_t)(USBHS_SetupReqValue>>8))
          {
            /* get usb device descriptor */
            case USB_DESCR_TYP_DEVICE:
              pUSBHS_Descr = MyDevDescr;
              len = DEF_USBD_DEVICE_DESC_LEN;
              break;

            /* get usb configuration descriptor */
            case USB_DESCR_TYP_CONFIG:
              /* Query current usb speed */
              if (( USBHSD->SPEED_TYPE & USBHS_SPEED_TYPE_MASK ) == USBHS_SPEED_HIGH)
              {
                /* High speed mode */
                USBHS_DevSpeed = USBHS_SPEED_HIGH;
                USBHS_DevMaxPackLen = DEF_USBD_HS_PACK_SIZE;
              }
              else
              {
                /* Full speed mode */
                USBHS_DevSpeed = USBHS_SPEED_FULL;
                USBHS_DevMaxPackLen = DEF_USBD_FS_PACK_SIZE;
              }

              /* Load usb configuration descriptor by speed */
              if (USBHS_DevSpeed == USBHS_SPEED_HIGH)
              {
                /* High speed mode */
                pUSBHS_Descr = MyCfgDescr_HS;
                len = DEF_USBD_CONFIG_HS_DESC_LEN;
              }
              else
              {
                /* Full speed mode */
                pUSBHS_Descr = MyCfgDescr_FS;
                len = DEF_USBD_CONFIG_FS_DESC_LEN;
              }
              break;

            /* get usb string descriptor */
            case USB_DESCR_TYP_STRING:
              switch( (uint8_t)(USBHS_SetupReqValue&0xFF) )
              {
                /* Descriptor 0, Language descriptor */
                case DEF_STRING_DESC_LANG:
                  pUSBHS_Descr = MyLangDescr;
                  len = DEF_USBD_LANG_DESC_LEN;
                  break;

                /* Descriptor 1, Manufacturers String descriptor */
                case DEF_STRING_DESC_MANU:
                  pUSBHS_Descr = MyManuInfo;
                  len = DEF_USBD_MANU_DESC_LEN;
                  break;

                /* Descriptor 2, Product String descriptor */
                case DEF_STRING_DESC_PROD:
                  pUSBHS_Descr = MyProdInfo;
                  len = DEF_USBD_PROD_DESC_LEN;
                  break;

                /* Descriptor 3, Serial-number String descriptor */
                case DEF_STRING_DESC_SERN:
                  pUSBHS_Descr = MySerNumInfo;
                  len = DEF_USBD_SN_DESC_LEN;
                  break;

                default:
                  errflag = 0xFF;
                  break;
              }
              break;

            /* get usb device qualify descriptor */
            case USB_DESCR_TYP_QUALIF:
              pUSBHS_Descr = MyQuaDesc;
              len = DEF_USBD_QUALFY_DESC_LEN;
              break;

            /* get usb BOS descriptor */
            case USB_DESCR_TYP_BOS:
              /* USB 2.00 DO NOT support BOS descriptor */
              errflag = 0xFF;
              break;

            /* get usb other-speed descriptor */
            case USB_DESCR_TYP_SPEED:
              if( USBHS_DevSpeed == USBHS_SPEED_HIGH )
              {
                /* High speed mode */
                memcpy( &TAB_USB_HS_OSC_DESC[ 2 ], &MyCfgDescr_FS[ 2 ], DEF_USBD_CONFIG_FS_DESC_LEN - 2 );
                pUSBHS_Descr = ( uint8_t * )&TAB_USB_HS_OSC_DESC[ 0 ];
                len = DEF_USBD_CONFIG_FS_DESC_LEN;
              }
              else if( USBHS_DevSpeed == USBHS_SPEED_FULL )
              {
                /* Full speed mode */
                memcpy( &TAB_USB_FS_OSC_DESC[ 2 ], &MyCfgDescr_HS[ 2 ], DEF_USBD_CONFIG_HS_DESC_LEN - 2 );
                pUSBHS_Descr = ( uint8_t * )&TAB_USB_FS_OSC_DESC[ 0 ];
                len = DEF_USBD_CONFIG_HS_DESC_LEN;
              }
              else
              {
                errflag = 0xFF;
              }
              break;

            default : 
              errflag = 0xFF;
              break;
          }

          /* Copy Descriptors to Endp0 DMA buffer */
          if (USBHS_SetupReqLen > len)
          {
            USBHS_SetupReqLen = len;
          }
          len = (USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
          memcpy( USBHS_EP0_Buf, pUSBHS_Descr, len );
          pUSBHS_Descr += len;
          break;

          /* Set usb address */
          case USB_SET_ADDRESS:
            USBHS_DevAddr = (uint16_t)(USBHS_SetupReqValue&0xFF);
            break;

          /* Get usb configuration now set */
          case USB_GET_CONFIGURATION:
            USBHS_EP0_Buf[0] = USBHS_DevConfig;
            if (USBHS_SetupReqLen > 1)
            {
              USBHS_SetupReqLen = 1;
            }
            break;

          /* Set usb configuration to use */
          case USB_SET_CONFIGURATION:
            USBHS_DevConfig = (uint8_t)(USBHS_SetupReqValue&0xFF);
            USBHS_DevEnumStatus = 0x01;
            is_connected = true;
            break;

          /* Clear or disable one usb feature */
          case USB_CLEAR_FEATURE:
            if (( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE)
            {
              /* clear one device feature */
              if ((uint8_t)(USBHS_SetupReqValue&0xFF) == 0x01)
              {
                /* clear usb sleep status, device not prepare to sleep */
                USBHS_DevSleepStatus &= ~0x01;
              }
              else
              {
                errflag = 0xFF;
              }
            }
            else if (( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP)
            {
              /* Set End-point Feature */
              if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
              {
                /* Clear End-point Feature */
                switch( (uint8_t)(USBHS_SetupReqIndex&0xFF) )
                {
                  case (DEF_UEP2 | DEF_UEP_IN):
                    /* Set End-point 2 IN NAK */
                    USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
                    break;

                  case (DEF_UEP2 | DEF_UEP_OUT):
                    /* Set End-point 2 OUT ACK */
                    USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
                    break;

                  case (DEF_UEP3 | DEF_UEP_IN):
                    /* Set End-point 3 IN NAK */
                    USBHSD->UEP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
                    break;

                  default:
                    errflag = 0xFF;
                    break;
                }
              }
              else
              {
                errflag = 0xFF;
              }
            }
            else
            {
              errflag = 0xFF;
            }
            break;

          /* set or enable one usb feature */
          case USB_SET_FEATURE:
            if (( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE)
            {
              /* Set Device Feature */
              if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP )
              {
                if( MyCfgDescr_FS[ 7 ] & 0x20 )
                {
                  /* Set Wake-up flag, device prepare to sleep */
                  USBHS_DevSleepStatus |= 0x01;
                }
                else
                {
                  errflag = 0xFF;
                }
              }
              else
              {
                errflag = 0xFF;
              }
            }
            else if (( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP)
            {
              /* Set End-point Feature */
              if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
              {
                /* Set end-points status stall */
                switch((uint8_t)(USBHS_SetupReqIndex&0xFF) )
                {
                  case (DEF_UEP2 | DEF_UEP_IN):
                    /* Set End-point 2 IN STALL */
                    USBHSD->UEP2_TX_CTRL = ( USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                    break;

                  case (DEF_UEP2 | DEF_UEP_OUT):
                    /* Set End-point 2 OUT STALL */
                    USBHSD->UEP2_RX_CTRL = ( USBHSD->UEP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
                    break;

                  case (DEF_UEP3 | DEF_UEP_IN):
                    /* Set End-point 3 IN STALL */
                    USBHSD->UEP3_TX_CTRL = ( USBHSD->UEP3_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                    break;

                  default:
                    errflag = 0xFF;
                    break;
                }
              }
            }
            break;

          /* This request allows the host to select another setting for the specified interface  */
          case USB_GET_INTERFACE:
              USBHS_EP0_Buf[0] = 0x00;
              if ( USBHS_SetupReqLen > 1 )
              {
                  USBHS_SetupReqLen = 1;
              }
              break;

          case USB_SET_INTERFACE:
              break;

          /* host get status of specified device/interface/end-points */
          case USB_GET_STATUS:
            USBHS_EP0_Buf[0] = 0x00;
            USBHS_EP0_Buf[1] = 0x00;
            if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
            {
              switch( (uint8_t)( USBHS_SetupReqIndex & 0xFF ) )
              {
                case (DEF_UEP2 | DEF_UEP_IN):
                  if( ( (USBHSD->UEP2_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                  {
                    USBHS_EP0_Buf[ 0 ] = 0x01;
                  }
                  break;

                case (DEF_UEP2 | DEF_UEP_OUT):
                  if( ( (USBHSD->UEP2_RX_CTRL) & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL )
                  {
                    USBHS_EP0_Buf[ 0 ] = 0x01;
                  }
                  break;

                case (DEF_UEP3 | DEF_UEP_IN):
                  if( ( (USBHSD->UEP3_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                  {
                    USBHS_EP0_Buf[ 0 ] = 0x01;
                  }
                  break;

                default:
                  errflag = 0xFF;
                  break;
                }
              }
              else if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
              {
                if( USBHS_DevSleepStatus & 0x01 )
                {
                  USBHS_EP0_Buf[ 0 ] = 0x02;
                }
              }

              if ( USBHS_SetupReqLen > 2 )
              {
                  USBHS_SetupReqLen = 2;
              }
              break;

            default:
              errflag = 0xFF;
              break;              
      }
    }

    /* errflag = 0xFF means a request not support or some errors occurred, else correct */
    if( errflag == 0xFF )
    {
      /* if one request not support, return stall */
      USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
      USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
    }
    else
    {
      /* end-point 0 data Tx/Rx */
      if( USBHS_SetupReqType & DEF_UEP_IN )
      {
        /* tx */
        len = (USBHS_SetupReqLen>DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
        USBHS_SetupReqLen -= len;
        USBHSD->UEP0_TX_LEN = len;
        USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
      }
      else
      {
        /* rx */
        if( USBHS_SetupReqLen == 0 )
        {
          USBHSD->UEP0_TX_LEN = 0;
          USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
        }
        else
        {
          USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
        }
      }
    }
    USBHSD->INT_FG = USBHS_UIF_SETUP_ACT;
  }
  else if (intflag & USBHS_UIF_BUS_RST)
  {
    /* usb reset interrupt processing */
    USBHS_DevConfig = 0;
    USBHS_DevAddr = 0;
    USBHS_DevSleepStatus = 0;
    USBHS_DevEnumStatus = 0;
    is_connected = false;
    is_open = false;

    USBHSD->DEV_AD = 0;
    usbhsCdcEndpInit();
    USBHSD->INT_FG = USBHS_UIF_BUS_RST;

    logUsb("USBHS_UIF_BUS_RST\n");
  }
  else if( intflag & USBHS_UIF_SUSPEND )
  {
    USBHSD->INT_FG = USBHS_UIF_SUSPEND;
    /* usb suspend interrupt processing */
    if (USBHSD->MIS_ST & USBHS_UMS_SUSPEND)
    {
      USBHS_DevSleepStatus |= 0x02;
      if( USBHS_DevSleepStatus == 0x03 )
      {
        /* Handling usb sleep here */
      }
    }
    else
    {
      USBHS_DevSleepStatus &= ~0x02;
    }
    logUsb("USBHS_UIF_SUSPEND\n");
    
    is_connected = false;
    is_open = false;
  }
  else
  {
    /* other interrupts */
    USBHSD->INT_FG = intflag;
  }

  // __asm volatile ("mret");
}


