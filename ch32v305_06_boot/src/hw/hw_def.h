#ifndef HW_DEF_H_
#define HW_DEF_H_



#include "bsp.h"


#define _DEF_FIRMWATRE_VERSION    "V230402R1"
#define _DEF_BOARD_NAME           "CH32V305_BOOT"



#define _USE_HW_VCP
#define _USE_HW_FLASH
#define _USE_HW_RESET


#define _USE_HW_LED
#define      HW_LED_MAX_CH          1

#define _USE_HW_UART
#define      HW_UART_MAX_CH         2

#define _USE_HW_LOG
#define      HW_LOG_CH              _DEF_UART1
#define      HW_LOG_BOOT_BUF_MAX    1024
#define      HW_LOG_LIST_BUF_MAX    1024

#define _USE_HW_CMD
#define      HW_CMD_MAX_DATA_LENGTH (1024+8)


#define FLASH_ADDR_BOOT             0x08000000
#define FLASH_ADDR_BOOT_VER         0x08000400

#define FLASH_ADDR_TAG              0x08008000
#define FLASH_ADDR_FW               0x08008400
#define FLASH_ADDR_FW_VER           0x08008800


#endif