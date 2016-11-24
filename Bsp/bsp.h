#ifndef _BSP_H
#define _BSP_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"

#define BUF_SIZE 128
typedef struct
{
  uint8_t len;
  uint8_t buffer[BUF_SIZE];
  
}buf_t;


#endif