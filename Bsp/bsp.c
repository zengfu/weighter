#include "bsp.h"

extern UART_HandleTypeDef huart1;


int fputc(int ch, FILE *f)
{
  uint8_t a=ch;
  HAL_UART_Transmit(&huart1, &a, 1, 100);
  return ch;      
}


