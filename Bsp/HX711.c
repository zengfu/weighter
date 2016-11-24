
#include "hx711.h"

static void nop(int x);
extern osMessageQId weigherweiHandle;
static uint32_t ReadCount(void);


void weight_task()
{
  uint32_t first;
  int32_t w;
  float a;
  first=ReadCount();
  
  while(1)
  {
    osDelay(1000);  
    w=ReadCount()-first;
    if(w<0)
      w=0;
    a=w/400.0;
    osMessagePut (weigherweiHandle,(uint32_t)a, 0);    
  }
}
static uint32_t ReadCount(void)
{
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
  uint32_t count=0;
  while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6));
  for(int i=0;i<24;i++)
  {
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);
    count=count<<1;
    //nop(10);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
    //nop(5);
    if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6))
      count++;
  }
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);
  count=count^0x800000;
  nop(5);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
  return count;
}

static void nop(int x)
{
  for(int i=x;i>0;i--)
  {
    ;
  }
}
