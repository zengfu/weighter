#include "pn532.h"


#define NFCPORT huart3

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern osMessageQId pn532rxHandle;
extern osSemaphoreId cardHandle;
extern DMA_HandleTypeDef hdma_usart3_rx;

static uint8_t handle(uint8_t *data);
static void send_key();
static void nfc_read();

uint8_t wakeup[24]={0x55,0x55,00,00,00,00,00,00,00,00,00,00,00,00,00,00,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,00};
uint8_t find[11]={0x00,0x00,0xff,0x04,0xfc,0xd4,0x4a,0x02,0x00,0xe0,0x00};
uint8_t readid[15]={00,00,0xff,0x08,0xf8,0xd4,0x40,0x01,0xc0,0xf1,0x00,0x01,0x08,0x31,00};
uint8_t nfc_cmd[50]={0};
static card_s card1;
static buf_t nfc_rx;

void nfc_rx_task()
{
  osEvent evt;
  nfc_ack_e state=idle;
  uint8_t tmp;
  uint8_t *data;
  uint8_t len;
  
  memset(&card1,0,sizeof(card1));
  /* Enable the UART Parity Error and Data Register not empty Interrupts */
  //SET_BIT(huart3.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
  HAL_UART_Receive_DMA(&NFCPORT,nfc_rx.buffer,BUF_SIZE);
  while(1)
  {
    /*the buffer is empty*/
    while(hdma_usart3_rx.Instance->NDTR==BUF_SIZE)
    {
      osDelay(500);
    }
    /*the buffer is changing*/
    while(nfc_rx.len!=(BUF_SIZE-hdma_usart3_rx.Instance->NDTR))
    {
      nfc_rx.len=BUF_SIZE-hdma_usart3_rx.Instance->NDTR;
      osDelay(100);
    }
    /*the buffer is freeze*/
    if(nfc_rx.len)
    {
      if(nfc_rx.len==6)
      {
        osDelay(200);
        continue;
      }
      HAL_UART_Transmit(&huart1,nfc_rx.buffer,nfc_rx.len,200);
      if(handle(nfc_rx.buffer))
      {
        
      }
      memset(nfc_rx.buffer,0,nfc_rx.len);
      nfc_rx.len=0;
      HAL_UART_DMAStop(&NFCPORT);
      HAL_UART_Receive_DMA(&NFCPORT,nfc_rx.buffer,BUF_SIZE);
      //handle data
      
      osSemaphoreRelease(cardHandle);
    }
  }
  
}
static uint8_t handle(uint8_t *data)
{
  //uuid
  if(data[12]==0x4b)
  {
    card1.capacity=data[17];
    card1.u_len=data[18];
    for(int i=0;i<data[18];i++)
    {
      card1.uuid[i]=data[19+i];
    }
    osMessagePut (pn532rxHandle,(uint32_t)1,0);
    return 0;
  }
  //wakeup
  if(data[12]==0x15)
    return 0;
  //key
  if(data[12]==0x41)
  {
    if(data[13]==0x00)
    {
      //osMessagePut (pn532rxHandle,(uint32_t)1,0);
      card1.key=1;
      return 0;
    }
    else
    {
      card1.key=0;
      return 1;
    }
  }
  
  
}
void nfc_tx_task()
{
  osSemaphoreWait(cardHandle,osWaitForever);
  HAL_UART_Transmit(&huart3,wakeup,24,200);
  
  
  while(1)
  {
    //find card
    osSemaphoreWait(cardHandle,osWaitForever);
    HAL_UART_Transmit(&huart3,find,11,200);
    osSemaphoreWait(cardHandle,osWaitForever);
    send_key();     
    if(card1.key)
    {
        
        //write();     
        osSemaphoreWait(cardHandle,osWaitForever);
        nfc_read();
     }
      
    
    //HAL_UART_Transmit(&huart3,find,11,200);
  }
}
static void nfc_read()
{
  uint8_t count=0;
  nfc_cmd[count++]=0x00;
  nfc_cmd[count++]=0x00;
  nfc_cmd[count++]=0xff;
  nfc_cmd[count++]=0x05;
  nfc_cmd[count++]=0xfb;
  nfc_cmd[count++]=0xd4;
  nfc_cmd[count++]=0x40;
  nfc_cmd[count++]=0x01;
  nfc_cmd[count++]=0x30;
  nfc_cmd[count++]=0x07;
  nfc_cmd[count++]=0xb4;
  nfc_cmd[count++]=0x00;
  HAL_UART_Transmit(&huart3,nfc_cmd,count,200);
  memset(nfc_cmd,0,count);
}
static void send_key()
{
  uint8_t count=0;
  uint32_t sum=0;
  nfc_cmd[count++]=0x00;
  nfc_cmd[count++]=0x00;
  nfc_cmd[count++]=0xff;
  nfc_cmd[count++]=0x0f;
  nfc_cmd[count++]=0xf1;
  nfc_cmd[count++]=0xd4;
  nfc_cmd[count++]=0x40;
  nfc_cmd[count++]=0x01;
  nfc_cmd[count++]=0x60;//key cmd
  nfc_cmd[count++]=0x07;
  nfc_cmd[count++]=0xff;
  nfc_cmd[count++]=0xff;
  nfc_cmd[count++]=0xff;
  nfc_cmd[count++]=0xff;
  nfc_cmd[count++]=0xff;
  nfc_cmd[count++]=0xff;
//  nfc_cmd[count++]=0x02;
//  nfc_cmd[count++]=0xf5;
//  nfc_cmd[count++]=0x13;
//  nfc_cmd[count++]=0xbe;
  for(int i=0;i<card1.u_len;i++)
  {
    nfc_cmd[count++]=card1.uuid[i];
  }
  for(int i=0;i<0x0f;i++)
  {
    sum+=nfc_cmd[i+5];
  }
  nfc_cmd[count++]=0xff-(uint8_t)sum+1;
  nfc_cmd[count++]=0;
  HAL_UART_Transmit(&huart3,nfc_cmd,count,200);
  memset(nfc_cmd,0,count);
  //HAL_UART_Transmit(&huart1,nfc_cmd,count,200);
}
//void USART3_IRQHandler(void)
//{
//  uint8_t tmp;
//  tmp=huart3.Instance->DR;
//  osMessagePut (pn532rxHandle,(uint32_t)tmp,0);
//}