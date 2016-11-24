#include "uart.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern osMessageQId weigherweiHandle;
extern osSemaphoreId checkHandle;

#define ssid "ChinaNet-xZ7a"
#define password "yp6sfnff"
#define ip "192.168.1.4"
#define port "6000"

#define ATPORT huart2

static char tx_buffer[50];
static buf_t at_rx;

static void low_transmit(char* data);
static uint8_t  check_ok(uint8_t *data,uint8_t len);

void at_recv_task()
{
  /* Enable the UART Parity Error and Data Register not empty Interrupts */
  //SET_BIT(ATPORT.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
  HAL_UART_Receive_DMA(&ATPORT,at_rx.buffer,BUF_SIZE);
  while(1)
  {
    /*the buffer is empty*/
    while(hdma_usart2_rx.Instance->NDTR==BUF_SIZE)
    {
      osDelay(500);
    }
    /*the buffer is changing*/
    while(at_rx.len!=(BUF_SIZE-hdma_usart2_rx.Instance->NDTR))
    {
      at_rx.len=BUF_SIZE-hdma_usart2_rx.Instance->NDTR;
      osDelay(100);
    }
    /*the buffer is freeze*/
    if(at_rx.len)
    {
      //information handle
      if(check_ok(at_rx.buffer,at_rx.len))
        continue;
      HAL_UART_Transmit(&huart1,at_rx.buffer,at_rx.len,200);
      
      memset(at_rx.buffer,0,at_rx.len);
      at_rx.len=0;
      HAL_UART_DMAStop(&ATPORT);
      HAL_UART_Receive_DMA(&ATPORT,at_rx.buffer,BUF_SIZE);
      //handle data
      
      
      osSemaphoreRelease(checkHandle);
    }
  }
}

static uint8_t  check_ok(uint8_t *data,uint8_t len)
{
  for(int i=0;i<len;i++)
  {
    if(data[i]=='O'&&data[i+1]=='K')
      return 0;
    if(data[i]=='+'&&data[i+1]=='I'&&data[i+2]=='P'&&data[i+3]=='D')
      return 0;
    if(data[i]=='>')
      return 0;
  }
  return 1;
}
void at_send_task()
{
  //osDelay(10000);
  memset(tx_buffer,0,50);
  //test cmd
  osSemaphoreWait(checkHandle,osWaitForever);
  sprintf(tx_buffer,"\r\nAT\r\n");
  low_transmit(tx_buffer);
  //set as station mode
  osSemaphoreWait(checkHandle,osWaitForever);
  sprintf(tx_buffer,"AT+CWMODE=1\r\n");
  low_transmit(tx_buffer);
  //join the ap
  osSemaphoreWait(checkHandle,osWaitForever);
  sprintf(tx_buffer,"AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,password);
  low_transmit(tx_buffer);
  //get ip
  osSemaphoreWait(checkHandle,osWaitForever);
  sprintf(tx_buffer,"AT+CIFSR\r\n");
  low_transmit(tx_buffer);
  //set mux
  osSemaphoreWait(checkHandle,osWaitForever);
  sprintf(tx_buffer,"AT+CIPMUX=1\r\n");
  low_transmit(tx_buffer);
  //connect the tcp server
  osSemaphoreWait(checkHandle,osWaitForever);
  sprintf(tx_buffer,"AT+CIPSTART=0,\"TCP\",\"%s\",%s\r\n",ip,port);
  low_transmit(tx_buffer);
  
  while(1)
  {
    uint8_t buf[4];
    uint32_t g;
    osEvent evt;
    evt=osMessageGet (weigherweiHandle,osWaitForever);
    g=(uint32_t)evt.value.v;
    if(g>1000)
    {
      buf[0]=(uint8_t)(g/1000)+0x30;
    }
    else
    {
      buf[0]=0+0x30;
    }
    if(g>100)
    {
      buf[1]=(uint8_t)(g%1000/100)+0x30;
    }
    else
    {
      buf[1]=0+0x30;
    }
    if(g>10)
    {
      buf[2]=(uint8_t)(g%100/10)+0x30;
    }
    else
    {
      buf[2]=0+0x30;
    }
    if(g>0)
    {
      buf[3]=(uint8_t)(g%10)+0x30;
    }
    else
    {
      buf[3]=0+0x30;
    }
    osSemaphoreWait(checkHandle,osWaitForever);
    sprintf(tx_buffer,"AT+CIPSEND=0,%c\r\n",'4');
    low_transmit(tx_buffer);
    osSemaphoreWait(checkHandle,osWaitForever);
    HAL_UART_Transmit(&ATPORT,buf,4,200);
    //low_transmit(buf);
    //osDelay(1000);
  }
}
static void low_transmit(char* data)
{
  uint8_t len;
  len=strlen(data);
  HAL_UART_Transmit(&ATPORT,(uint8_t*)tx_buffer,len,200);
  memset(tx_buffer,0,len);
}
//void USART2_IRQHandler(void)
//{
//  uint8_t tmp;
//  tmp=ATPORT.Instance->DR;
//  osMessagePut (uartqHandle,(uint32_t)tmp,0);
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  osMessagePut (uartqHandle,(uint32_t)rx_buf,0);
//  HAL_UART_Receive_IT(&huart1,&rx_buf,1);
//}