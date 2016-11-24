#ifndef _PN532_H
#define _PN532_H

#include "bsp.h"

void nfc_rx_task();
void nfc_tx_task();


typedef enum
{
  idle=0U,
  pre,
  sof_1,
  sof_2,
  ack_s,
  nack_s,
  data_s,
  data_in,
  data_done,
  dcs,
  error,
  ack_s1
}nfc_ack_e;

typedef struct
{
  uint8_t  u_len;
  uint8_t  uuid[12];
  uint8_t capacity;
  uint8_t key;
}card_s;
#endif