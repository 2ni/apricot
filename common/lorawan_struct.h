#ifndef __STRUCT__
#define __STRUCT__

#include <avr/io.h>

typedef struct {
  uint8_t *data;
  uint8_t len;
} Packet;

typedef enum {
  OK,            // return 0 -> all good
  NO_ACK,
  TIMEOUT,
  ERROR
} Status;

typedef struct {
  uint8_t nwkskey[16];
  uint8_t appskey[16];
  uint8_t devaddr[4];
  uint16_t counter;
  uint8_t datarate;
} Lora_session;

typedef struct {
  uint8_t deveui[8];
  uint8_t appeui[8];
  uint8_t appkey[16];
  uint16_t devnonce;
} Lora_otaa;

/*
typedef struct {
  uint8_t tx_datarate; // see rfm95_set_datarate function
  uint8_t tx_channel;  // see rfm95_set_channel
  uint8_t rx_datarate;
  uint8_t rx_channel;
  uint8_t power;       // 0x00 - 0x0F
  uint8_t confirm;     // 0x00 unconfirmed, 0x01 confirmed
  uint8_t hopping;     // 0x00 no channel hopping, 0x01 channel hopping
} Lora_settings;
*/

#endif
