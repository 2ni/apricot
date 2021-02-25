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
  uint8_t  nwkskey[16];
  uint8_t  appskey[16];
  uint8_t  devaddr[4];
  uint16_t counter;
  uint8_t  txdatarate;     // datarate with which we send
  uint8_t  rxdelay;        // delay for 1st rx window in seconds
  uint8_t  rxoffset;       // datarate offset between tx and rx
  uint8_t  rx2datarate;    // datarate for 2nd rx window
  uint8_t  txpower;        // power with which we send
  uint16_t chmask;         // which channels can be used 0: channel 1, 1: channel 2,.. 15: channel 16
  uint32_t frequencies[9]; // frequencies which can be used, rx2 frq is frequencies[8]
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
