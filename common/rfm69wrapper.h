#ifndef __RFM69WRAPPER_h__
#define __RFM69WRAPPER_h__

#include <avr/io.h>
#include "pins.h"
#include "rfm69.h"

#define RFM69_MAX_DATA_LEN   57 // 65 - 8 overhead (uint8_t len, uint24_t dest, uint24_t source, uint8_t ctl)

class RFM69WRAPPER {
  public:
    typedef struct {
      uint8_t type;
      uint8_t len;
      uint8_t payload[15];
    } WPacket;

    RFM69WRAPPER();
    void    init(uint32_t gateway_id, uint8_t network_id);
    void    send(WPacket *packets, uint8_t packets_len, WPacket *responses, uint8_t *response_len, uint32_t *from = 0);
    uint8_t decode(uint8_t *i, RFM69::Packet *response, RFM69WRAPPER::WPacket *packet);
    void    decode_first_byte(uint8_t data, uint8_t *type, uint8_t *len);
    uint8_t encode_first_byte(uint8_t type, uint8_t len);

  private:
    uint32_t gateway_id;
    uint8_t  rssi_limit_reached;
    uint8_t  rssi_last;
    uint8_t  rssi_reset;
    uint8_t  rssi_request;
    uint8_t  fail_count;
    RFM69    rfm69;
};

#endif
