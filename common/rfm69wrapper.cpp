#include <util/delay.h>
#include <avr/interrupt.h>

#include "rfm69wrapper.h"
#include "rfm69.h"
#include "pins.h"
#include "uart.h"
#include "mcu.h"


RFM69WRAPPER::RFM69WRAPPER() {
}

/*
 * include this function in the isr
 * eg
 * ISR(PORTC_PORT_vect) {
 *   rf.isr();
 * }
 */
void RFM69WRAPPER::isr() {
  this->rfm69.isr = 1;
  this->rfm69.pin_interrupt.port->INTFLAGS |= (1<<this->rfm69.pin_interrupt.pin); // clear interrupt flag
}

uint8_t RFM69WRAPPER::init(uint32_t gateway_id, uint8_t network_id) {
  this->gateway_id = gateway_id;
  this->rssi_limit_reached = 0;
  this->rssi_last = 0;
  this->rssi_reset = 1; // ensure we start ATC from scratch at the beginning
  this->fail_count = 0;

  uint8_t version = this->rfm69.init(get_deviceid(), network_id);
  if (!version) {
    DL("rfm69 init failed");
    return 1;
  } else {
    DF("RFM69 Version: 0x%02x\n", version);
    return 0;
  }
}

void RFM69WRAPPER::send(WPacket *packets, uint8_t packets_len, WPacket *responses, uint8_t *responses_len, uint32_t *from) {
  uint8_t stream[RFM69_MAX_DATA_LEN];
  uint8_t stream_len = 0;
  uint8_t ii = 0;
  int8_t pwr_change;
  RFM69::Packet response;
  DF("sending %u packets | pwr: %u | ", packets_len, rfm69.get_power_level());

  // add rssi packet to stream if necessary (as 1st packet!)
  if (this->rssi_limit_reached || this->rssi_reset || this->rssi_request) {
    DF("lim: %u res: %u requ: %u rssi: %udBm | ", this->rssi_limit_reached?1:0, this->rssi_reset?1:0, this->rssi_request?1:0, this->rssi_last);
    stream[stream_len++] = (0x03<<4) | 0x02; // type: rssi (0x03), 2bytes
    stream[stream_len++] = (this->rssi_limit_reached ? 0x80 : 0x00)
      | (this->rssi_reset ? 0x40 : 0x00)
      | (this->rssi_request ? 0x20 : 0x00) ;  // ctrl (limit, reset, request)
    stream[stream_len++] = this->rssi_last;

    this->rssi_limit_reached = 0;
    this->rssi_request = 0;
  }
  uart_arr("raw send", stream, stream_len, 0);

  // copy packets to send to stream
  for (ii=0; ii<packets_len; ii++) {
    stream[stream_len++] = this->encode_first_byte(packets[ii].type, packets[ii].len);
    for (uint8_t iii=0; iii<packets[ii].len; iii++) {
      stream[stream_len++] = packets[ii].payload[iii];
    }
  }

  *responses_len = 0;
  DL("");
  // handle response
  if (rfm69.send_retry(this->gateway_id, stream, stream_len, &response, 3)) {
    this->rssi_reset = 0;
    this->fail_count = 5;
    this->rssi_last = (uint8_t)(-response.rssi);
    DF("  " OK("ack") " (%idBm) | ", response.rssi);
    if (from) *from = response.from;
    uint8_t ii = 0;
    WPacket decoded;
    while (this->decode(&ii, &response, &decoded)) {
      switch (decoded.type) {
        case 0x03: // rssi
          pwr_change = (decoded.payload[0] & 0x0f) | (decoded.payload[0] & 0x08 ? 0xf0 : 0x00); // convert int4_t to int8_t
          this->rssi_request = decoded.payload[0] & 0x20;
          this->rssi_limit_reached = pwr_change && !this->rfm69.set_power_level_relative(pwr_change);
          DF("pwr change: %i rssi_request: %i | ", pwr_change, this->rssi_request?1:0);
        break;
        default:
          // copy decoded to responses
          responses[*responses_len].type = decoded.type;
          responses[*responses_len].len = decoded.len;
          for (uint8_t i=0; i<decoded.len; i++) {
            responses[*responses_len].payload[i] = decoded.payload[i];
          }
          (*responses_len)++;
        break;
       }
    }
    uart_arr("raw", response.payload, response.len);
  } else {
    D(NOK("no response | "));
    if (--this->fail_count == 0) {
      // failed receiving data for more than x times
      this->rssi_reset = 1;
      this->rfm69.set_power_level(23);
      D("reset power | ");
    }
    DL("");
  }
  this->rfm69.sleep();
}

void RFM69WRAPPER::decode_first_byte(uint8_t data, uint8_t *type, uint8_t *len) {
  *type = data >> 4;
  *len = data & 0x0f;
}

uint8_t RFM69WRAPPER::encode_first_byte(uint8_t type, uint8_t len) {
  return (type << 4) | (len & 0x0f);
}

uint8_t RFM69WRAPPER::decode(uint8_t *i, RFM69::Packet *response, RFM69WRAPPER::WPacket *packet) {
  if (*i == response->len) return 0;

  uint8_t data_type;
  uint8_t data_len;
  this->decode_first_byte(response->payload[*i], &data_type, &data_len);
  (*i)++;
  // we skip rssi packet and get the next packet
  /*
  if (data_type == 0x03) {
    *i += data_len;
    if (*i == response->len) return 0;
    this->decode_first_byte(response->payload[*i], &data_type, &data_len);
    (*i)++;
  }
  */

  packet->type = data_type;
  packet->len = data_len;
  for (uint8_t ii=0; ii<data_len; ii++) {
    packet->payload[ii] = response->payload[*i+ii];
  }
  *i += data_len;
  return 1;
}
