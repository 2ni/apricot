/*
 * https://github.com/nayem-cosmic/RFM69-Library-AVR
 */
#ifndef __RFM69_h__
#define __RFM69_h__

#include <avr/io.h>
#include "pins.h"

#define RFM69_MAX_DATA_LEN       61 // 65 - 4 overhead (uint8_t len, uint8_t dest, uint8_t source, uint8_t ctl)
#define RFM69_CTL_SENDACK       0x80
#define RFM69_CTL_REQACK        0x40
#define RFM69_BROADCAST_ADDR    0

class RFM69 {
  public:
    typedef enum {
      SLEEP,   // xtal off
      STANDBY, // xtal on
      SYNTH,   // pll on
      RX,
      TX
    } Mode;

    typedef struct {
      uint8_t message[RFM69_MAX_DATA_LEN+1];
      uint8_t from;
      int16_t rssi;
    } Packet;

    RFM69(pins_t pin_cs, pins_t pin_interrupt);
    RFM69();
    uint8_t  init(uint16_t freq, uint8_t node_id, uint8_t network_id);
    uint8_t  init(uint8_t node_id, uint8_t network_id);
    uint8_t  read_reg(uint8_t addr);
    void     write_reg(uint8_t addr, uint8_t value);
    void     encrypt(const char* key);
    void     set_high_power();
    void     set_network(uint8_t id);
    void     set_power_level(uint8_t level);
    uint8_t  send(uint8_t to, const void* buffer, uint8_t buffer_len, RFM69::Packet *response = 0);
    uint8_t  send_retry(uint8_t to, const void* buffer, uint8_t buffer_len, RFM69::Packet *response, uint8_t retries);
    uint8_t  listen(RFM69::Packet *response, uint8_t timeout_enabled = 1);
    int16_t  read_rssi(uint8_t force = 0);
    void     send_frame(uint8_t to, const void* buffer, uint8_t size, uint8_t request_ack, uint8_t send_ack);
    void     sleep();

    static RFM69 *rfm69_ptr;
    volatile uint8_t isr;
    pins_t pin_interrupt;

  private:
    Mode mode;
    pins_t pin_cs;
    uint8_t power_level;
    uint8_t node_id;
    uint8_t spy_mode;
    void    init_vars(pins_t pins_cs, pins_t pin_interrupt);

    void     set_mode(Mode mode);
    void     select();
    void     unselect();
    void     set_high_power_regs(uint8_t enable);
};

#endif
