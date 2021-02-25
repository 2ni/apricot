#ifndef __RFM95_h__
#define __RFM95_h__

#include "lorawan_struct.h"
#include "pins.h"

class RFM95 {
  public:
    // RFM95(pins_t *cs = &pins_csrfm, pins_t *dio0 = &pins_dio0, pins_t *dio1 = &pins_dio1);
    RFM95(pins_t *cs = &pins_csrfm);

    uint8_t  init();
    void     select();
    void     unselect();
    uint8_t  read_reg(uint8_t addr);
    void     write_reg(uint8_t addr, uint8_t data);
    void     send(const Packet *packet, const uint32_t frq, const uint8_t datarate, const uint8_t power=12);
    void     receive_continuous(uint32_t frq, uint8_t datarate);
    Status   wait_for_single_package(uint32_t frq, uint8_t datarate);
    Status   read(Packet *packet);
    uint8_t  set_mode(uint8_t mode);
    void     set_power(int8_t power);
    uint32_t get_random(uint8_t bits=32);
    void     set_datarate(uint8_t rate);
    void     set_channel(uint8_t channel);
    void     set_frq(uint32_t frq);

  private:
    const uint32_t FREQUENCIES[8] = { 8681000, 8683000, 8685000, 8671000, 8673000, 8675000, 8677000, 8679000 };
    const uint32_t FREQUENCY_UP = 8695250;

    pins_t *cs, *dio0, *dio1;
};

#endif
