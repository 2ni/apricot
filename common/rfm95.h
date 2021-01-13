#ifndef __RFM95_h__
#define __RFM95_h__

#include "lorawan_struct.h"
#include "pins.h"

class RFM95 {
  public:
    RFM95(pins_t *cs = &PC3, pins_t *dio0 = &PC4, pins_t *dio1 = &PC5);

    uint8_t  init();
    void     select();
    void     unselect();
    uint8_t  read_reg(uint8_t addr);
    void     write_reg(uint8_t addr, uint8_t data);
    void     send(const Packet *packet, const uint8_t channel, const uint8_t datarate);
    void     receive_continuous(uint8_t channel, uint8_t datarate);
    Status   wait_for_single_package(uint8_t channel, uint8_t datarate);
    Status   read(Packet *packet);
    void     set_mode(uint8_t mode);
    void     setpower(int8_t power);
    void     sleep();
    uint32_t get_random(uint8_t bits=32);
    void     set_datarate(uint8_t rate);
    void     set_channel(uint8_t channel);
    void     set_frq(uint32_t frq);

  private:
    const uint32_t FREQUENCIES[8] = { 868100, 868300, 868500, 867100, 867300, 867500, 867700, 867900 };
    const uint32_t FREQUENCY_UP = 869525;

    pins_t *pins_cs, *pins_dio0, *pins_dio1;
};

#endif
