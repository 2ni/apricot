#ifndef __QUEUETRACKSWITCH_H__
#define __QUEUETRACKSWITCH_H__

#include <avr/io.h>
#include "dcc_structs.h"

class QUEUE {
  public:
    QUEUE();
    uint8_t empty();
    uint8_t full();
    void push(DCC::PACKET &packet);
    void pull(DCC::PACKET &packet);
  private:
    static const uint8_t QUEUE_SIZE = 8;
    DCC::PACKET packets[QUEUE_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;
};
#endif
