## Packet format

<img src="rfm69-packet.png" height="130px" />

## datapacket types
A datapacket consist of a type/len byte followed by len  data bytes. The 1st byte defines what kind of packet it is (4bits) and how long it is (4bits).

A node can therefore send and receive 16 different types of datapackets each, which can be up to 16 bytes long. Types 0-7 are reserved for generic types, 8-15 are node dependent.

### download types (node -> gateway)
| type | name     | length  | description                                          |
| ---- | -----    | ------  | -----------                                          |
| 0x00 | debug    | 1       | used to debug, eg as counter                         |
| 0x01 | vcc      | 2       | voltage of power source, 522 = 5.22v                 |
| 0x02 | temp     | 2       | temperature on pcb, p.475 in datasheet               |
| 0x03 | rssi     | 1 or  2 | RSSI ctrl, [ last received RSSI as uint8_t ]         |
| ...  |          |         |                                                      |
| 0x08 | humidtiy | 4       | returns humidity (2 bytes) and temperature (2 bytes) |

### upload types (gateway -> node, ie returned with ack)
| type | name      | length | description                                          |
| -    | -         | -      | -                                                    |
| 0x00 | debug     | 1      | used to debug, eg as counter                         |
| 0x01 | timestamp | 4      | current timestamp                                    |
| 0x02 |           |        |                                                      |
| 0x03 | rssi      | 1 or 2 | RSSI ctrl, [ last received RSSI as uint8_t ]         |

### RSSI ctrl
The RSSI ctrl byte gives information about power transmission. The lower 4 bits define the amount of change to the power transmission. The higher 4 bits are the control bits. The highest bit is set if a limit is reached, eg power = 0 and change = -1.

<img src="rfm69-datapacket-rssi.png" height="104px" />
