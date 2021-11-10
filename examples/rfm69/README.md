## Packet format

<img src="rfm69-packet.png" width="1000" />

## datapacket types
A datapacket consist of a type/len byte followed by len  data bytes. The 1st byte defines what kind of packet it is (4bits) and how long it is (4bits).

A node can therefore send and receive 16 different types of datapackets each, which can be up to 16 bytes long. Types 0-7 are reserved for generic types, 8-15 are node dependent.

**download types (node -> gateway)**
| type | name     | length | description                                          |
| ---- | -----    | ------ | -----------                                          |
| 0x00 | debug    | 1      | used to debug, eg as counter                         |
| 0x01 | vcc      | 2      | voltage of power source                              |
| 0x02 | rssi     | 1      | last received RSSI as uint8_t                        |
| ...  |          |        |                                                      |
| 0x08 | humidtiy | 4      | returns humidity (2 bytes) and temperature (2 bytes) |

**upload types (gateway -> node, ie returned with ack)** 
| type | name      | length | description                   |
| -    | -         | -      | -                             |
| 0x00 | debug     | 1      | used to debug, eg as counter  |
| 0x01 | timestamp | 4      | current timestamp             |
| 0x02 | rssi      | 1      | last received RSSI as uint8_t |
| 0x03 | trxpwr    | 1      | increment/decrement power     |
