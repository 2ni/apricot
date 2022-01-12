# A simple simulator for dcc signals.

### Example of a 14bit multifunction decoder (extended address mode)
```
{preamble} 0 11AAAAAA AAAAAAAA 0 CCCDDDDD 0 EEEEEEEE 1
A: address bit
C: type bit (see [doc](https://dccwiki.com/Digital_Packet))
D: data bit
E: control bit (xor)
```

<img src="example-multifunction-decoder-signal.png" width="800px" />


### Links
- best doc: http://vhdm.de/index.php?option=com_content&view=article&id=49&Itemid=61&eprivacy=1
- cv: https://dccwiki.com/Configuration_Variable
- decoder programming: https://dccwiki.com/Decoder_Programming
- address range: https://dccwiki.com/Address_Range
- examples: https://www.opendcc.de/elektronik/opendcc/opendcc_sw_lenz.html
- nmra extended packet formats: https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf
- extended accessory decoder 11bit: https://www.iascaled.com/blog/high-current-dcc-accessory-decoder/


- https://www.nmra.org/index-nmra-standards-and-recommended-practices
- https://dccwiki.com/Digital_packet
- https://dccwiki.com/DCC_Tutorial_(Power)
- http://normen.railcommunity.de/RCN-211.pdf
- https://github.com/mrrwa/NmraDcc/blob/master/NmraDcc.cpp
- https://www.opendcc.de/elektronik/opendecoder/OpenDecoder_V04.zip
- http://www.opendcc.net/elektronik/opendecoder/opendecoder_sw_rm_e.html
- https://www.lucadentella.it/en/2017/11/25/dcc-decoder-accessori-per-led/


### Knowledge
- high: 58us -vcc, 58us +vcc
- low: 116us -vcc, 116us +vcc
