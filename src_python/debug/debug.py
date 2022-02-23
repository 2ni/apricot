"""
https://github.com/chris-heo/updizombie
protocol: https://github.com/mraardvark/pyedbglib/blob/master/pyedbglib/protocols/avr8protocol.py


https://aykevl.nl/2020/06/simavr-debug
https://github.com/stemnic/pyAVRdbg
https://github.com/Polarisru/updiprog
https://github.com/microchip-pic-avr-tools/pymcuprog

port: eg /dev/cu.usbserial-1430

- to get address of global variables toolchain_microchip/bin/avr-nm main.elf | grep " B \| D "
- show info                          toolchain_microchip/bin/avr-readelf -a main.elf

avr-gdb main.elf
(gdb) target remote :1234
(gdb) b main // set breakpoint at main
(gdb) c      // continue

"""

import updi.link as link
import updi.constants as constants
import time


def regprop(address, length, doc=None):
    def getter(self):
        return self.mcu.reg_read(self.baseaddress + address, length)

    def setter(self, value):
        self.mcu.reg_write(self.baseaddress + address, length, value)

    return property(getter, setter, doc=doc)


class ATtiny3217Debug():
    def __init__(self, link):
        self.link = link

    def attach(self):
        self.link.key(constants.UPDI_KEY_64, b'OCD     ')
        self.link.stcs(4, 0x01)  # stop

    def detach(self):
        self.link.stcs(4, 0x02)  # go
        # self.link.stcs(updi.constants.UPDI_CS_CTRLB,
        #    (1 << updi.constants.UPDI_CTRLB_CCDETDIS_BIT) |
        #    (1 << updi.constants.UPDI_CTRLB_UPDIDIS_BIT)
        # )

    def reg_write(self, address, length, value):
        if length == 1:
            self.link.st(address, value)
        elif length == 2:
            self.link.st(address, value & 0xFF)
            self.link.st(address + 1, (value >> 8) & 0xFF)
        else:
            raise Exception("value length is not supported")

    def reg_read(self, address, length):
        if length == 1:
            return self.link.ld(address)
        elif length == 2:
            result = self.link.ld(address)
            result |= self.link.ld(address + 1) << 8

            return result
        else:
            raise Exception("value length is not supported: %u" % length)


class ATtiny3217_PORT():
    def __init__(self, mcu, baseaddress):
        self.mcu = mcu
        self.baseaddress = baseaddress

    DIR = regprop(0x00, 1, "Data Direction")
    DIRSET = regprop(0x01, 1, "Data Direction Set")
    DIRCLR = regprop(0x02, 1, "Data Direction Clear")
    DIRTGL = regprop(0x03, 1, "Data Direction Toggle")
    OUT = regprop(0x04, 1, "Output Value")
    OUTSET = regprop(0x05, 1, "Output Value Set")
    OUTCLR = regprop(0x06, 1, "Output Value Clear")
    OUTTGL = regprop(0x07, 1, "Output Value Toggle")
    IN = regprop(0x08, 1, "Input Value")
    INTFLAGS = regprop(0x09, 1, "Interrupt Flags")
    PIN0CTRL = regprop(0x10, 1, "Pin 0 Control")
    PIN1CTRL = regprop(0x11, 1, "Pin 1 Control")
    PIN2CTRL = regprop(0x12, 1, "Pin 2 Control")
    PIN3CTRL = regprop(0x13, 1, "Pin 3 Control")
    PIN4CTRL = regprop(0x14, 1, "Pin 4 Control")
    PIN5CTRL = regprop(0x15, 1, "Pin 5 Control")
    PIN6CTRL = regprop(0x16, 1, "Pin 6 Control")
    PIN7CTRL = regprop(0x17, 1, "Pin 7 Control")


class ATtiny3217(ATtiny3217Debug):
    def __init__(self, link):
        super().__init__(link)
        self.PORTA = ATtiny3217_PORT(self, 0x0400)
        self.PORTB = ATtiny3217_PORT(self, 0x0420)
        self.PORTC = ATtiny3217_PORT(self, 0x0440)


PIN1_bm = 0x01
PIN2_bm = 0x02
PIN3_bm = 0x08
PIN4_bm = 0x10
PIN5_bm = 0x20
PIN6_bm = 0x40
PIN7_bm = 0x80


"""
MAIN CODE
"""

try:
    link = link.UpdiDatalink("/dev/cu.usbserial-1430", 115200)
except Exception as ex:
    print("something went wrong:", ex)
    exit()

mcu = ATtiny3217(link)
mcu.PORTB.OUTSET = PIN5_bm
print(hex(int(mcu.PORTB.IN)))
time.sleep(1)
mcu.PORTB.OUTCLR = PIN5_bm

"""
link.st(0x0425, 0x20)  # set PORTB.PB5
time.sleep(1)
link.st(0x0426, 0x20)  # clr PORTB.PB5

#  reset device
link.stcs(constants.UPDI_ASI_RESET_REQ, constants.UPDI_RESET_REQ_VALUE)  # apply updi reset condition
link.stcs(constants.UPDI_ASI_RESET_REQ, 0x00)  # release updi reset condition
"""
