### SETUP
```
git clone git@github.com:mraardvark/pyupdi.git
```

#### adapt physical.py
the board controls the serial and updi with the dtr line.
dtr=1 -> updi mode (dtr is 0v)
dtr=0 -> uart debug mode (default, dtr is 3.3v)

```
- self.ser = serial.Serial(port, baud, parity=serial.PARITY_EVEN, timeout=1, stopbits=serial.STOPBITS_TWO)
+ self.ser = serial.serial_for_url(port, baud, parity=serial.PARITY_EVEN, timeout=1, stopbits=serial.STOPBITS_TWO, rtscts=False, dsrdtr=False, do_not_open=True)
+ self.ser.rts = 0  # needed so dtr reall gets 0v
+ self.ser.dtr = 1
+ self.ser.open()
pip install -e pyupdi
```

### TOOLCHAIN
- download the newest [toolchain](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers) from microchip (you need to be logged in)
- download the newest [pack](http://packs.download.atmel.com/) from atmel (search for attiny3217 to get the correct pack)
```
mkdir toolchain_microchip
tar xfz avr8-gnu-toolchain-osx-3.6.2.514-darwin.any.x86_64.tar.gz -C toolchain_microchip
mkdir toolchain_microchip/pack
unzip Atmel.ATtiny_DFP.1.8.332.atpack -d toolchain_microchip/pack/
```

don't forget to include <avr/io.h> in your main code!
