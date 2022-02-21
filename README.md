## DESCRIPTION
This is a developer board for the chip ATtiny3217 from microchip. It's buils so it can be placed on a breadboard and extended with whatever you wish. The board can be extended with a RFM69 transmitter to send some data via radio. If wished a RFM95 can be soldered to act as a lorawan node.

The code also support ATtiny1604 (beta).

It's the successor of the [forgetmenot board](https://github.com/2ni/forgetmenot). [Schematics and pcb](https://easyeda.com/vkbs/apricot) are also available.

There are many examples in the [examples](/examples) section such as:
- [lorawan node](/examples/lorawan) for the [thethingsnetwork](https://www.thethingsnetwork.org/) (code probably needs to be updated for V3 support)
- [rfm69 node/gateway network](/examples/rfm69), see readme in the folder fore more information
- [ssd1306 display](/examples/ssd1306) example
- [some sensor](/examples/sensors) handling, such as ISL29035 (light), SHT20 (temperature and humidity), LPS22HBTR (pressure)
- [touch library](/examples/touch) to work with touch buttons
- [sleep](/examples/sleep) examples
- complete [node](/examples/humidity-node) with humidity, temperature sensor shown on a display and sent to a RFM69 gw
- [infrared transmitter](examples/infrared_v2) which can receive and transmit NEC signals (remote control) in parallel
- more examples and libraries are planned (eg infrared remote control, mppt solar charger, dcc train switches, ...)


Main features:
- only 1 usb needed for programming and debug output on uart
- onboard 1 led and sensors (light sensor ISL29035, temperature&humidity sensor SHT20, pressure sensor LPS22HBTR)
- dual power input handling
- 32.768kHz quartz for precise sleep times (TOSC1, TOSC2)
- esd protection
- pinout supporting i2c, spi and general purpose pins
- extended functions to print information via serial terminal
- serial terminal is included, just run make serial to start it after having compiled an example
- makefile to simplify handling
- in the [common](/common) directory all base libraries such as twi, spi, touch, sensors are available
- functions to simplify handling with pins, ie also adc, interrupts etc
- a common timer counter is available if needed which is also updated in sleep mode, see [clock h](/common/clock.h)
- for lorawan connect the following pins from the rfm95 to the main board (D0, D1, D2 is not needed):
```
MI  - PC1
MO  - PC2
SCK - PC0
CS  - PC3
GND - GND
3.3 - 3.3
```

### COMMANDS
```
./activate example/blink  // you 1st need so choose which project you'd like to use
make flash                // compile, upload and starts the debugging usart
make serial               // only starts the debugging usart
make reset                // resets the mcu and starts the debugging usart
```

### SETUP

TODO: use [pymcuprog](https://github.com/microchip-pic-avr-tools/pymcuprog) instead of pyupdi

You'll need the toolchain from microchip to compile the sources.

The shared usart for debugging and programming makes use of the DTR line to control which part is connected to the usb. For this matter a special [serial terminal](/serialterminal.py) and programmer is used. As programmer a patched [pyupdi](https://github.com/mraardvark/pyupdi) comes to hand.
- DTR=1 -> updi mode (DTR outputs 0v)
- DTR=0 -> uart debug mode (default, DTR outputs 3.3v)

#### PYTHON
```
pyenv virtualenv 3.9.0 apricot
pyenv local apricot
pip install -r requirements
```

#### MICROCHIP TOOLCHAIN
- download the newest [toolchain](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers) "AVR-8-bit Toolchain 3.6.2 - Mac OS X 64-bit "from microchip (you need an account and to be logged in)
- download the newest [pack](http://packs.download.atmel.com/) from atmel (search for attiny3217 to get the correct pack)
```
mkdir toolchain_microchip
tar xfz avr8-gnu-toolchain-osx-3.6.2.514-darwin.any.x86_64.tar.gz -C toolchain_microchip
cd toolchain_microchip; mv avr8-gnu-toolchain-darwin_x86_64/* .; rm -rf avr8-gnu-toolchain-darwin_x86_64/
cd ..
mkdir toolchain_microchip/pack
unzip Atmel.ATtiny_DFP.1.8.332.atpack -d toolchain_microchip/pack/
```

don't forget to include <avr/io.h> in your main code!

#### PYUPDI
```
git clone git@github.com:mraardvark/pyupdi.git
make patchpyupdi
pip install -e pyupdi # IMPORTANT! install pyupdi to have correct pathes to use it
```

This should result in the following change:
```
$ cd pydupi
$ git diff .
diff --git a/updi/physical.py b/updi/physical.py
index 96be64b..f1afb26 100644
--- a/updi/physical.py
+++ b/updi/physical.py
@@ -31,7 +31,15 @@ class UpdiPhysical(object):
             Standard COM port initialisation
         """
         self.logger.info("Opening {} at {} baud".format(port, baud))
-        self.ser = serial.Serial(port, baud, parity=serial.PARITY_EVEN, timeout=1, stopbits=serial.STOPBITS_TWO)
+        self.ser = serial.serial_for_url(port, baud, parity=serial.PARITY_EVEN, timeout=1, stopbits=serial.STOPBITS_TWO, rtscts=False, dsrdtr=False, do_not_open=True)
+        self.ser.rts = 0  # needed so dtr really gets 0v
+        self.ser.dtr = 1
+
+        self.ser.open()
+        # dtr is only set when port is opened, and stable low after ~3ms.
+        # during that time some crap from uart can come in, which disturbs the updi communication
+        time.sleep(.01)
+        self.ser.flushInput()

     def _loginfo(self, msg, data):
         if data and isinstance(data[0], str):
@@ -56,7 +64,12 @@ class UpdiPhysical(object):
         # Which is slightly above the recommended 24.6ms
         self.ser.close()

-        temporary_serial = serial.Serial(self.port, 300, stopbits=serial.STOPBITS_ONE, timeout=1)
+        temporary_serial = serial.serial_for_url(self.port, 300, stopbits=serial.STOPBITS_ONE, timeout=1, rtscts=False, dsrdtr=False, do_not_open=True)
+        temporary_serial.rts = 0  # needed so dtr really gets 0v
+        temporary_serial.dtr = 1
+        temporary_serial.open()
+        time.sleep(.01)
+        self.ser.flushInput()
```
