### DESCRIPTION
This is a lorawan breakout board based on the attiny3217. It's the successor of the [forgetmenot board](https://github.com/2ni/forgetmenot). [Schematics and pcb](https://easyeda.com/vkbs/apricot) are also available.

Main features:
- only 1 usb needed for programming and debug output on uart
- uses an RFM95 for lorawan
- onboard 1 led and sensors (light sensor ISL29035, temperature&humidity sensor SHT20, pressure sensor LPS22HBTR)
- dual power input handling
- 32.768kHz quartz for precise sleep times (TOSC1, TOSC2)
- esd protection
- pinout supporting i2c, spi and general purpose pins
- many [examples](/examples)


### COMMANDS
```
./activate example/blink  // you 1st need so choose which project you'd like to use
make flash                // compile, upload and starts the debugging usart
make serial               // only starts the debugging usart
make reset                // resets the mcu and starts the debugging usart
```

### SETUP
You'll need the toolchain from microchip to compile the sources.

The shared usart for debugging and programming makes use of the DTR line to control which part is connected to the usb. For this matter a special [serial terminal](/serialterminal.py) and programmer is used. As programmer a patched [pyupdi](https://github.com/mraardvark/pyupdi) comes to hand.
- DTR=1 -> updi mode (DTR outputs 0v)
- DTR=0 -> uart debug mode (default, DTR outputs 3.3v)

#### MICROCHIP TOOLCHAIN
- download the newest [toolchain](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers) from microchip (you need an account and to be logged in)
- download the newest [pack](http://packs.download.atmel.com/) from atmel (search for attiny3217 to get the correct pack)
```
mkdir toolchain_microchip
tar xfz avr8-gnu-toolchain-osx-3.6.2.514-darwin.any.x86_64.tar.gz -C toolchain_microchip
mkdir toolchain_microchip/pack
unzip Atmel.ATtiny_DFP.1.8.332.atpack -d toolchain_microchip/pack/
```

don't forget to include <avr/io.h> in your main code!

#### PYUPDI
```
git clone git@github.com:mraardvark/pyupdi.git
make patchpyupdi
pip install -e pyupdi
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
+        self.ser.rts = 0  # needed so dtr reall gets 0v
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
+        temporary_serial.rts = 0  # needed so dtr reall gets 0v
+        temporary_serial.dtr = 1
+        temporary_serial.open()
+        time.sleep(.01)
+        self.ser.flushInput()
```
