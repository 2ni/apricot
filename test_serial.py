import serial
import sys
import time

"""
client = serial.Serial()
client.baudrate = 8600
client.port = "/dev/cu.usbserial-1410"
client.rts = 0
client.dtr = 1
print(client)
client.open()
"""

client = serial.serial_for_url("/dev/cu.usbserial-1410", 115200, rtscts=False, dsrdtr=False, do_not_open=True)
client.rts = 0  # must be set to 0 or dtr won't be 0v if 1
client.dtr = 1  # 1=0v (updi) 0=3.3v (uart)
client.open()


try:
    while True:
        print(client.dtr)
        time.sleep(1)
        client.dtr ^= 1
except KeyboardInterrupt:
    client.close()
    print("closed")
    sys.exit(0)
