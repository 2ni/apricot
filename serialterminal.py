#!/usr/bin/env python

'''
howto send test string:
import serial
ser = serial.Serial('/dev/cu.[wch]usbserial1410', 19200, timeout=.1)
ser.write("Hello".encode())
'''

import serial
from serial.tools import list_ports
import argparse
from datetime import datetime as dt
import sys
import re
import math
from pynput import keyboard

parser = argparse.ArgumentParser(description='simpler serial port listener', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-p', '--port', type=str, default='', required=True, help='give port to listen from, eg "/dev/cu.[wch]usbserial1410" or "1"')
parser.add_argument('-b', '--baudrate', type=int, default=19200, help='give port speed in baud, eg 19200')
parser.add_argument('-d', '--datestamp', action='store_true', help='show datestamp on output')
parser.add_argument('-a', '--absolute', action='store_true', help='show absolute datestamps (current time)')

args = parser.parse_args()

# if port given, wait for it
# port can be given as /dev/cu.[wch]usbserial1440 or 4
print("waiting for serial...")
waiting_for_port = True
while waiting_for_port:
    for p in list_ports.comports():
        if re.search(r"^\d$", args.port) and re.search(r"usbserial.*?{port}0$".format(port=args.port), p.device):
            args.port = p.device
            waiting_for_port = False
        elif args.port == p.device:
            waiting_for_port = False

print("-- Miniterm on {port} {baud},8,N,1 --".format(port=args.port, baud=args.baudrate))
printTimestamp = True
if not args.datestamp:
    printTimestamp = False

# set dtr to switch between UPDI and UART
# 8bits, parity none, stop bit
# client = serial.Serial(args.port, args.baudrate, timeout=1)
client = serial.serial_for_url(args.port, args.baudrate, rtscts=False, dsrdtr=False, do_not_open=True)
client.rts = 0  # must be set to 0 or dtr won't be 0v if 1
client.dtr = 0  # 1=0v (updi) 0=3.3v (uart)
client.open()
start = dt.now()
last = start
timestampAbsolute = False


# keyboard listener see
# https://stackoverflow.com/questions/11918999/key-listeners-in-python#answer-43106497
def on_press(key):
    try:
        k = key.char  # single-char keys
        # print("single: " + k)
        client.write(k.encode())
    except AttributeError:
        try:
            k = key.name  # other keys
            if k == "enter":
                client.write("\n".encode())
            elif k == "backspace":
                client.write("\r".encode())
        except:
            pass


listener = keyboard.Listener(on_press=on_press)
listener.start()

try:
    while True:
        data = client.read()
        if data:
            if printTimestamp:
                now = dt.now()

                if args.absolute:
                    seconds = int(now.strftime('%S'))
                    micros = int(now.strftime('%f'))
                    precision = round(micros / 1000)
                    if precision == 1000:
                        seconds += 1
                        precision = 0

                    print('{now}:{seconds:02d}.{precision:03d}: '.format(
                        now=now.strftime("%H:%M"),
                        seconds=seconds,
                        precision=precision), end='')
                    # print('{}: '.format(now.strftime("%H:%M:%S.%f")[:-5]), end='')
                else:
                    secondsSinceLast = (now - last).total_seconds()
                    diff = ' +{s:03d}.{ms:04d}'.format(
                        s=int(secondsSinceLast),
                        ms=round((secondsSinceLast - int(secondsSinceLast)) * 10000)
                    )
                    last = now

                    secondsFromStart = (now - start).total_seconds()
                    hours = math.floor(secondsFromStart / 3600)
                    minutes = math.floor((secondsFromStart / 60) % 60)
                    seconds = math.floor(secondsFromStart % 60)
                    milliseconds = round((secondsFromStart - int(secondsFromStart)) * 1000)

                    print('{h:02d}:{m:02d}:{s:02d}.{ms:03d}{diff}: '.format(
                        h=hours,
                        m=minutes,
                        s=seconds,
                        ms=milliseconds,
                        diff=diff), end='')

                printTimestamp = False

            try:
                print(data.decode('utf-8'), end='', flush=True)
            except UnicodeDecodeError:
                pass

            if args.datestamp and ord(data) == 10:
                printTimestamp = True

except KeyboardInterrupt:
    pass
except OSError:
    print("no port found")
    sys.exit(0)
