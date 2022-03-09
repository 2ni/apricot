"""
https://aykevl.nl/2020/06/simavr-debug
https://github.com/stemnic/pyAVRdbg
https://github.com/microchip-pic-avr-tools/pyedbglib
https://github.com/microchip-pic-avr-tools/pymcuprog
https://github.com/chris-heo/updizombie/issues/1

https://sourceware.org/gdb/current/onlinedocs/gdb/Remote-Protocol.html
toolchain_microchip/bin/avr-gdb main.elf

required cmds: g, G, m, M, c and s
(gdb) target remote :1234
(gdb) Z type,addr,kind //  set breakpoint of type at addr of kind (type: 0=sw, 1=hw)
(gdb) z type,addr,kind //  clear breakpoint of type at addr of kind
(gdb) b main           // breakpoint at beginning of prg
(gdb) b                // breakpoint at current line
(gdb) b N              // breakpoint at line N
(gdb) b +N             // breakpoint at current line + N lines
(gdb) info break       // list breakpoints
(gdb) r                // run until breakpoint or error
(gdb) c                // continue until breakpoint or error
(gdb) s                // run next step
(gdb) s N              // run N next steps
(gdb) p var            // print current value of var
(gdb) u                // up a level in the stack
(gdb) d                // down a level in the stack
(gdb) q                // quit
"""

import socket
import select
from updi.nvm import UpdiNvmProgrammer

HOST = "127.0.0.1"
PORT = 12555

SIGVAL_last = "S00"

nvm = UpdiNvmProgrammer(comport="/dev/cu.usbserial-1430", baud=115200, device="attiny3217")


def send_packet(socket, data):
    checksum = sum(data.encode("ascii")) % 256
    message = "$" + data + "#" + format(checksum, "02x")
    if data == "":
        message = "$#00"

    print("<- " + message)
    socket.sendall(message.encode("ascii"))


def handle_command(socket, command):
    match command[0]:
        case "H":  # ignore Hg0, HC-1
            send_packet(socket, "")
        case "q":
            if len(command) > 1:
                query = command[1:]
                if query == "Attached":
                    send_packet(socket, "0")
                    return
                elif "Supported" in query:
                    # Since we are using a tcp connection we do not want to split up messages
                    # into different packets, so packetsize is set absurdly large
                    send_packet(socket, "PacketSize=10000000000")
                    return
                elif "Symbol::" in query:
                    send_packet(socket, "OK")
                    return
                elif "C" == query[0]:
                    send_packet(socket, "")
                    return
                elif "Offsets" in query:
                    send_packet(socket, "Text=000;Data=000;Bss=000")
                    return

            send_packet(socket, "")
        case "?":
            send_packet(socket, SIGVAL_last)
        case "g":
            packets = []
            packets += list(nvm.application.read_data(0x0000, 32))  # read 32 general purpose registers R0 - R31, see https://en.wikipedia.org/wiki/Atmel_AVR_instruction_set

            packets += [nvm.application.datalink.ld(0x003F)]  # status register, see CPU_SREG iotn3217.h
            packets += [nvm.application.datalink.ld(0x003D), nvm.application.datalink.ldcs(0x003E)]  # stack pointer LSB-MSB, see CPU_SPL/CPU_SPH iotn3217.h

            packets = "".join([format(x, "02x") for x in packets])

            send_packet(socket, packets)
        case "p":
            if len(command) > 1:
                match command[1:]:
                    case "22":
                        # gdb defines pc register for avr to be REG34(0x22)
                        # of interest: AVR8_MEMTYPE_OCD_PC = 0x14 (from pyedgelib/avr8protocol.py)
                        pass


"""
stack_pointer_read()
AVR8_MEMTYPE_OCD = 0xD1
memory_read(AVR8_MEMTYPE_OCD, 0x18, 0x02)
check_response(jtagice3_command_response(bytearray([CMD_AVR8_MEMORY_READ, CMD_VERSION0, AVR8_MEMTYPE_OCD]) + binary.pack_le32(0x18) + binary.pack_le32(0x02)))
check_response(jtagice3_command_response(bytearray([CMD_AVR8_MEMORY_READ, CMD_VERSION0, AVR8_MEMTYPE_OCD]) + binary.pack_le32(0x18) + binary.pack_le32(0x02)))
header = bytearray([self.JTAGICE3_TOKEN, self.JTAGICE3_PROTOCOL_VERSION, self.sequence_id & 0xFF, (self.sequence_id >> 8) & 0xFF, self.handler])
# Send command, receive response
packet = header + bytearray(command)
response = self.avr_command_response(packet)


readRegs()
AVR8_MEMTYPE_REGFILE = 0xB8
CMD_AVR8_MEMORY_READ = 0x21
CMD_VERSION0 = 0
data = memory_read(AVR8_MEMTYPE_REGFILE, 0, 32)
memory_read(memtype, address, num_bytes):
    check_response(self.jtagice3_command_response(bytearray([self.CMD_AVR8_MEMORY_READ, self.CMD_VERSION0, memtype]) + binary.pack_le32(address) + binary.pack_le32(num_bytes)))

readSREG()
memory_read(AVR8_MEMTYPE_OCD, 0x1C, 0x01)

"""


def handle_data(socket, data):
    if data.decode("ascii").count("$") > 0:
        for n in range(data.decode("ascii").count("$")):
            valid_data = True
            data = data.decode("ascii")
            checksum = (data.split("#")[1])[:2]
            packet_data = (data.split("$")[1]).split("#")[0]
            if int(checksum, 16) != sum(packet_data.encode("ascii")) % 256:
                print("wrong checksum")
                valid_data = False

            if valid_data:
                socket.sendall(b"+")
                print("<- +")
            else:
                socket.sendall(b"-")
                print("<- -")

            print("packet_data", packet_data)
            handle_command(socket, packet_data)

    elif data == b"\x03":
        #  stop()
        socket.sendall(b"+")
        print("<- +")


try:
    print("Waiting for gdb client @ {host}:{port}. Call > target remote :{port}".format(host=HOST, port=PORT))
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        conn.setblocking(0)
        with conn:
            print("{client} connected...".format(client=addr))
            while True:
                ready = select.select([conn], [], [], .5)
                if ready[0]:
                    data = conn.recv(1024)
                    if len(data) > 0:
                        print("-> {}".format(data.decode("ascii")))
                        handle_data(conn, data)
except KeyboardInterrupt:
    print("Done. Bye.")
