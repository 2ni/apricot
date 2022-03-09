from enum import Enum


class UpdiState(Enum):
    Idle = 0
    Synced = 1
    Data = 2


class UpdiOpcode(Enum):
    LDS = 0
    STS = 2
    LD = 1
    ST = 3
    LDCS = 4
    STCS = 6
    REPEAT = 5
    KEY = 7


class UpdiFrame():
    def __init__(self, sfr, time, opcode=None):
        self.sfr = sfr
        self.starttime = time
        self.endtime = time
        self.opcode = opcode
        self.errors = []

    def error(self, msg):
        if self.sfr:
            msg += " @ line %u" % self.sfr.lineno
        self.errors.append(msg)

    def readbyte(self, expected=None):
        entry = self.sfr.readline()
        if entry is None:
            msg = "unexpected EOF"
            if expected:
                msg += " " + expected
            self.error(msg)
            return None
        self.endtime = entry[0]
        return entry[1]


class UpdiFrameError(UpdiFrame):
    def __init__(self, sfr, time, data):
        super().__init__(sfr, time)

        self.data = [data]

    def push_byte(self, time, data):
        self.endtime = time
        self.data.append(data)

    def finish(self, time):
        self.endtime = time


class UpdiFrameLDS(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        sizea = (self.opcode >> 2) & 0x03
        sizeb = (self.opcode >> 0) & 0x03

        self.addrsize = 0
        self.datasize = 0
        self.addr = 0
        self.data = 0

        if sizea <= 1:
            self.addrsize = sizea + 1
        else:
            self.error("invalid value for Size A: %u" % sizea)
            return False

        if sizeb <= 1:
            self.datasize = sizeb + 1
        else:
            self.error("invalid value for Size B: %u" % sizeb)
            return False

        for i in range(0, self.addrsize):
            b = self.readbyte("Addr")
            if b is not None:
                self.addr |= b << (i * 8)

        for i in range(0, self.datasize):
            b = self.readbyte("Data")
            if b is not None:
                self.data |= b << (i * 8)

        return True

    def report(self):
        msg = "LDS from "
        if self.addrsize == 1:
            msg += "0x%02X" % self.addr
        elif self.addrsize == 2:
            msg += "0x%04X" % self.addr
        else:
            msg += "(invalid)"
        msg += ": "

        if self.datasize == 1:
            msg += "0x%02X" % self.data
        elif self.datasize == 2:
            msg += "0x%04X" % self.data
        else:
            msg += "(invalid)"

        return msg


class UpdiFrameSTS(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        sizea = (self.opcode >> 2) & 0x03
        sizeb = (self.opcode >> 0) & 0x03
        self.addrsize = 0
        self.datasize = 0
        self.addr = 0
        self.data = 0

        if sizea <= 1:
            self.addrsize = sizea + 1
        else:
            self.error("invalid value for Size A: %u" % sizea)
            return False

        if sizeb <= 1:
            self.datasize = sizeb + 1
        else:
            self.error("invalid value for Size B: %u" % sizeb)
            return False

        for i in range(0, self.addrsize):
            b = self.readbyte("Addr")
            if b is not None:
                self.addr |= b << (i * 8)

        self.ack1 = False
        b = self.readbyte("ACK1")
        if b is not None:
            if b == 0x40:
                self.ack1 = True
            else:
                return False
        else:
            return False

        for i in range(0, self.datasize):
            b = self.readbyte("Data")
            if b is not None:
                self.data |= b << (i * 8)

        self.ack2 = False
        b = self.readbyte("ACK")
        if b is not None:
            if b == 0x40:
                self.ack2 = True
            else:
                return False
        else:
            return False

        return True

    def report(self):
        msg = "STS: "
        if self.addrsize == 1:
            msg += "0x%02X" % self.addr
        elif self.addrsize == 2:
            msg += "0x%04X" % self.addr
        else:
            msg += "(invalid)"
        msg += ": "

        if self.datasize == 1:
            msg += "0x%02X" % self.data
        elif self.datasize == 2:
            msg += "0x%04X" % self.data
        else:
            msg += "(invalid)"

        return msg


class UpdiFrameLD(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        self.ptraccess = ["(*ptr)", "(*ptr++)", "ptr", "(invalid)"][(self.opcode >> 2) & 0x03]
        self.datasize = 0
        self.data = 0

        tmp = self.opcode & 0x03
        if tmp < 2:
            self.datasize = tmp + 1
        else:
            self.error("invalid value for Size: %u" % tmp)
            return False

        for i in range(0, self.datasize):
            b = self.readbyte("Data")
            if b is not None:
                self.data |= b << (i * 8)

    def report(self):
        msg = "LD %s: " % self.ptraccess
        if self.datasize == 1:
            msg += "0x%02X" % self.data
        elif self.datasize == 2:
            msg += "0x%04X" % self.data
        else:
            msg += "(invalid)"
        return msg


class UpdiFrameST(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        self.ptraccess = ["(*ptr)", "(*ptr++)", "ptr", "(invalid)"][(self.opcode >> 2) & 0x03]
        self.datasize = 0
        self.data = 0
        self.ack = False

        tmp = self.opcode & 0x03
        if tmp < 2:
            self.datasize = tmp + 1
        else:
            self.error("invalid value for Size: %u" % tmp)
            return False

        for i in range(0, self.datasize):
            b = self.readbyte("Data")
            if b is not None:
                self.data |= b << (i * 8)

        b = self.readbyte("ACK")
        if b is not None:
            if b == 0x40:
                self.ack = True

    def report(self):
        msg = "ST %s: " % self.ptraccess
        if self.datasize == 1:
            msg += "0x%02X" % self.data
        elif self.datasize == 2:
            msg += "0x%04X" % self.data
        else:
            msg += "(invalid)"

        if self.ack:
            msg += " ACK"
        else:
            msg += "NACK"
        return msg


class UpdiFrameLDCS(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        self.csaddr = self.opcode & 0x0F

        self.data = 0
        b = self.readbyte("Data")
        if b is not None:
            self.data = b

    def report(self):
        msg = "LDCS: @%u=" % self.csaddr
        if self.data is not None:
            msg += "0x%02X" % self.data
        else:
            msg += "(invalid)"

        return msg


class UpdiFrameSTCS(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        self.csaddr = self.opcode & 0x0F

        self.data = None
        b = self.readbyte("Data")
        if b is not None:
            self.data = b

    def report(self):
        msg = "STCS: @%u=" % self.csaddr
        if self.data:
            msg += "0x%02X" % self.data
        else:
            msg += "(invalid)"

        return msg


class UpdiFrameREPEAT(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        tmp = self.opcode & 0x03
        self.repeatsize = 0
        if tmp != 0:
            self.error("data size is != 0: %u" % tmp)
            return False
        else:
            self.repeatsize = tmp + 1

        self.repeats = 0
        if tmp is not None:
            for i in range(0, self.repeatsize):
                b = self.readbyte("RepData")
                if b is not None:
                    self.repeats |= b << (i * 8)
            self.repeats += 1

    def report(self):
        msg = "REPEAT %u" % self.repeats
        return msg


class UpdiFrameKEY(UpdiFrame):
    def __init__(self, sfr, time, opcode):
        super().__init__(sfr, time, opcode)

    def run(self):
        self.sib = (self.opcode >> 2) & 1
        self.sizec = self.opcode & 0x03

        self.key = []

        self.keylen = None

        if self.sizec == 0:
            self.keylen = 8
        elif self.sizec == 1:
            self.keylen = 16
        elif self.sizec == 2:
            self.keylen = 32  # guesstimated from RE
        else:
            self.error("invalid key length: %u" % self.sizec)

        if self.keylen is not None:
            for i in range(0, self.keylen):
                b = self.readbyte("Key")
                if b is not None:
                    self.key.append(b)

        return True

    def report(self):
        msg = "KEY "
        if self.sib == 1:
            msg += "Receive"
        else:
            msg += "Send"

        msg += " "
        if self.keylen is None:
            msg += "(invalid key size: %u)" % self.sizec
        else:
            msg += "%u bytes: %s" % (
                self.keylen,
                " ".join(["0x%02X" % x for x in self.key])
            )

        return msg


UpdiFrameTypes = [
    UpdiFrameLDS,
    UpdiFrameLD,
    UpdiFrameSTS,
    UpdiFrameST,
    UpdiFrameLDCS,
    UpdiFrameREPEAT,
    UpdiFrameSTCS,
    UpdiFrameKEY,
]


class UpdiReader():
    def __init__(self, sfr):
        self.sfr = sfr
        self.currframe = None
        self.frames = []
        self.errors = []

    def finish(self, time):
        pass

    def error_add(self, msg):
        self.errors.append(msg)

    def run(self):
        errfr = None
        while True:
            entry = self.sfr.readline()
            if entry is None:
                break

            time = entry[0]
            data = entry[1]

            if data == 0x55:
                if errfr:
                    print("%0.8f\t%0.8f\tErrors: %s" % (errfr.starttime, errfr.endtime, " ".join(["0x%02X" % x for x in errfr.data])))
                    self.frames.append(errfr)
                    errfr = None

                # we got a frame, get the opcode
                entry = self.sfr.readline()
                if entry is None:
                    self.error_add("Unexpected end at line %u" % sfr.lineno)
                    return

                time = entry[0]
                data = entry[1]
                opcode = data >> 5

                fr = UpdiFrameTypes[opcode](sfr, time, data)
                fr.run()
                print("%0.8f\t%0.8f\t%s" % (fr.starttime, fr.endtime, fr.report()))
                self.frames.append(fr)

            else:
                if not errfr:
                    errfr = UpdiFrameError(sfr, time, data)
                else:
                    errfr.push_byte(time, data)

        print("done.")
        return self.frames

    def frame_finish(self, time):
        if self.currframe:
            self.currframe.finish(time)
            self.frames.append(self.currframe)
            self.currframe = None

    def push_errframe(self, time, data):
        if self.currframe:
            if self.currframe is UpdiFrameError:
                self.currframe.push_byte(time, data)
            else:
                self.frame_finish(time)
                self.currframe = UpdiFrameError(self.sfr, time, data)


class SaleaeFileReader():

    def __init__(self, fhandle):
        self.fhandle = fhandle
        self.lineno = 0

    def readline(self):

        if self.lineno == 0:
            # skip header
            self.fhandle.readline()
            self.lineno += 1

        line = self.fhandle.readline()
        if line == '':
            return None  # EOF

        self.lineno += 1

        time, data, _, _ = line.split(",")
        time = float(time)
        data = int(data, 16)
        return time, data


# cat stop-orig.csv | grep -v Break | awk -F',' 'BEGIN { OFS="," } {print $3, $5, $1, $2}'> stop.txt
infile = "stop2.txt"
ifh = open(infile, "r")

sfr = SaleaeFileReader(ifh)

ur = UpdiReader(sfr)
ur.run()
