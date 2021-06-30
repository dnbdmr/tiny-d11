import ctypes
c_uint32 = ctypes.c_uint32

import time
import serial
import sys

class Flags_bits(ctypes.LittleEndianStructure):
    _fields_ = [
            ("second", c_uint32, 6),
            ("minute", c_uint32, 6),
            ("hour", c_uint32, 5),
            ("day", c_uint32, 5),
            ("month", c_uint32, 4),
            ("year", c_uint32, 6),
        ]

class time_flags(ctypes.Union):
    _fields_ = [("bit", Flags_bits),
                ("reg", c_uint32)]

    def from_local(self, cur_time):
        self.bit.second = cur_time.tm_sec
        self.bit.minute = cur_time.tm_min
        self.bit.hour = cur_time.tm_hour
        self.bit.day = cur_time.tm_mday
        self.bit.month = cur_time.tm_mon
        self.bit.year = cur_time.tm_year - 2000

    def to_local(self):
        self_time = (
                self.bit.year + 2000,
                self.bit.month,
                self.bit.day,
                self.bit.hour,
                self.bit.minute,
                self.bit.second,
                time.localtime().tm_wday,
                time.localtime().tm_yday,
                time.localtime().tm_isdst )
        return time.struct_time(self_time)

if __name__ == "__main__":
    cur_time = time_flags()
    cur_time.from_local(time.localtime())

    board_time = time_flags()

    ser = serial.Serial("/dev/serial/by-id/usb-DNBDMR_TinyUSB_D11D_CEF3EF1D514D4B53334A2020FF071B46-if00", baudrate=115200, timeout=1)

    if (sys.argv[1] == '-s'):
        ser.write(('S ' + str(cur_time.reg) + '\n').encode('ascii'))
    elif (sys.argv[1] == '-c'):
        ser.write('T\n'.encode('ascii'))
        ser.readline()
        board_time.reg = int(ser.readline())
        print('Local time: ' + time.asctime(cur_time.to_local()))
        print('Board time: ' + time.asctime(board_time.to_local()))
        diff = time.mktime(cur_time.to_local()) - time.mktime(board_time.to_local())
        print('Local - Board: ' + str(diff))

    ser.close()
