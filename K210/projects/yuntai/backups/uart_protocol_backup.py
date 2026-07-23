# -*- coding: utf-8 -*-

from machine import UART
from fpioa_manager import fm

fm.register(6, fm.fpioa.UART1_TX, force=True)

uart = UART(
    UART.UART1, 115200, 8, 0, 1,
    timeout=100, read_buf_len=64
)

frame = bytearray(7)
frame[0] = 0xAA
frame[1] = 0x55
frame[2] = 0x03


def send_laser(x, y, valid):
    if valid:
        frame[3] = (x >> 8) & 0xFF
        frame[4] = x & 0xFF
        frame[5] = y & 0xFF
    else:
        frame[3] = 0
        frame[4] = 0
        frame[5] = 0xFF

    checksum = 0
    for i in range(6):
        checksum = (checksum + frame[i]) & 0xFF

    frame[6] = checksum
    uart.write(frame)


print("UART module ready")
