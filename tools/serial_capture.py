#!/usr/bin/env python3
"""Passive UART logger for the fan's ESP32-C3. Read-only: never writes to the port.

Usage: serial_capture.py PORT BAUD SECONDS OUTFILE
Holds DTR/RTS deasserted so the CH340 can't toggle the C3's EN/BOOT.
"""
import sys
import time

import serial

port, baud, secs, out = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]

ser = serial.Serial()
ser.port = port
ser.baudrate = baud
ser.bytesize, ser.parity, ser.stopbits = 8, "N", 1
ser.timeout = 0.2
ser.dtr = False
ser.rts = False
ser.open()

end = time.time() + secs
with open(out, "ab") as f:
    f.write(f"\n===== capture {time.strftime('%Y-%m-%d %H:%M:%S')} @ {baud} baud, {secs:.0f}s =====\n".encode())
    while time.time() < end:
        data = ser.read(4096)
        if data:
            f.write(data)
            f.flush()
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()
ser.close()
