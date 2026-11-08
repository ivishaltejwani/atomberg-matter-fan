#!/usr/bin/env python3
"""Send commands to fan_lab's serial terminal and print what comes back.

Usage: fanterm.py [--listen SECS] [CMD ...]
Keeps RTS/DTR off so BOOT stays high.
"""
import sys
import time

import serial

PORT = "/dev/cu.wchusbserial2140"

args = sys.argv[1:]
secs = 2.0
if args[:1] == ["--listen"]:
    secs = float(args[1])
    args = args[2:]

s = serial.Serial()
s.port, s.baudrate, s.timeout = PORT, 115200, 0.2
s.dtr = s.rts = False
s.open()
time.sleep(0.2)
s.reset_input_buffer()
for cmd in args:
    s.write(cmd.encode() + b"\n")
    s.flush()
    time.sleep(0.3)
end = time.time() + secs
while time.time() < end:
    d = s.read(4096)
    if d:
        sys.stdout.write(d.decode(errors="replace"))
        sys.stdout.flush()
s.close()
