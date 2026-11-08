#!/usr/bin/env python3
"""Map the fan C3's GPIOs from the ROM download mode via register reads/writes.

Nothing is written to flash. Put the C3 in download mode first: BOOT tied to
GND, then tap EN.

  pinprobe.py scan                 pull-down/pull-up read of every pin
  pinprobe.py walk [secs]          each pin HIGH secs, LOW secs, then released
  pinprobe.py blink PIN [secs]     blink one pin at 1 Hz
  pinprobe.py hold PIN LEVEL secs  drive one pin to 0/1 for secs
  pinprobe.py watch [secs]         count input edges (press remote buttons)
  pinprobe.py charlie SECS PIN...  rapidly cycle all HIGH/LOW pairs (charlieplex test)
  pinprobe.py pairs SECS PIN...    hold each HIGH/LOW pair for SECS, one at a time
"""
import sys
import time

from esptool.cmds import detect_chip

PORT = "/dev/cu.wchusbserial2140"
# GPIO9 is held low by BOOT, 20/21 are UART0, 11-17 are flash.
PINS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 18, 19]

GPIO = 0x60004000
OUT_W1TS, OUT_W1TC = GPIO + 0x08, GPIO + 0x0C
EN_W1TS, EN_W1TC = GPIO + 0x24, GPIO + 0x28
IN_REG = GPIO + 0x3C
OUT_SEL = lambda p: GPIO + 0x554 + 4 * p  # noqa: E731
IO_MUX = lambda p: 0x60009004 + 4 * p  # noqa: E731
USB_CONF0 = 0x60043018

FUN_PD, FUN_PU, FUN_IE = 1 << 7, 1 << 8, 1 << 9
MCU_SEL_GPIO = 1 << 12  # function 1 = GPIO, FUN_DRV = 0 (~5 mA)
SIG_GPIO_OUT = 128


def connect():
    # Auto-reset doesn't reliably reach the fan: BOOT is tied to GND and EN tapped by hand.
    esp = detect_chip(PORT, 115200, "no-reset")
    # Release GPIO18/19 from the USB-Serial-JTAG PHY (pad enable + D+ pull-up).
    esp.write_reg(USB_CONF0, 0, mask=(1 << 14) | (1 << 9))
    return esp


def as_input(esp, p, pulls=0):
    esp.write_reg(EN_W1TC, 1 << p)
    esp.write_reg(IO_MUX(p), MCU_SEL_GPIO | FUN_IE | pulls)


def as_output(esp, p, level):
    esp.write_reg(OUT_W1TS if level else OUT_W1TC, 1 << p)
    esp.write_reg(OUT_SEL(p), SIG_GPIO_OUT)
    esp.write_reg(IO_MUX(p), MCU_SEL_GPIO | FUN_IE)
    esp.write_reg(EN_W1TS, 1 << p)


def read(esp, p):
    return (esp.read_reg(IN_REG) >> p) & 1


def scan(esp):
    print("pin   pd pu  class")
    classes = {}
    for p in PINS:
        as_input(esp, p, FUN_PD); time.sleep(0.01); dn = read(esp, p)
        as_input(esp, p, FUN_PU); time.sleep(0.01); up = read(esp, p)
        as_input(esp, p)
        c = "HIGH" if dn and up else "LOW" if not dn and not up else "float"
        classes[p] = c
        note = {"HIGH": "held high: pull-up / IR out / I2C",
                "LOW": "held low: pull-down / active-high LED / driven low",
                "float": "follows pull: unconnected / input / active-low LED"}[c]
        print(f"GPIO{p:<3} {dn}  {up}   {c:5} {note}")
    return classes


def walk(esp, secs):
    print(f"order: {', '.join(f'GPIO{p}' for p in PINS)}")
    for i, p in enumerate(PINS, 1):
        print(f"step {i:2}: GPIO{p} HIGH", flush=True); as_output(esp, p, 1); time.sleep(secs)
        print(f"step {i:2}: GPIO{p} LOW", flush=True);  as_output(esp, p, 0); time.sleep(secs)
        as_input(esp, p)
        time.sleep(1)


def blink(esp, p, secs):
    end = time.time() + secs
    level = 1
    while time.time() < end:
        as_output(esp, p, level); level ^= 1; time.sleep(0.5)
    as_input(esp, p)


def pair(esp, hi, lo, pins):
    for p in pins:
        if p not in (hi, lo):
            as_input(esp, p)
    as_output(esp, hi, 1)
    as_output(esp, lo, 0)


def charlie(esp, pins, secs):
    """Cycle every ordered (HIGH, LOW) pair among pins; others left floating."""
    pairs = [(a, b) for a in pins for b in pins if a != b]
    end = time.time() + secs
    while time.time() < end:
        for a, b in pairs:
            pair(esp, a, b, pins)
    for p in pins:
        as_input(esp, p)


def charlie_steps(esp, pins, secs):
    """Hold each ordered pair for secs so a single LED can be identified."""
    pairs = [(a, b) for a in pins for b in pins if a != b]
    for i, (a, b) in enumerate(pairs, 1):
        print(f"step {i:2}: GPIO{a} HIGH, GPIO{b} LOW", flush=True)
        pair(esp, a, b, pins)
        time.sleep(secs)
        as_input(esp, a); as_input(esp, b)
        time.sleep(0.7)


def watch(esp, secs):
    for p in PINS:
        as_input(esp, p)
    mask = sum(1 << p for p in PINS)
    edges = {p: 0 for p in PINS}
    samples = 0
    last = esp.read_reg(IN_REG) & mask
    end = time.time() + secs
    print(f"watching {secs}s, press remote buttons now", flush=True)
    while time.time() < end:
        v = esp.read_reg(IN_REG) & mask
        samples += 1
        for p in PINS:
            if (v ^ last) >> p & 1:
                edges[p] += 1
        last = v
    print(f"{samples} samples")
    for p in PINS:
        print(f"GPIO{p:<3} edges={edges[p]}{'   <-- active' if edges[p] > 3 else ''}")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "scan"
    esp = connect()
    if cmd == "scan":
        scan(esp)
    elif cmd == "walk":
        walk(esp, float(sys.argv[2]) if len(sys.argv) > 2 else 3)
    elif cmd == "blink":
        blink(esp, int(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else 8)
    elif cmd == "hold":
        p, lvl, secs = int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
        as_output(esp, p, lvl); time.sleep(secs); as_input(esp, p)
    elif cmd == "charlie":
        charlie(esp, [int(x) for x in sys.argv[3:]], float(sys.argv[2]))
    elif cmd == "pairs":
        charlie_steps(esp, [int(x) for x in sys.argv[3:]], float(sys.argv[2]))
    elif cmd == "watch":
        watch(esp, float(sys.argv[2]) if len(sys.argv) > 2 else 15)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
