#!/usr/bin/env python3
"""Flash the fan's C3 over USB without touching EN or BOOT.

The dev kit's auto-reset can only pull BOOT (via RTS); it can't pulse EN. So:
  1. ask the running fan_lab firmware for download mode ('dl' command),
     or rely on the C3 already being in download mode (it is after power-up);
  2. write the merged image at 0x0 with --before no-reset;
  3. hold BOOT high (RTS/DTR off) and reset via the RTC watchdog,
     which re-latches the strapping pins and boots the new app.

Usage: c3flash.py MERGED_BIN            (writes at 0x0, wipes NVS / Matter pairing)
       c3flash.py ADDR FILE [ADDR FILE ...]   (e.g. 0x10000 app.bin, keeps NVS)
Afterwards the fan's QR code is saved to ~/Documents/Fan QR Codes/fan-<mac>.png.
"""
import subprocess
import sys
import time

import serial
from esptool.cmds import detect_chip

import matter_creds

PORT = "/dev/cu.wchusbserial2140"


def open_port():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, 115200, 0.3
    s.dtr = s.rts = False  # BOOT high
    s.open()
    return s


def request_download_mode():
    s = open_port()
    s.write(b"\ndl\n")
    s.flush()
    time.sleep(0.8)
    s.close()


def watchdog_boot():
    """Reboot into the app; returns the chip MAC (used to name the fan's QR file)."""
    # write-flash leaves the stub running at the flashing baud rate.
    esp = detect_chip(PORT, 460800, "no-reset")
    mac = "".join(f"{b:02x}" for b in esp.read_mac())
    p = esp._port
    p.dtr = False; p.rts = False; p.dtr = False  # BOOT high so the app boots
    time.sleep(0.05)
    try:
        esp.watchdog_reset()
    except Exception:
        pass  # the chip resets before it can answer
    p.close()
    return mac


def save_qr(mac):
    """Copy the fan's own pairing code (generated on its first boot) to ~/Documents/Fan QR Codes."""
    time.sleep(10)  # boot + first-boot code generation + Matter start
    try:
        matter_creds.save_only(f"fan-{mac}")
    except SystemExit as e:
        print(f"QR not saved ({e}); run: tools/matter_creds.py --save fan-{mac}")


def main():
    args = sys.argv[1:]
    regions = ["0x0", args[0]] if len(args) == 1 else args
    request_download_mode()
    r = subprocess.run([sys.executable, "-m", "esptool", "--chip", "esp32c3", "-p", PORT, "-b", "460800",
                        "--before", "no-reset", "--after", "no-reset",
                        "write-flash", *regions])
    if r.returncode:
        sys.exit("flash failed: C3 not in download mode (replug USB once, then rerun)")
    mac = watchdog_boot()
    print("flashed and rebooted into the new app")
    save_qr(mac)


if __name__ == "__main__":
    main()
