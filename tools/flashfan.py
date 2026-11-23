#!/usr/bin/env python3
"""Flash build/atomberg_matter onto whichever fan is on the serial adapter, encrypted or not, no BOOT wire.

  tools/flashfan.py [--devhost 192.168.1.7] [--port /dev/cu.usbserial-2140]

1. asks the running firmware for download mode ('dl'); a chip already in download mode is fine too
2. reads the chip's flash-encryption state and writes bootloader / partitions / blank otadata / app
   per file (with --encrypt on an encrypted chip; never the merged image)
3. boots the app with an RTC watchdog reset, sets the dev host, prints status and the pairing code
NVS is not touched: pairing, pairing code and saved state survive. otadata is blanked so the freshly written
slot 0 boots even if the fan was running an OTA build from slot 1.
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time

import serial
from esptool.cmds import detect_chip

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
B = os.path.join(ROOT, "build", "atomberg_matter", "atomberg_matter.ino")
NOISE = ("i2c", "I2C", "nack", "telemetry", "mcf:")


def talk(port, cmd=None, wait=4.0):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.3
    s.dtr = s.rts = False
    s.open()
    if cmd:
        time.sleep(0.2)
        s.reset_input_buffer()
        s.write(("\r\n" + cmd + "\r\n").encode())
        s.flush()
    end, buf = time.time() + wait, b""
    while time.time() < end:
        buf += s.read(4096)
    s.close()
    return [l for l in buf.decode(errors="replace").splitlines() if l.strip() and not any(n in l for n in NOISE)]


def esptool(port, *args):
    return subprocess.run([sys.executable, "-m", "esptool", "--chip", "esp32c3", "-p", port, "-b", "115200",
                           "--before", "no-reset", "--after", "no-reset", *args], capture_output=True, text=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-2140")
    ap.add_argument("--devhost")
    a = ap.parse_args()

    before = talk(a.port, "info", 3)
    print("before:", next((l for l in before if l.startswith("wifi=")), "(no reply from an app: assuming download mode)"))
    talk(a.port, "dl", 2)
    time.sleep(1.5)

    r = esptool(a.port, "get-security-info")
    if r.returncode:
        sys.exit("chip is not in download mode:\n" + (r.stdout + r.stderr)[-400:])
    encrypted = "Flash Encryption: Enabled" in r.stdout
    print("flash encryption:", "ENABLED (writing with --encrypt)" if encrypted else "disabled (plain write)")

    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(b"\xff" * 0x2000)
        otadata = f.name
    regions = ["0x0", B + ".bootloader.bin", "0x8000", B + ".partitions.bin", "0xe000", otadata, "0x10000", B + ".bin"]
    t = time.time()
    r = esptool(a.port, "write-flash", *(["--encrypt"] if encrypted else []), *regions)
    os.unlink(otadata)
    wrote = [l for l in r.stdout.splitlines() if l.startswith("Wrote")]
    print("\n".join(wrote))
    if r.returncode or len(wrote) != 4:
        sys.exit("FLASH FAILED:\n" + (r.stdout + r.stderr)[-600:])
    print(f"flashed in {time.time() - t:.0f} s")

    esp = detect_chip(a.port, 115200, "no-reset")
    mac = "".join(f"{b:02x}" for b in esp.read_mac())
    p = esp._port
    p.dtr = False
    p.rts = False
    time.sleep(0.05)
    try:
        esp.watchdog_reset()
    except Exception:
        pass  # the chip resets before it can answer
    p.close()
    print("fan", mac, "rebooted")

    boot = talk(a.port, None, 12)
    for l in boot:
        if any(k in l for k in ("atomberg_matter", "not paired", "restarting", "restoring", "bluetooth released", "Guru", "ALLOC")):
            print("  ", l[:160])
    if a.devhost:
        cur = " ".join(talk(a.port, "devhost", 2))
        if f"dev host: {a.devhost} " not in cur:
            talk(a.port, "devhost " + a.devhost, 2)
            print("dev host set to", a.devhost, "(fan reboots)")
            time.sleep(10)
    lines = talk(a.port, "info", 3) + talk(a.port, "devhost", 2)
    for l in lines:
        if l.startswith(("wifi=", "dev host")):
            print(l[:230])
    if any("commissioned=0" in l for l in lines):
        for l in talk(a.port, "pair", 2):
            if l.startswith(("manual code", "payload")):
                print(l)
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import matter_creds
        matter_creds.PORT = a.port
        try:
            matter_creds.save_only(f"fan-{mac}")
        except SystemExit as e:
            print("QR not saved:", e)


if __name__ == "__main__":
    main()
