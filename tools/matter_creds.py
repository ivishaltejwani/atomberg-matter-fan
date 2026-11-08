#!/usr/bin/env python3
"""Give a fan its own Matter pairing code.

Generates a random passcode + discriminator, the SPAKE2+ verifier for them, sends them to the
fan's 'factory set' serial command (stored in NVS namespace chip-factory, which survives
Matter decommission), then reads back the fan's QR/manual code and saves a QR PNG.

  matter_creds.py --selftest        check the verifier maths against the Matter test vector
  matter_creds.py NAME              provision the connected fan, save ~/Documents/Fan QR Codes/NAME.{json,png}
                                    (reuses creds/NAME.json if it exists, so the QR never changes)
  matter_creds.py --save NAME       keep the code the fan generated itself on first boot, just
                                    read it back and save ~/Documents/Fan QR Codes/NAME.{json,png}
"""
import base64
import hashlib
import json
import os
import secrets
import sys
import time

import qrcode
import serial
from cryptography.hazmat.primitives.asymmetric import ec

PORT = "/dev/cu.wchusbserial2140"
P256_N = 0xFFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551
CREDS_DIR = os.path.expanduser("~/Documents/Fan QR Codes")

# Matter spec 5.1.7.1: passcodes that must never be used.
INVALID_PASSCODES = {0, 11111111, 22222222, 33333333, 44444444, 55555555, 66666666, 77777777,
                     88888888, 99999999, 12345678, 87654321}


def spake2p_verifier(passcode, salt, iterations):
    """w0 (32 B) || L = w1*G (65 B uncompressed), from PBKDF2-SHA256(passcode LE32, salt) -> 80 B."""
    ws = hashlib.pbkdf2_hmac("sha256", passcode.to_bytes(4, "little"), salt, iterations, 80)
    w0 = int.from_bytes(ws[:40], "big") % P256_N
    w1 = int.from_bytes(ws[40:], "big") % P256_N
    L = ec.derive_private_key(w1, ec.SECP256R1()).public_key().public_numbers()
    point = b"\x04" + L.x.to_bytes(32, "big") + L.y.to_bytes(32, "big")
    return w0.to_bytes(32, "big") + point


def selftest():
    # From CHIPDeviceConfig.h: spake2p gen-verifier -i 1000 -s "U1BBS0UyUCBLZXkgU2FsdA==" -p 20202021
    expected = ("uWFwqugDNGiEck/po7KHwwMwwqZgN10XuyBajPGuyzUEV/iree4lOrao5GuwnlQ65CJzbeUB49s31EH+NEkg0JVI5MGCQGMMT/SRPFNRODm3wH/MBiehuFc6FJ/"
                "NH6Rmzw==")
    got = base64.b64encode(spake2p_verifier(20202021, b"SPAKE2P Key Salt", 1000)).decode()
    print("selftest", "PASS" if got == expected else "FAIL")
    return got == expected


def new_passcode():
    while True:
        p = secrets.randbelow(99999998) + 1
        if p not in INVALID_PASSCODES:
            return p


def talk(s, line, wait):
    s.reset_input_buffer()
    s.write(line.encode() + b"\n")
    s.flush()
    out = b""
    end = time.time() + wait
    while time.time() < end:
        out += s.read(4096)
    return out.decode(errors="replace")


def read_codes(s):
    reply = talk(s, "pair", 2)
    print(reply.strip())
    manual = qr = None
    for ln in reply.splitlines():
        if ln.startswith("manual code:"):
            manual = ln.split(":", 1)[1].strip()
        if ln.startswith("payload:"):
            qr = ln.split(":", 1)[1].strip()
    if not manual or not qr:
        sys.exit("could not read the pairing code from the fan")
    return manual, qr


def save(name, rec):
    os.makedirs(CREDS_DIR, exist_ok=True)
    rec = {"name": name, **rec, "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(os.path.join(CREDS_DIR, f"{name}.json"), "w") as f:
        json.dump(rec, f, indent=2)
    qrcode.make(rec["qr_payload"]).save(os.path.join(CREDS_DIR, f"{name}.png"))
    print(f"saved {os.path.join(CREDS_DIR, name)}.png and .json  (manual code {rec['manual_code']})")


def open_fan():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, 115200, 0.2
    s.dtr = s.rts = False  # keep BOOT high
    s.open()
    time.sleep(0.2)
    return s


def save_only(name):
    s = open_fan()
    manual, qr = read_codes(s)
    s.close()
    save(name, {"manual_code": manual, "qr_payload": qr})


def provision(name):
    if not selftest():
        sys.exit("verifier maths wrong, not provisioning")
    path = os.path.join(CREDS_DIR, f"{name}.json")
    old = None
    if os.path.exists(path):
        with open(path) as f:
            old = json.load(f)
    if old and "passcode" in old:
        passcode, discriminator, iterations = old["passcode"], old["discriminator"], old["iterations"]
        salt_b64, ver_b64 = old["salt_b64"], old["verifier_b64"]
        print(f"reusing existing code for {name} ({old['manual_code']})")
    else:
        passcode = new_passcode()
        discriminator = secrets.randbelow(4096)
        iterations = 1000
        salt = secrets.token_bytes(32)
        salt_b64 = base64.b64encode(salt).decode()
        ver_b64 = base64.b64encode(spake2p_verifier(passcode, salt, iterations)).decode()

    s = open_fan()
    reply = talk(s, f"factory set {discriminator} {passcode} {iterations} {salt_b64} {ver_b64}", 2)
    print(reply.strip())
    if "factory data saved" not in reply:
        sys.exit("fan did not accept the factory data")
    time.sleep(12)  # decommission + reboot + Matter start
    manual, qr = read_codes(s)
    s.close()
    save(name, {"discriminator": discriminator, "passcode": passcode, "iterations": iterations,
                "salt_b64": salt_b64, "verifier_b64": ver_b64, "manual_code": manual, "qr_payload": qr})


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        sys.exit(0 if selftest() else 1)
    if len(sys.argv) == 3 and sys.argv[1] == "--save":
        save_only(sys.argv[2])
    elif len(sys.argv) == 2:
        provision(sys.argv[1])
    else:
        sys.exit(__doc__)
