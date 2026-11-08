# Atomberg Renesa Halo: Matter firmware

Replacement firmware for the ESP32-C3 in the Atomberg Renesa Halo ceiling fan (MCF8316C motor driver).
Works with Apple Home (or any Matter controller): on/off, speed slider, the original IR remote, the LED ring.
Hardware notes and everything learned along the way are in [findings.md](findings.md).
The full build story, with photos: [How I turned my Atomberg fan into a Matter fan](blog/how-i-made-my-atomberg-fan-a-matter-fan.md).

**To flash a ready-built image, see [FLASHING.md](FLASHING.md).**

## First install (serial cable, once per fan)

1. Arduino IDE / `arduino-cli` with **esp32 core 3.3.0**, board *ESP32C3 Dev Module*.
2. Make your own OTA signing key (once, keep `keys/` private and backed up):
   ```
   python3 -m venv .venv && .venv/bin/pip install esptool pyserial cryptography qrcode pillow
   .venv/bin/python tools/devlink.py keygen
   ```
   This writes `keys/ota_signing_key.pem` (secret) and `sketches/atomberg_matter/ota_pubkey.h` (built into the
   firmware). Without it the firmware still works, but has no OTA.
3. Build and flash over the serial header (TX/RX/GND/3V3, IO9 = BOOT):
   ```
   arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app \
     --output-dir build/atomberg_matter sketches/atomberg_matter
   .venv/bin/python tools/c3flash.py build/atomberg_matter/atomberg_matter.ino.merged.bin
   ```
   The fan generates its own unique pairing code on first boot. `c3flash.py` saves the QR code to
   `~/Documents/Fan QR Codes/fan-<chip id>.png`.
4. Pair it: Apple Home → Add Accessory → scan that QR.
5. Optional, for OTA: tell the fan which computer serves updates (serial only), e.g. `devhost 192.168.1.10`
   (`tools/fanterm.py "devhost 192.168.1.10"`). Give that computer a fixed IP (DHCP reservation).

## Updates without a cable

Run the dev-link server on that computer and leave it running:
```
.venv/bin/python tools/devlink.py serve
```
then open **http://localhost:8090**: fan status, live log, commands, and an upload box. To update, either
rebuild the sketch, or in Arduino IDE use *Sketch → Export Compiled Binary* and drop the `.ino.bin` on the page.
The server signs it with your key; the fan installs it within ~30 s and reboots.

Security model:
- The fan installs only firmware whose manifest is signed with **your** key and newer than what it has installed.
  The downloaded image must match the signed SHA-256. Anyone else on the network can't push firmware, and
  can't replay old signed builds.
- A build that crashes or doesn't come up on WiFi within 60 s is rolled back automatically.
- Over WiFi the fan only accepts read-only commands (and "stop motor"). Changing the update server, pairing
  codes, factory reset etc. need the serial cable (factory reset also works with the remote: Power ×10).
- The dashboard only listens on 127.0.0.1. The LAN only sees the signed manifest and the firmware image.
- Flash encryption and secure boot are off, so anyone with physical access to the board can reflash it.

## Remote

Power: on/off · 1–6: speed · LED: speed gauge → all on → all off · **Power ×10 quickly: factory reset**
(unpairs; the pairing code stays the same).
