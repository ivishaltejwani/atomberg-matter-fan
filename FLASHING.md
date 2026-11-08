# How to upload the firmware

For the ESP32-C3 in the Atomberg Renesa Halo. Ready-built images are in [`firmware/`](firmware/).

| File | Address | Use |
|---|---|---|
| `atomberg_matter.ino.bootloader.bin` | `0x0` | Bootloader |
| `atomberg_matter.ino.partitions.bin` | `0x8000` | Partition table |
| `atomberg_matter.ino.bin` | `0x10000` | Matter app |
| `atomberg_matter.ino.merged.bin` | `0x0` | All of the above in one 4 MB image. Plain flashing only |

The prebuilt images contain the original author's OTA public key, so they only accept OTA updates signed with that
key. If you are setting up your own fan, build from source with your own key (see the [README](README.md)).

## 1. What you need

- A 3.3 V USB-serial adapter. A CP2102 works.
- esptool: `python3 -m venv .venv && .venv/bin/pip install esptool pyserial`

Replace `PORT` below with your adapter, for example `/dev/cu.usbserial-2140` on macOS.

## 2. Wire it

Keep the **24 V supply and mains off**.

| Adapter | Fan board |
|---|---|
| GND | GND |
| 3V3 | 3V3 |
| TX | RX (GPIO20) |
| RX | TX (GPIO21) |
| 3V3 | RST / EN |
| GND | BOOT (GPIO9) |

## 3. Enter download mode

The chip only reads BOOT at power-up, so moving wires while it is running does nothing. Unplug the adapter from USB,
wait 5 seconds and plug it back in with BOOT already on GND.

## 4. Check whether flash encryption is on

```
.venv/bin/esptool --chip esp32c3 -p PORT -b 115200 --before no-reset --after no-reset get-security-info
```

Look at the `Flash Encryption:` line and use the matching step below. These fans ship with it **enabled**.

`Invalid head of packet (0x73)` means the stock firmware is running and the chip is not in download mode (the 0x73 is
the `s` of its `string not matched !` reply). Go back to step 3 and check that BOOT really reaches GPIO9.

## 5a. Flash encryption ENABLED

```
B=firmware
.venv/bin/esptool --chip esp32c3 -p PORT -b 115200 --before no-reset --after no-reset \
  write-flash --encrypt \
  0x0 $B/atomberg_matter.ino.bootloader.bin \
  0x8000 $B/atomberg_matter.ino.partitions.bin \
  0x10000 $B/atomberg_matter.ino.bin
```

- Use the three separate files. **Do not use the merged image here**: encrypting it would turn the blank settings
  area into garbage.
- `Cannot verify written data if encrypted` is normal.
- Arduino IDE and other plain uploads will boot-loop with `invalid header` on an encrypted chip.
- No eFuse has to be burned for this method.
- Stay at 115200. A read at 460800 hung the flasher over a CP2102 until the fan was power-cycled.

## 5b. Flash encryption DISABLED

```
.venv/bin/esptool --chip esp32c3 -p PORT -b 460800 --before no-reset --after no-reset \
  write-flash 0x0 firmware/atomberg_matter.ino.merged.bin
```

This wipes the settings area, including Matter pairing. To keep pairing, write only the app at `0x10000`.

## 6. Boot it

1. Remove the BOOT to GND wire.
2. Unplug and replug USB. Keep 24 V off for this first boot.
3. Open a serial terminal at 115200 and send `pair` to get the manual pairing code and QR payload. A new code is
   generated on first boot.

The firmware keeps the motor stopped at boot.

## 7. Pair

Apple Home → Add Accessory, then scan the QR or type the manual code. The fan appears as an uncertified device
because it uses the Matter test vendor ID.

## 8. Enable updates over WiFi (optional)

Do this over serial **before removing the adapter**, because it cannot be changed over WiFi:

```
devhost 192.168.1.10
```

Use the IP of the computer running `tools/devlink.py serve`. See the [README](README.md) for how signed updates work.

## Tested status

| | Plain flash, encryption disabled (fan 1) | Encrypted flash (fan 2) |
|---|---|---|
| Boots and runs | Yes | Yes |
| Matter pairing | Yes | Not yet tested |
| Motor control | Yes | Not yet tested |
| OTA update and rollback | Yes | Not yet tested |

Do not burn eFuses unless you have to. Disabling flash encryption with `SPI_BOOT_CRYPT_CNT` is permanent.
