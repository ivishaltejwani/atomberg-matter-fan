# Atomberg Renesa Prime – C3 (U6) findings, 2026-10-20

## Status
- **Stock firmware was erased by the user and replaced with a blank sketch. No flash backup exists.**
- Partition table, app strings, NVS contents and the stock command list are lost.
- eFuses (security config, MAC, chip revision) are unaffected and can still be read.

## GPIO map (probed from ROM download mode with `tools/pinprobe.py`, no flash writes)

LEDs are a 2 x 3 matrix: an LED lights when its row AND column pins are both HIGH.

| row \ column | GPIO1 | GPIO4 | GPIO10 |
|---|---|---|---|
| GPIO5 | D9  | D14 | D12 |
| GPIO6 | D8  | D13 | D11 |

The 6 LEDs (ring around the IR receiver) indicate fan speed 1–6. **Confirmed on hardware:**
speed 1..6 = **D12, D11, D9, D8, D14, D13** (clockwise around U7). One LED lit per speed.

Remote (Atomberg IR): Power, speed 1–6 (ring + centre 6), TIMER, LED, SLEEP.
Protocol: **NEC, extended 16-bit address 0xF300** (addr byte 0x00, second byte 0xF3, not 0xFF), command byte + inverse.
Holding a button sends NEC repeat frames. Captured 2026-10-20 with fan_lab (log in `ir_codes_log.txt`); mapping assumes buttons pressed in listed order.

| Button | cmd | raw (LSB-first 32-bit) |
|---|---|---|
| Power | 0x91 | 0x6E91F300 |
| 1 | 0x8B | 0x748BF300 |
| 2 | 0x90 | 0x6F90F300 |
| 3 | 0x8A | 0x758AF300 |
| 4 | 0x93 | 0x6C93F300 |
| 5 | 0x88 | 0x7788F300 |
| 6 | 0x8F | 0x708FF300 |
| TIMER | 0x96 | 0x6996F300 |
| LED | 0x97 | 0x6897F300 |
| SLEEP | 0x8E | 0x718EF300 |

Remaining pins (passive scan with internal pull-down / pull-up, 3.3 V only, MCF8316C unpowered):

| GPIO | reads | likely role |
|---|---|---|
| 0, 7 | held LOW | MCF8316C inputs with internal pull-downs (SPEED / DIR / BRAKE / DRVOFF) |
| **3** | held HIGH | **IR receiver (U7) output**: 102 edges while pressing remote, all other pins 0 |
| 2, 8 | held HIGH | likely I2C SDA/SCL to MCF8316C (external pull-ups; both also strap pins) |
| 18, 19 | float | MCF8316C (FG / nFAULT?) or unused |
| 9 | BOOT pad | strap |
| 20, 21 | UART0 | debug header TX/RX |

## Security (esptool get-security-info + espefuse summary, read-only; `security_info.txt`, `efuse_summary.txt`)
- Chip: ESP32-C3 (QFN32) rev v0.4, 4 MB embedded flash (XMC), MAC 44:b1:76:53:6d:88.
- **Flash encryption: ENABLED**, SPI_BOOT_CRYPT_CNT = 0b001, still writable (not write-protected).
  XTS-AES-128 key in BLOCK_KEY0, read- and write-protected (as expected).
- **DIS_DOWNLOAD_MANUAL_ENCRYPT = False** → `esptool write-flash --encrypt` works over UART.
  Effectively development-mode flash encryption.
- Secure boot: disabled. Secure download mode: off. Download mode: enabled.
- JTAG permanently disabled (DIS_PAD_JTAG, DIS_USB_JTAG). DIS_DOWNLOAD_ICACHE set. UART ROM print enabled.
- Plaintext uploads (Arduino IDE / pio upload) therefore boot-looped with `invalid header`.
- **2026-10-20: flash encryption permanently disabled at the user's request**:
  `espefuse burn-efuse SPI_BOOT_CRYPT_CNT 0x3` (0b001 → 0b011), verified `Flash Encryption: Disabled`.
  Log in `efuse_burn_crypt_cnt.txt`. Plain uploads now work. Only one bit is left, so it can be re-enabled once at most.

## Serial / UART0 (GPIO20/21 pads)
- 115200 8N1. Fan TX -> dev kit "TX" header (= CH340 RXD), fan RX -> dev kit "RX" header.
- The ROM boot banner **does** print (seen once BOOT/IO9 wiring was sorted out); the early silent
  captures were a wiring/strap problem, not an eFuse.
- After the user's Arduino uploads the ROM loops on `invalid header: 0x1435f804`
  (`boot:0xf SPI_FAST_FLASH_BOOT`, TG0WDT/RTCWDT resets). A plaintext image decrypting to garbage →
  **flash encryption is probably enabled**. Not yet confirmed with `espefuse summary`.
- **The stock app logged nothing on its own.** It ran a line-based command parser on UART0 that
  answered unknown input with `string not matched !\r\n`. That reply is why esptool showed
  `Invalid head of packet (0x73)` (0x73 = 's').
- The command list was never recovered.

## Programmer setup
- USB-serial: CH340K on an ESP32-C6 dev kit (`/dev/cu.wchusbserial2140`), esptool v5.6.0 in `.venv/`.
- The C6 module was later desoldered from the dev kit.
- Auto-reset (BOOT→IO9, EN→EN) works for Arduino IDE uploads but was unreliable from esptool/pyserial on
  macOS, and opening the port can drop the C3 into download mode. For probing, tie BOOT directly to GND.
- In download mode the ROM accepts register read/write, so `tools/pinprobe.py` can drive and read GPIOs
  without anything in flash (works despite the encryption problem).
- **Measured:** the dev kit's RTS drives the fan's BOOT/GPIO9 low; nothing reaches the fan's EN. At USB power-up
  GPIO9 starts low, so the C3 always powers up in download mode (GPIO_STRAP=0x7).
- **Working no-touch flow:** `tools/c3flash.py build/fan_lab/fan_lab.ino.merged.bin`:
  sends `dl` to fan_lab (sets RTC_CNTL_FORCE_DOWNLOAD_BOOT + restart), writes at 0x0, then RTS/DTR off + RTC watchdog
  reset (esptool `watchdog_reset`) to boot the app. After a USB replug the C3 is already in download mode; the same script works.
- `tools/fanterm.py [--listen S] CMD...` talks to fan_lab's serial terminal.
- WiFi: fan_lab joins the office SSID (192.168.1.16, mDNS atomberg-fan.local, OTA port 3232), but the Mac on wired
  Ethernet can't reach it by unicast (AP client isolation?), so testing runs over the serial terminal for now.

## Motor driver (MCF8316C, U5): confirmed 2026-10-20
- **I2C: SDA = GPIO19, SCL = GPIO18, 7-bit address 0x01**, 100 kHz. Driver powered from +24V/DGND bench supply.
- Protocol (datasheet SLLSFV2A §6.6.2): 24-bit control word `CW23 read | CW22 CRC_EN | CW21-20 DLEN (01b=32-bit) |
  CW19-12 sec/page = 0 | CW11-0 addr`, then repeated start, data LSB-first. 32-bit read = `0x900000 | addr`.
- **EEPROM backup (read-only): `mcf_eeprom_backup.txt`** (0x80–0xAE + status 0xE0–0xE8). Nonzero
  MOTOR_STARTUP/CLOSED_LOOP/FAULT values (TI reset = 0) → **Atomberg's tuning is in the MCF EEPROM**.
- Decoded: PIN_CONFIG = 0x3A → **SPEED_MODE = 2 (I2C speed control)**, BRAKE_INPUT = 2 (pin overridden, no brake).
  DEVICE_CONFIG2 = 0xB000 → **DEV_MODE = 0 (standby, not sleep)**, so the buck and I2C stay on with the motor stopped.
  Faults 0, ALGO_STATUS = 0x04.
- I2C reads work with GPIO0 released, so GPIO0 isn't needed to wake the driver. In I2C mode the SPEED pin doesn't set speed.
- **Speed command** (do NOT write without the user's OK): ALGO_DEBUG1 0xEC, bit31 OVERRIDE = 1,
  bits30-16 DIGITAL_SPEED_CTRL (32767 = 100%). 0 → standby.
- **2026-10-20: first spin confirmed.** `mcf speed 5 1000` (5% for 1 s, then 0) → motor moved, user confirmed.
  No gate-driver or controller faults afterwards; ALGO_STATUS 0x00000004 → 0x029A0004. Log in `mcf_spin_log.txt`.
- GPIO0 / 7 / 2 / 8 roles still open (candidates DIR, BRAKE, FG, nFAULT, EXT_WD). GPIO7 toggles while powered.

## Motor driver notes from before power-up
- The motor tuning (motor parameters, closed-loop settings, speed input mode) normally lives in the
  **MCF8316C's own EEPROM**, not the C3's flash, so it most likely survived the erase.
- The MCF8316C is configured over I2C (default 7-bit target address 0x01). Next step: read-only I2C dump of the
  EEPROM/shadow registers before writing anything to the driver.
- Still needed: trace C3 GPIOs to the MCF8316C SDA/SCL, SPEED/PWM input, DIR, BRAKE, nFAULT, FG, plus the IR
  receiver, LEDs and buzzer.

## Files
- `fan_boot_log.txt`: serial captures (all empty apart from headers).
- `chip_id.txt`: failed esptool connect output (app replying instead of ROM).
- `tools/serial_capture.py`: passive UART logger (DTR/RTS deasserted).

## Matter firmware (`sketches/atomberg_matter`)
- Arduino-ESP32 3.3.0 Matter library (`MatterFan`), PartitionScheme=huge_app (1.9 MB / 3 MB). Build:
  `arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app --output-dir build/atomberg_matter sketches/atomberg_matter`
- Slider 0–100% → motor proportionally, scaled by `MOTOR_MAX_PCT` (bench cap **30%**; set 100 once mounted).
  Remote 1–6 → slider n/6; Power toggles; LED cycles speed LED → all on → all off; TIMER/SLEEP not implemented. Motor stopped at boot.
- First flash: merged bin (new partition table). **Updates: `tools/c3flash.py 0x10000 build/atomberg_matter/atomberg_matter.ino.bin`**
  so NVS (Matter pairing) is kept.
- **Pairing code:** on first boot (no NVS `chip-factory` data) the firmware generates its own random passcode,
  discriminator and salt, computes the SPAKE2+ verifier with mbedTLS (checked against the Matter test vector first) and
  stores them in `chip-factory`. Decommission / remote reset / app updates keep it; a full flash or `factory clear` makes a new one.
  The code is printed on serial at boot while unpaired and by `pair` (built with the SDK's GetQRCode/GetManualPairingCode,
  because `Matter.getManualPairingCode()` in Arduino 3.3.0 returns a hardcoded test string). `tools/c3flash.py` saves it
  automatically after every flash to `~/Documents/Fan QR Codes/fan-<mac>.png`.- Still uses the test vendor/product ID, so ecosystems show it as an uncertified device.

## Matter firmware: memory, Home sync, OTA (2026-10-21)
- **Bluetooth is freed after pairing** (`releaseBle()`): the prebuilt sdkconfig has CONFIG_USE_BLE_ONLY_FOR_COMMISSIONING off,
  so NimBLE + the controller kept ~70 KB forever. With Apple's 2 fabrics (Home + iPhone) CASE ran out of heap
  (`BIGNUM - Memory allocation failed`, `PacketBuffer pool EMPTY`) and Home showed No Response. Heap is now ~85-96 KB.
  BLE is released when the phone's BLE link closes during pairing, and at every boot once paired; failed pairing or
  removal from Home reboots so BLE advertises again.
- **Remote → Home sync:** MatterFan writes FanMode/PercentSetting/PercentCurrent with set_val inside its own callback, so
  attribute::update sees no change and never reports. `reportFanState()` marks all three dirty after every change.
- `Matter.getManualPairingCode()` in Arduino 3.3.0 is a hardcoded test string; `pair` uses the SDK's GetQRCode instead.
- LEDs: speed n lights LEDs 1..n (gauge). LED button cycles gauge → all on → all off. MOTOR_MAX_PCT is now **100**.
- **Partition table `sketches/atomberg_matter/partitions.csv`:** two 0x1F0000 OTA slots; nvs/otadata at the huge_app
  offsets, so pairing survived the switch. App is ~1.97 MB of 1.94 MiB (2,031,616 B) slot: watch the size
  (esp_http_client was +130 KB, so devlink uses plain sockets).
- **Dev link over WiFi (`devlink.h` + `tools/devlink.py serve`):** the fan pulls everything, because the wired Mac can't
  reach WiFi clients by unicast (but ARP works, and fan → Mac works). The fan polls `http://<devhost>:8070/fw.sha`
  every 30 s and installs `/fw.bin` when it differs, so a rebuild is enough to deploy. Logs (incl. ESP-IDF errors) go to
  UDP 8071 (`logs/devlink.log`); `devlink.py send CMD` sends commands back. Dev host is set with `devhost 192.168.1.10`
  (NVS "devlink"); none set = the link is off.
- **Rollback verified on hardware:** new image boots pending-verify (state 1) and is confirmed after 60 s with WiFi up.
  A build that aborts at boot was rolled back by the bootloader, and the fan refuses to reinstall a rolled-back SHA.
  (`verifyRollbackLater` must be `extern "C"`, or the Arduino core marks every image valid at boot.)
- **Signed OTA (2026-10-21):** `devlink.py keygen` → `keys/ota_signing_key.pem` (secret) + `ota_pubkey.h` (compiled in;
  no header = no OTA). Manifest `/fw.manifest` = "ATOMOTA1" | version u32 | length u32 | SHA-256(fw.bin) | ECDSA P-256 r‖s.
  Fan installs only if the signature verifies, version > NVS `devlink/otaver`, and the image matches length + SHA-256.
  otaver is stored before the reboot, so a rolled-back build is never retried. Any rejection pauses checks for 1 h
  (bounds flash wear from a hostile server). Version = mtime of the offered .bin (newest of build output / dashboard upload).
- Tested against a fake hostile server: wrong-key manifest rejected without download; tampered image rejected on SHA;
  older signed version ignored; `factory clear`, `devhost <ip>`, `decommission`, `pair`, `reboot`, `dl` refused over WiFi.
- Network commands are read-only (handleNetworkLine allowlist); pairing codes print on serial only.
- Dashboard: `devlink.py serve` → http://localhost:8090 (127.0.0.1 only, same-origin checked). LAN :8070 serves only
  the signed manifest + image. `--legacy` re-enables the old unsigned `/fw.sha` for migrating pre-signing firmware.
