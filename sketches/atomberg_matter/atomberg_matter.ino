// Matter firmware for the Atomberg Renesa Halo fan board (ESP32-C3-MINI-1 + MCF8316C).
// Matter Fan endpoint (on/off + speed slider), Atomberg IR remote, 6-LED speed indicator.
// Arduino-ESP32 core 3.x, board "ESP32C3 Dev Module", Partition Scheme "Huge APP",
// USB CDC On Boot: Disabled, partitions.csv in this folder (two OTA slots). See findings.md.
// WiFi is provisioned by the Matter controller over BLE during pairing (no credentials in code).

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Matter.h>
#include <nvs.h>
#include "soc/rtc_cntl_reg.h"
#include "bootloader_random.h"
#include "esp_random.h"
#include "mbedtls/base64.h"
#include "mbedtls/ecp.h"
#include "mbedtls/pkcs5.h"
#include <app/server/OnboardingCodesUtil.h>
#include <setup_payload/ManualSetupPayloadGenerator.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>
#include <app/reporting/reporting.h>
#include "esp_netif.h"
#include "esp_wifi.h"
#include <app/InteractionModelEngine.h>
#include <app/server/Server.h>
#include <platform/internal/BLEManager.h>
#include "esp_bt.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include <platform/internal/GenericDeviceInstanceInfoProvider.h>
#include <platform/ESP32/ESP32Config.h>
#include <platform/ESP32/ConfigurationManagerImpl.h>
#include <mdns.h>
#include "lwip/nd6.h"
#include "lwip/tcpip.h"
#include "esp_netif_net_stack.h"
#include <app/FailSafeContext.h>
#include "devlink.h"  // OTA + logs + commands over WiFi; defines Log (Serial + network)

// CASE session setup during pairing needs internal RAM; the default 8 KB loop stack is more than loop() uses.
SET_LOOP_TASK_STACK_SIZE(4 * 1024);

// Slider 1..100% maps onto MOTOR_MIN_PCT..MOTOR_MAX_PCT of the driver's MAX_SPEED (67.5 electrical Hz).
// Measured 2026-10-21: below ~15% sensorless closed loop can't hold speed (1% = 0.7 Hz jittered), and
// speed 1 at 16% (10.8 Hz) felt almost stopped. Remote 1..6 = 33/47/60/73/87/100%.
static const float MOTOR_MIN_PCT = 20.0f, MOTOR_MAX_PCT = 100.0f;
// Slowing down is slewed in firmware: the driver (AVS on, CL_DEC) dropped 67 -> 11 Hz in ~1.5 s, which
// looked like the fan stopping. Speeding up keeps the driver's own CL_ACC ramp (~7.5 Hz/s).
// The slew must stay slower than the fan coasts down on its own, or the speed loop asks for braking, AVS
// clamps torque to 0 and the fan pulses (seen with a flat 10 %/s: iq_ref 0 / -0.23 / 0 ...). Air drag
// grows with speed^2, so the fan coasts fast from the top and slowly near the bottom: rate follows that.
static float motorDecelPctPerS(float pct) { return 2.0f + 13.0f * (pct / 100) * (pct / 100); }

// ---- LED matrix: LED lights when its row AND column are HIGH ----
static const uint8_t ROWS[2] = {5, 6};
static const uint8_t COLS[3] = {1, 4, 10};
struct Led { uint8_t d, row, col; };
static const Led LEDS[] = {
  {9, 0, 0}, {14, 0, 1}, {12, 0, 2},   // row GPIO5
  {8, 1, 0}, {13, 1, 1}, {11, 1, 2},   // row GPIO6
};
static const uint8_t SPEED_LED[6] = {12, 11, 9, 8, 14, 13};  // speed level 1..6, clockwise
static volatile uint8_t ledMask[2] = {0, 0};

static void setLeds(const uint8_t *ds, size_t n) {
  uint8_t m[2] = {0, 0};
  for (size_t i = 0; i < n; i++)
    for (const Led &l : LEDS)
      if (l.d == ds[i]) m[l.row] |= 1 << l.col;
  ledMask[0] = m[0];
  ledMask[1] = m[1];
}

// Multiplex one row per 2 ms tick. Runs on the existing esp_timer task, so no task stack of its own.
static void ledTick(void *) {
  static uint8_t r = 0;
  r ^= 1;
  digitalWrite(ROWS[0], LOW);
  digitalWrite(ROWS[1], LOW);
  for (int c = 0; c < 3; c++) digitalWrite(COLS[c], (ledMask[r] >> c) & 1);
  if (ledMask[r]) digitalWrite(ROWS[r], HIGH);
}

// ---- MCF8316C over I2C (target 0x01, SDA=GPIO19, SCL=GPIO18) ----
static const uint8_t MCF_ADDR = 0x01, MCF_SDA = 19, MCF_SCL = 18;
static const uint16_t MCF_ALGO_DEBUG1 = 0xEC;  // bit31 OVERRIDE, bits30-16 DIGITAL_SPEED_CTRL
static SemaphoreHandle_t i2cLock;

static bool mcfWrite32(uint16_t reg, uint32_t val) {
  uint32_t cw = (1UL << 20) | (reg & 0xFFF);  // write, 32-bit, no CRC
  xSemaphoreTake(i2cLock, portMAX_DELAY);
  Wire.beginTransmission(MCF_ADDR);
  Wire.write(cw >> 16);
  Wire.write(cw >> 8);
  Wire.write(cw);
  for (int i = 0; i < 4; i++) Wire.write(val >> (8 * i));
  bool ok = Wire.endTransmission() == 0;
  xSemaphoreGive(i2cLock);
  return ok;
}

static bool mcfRead32(uint16_t reg, uint32_t &val) {
  uint32_t cw = (1UL << 23) | (1UL << 20) | (reg & 0xFFF);  // read, 32-bit
  xSemaphoreTake(i2cLock, portMAX_DELAY);
  Wire.beginTransmission(MCF_ADDR);
  Wire.write(cw >> 16);
  Wire.write(cw >> 8);
  Wire.write(cw);
  bool ok = Wire.endTransmission(false) == 0 && Wire.requestFrom(MCF_ADDR, (uint8_t)4) == 4;
  val = 0;
  for (int i = 0; ok && i < 4; i++) val |= (uint32_t)Wire.read() << (8 * i);
  xSemaphoreGive(i2cLock);
  return ok;
}

static float motorPct = 0;     // last speed command written to the driver
static float motorTarget = 0;  // where motorRampTick() is heading

static uint32_t motorWord = 1UL << 31;  // last value written to ALGO_DEBUG1 (what the driver should hold)
static bool motorWriteOk = true;         // false if that write failed: the guard retries it

static void writeMotor(float pct, bool quiet) {
  pct = constrain(pct, 0.0f, 100.0f);
  uint32_t raw = (uint32_t)(pct / 100.0f * 32767.0f) & 0x7FFF;
  motorWord = (1UL << 31) | (raw << 16);
  bool ok = mcfWrite32(MCF_ALGO_DEBUG1, motorWord);
  motorWriteOk = ok;
  motorPct = pct;
  if (!quiet || !ok) Log.printf("motor %.1f%%%s\n", pct, ok ? "" : " (I2C WRITE FAILED)");
}

// Immediate: stops (coast) and speed-ups. applyFan() uses the ramp for slow-downs.
static void setMotor(float pct) {
  motorTarget = pct;
  writeMotor(pct, false);
}

// Called from loop(): walks a slow-down towards motorTarget in small 20 ms steps.
static void motorRampTick() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 20) return;
  float dt = min(now - last, 100UL) / 1000.0f;
  last = now;
  if (motorTarget <= 0 || motorTarget >= motorPct) return;
  float next = max(motorTarget, motorPct - motorDecelPctPerS(motorPct) * dt);
  writeMotor(next, next != motorTarget);
}

// Called from loop(): the MCF8316C can reset on its own (supply dip), which clears ALGO_DEBUG1 to 0, so the
// motor stops while we still think it runs. Once a second, read the speed command back and re-send it if the
// driver lost it, or if the last write failed. Only re-sends the speed the user already asked for.
static uint32_t motorRestores = 0;
static void motorGuardTick() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  uint32_t v;
  if (!mcfRead32(MCF_ALGO_DEBUG1, v)) return;  // driver busy/unpowered: try again next second
  uint32_t want = motorWord & 0xFFFF0000UL, have = v & 0xFFFF0000UL;
  if (have == want && motorWriteOk) return;
  uint32_t status = 0, gf = 0, cf = 0;
  mcfRead32(0xE4, status);
  mcfRead32(0xE0, gf);
  mcfRead32(0xE2, cf);
  motorRestores++;
  Log.printf("motor guard: driver holds 0x%08lX, expected 0x%08lX (status=0x%08lX gate=0x%08lX ctrl=0x%08lX), re-sending %.1f%% [#%lu]\n",
             (unsigned long)v, (unsigned long)want, (unsigned long)status, (unsigned long)gf, (unsigned long)cf,
             motorPct, (unsigned long)motorRestores);
  motorWriteOk = mcfWrite32(MCF_ALGO_DEBUG1, motorWord);
}

// ---- MCF8316C telemetry (read-only): what the motor is actually doing ----
// Algorithm variables (datasheet 8.5): values are fractions of 2^27, speeds scaled by MAX_SPEED.
static const uint16_t MCF_CLOSED_LOOP4 = 0x8E, MCF_GATE_FAULT = 0xE0, MCF_CTRL_FAULT = 0xE2,
                      MCF_ALGO_STATE = 0x190, MCF_VM = 0x476, MCF_SPEED_REF_CL = 0x5D4, MCF_IQ_REF_CL = 0x608,
                      MCF_SPEED_FDBK = 0x75A, MCF_VD = 0x4E6, MCF_VQ = 0x4E8;
static const char *const MCF_STATES[] = {"idle", "isd", "tristate", "brake_on_start", "ipd", "slow_first_cycle",
                                         "align", "open_loop", "closed_loop_unaligned", "closed_loop",
                                         "active_braking", "soft_stop", "recirculation_stop", "brake_on_stop",
                                         "fault", "mpet_motor_stop_check", "mpet_motor_stop_wait"};
static float mcfMaxSpeedHz = 0;  // electrical Hz at 100% (MAX_SPEED in CLOSED_LOOP4)
static bool telemetryOn = true;

static float mcfFrac(uint32_t raw) { return (int32_t)raw / 134217728.0f; }  // / 2^27, signed

static void mcfTelemetry() {
  uint32_t state, vm, ref, spd, iq, gf, cf, vd, vq;
  if (!mcfMaxSpeedHz) {
    uint32_t cl4;
    if (mcfRead32(MCF_CLOSED_LOOP4, cl4)) mcfMaxSpeedHz = (cl4 & 0x3FFF) / 6.0f;
  }
  if (!(mcfRead32(MCF_ALGO_STATE, state) && mcfRead32(MCF_VM, vm) && mcfRead32(MCF_SPEED_REF_CL, ref) &&
        mcfRead32(MCF_SPEED_FDBK, spd) && mcfRead32(MCF_IQ_REF_CL, iq) && mcfRead32(MCF_GATE_FAULT, gf) &&
        mcfRead32(MCF_CTRL_FAULT, cf) && mcfRead32(MCF_VD, vd) && mcfRead32(MCF_VQ, vq))) {
    Log.println("mcf: telemetry read failed");
    return;
  }
  state &= 0xFFFF;
  float refHz = mcfFrac(ref) * mcfMaxSpeedHz, spdHz = mcfFrac(spd) * mcfMaxSpeedHz;
  // Applied phase voltage vs what the supply can give in linear SVPWM (VM / sqrt(3)): voltage headroom.
  float vmV = vm * 60.0f / 134217728.0f, k = 60.0f / sqrtf(3.0f);
  float vApplied = sqrtf(sq(mcfFrac(vd) * k) + sq(mcfFrac(vq) * k)), vAvail = vmV / sqrtf(3.0f);
  Log.printf("mcf: cmd=%.1f%% state=%s ref=%.1fHz speed=%.1fHz (%.0f%% of max %.1fHz) vm=%.1fV volt=%.1f/%.1fV (%.0f%%) iq_ref=%.3f%s",
             motorPct, state < sizeof MCF_STATES / sizeof *MCF_STATES ? MCF_STATES[state] : "?", refHz, spdHz,
             mcfMaxSpeedHz ? spdHz * 100 / mcfMaxSpeedHz : 0, mcfMaxSpeedHz, vmV, vApplied, vAvail,
             vAvail ? vApplied * 100 / vAvail : 0, mcfFrac(iq), (gf | cf) ? "" : "\n");
  if (gf | cf) Log.printf(" FAULT gate=0x%08lX ctrl=0x%08lX\n", (unsigned long)gf, (unsigned long)cf);
}

// 'telem burst [s]': speed + torque reference at 50 Hz for s seconds (default 4), then a summary, to see
// fast hunting that the 1 s telemetry can't. Blocks loop() while it samples (remote input is queued).
static void telemetryBurst(int seconds) {
  static const int RATE = 50, MAX_N = 50 * 8;
  static float spd[MAX_N], iq[MAX_N];
  int n = constrain(seconds, 1, 8) * RATE;
  if (!mcfMaxSpeedHz) mcfTelemetry();
  uint32_t next = millis();
  for (int i = 0; i < n; i++) {
    uint32_t a = 0, b = 0;
    mcfRead32(MCF_SPEED_FDBK, a);
    mcfRead32(MCF_IQ_REF_CL, b);
    spd[i] = fabsf(mcfFrac(a) * mcfMaxSpeedHz);
    iq[i] = fabsf(mcfFrac(b));
    next += 1000 / RATE;
    while ((int32_t)(millis() - next) < 0) delay(1);
  }
  float mean = 0, imean = 0, smin = 1e9, smax = 0, imin = 1e9, imax = 0;
  for (int i = 0; i < n; i++) {
    mean += spd[i] / n; imean += iq[i] / n;
    smin = min(smin, spd[i]); smax = max(smax, spd[i]); imin = min(imin, iq[i]); imax = max(imax, iq[i]);
  }
  float var = 0;
  int crossings = 0;
  for (int i = 0; i < n; i++) {
    var += (spd[i] - mean) * (spd[i] - mean) / n;
    if (i && (spd[i - 1] - mean) * (spd[i] - mean) < 0) crossings++;
  }
  Log.printf("burst %ds @%dHz: speed mean=%.2fHz min=%.2f max=%.2f sd=%.3f osc~%.1fHz | iq_ref mean=%.3f min=%.3f max=%.3f\n",
             n / RATE, RATE, mean, smin, smax, sqrtf(var), crossings / 2.0f / (n / (float)RATE), imean, imin, imax);
  String line = "burst speed:";
  for (int i = 0; i < n && i < 100; i++) line += String(" ") + String(spd[i], 2);
  Log.println(line);
  line = "burst iq:";
  for (int i = 0; i < n && i < 100; i++) line += String(" ") + String(iq[i], 3);
  Log.println(line);
}

// Called from loop(): once a second while the motor runs, and for 15 s after it is told to stop.
static void telemetryTick() {
  static uint32_t last = 0, stoppedAt = 0;
  static bool wasRunning = false;
  bool running = motorPct > 0;
  if (wasRunning && !running) stoppedAt = millis();
  wasRunning = running;
  bool coasting = stoppedAt && millis() - stoppedAt < 15000;
  if (!telemetryOn || !(running || coasting) || millis() - last < 1000) return;
  last = millis();
  mcfTelemetry();
}

// ---- fan state ----
static MatterFan Fan;
// LED button cycles: speed gauge -> whole ring on -> all off -> speed gauge.
enum LedMode : uint8_t { LED_SPEED, LED_ALL_ON, LED_ALL_OFF };
static LedMode ledMode = LED_SPEED;

static uint8_t levelFor(uint8_t percent) {  // 1..6 for the LEDs and the remote
  return percent == 0 ? 0 : constrain((percent * 6 + 99) / 100, 1, 6);
}

static void showLeds(bool on, uint8_t percent) {
  uint8_t lvl = levelFor(percent);
  if (ledMode == LED_ALL_ON) setLeds(SPEED_LED, 6);
  else if (ledMode == LED_SPEED && on && lvl) setLeds(SPEED_LED, lvl);  // gauge: speed n lights LEDs 1..n
  else setLeds(nullptr, 0);
}

// MatterFan writes FanMode/PercentSetting/PercentCurrent with set_val from inside its own attribute
// callback, so the following attribute::update sees no change and never reports it: Home missed every
// remote change. Store the real speed and mark all three dirty, unconditionally.
// Runs on the Matter task (ScheduleWork), so it is safe from loop() and from Matter callbacks alike.
static void reportFanState(uint8_t pct) {
  chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg) {
    using namespace chip::app::Clusters;
    uint16_t ep = Fan.getEndPointId();
    esp_matter::attribute_t *attr = esp_matter::attribute::get(ep, FanControl::Id, FanControl::Attributes::PercentCurrent::Id);
    esp_matter_attr_val_t val = esp_matter_uint8((uint8_t)arg);
    if (attr) esp_matter::attribute::set_val(attr, &val);
    MatterReportingAttributeChangeCallback(ep, FanControl::Id, FanControl::Attributes::FanMode::Id);
    MatterReportingAttributeChangeCallback(ep, FanControl::Id, FanControl::Attributes::PercentSetting::Id);
    MatterReportingAttributeChangeCallback(ep, FanControl::Id, FanControl::Attributes::PercentCurrent::Id);
  }, pct);
}

static void noteFanState(bool on, uint8_t pct);

// "On" is always FanMode High (3), never the library's FAN_MODE_ON (4). Mode 4 is a deprecated alias that a
// fan must not report (Matter spec: a write of On becomes High). Apple Home has no mapping for it: its on/off
// value for the fan came back empty and the tile showed "Unavailable" / "No Response" although speed changes
// went through (homed log 2026-11-23: "Nil updated value for characteristic", fan reporting FanMode 4).
// MatterFan::setOnOff()/toggle() write 4, so they are only used here to switch OFF.
static bool fanSetOn(bool on, bool performUpdate = true) {
  return Fan.setMode(on ? MatterFan::FAN_MODE_HIGH : MatterFan::FAN_MODE_OFF, performUpdate);
}

// Called from loop(): whatever still leaves FanMode at 4 (a value stored by older firmware, a controller
// writing On) is turned into High within a second, and reported.
static void fanModeTick() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  using namespace chip::app::Clusters;
  uint16_t ep = Fan.getEndPointId();
  esp_matter::attribute_t *attr = esp_matter::attribute::get(ep, FanControl::Id, FanControl::Attributes::FanMode::Id);
  esp_matter_attr_val_t val = esp_matter_invalid(NULL);
  if (!attr || esp_matter::attribute::get_val(attr, &val) != ESP_OK || val.val.u8 != MatterFan::FAN_MODE_ON) return;
  val.val.u8 = MatterFan::FAN_MODE_HIGH;
  esp_err_t err = esp_matter::attribute::update(ep, FanControl::Id, FanControl::Attributes::FanMode::Id, &val);
  Log.printf("fan mode ON(4) -> HIGH(3) (%s)\n", esp_err_to_name(err));
}

// Single place where Matter state turns into motor + LEDs.
static bool applyFan(MatterFan::FanMode_t mode, uint8_t percent) {
  bool on = mode != MatterFan::FAN_MODE_OFF && percent > 0;
  float pct = on ? MOTOR_MIN_PCT + (MOTOR_MAX_PCT - MOTOR_MIN_PCT) * (percent - 1) / 99.0f : 0;
  if (pct > 0 && pct < motorPct) motorTarget = pct;  // slow down gradually
  else setMotor(pct);
  showLeds(on, percent);
  reportFanState(on ? percent : 0);
  Log.printf("fan %s %u%% (mode %s)\n", on ? "ON" : "OFF", percent, Fan.getFanModeString(mode));
  noteFanState(on, on ? percent : 0);
  return true;
}

// ---- IR remote on GPIO3: NEC, address 0xF300 ----
static const int IR_PIN = 3;
static const uint16_t REMOTE_ADDR = 0xF300;
static QueueHandle_t remoteQueue;  // command bytes, handled in loop() (Matter calls stay on one task)

static inline bool IRAM_ATTR near(uint32_t v, uint32_t t) { return v > t * 7 / 10 && v < t * 13 / 10; }

// NEC decoded from the receiver's falling edges (start of each mark), timed between edges:
// leader 9 ms + 4.5 ms = 13.5 ms, bit 0 = 1.125 ms, bit 1 = 2.25 ms, 32 bits LSB first.
// No RMT channel or task, so the remote costs almost no RAM and can stay on during pairing.
static void IRAM_ATTR irEdge() {
  static int64_t last = 0;
  static int bits = -1;  // -1 = waiting for a leader
  static uint32_t v = 0;
  int64_t now = esp_timer_get_time();
  uint32_t dt = now - last;
  last = now;
  if (near(dt, 13500)) { bits = 0; v = 0; return; }
  if (bits < 0) return;
  if (near(dt, 2250)) v |= 1UL << bits;
  else if (!near(dt, 1125)) { bits = -1; return; }
  if (++bits < 32) return;
  bits = -1;
  uint8_t cmd = v >> 16;
  if ((v & 0xFFFF) == REMOTE_ADDR && (uint8_t)(cmd ^ (v >> 24)) == 0xFF) {
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(remoteQueue, &cmd, &woken);
    if (woken) portYIELD_FROM_ISR();
  }
}

// ---- remote factory reset: Power pressed RESET_PRESSES times, each within RESET_GAP_MS ----
static const int RESET_PRESSES = 10;
static const uint32_t RESET_GAP_MS = 1500;
static int powerBurst = 0;
static uint32_t lastPowerMs = 0;

static void showBurstProgress(int presses) {
  uint8_t n = presses * 6 / RESET_PRESSES;  // fill the ring as presses add up
  setLeds(SPEED_LED, n);
}

static void resetAnimation() {
  for (int round = 0; round < 3; round++)
    for (int i = 0; i < 6; i++) { setLeds(&SPEED_LED[i], 1); delay(60); }
  for (int i = 0; i < 3; i++) {
    setLeds(SPEED_LED, 6); delay(300);
    setLeds(nullptr, 0); delay(300);
  }
}

static void factoryResetFromRemote() {
  Log.println("remote: factory reset");
  Fan.setOnOff(false);
  setMotor(0);
  resetAnimation();
  Matter.decommission();  // clears pairing + WiFi (chip-factory pairing code is kept) and reboots
}

static void handlePowerButton() {
  uint32_t now = millis();
  powerBurst = (powerBurst > 0 && now - lastPowerMs < RESET_GAP_MS) ? powerBurst + 1 : 1;
  lastPowerMs = now;
  if (powerBurst == 1) {
    Log.println("remote: power");
    fanSetOn(!Fan.getOnOff());
    return;
  }
  Log.printf("remote: power x%d\n", powerBurst);
  if (powerBurst >= RESET_PRESSES) factoryResetFromRemote();
  else showBurstProgress(powerBurst);
}

// Called from loop(): a burst that stops short of a reset restores the normal display.
static void checkPowerBurstTimeout() {
  if (powerBurst > 1 && millis() - lastPowerMs > RESET_GAP_MS) {
    powerBurst = 0;
    showLeds(Fan.getOnOff(), Fan.getSpeedPercent());
  }
}

static void handleRemote(uint8_t cmd) {
  static const uint8_t SPEED_CMDS[6] = {0x8B, 0x90, 0x8A, 0x93, 0x88, 0x8F};  // buttons 1..6
  for (int i = 0; i < 6; i++) {
    if (cmd == SPEED_CMDS[i]) {
      uint8_t pct = (i + 1) * 100 / 6;  // proportional for now
      Log.printf("remote: speed %d -> %u%%\n", i + 1, pct);
      Fan.setSpeedPercent(pct);  // fires the Matter callbacks, which drive the motor
      return;
    }
  }
  switch (cmd) {
    case 0x91: handlePowerButton(); break;
    case 0x97:
      ledMode = (LedMode)((ledMode + 1) % 3);
      showLeds(Fan.getOnOff(), Fan.getSpeedPercent());
      Log.printf("remote: leds %s\n", ledMode == LED_SPEED ? "speed" : ledMode == LED_ALL_ON ? "all on" : "all off");
      break;
    case 0x96: Log.println("remote: timer (not implemented)"); break;
    case 0x8E: Log.println("remote: sleep (not implemented)"); break;
    default: Log.printf("remote: unknown cmd 0x%02X\n", cmd); break;
  }
}

// ---- per-device pairing code, stored where the Matter stack looks first (NVS "chip-factory") ----
// Generated on first boot (or written by tools/matter_creds.py); survives Matter.decommission().
// Salt/verifier are base64 strings.
static const char *FACTORY_NS = "chip-factory";
static const uint32_t SPAKE2P_ITERATIONS = 1000;

static int mbedRng(void *, unsigned char *buf, size_t len) {
  esp_fill_random(buf, len);
  return 0;
}

// SPAKE2+ verifier = w0 (32 B) || L = w1*G (65 B uncompressed), w0/w1 from PBKDF2-SHA256(passcode LE32, salt) -> 80 B.
static bool spake2pVerifier(uint32_t passcode, const uint8_t *salt, size_t saltLen, uint32_t iter, uint8_t out[97]) {
  uint8_t pw[4] = {(uint8_t)passcode, (uint8_t)(passcode >> 8), (uint8_t)(passcode >> 16), (uint8_t)(passcode >> 24)};
  uint8_t ws[80];
  if (mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256, pw, 4, salt, saltLen, iter, sizeof ws, ws) != 0) return false;
  mbedtls_ecp_group grp;
  mbedtls_mpi w, w0, w1;
  mbedtls_ecp_point L;
  mbedtls_ecp_group_init(&grp);
  mbedtls_mpi_init(&w); mbedtls_mpi_init(&w0); mbedtls_mpi_init(&w1);
  mbedtls_ecp_point_init(&L);
  size_t olen = 0;
  bool ok = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1) == 0 &&
            mbedtls_mpi_read_binary(&w, ws, 40) == 0 && mbedtls_mpi_mod_mpi(&w0, &w, &grp.N) == 0 &&
            mbedtls_mpi_read_binary(&w, ws + 40, 40) == 0 && mbedtls_mpi_mod_mpi(&w1, &w, &grp.N) == 0 &&
            mbedtls_mpi_write_binary(&w0, out, 32) == 0 &&
            mbedtls_ecp_mul(&grp, &L, &w1, &grp.G, mbedRng, nullptr) == 0 &&
            mbedtls_ecp_point_write_binary(&grp, &L, MBEDTLS_ECP_PF_UNCOMPRESSED, &olen, out + 32, 65) == 0 && olen == 65;
  mbedtls_ecp_point_free(&L);
  mbedtls_mpi_free(&w); mbedtls_mpi_free(&w0); mbedtls_mpi_free(&w1);
  mbedtls_ecp_group_free(&grp);
  return ok;
}

// Matter test vector (CHIPDeviceConfig.h): passcode 20202021, salt "SPAKE2P Key Salt", 1000 iterations.
static bool spake2pSelftest() {
  static const char *expected = "uWFwqugDNGiEck/po7KHwwMwwqZgN10XuyBajPGuyzUEV/iree4lOrao5GuwnlQ65CJzbeUB49s31EH+NEkg0JVI"
                                "5MGCQGMMT/SRPFNRODm3wH/MBiehuFc6FJ/NH6Rmzw==";
  uint8_t v[97];
  char b64[140];
  size_t n = 0;
  return spake2pVerifier(20202021, (const uint8_t *)"SPAKE2P Key Salt", 16, 1000, v) &&
         mbedtls_base64_encode((uint8_t *)b64, sizeof b64, &n, v, sizeof v) == 0 && strcmp(b64, expected) == 0;
}

static bool saveFactoryData(uint32_t disc, uint32_t pin, uint32_t iter, const char *salt, const char *ver) {
  nvs_handle_t h;
  if (nvs_open(FACTORY_NS, NVS_READWRITE, &h) != ESP_OK) return false;
  bool ok = nvs_set_u32(h, "discriminator", disc) == ESP_OK && nvs_set_u32(h, "pin-code", pin) == ESP_OK &&
            nvs_set_u32(h, "iteration-count", iter) == ESP_OK && nvs_set_str(h, "salt", salt) == ESP_OK &&
            nvs_set_str(h, "verifier", ver) == ESP_OK && nvs_commit(h) == ESP_OK;
  nvs_close(h);
  return ok;
}

static bool hasFactoryData() {
  nvs_handle_t h;
  uint32_t disc;
  if (nvs_open(FACTORY_NS, NVS_READONLY, &h) != ESP_OK) return false;
  bool has = nvs_get_u32(h, "discriminator", &disc) == ESP_OK;
  nvs_close(h);
  return has;
}

// First boot: give this fan its own random pairing code instead of the shared test code.
// Runs before Matter.begin(), so the radio is off; bootloader_random_enable() supplies real entropy.
static void ensureFactoryData() {
  if (hasFactoryData()) return;
  if (!spake2pSelftest()) {
    Log.println("pairing code generation selftest FAILED, using the test code");
    return;
  }
  static const uint32_t invalid[] = {0, 11111111, 22222222, 33333333, 44444444, 55555555, 66666666,
                                     77777777, 88888888, 99999999, 12345678, 87654321};
  bootloader_random_enable();
  uint32_t pin;
  bool bad;
  do {
    pin = esp_random() % 99999998 + 1;
    bad = false;
    for (uint32_t v : invalid) bad |= pin == v;
  } while (bad);
  uint32_t disc = esp_random() & 0xFFF;
  uint8_t salt[32];
  esp_fill_random(salt, sizeof salt);
  bootloader_random_disable();

  uint8_t ver[97];
  char saltB64[48], verB64[140];
  size_t n;
  bool ok = spake2pVerifier(pin, salt, sizeof salt, SPAKE2P_ITERATIONS, ver) &&
            mbedtls_base64_encode((uint8_t *)saltB64, sizeof saltB64, &n, salt, sizeof salt) == 0 &&
            mbedtls_base64_encode((uint8_t *)verB64, sizeof verB64, &n, ver, sizeof ver) == 0 &&
            saveFactoryData(disc, pin, SPAKE2P_ITERATIONS, saltB64, verB64);
  Log.println(ok ? "generated a new pairing code for this fan" : "pairing code generation FAILED, using the test code");
}

static void cmdFactory(const String &rest) {
  nvs_handle_t h;
  if (rest.startsWith("set ")) {
    unsigned long disc, pin, iter;
    char salt[64], ver[160];
    if (sscanf(rest.c_str() + 4, "%lu %lu %lu %63s %159s", &disc, &pin, &iter, salt, ver) != 5 || disc > 4095 ||
        pin < 1 || pin > 99999998 || iter < 1000 || iter > 100000) {
      Log.println("usage: factory set <disc 0-4095> <passcode> <iterations> <salt_b64> <verifier_b64>");
      return;
    }
    if (!saveFactoryData(disc, pin, iter, salt, ver)) { Log.println("factory data write FAILED"); return; }
    Log.println("factory data saved, unpairing and rebooting");
    setMotor(0);
    delay(100);
    Matter.decommission();
  } else if (rest == "show") {
    uint32_t disc = 0, iter = 0;
    if (nvs_open(FACTORY_NS, NVS_READONLY, &h) != ESP_OK) { Log.println("no factory data (test code in use)"); return; }
    bool has = nvs_get_u32(h, "discriminator", &disc) == ESP_OK;
    nvs_get_u32(h, "iteration-count", &iter);
    nvs_close(h);
    if (has) Log.printf("factory data: discriminator=%lu iterations=%lu\n", disc, iter);
    else Log.println("no factory data (test code in use)");
  } else if (rest == "clear") {
    if (nvs_open(FACTORY_NS, NVS_READWRITE, &h) == ESP_OK) { nvs_erase_all(h); nvs_commit(h); nvs_close(h); }
    Log.println("factory data cleared, unpairing and rebooting with a new random code");
    setMotor(0);
    delay(100);
    Matter.decommission();
  } else {
    Log.println("usage: factory set ... | show | clear");
  }
}

// Matter.getManualPairingCode()/getOnboardingQRCodeUrl() return hardcoded test-code strings, so build the
// codes from what the commissionable data provider really uses (chip-factory NVS).
// Serial only: the pairing code must not go out over the WiFi dev link.
static void printPairingCodes() {
  char manual[chip::kManualSetupLongCodeCharLength + 1];
  char qr[chip::QRCodeBasicSetupPayloadGenerator::kMaxQRCodeBase38RepresentationLength + 1];
  chip::MutableCharSpan manualSpan(manual), qrSpan(qr);
  chip::RendezvousInformationFlags flags(chip::RendezvousInformationFlag::kBLE);
  if (GetManualPairingCode(manualSpan, flags) != CHIP_NO_ERROR || GetQRCode(qrSpan, flags) != CHIP_NO_ERROR) {
    Serial.println("could not build pairing codes");
    return;
  }
  Serial.printf("manual code: %.*s\nQR: https://project-chip.github.io/connectedhomeip/qrcode.html?data=%.*s\n"
                "payload: %.*s\n", (int)manualSpan.size(), manual, (int)qrSpan.size(), qr, (int)qrSpan.size(), qr);
}

// ---- free Bluetooth once paired ----
// The prebuilt sdkconfig leaves CONFIG_USE_BLE_ONLY_FOR_COMMISSIONING off, so NimBLE + the BT controller
// hold their RAM forever. With two Apple fabrics (Home + iPhone) the CASE sessions then run out of heap
// ("BIGNUM alloc failed", "PacketBuffer pool EMPTY") and Home shows No Response. BLE is only needed for
// pairing; a reset reboots with it enabled again.
static bool bleReleased = false;
static void heapWatchStart();

static void releaseBle() {
  if (bleReleased) return;
  bleReleased = true;
  uint32_t before = ESP.getFreeHeap();
  chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) { chip::DeviceLayer::Internal::BLEMgr().Shutdown(); });
  delay(500);  // let CHIP stop advertising / the GATT service first
  chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t before) {
    if (ble_hs_is_enabled()) {
      nimble_port_stop();
      nimble_port_deinit();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) esp_bt_controller_disable();
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) esp_bt_controller_deinit();
    esp_err_t err = esp_bt_mem_release(ESP_BT_MODE_BLE);
    Log.printf("bluetooth released (%s): heap %lu -> %lu\n", esp_err_to_name(err), (unsigned long)before,
                  (unsigned long)ESP.getFreeHeap());
    heapWatchStart();  // only now: its stack would come out of the RAM that pairing needs
    devlinkStart();    // same for the dev link (OTA, logs)
  }, before);
}

// An unpaired fan must not be on WiFi. Bluetooth (needed to pair) and a connected WiFi station together
// leave ~10 KB of heap instead of ~26 KB, and pairing then fails ("stuck on setting up" in Home). Stale
// credentials survive whenever the reset that should wipe them is cut short, so drop them at every
// unpaired boot: the controller sends fresh ones while pairing.
// By the time we know the fan is unpaired the station is already started and connecting, and its buffers are
// allocated, so wipe the stored credentials and restart once: the next boot never starts WiFi.
// Runs on the Matter task, so Matter's own WiFi state machine cannot reconnect in between (done from setup(),
// esp_wifi_set_config failed with "sta is connecting"). Only the station credentials are wiped; not
// esp_wifi_restore(), which also resets the stored mode.
static RTC_NOINIT_ATTR uint32_t wifiCleanups;  // survives the restart below: never loop on it
static volatile bool restartAfterWifiCleanup = false;

static void dropStaleWifi() {
  if (esp_reset_reason() != ESP_RST_SW) wifiCleanups = 0;
  chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
    wifi_config_t cfg = {};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK || !cfg.sta.ssid[0]) return;
    chip::DeviceLayer::ConnectivityMgr().SetWiFiStationMode(chip::DeviceLayer::ConnectivityManager::kWiFiStationMode_Disabled);
    wifi_config_t empty = {};
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5 && err != ESP_OK; i++) {
      esp_wifi_disconnect();
      esp_wifi_stop();
      esp_wifi_set_mode(WIFI_MODE_STA);
      err = esp_wifi_set_config(WIFI_IF_STA, &empty);
      if (err != ESP_OK) vTaskDelay(pdMS_TO_TICKS(100));
    }
    Log.printf("not paired: old WiFi credentials cleared (%s)\n", esp_err_to_name(err));
    if (err == ESP_OK && wifiCleanups++ < 2) restartAfterWifiCleanup = true;
  });
}

// During pairing the phone drops its Bluetooth link once the fan has WiFi credentials, then opens CASE
// sessions over WiFi. That is the RAM peak, so free BLE as soon as the link closes. If pairing then
// fails (fail-safe expiry) or Home removes the fan, reboot so BLE is back for the next attempt.
static volatile bool bleLinkClosed = false, rebootForBle = false;

static void onMatterEvent(matterEvent_t ev, const chip::DeviceLayer::ChipDeviceEvent *) {
  switch (ev) {
    case MATTER_CHIPOBLE_CONNECTION_CLOSED:
      if (chip::DeviceLayer::ConnectivityMgr().IsWiFiStationProvisioned()) bleLinkClosed = true;
      break;
    case MATTER_FAIL_SAFE_TIMER_EXPIRED:
      Log.println("matter: pairing fail-safe expired");
      if (bleReleased && chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) rebootForBle = true;
      break;
    case MATTER_COMMISSIONING_COMPLETE: Log.println("matter: commissioning complete"); break;
    case MATTER_FABRIC_COMMITTED: Log.println("matter: fabric added"); break;
    case MATTER_FABRIC_REMOVED:
      Log.println("matter: fabric removed");
      if (bleReleased && chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) rebootForBle = true;
      break;
    default: break;
  }
}

// The SDK closes the pairing window 15 minutes after boot (CONFIG_CHIP_DISCOVERY_TIMEOUT_SECS) and stops
// advertising: an unpaired fan on the ceiling then could not be found until someone cut its power. Keep an
// unpaired fan pairable for as long as it takes. Called from loop().
static void pairingWindowTick() {
  static uint32_t last = 0;
  if (millis() - last < 10000) return;
  last = millis();
  if (bleReleased) return;  // paired (or about to reboot back into pairing mode)
  chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
    auto &server = chip::Server::GetInstance();
    auto &window = server.GetCommissioningWindowManager();
    if (server.GetFabricTable().FabricCount() != 0 || window.IsCommissioningWindowOpen()) return;
    CHIP_ERROR err = window.OpenBasicCommissioningWindow(chip::System::Clock::Seconds32(900),
                                                         chip::CommissioningWindowAdvertisement::kAllSupported);
    Log.printf("pairing window reopened (%s)\n", err == CHIP_NO_ERROR ? "ok" : "FAILED");
  });
}

// 'matter' command: is a controller actually connected? Read on the Matter task.
// The SDK makes ReadHandler's state private but befriends this (test-only) class; we never link the real one.
namespace chip { namespace app { namespace reporting {
class TestReportScheduler {
 public:
  static void Print(ReadHandler *h) {
    uint16_t minI = 0, maxI = 0;
    h->GetReportingIntervals(minI, maxI);
    SubscriptionId id = 0;
    h->GetSubscriptionId(id);
    uint64_t peer = h->GetSession() ? h->GetSubjectDescriptor().subject : 0;
    Log.printf("  %s id=%08lX fabric=%u peer=%08lX%08lX interval=%u..%us %s%s%s\n",
               h->IsType(ReadHandler::InteractionType::Subscribe) ? "subscription" : "read", (unsigned long)id,
               h->GetAccessingFabricIndex(), (unsigned long)(peer >> 32), (unsigned long)peer, minI, maxI,
               h->IsActiveSubscription() ? "established" : "NOT-ESTABLISHED", h->IsPriming() ? " priming" : "",
               h->IsAwaitingReportResponse() ? " awaiting-ack" : "");
  }
};
}}}

static void printMatterStatus() {
  chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
    auto &server = chip::Server::GetInstance();
    int secure = 0, caseSessions = 0;
    uint64_t now = chip::System::SystemClock().GetMonotonicTimestamp().count();
    server.GetSecureSessionManager().GetSecureSessions().ForEachSession([&](auto *session) {
      secure++;
      bool isCase = session->GetSecureSessionType() == chip::Transport::SecureSession::Type::kCASE;
      if (isCase) caseSessions++;
      char addr[chip::Transport::PeerAddress::kMaxToStringSize];
      session->GetPeerAddress().ToString(addr);
      Log.printf("  session lsid=%u fabric=%u peer=%08lX%08lX %s %s %s idle=%llus\n", session->GetLocalSessionId(),
                    session->GetFabricIndex(), (unsigned long)(session->GetPeerNodeId() >> 32),
                    (unsigned long)session->GetPeerNodeId(), isCase ? "CASE" : "PASE",
                    session->IsActiveSession() ? "active" : session->IsEstablishing() ? "establishing" : "defunct/other",
                    addr, (now - session->GetLastPeerActivityTime().count()) / 1000);
      return chip::Loop::Continue;
    });
    auto *im = chip::app::InteractionModelEngine::GetInstance();
    for (unsigned i = 0; i < im->GetNumActiveReadHandlers(); i++)
      if (auto *h = im->ActiveHandlerAt(i)) chip::app::reporting::TestReportScheduler::Print(h);
    Log.printf("matter: fabrics=%u sessions=%d (case=%d) subscriptions=%lu reads=%lu\n",
                  server.GetFabricTable().FabricCount(), secure, caseSessions,
                  (unsigned long)im->GetNumActiveReadHandlers(chip::app::ReadHandler::InteractionType::Subscribe),
                  (unsigned long)im->GetNumActiveReadHandlers(chip::app::ReadHandler::InteractionType::Read));
  });
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_ip6_addr_t ip6[CONFIG_LWIP_IPV6_NUM_ADDRESSES];  // esp_netif_get_all_ip6 can return that many
  int n = sta ? esp_netif_get_all_ip6(sta, ip6) : 0;
  for (int i = 0; i < n; i++) Log.printf("ipv6: " IPV6STR "\n", IPV62STR(ip6[i]));
}

// ---- heap diagnostics: catch the short dips that break CASE / subscription priming ----
struct AllocFail { uint32_t ms, size, caps, freeNow; char task[12]; };
static AllocFail allocFails[16];
static volatile uint32_t allocFailCount = 0;
static uint32_t allocFailLogged = 0;

static void onAllocFail(size_t size, uint32_t caps, const char *) {
  AllocFail &f = allocFails[allocFailCount % 16];
  f.ms = millis(); f.size = size; f.caps = caps; f.freeNow = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  const char *t = pcTaskGetName(nullptr);
  strncpy(f.task, t ? t : "?", sizeof f.task - 1);
  f.task[sizeof f.task - 1] = 0;
  allocFailCount++;
}

static const uint32_t HEAP_DIP = 30000;
static volatile bool dipActive = false, dipReady = false;
static volatile uint32_t dipStart = 0, dipEnd = 0, dipMin = 0, dipLargest = 0;
static uint32_t dipCount = 0;

static void heapWatchTask(void *) {
  for (;;) {
    uint32_t f = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    if (f < HEAP_DIP) {
      if (!dipActive) { dipActive = true; dipStart = millis(); dipMin = UINT32_MAX; }
      if (f < dipMin) { dipMin = f; dipLargest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT); }
    } else if (dipActive && !dipReady) {
      dipActive = false; dipEnd = millis(); dipReady = true;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void heapDiagBegin() {
  heap_caps_register_failed_alloc_callback(onAllocFail);
}

static void heapWatchStart() {  // called once Bluetooth is released (see releaseBle)
  static bool started = false;
  if (started) return;
  started = true;
  xTaskCreate(heapWatchTask, "heapwatch", 2048, nullptr, 10, nullptr);
}

static void heapDiagTick() {  // from loop(): logging is not safe inside the hooks
  while (allocFailLogged < allocFailCount) {
    if (allocFailCount - allocFailLogged > 16) allocFailLogged = allocFailCount - 16;
    const AllocFail &f = allocFails[allocFailLogged % 16];
    Log.printf("ALLOC FAILED: %lu bytes caps=0x%lx task=%s free=%lu at up=%lus\n", (unsigned long)f.size,
               (unsigned long)f.caps, f.task, (unsigned long)f.freeNow, (unsigned long)(f.ms / 1000));
    allocFailLogged++;
  }
  if (dipReady) {
    dipCount++;
    Log.printf("heap dip #%lu: min free %lu (largest block %lu) for %lu ms at up=%lus\n", (unsigned long)dipCount,
               (unsigned long)dipMin, (unsigned long)dipLargest, (unsigned long)(dipEnd - dipStart),
               (unsigned long)(dipStart / 1000));
    dipReady = false;
  }
}

// ---- WiFi: mains powered, so never power-save ----
// In modem sleep the radio only wakes for DTIM beacons and misses multicast: mDNS queries and IPv6
// neighbour solicitations from the HomePod. The fan then can't resolve the hub, and the hub can't reach
// the fan or keeps flipping it between reachable and unreachable. Matter can re-enable power save when
// WiFi reconnects, so check every few seconds and keep it off.
static void wifiPowerTick() {
  static uint32_t last = 0;
  static bool logged = false;
  if (millis() - last < 3000) return;
  last = millis();
  // Never while Bluetooth is up (pairing): WiFi/BLE share one radio and coexistence needs modem sleep.
  // With power save off, WiFi starves BLE as soon as it starts connecting and the phone reports
  // "Unable to pair". Only after pairing has released BLE is it safe to keep the radio awake.
  if (!bleReleased || esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) return;
  wifi_ps_type_t ps;
  if (esp_wifi_get_ps(&ps) != ESP_OK || ps == WIFI_PS_NONE) return;
  esp_err_t err = esp_wifi_set_ps(WIFI_PS_NONE);
  if (!logged || err != ESP_OK) Log.printf("wifi: power save %d -> off (%s)\n", (int)ps, esp_err_to_name(err));
  logged = true;
}

// ---- IPv6: ask for the network prefix instead of waiting for it ----
// Apple's hubs talk Matter over the routable IPv6 address (the fd.. prefix a HomePod / border router announces).
// The router only announces it every several minutes, and the solicitation lwIP sends at link-up gets no
// answer here, so after every power-up the fan sat with a link-local address only and Home showed
// "No Response" until the next periodic announcement (measured: 4 and 8 minutes). Solicit until one arrives.
static void routerSolicitTick() {
  static uint32_t last = 0, tries = 0;
  static bool had = false;
  if (millis() - last < (tries < 24 ? 5000u : 60000u)) return;
  last = millis();
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  wifi_ap_record_t ap;
  if (!sta || esp_wifi_sta_get_ap_info(&ap) != ESP_OK) { tries = 0; had = false; return; }
  esp_ip6_addr_t a[CONFIG_LWIP_IPV6_NUM_ADDRESSES];
  int n = esp_netif_get_all_ip6(sta, a);
  bool routable = false;
  for (int i = 0; i < n; i++) routable |= esp_netif_ip6_get_addr_type(&a[i]) != ESP_IP6_ADDR_IS_LINK_LOCAL;
  if (routable) {
    if (!had) Log.printf("ipv6: routable address at up=%lus after %lu solicitation(s)\n", millis() / 1000, (unsigned long)tries);
    had = true;
    tries = 0;
    return;
  }
  had = false;
  tries++;
  tcpip_callback([](void *arg) { nd6_restart_netif((struct netif *)arg); }, esp_netif_get_netif_impl(sta));
}

// ---- Matter over IPv6 only ----
// The fan also has an IPv4 address (DHCP; the dev link uses it), and mDNS advertised it. Apple's hub then
// sometimes picked IPv4, and on this network the fan cannot resolve the hub's MAC by ARP: replies queued,
// SendMessage failed with lwIP ERR_MEM for 30+ s, CASE and subscriptions timed out and Home showed
// "No Response" (log 2026-11-16 15:15). Over IPv6 the same hub connects and stays connected. Matter only
// requires IPv6, so keep IPv4 out of mDNS: controllers then never try it. mdns re-enables IPv4 on every
// DHCP event, hence the repeat.
static void mdnsIpv6OnlyTick() {
  static uint32_t last = 0;
  static bool logged = false;
  if (millis() - last < 10000) return;
  last = millis();
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  wifi_ap_record_t ap;
  if (!sta || esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return;
  esp_err_t err = mdns_netif_action(sta, MDNS_EVENT_DISABLE_IP4);
  if (!logged || err != ESP_OK) Log.printf("mdns: IPv4 hidden, Matter is advertised over IPv6 only (%s)\n", esp_err_to_name(err));
  logged = true;
}

// ---- fan state survives reboots and power cuts (NVS "fanstate"), like the stock fan ----
static bool savedOn = false;
static uint8_t savedPct = 0;
static uint32_t stateChangedAt = 0;
static bool statePending = false;

static void noteFanState(bool on, uint8_t pct) {
  if (on == savedOn && pct == savedPct) { statePending = false; return; }
  statePending = true;
  stateChangedAt = millis();
}

static void fanStateTick() {  // write 10 s after the last change, to spare the flash while someone slides
  if (!statePending || millis() - stateChangedAt < 10000) return;
  statePending = false;
  bool on = Fan.getOnOff();
  uint8_t pct = Fan.getSpeedPercent();
  nvs_handle_t h;
  if (nvs_open("fanstate", NVS_READWRITE, &h) != ESP_OK) return;
  nvs_set_u8(h, "on", on);
  nvs_set_u8(h, "pct", pct);
  nvs_commit(h);
  nvs_close(h);
  savedOn = on;
  savedPct = pct;
}

static void restoreFanState() {
  nvs_handle_t h;
  uint8_t on = 0, pct = 0;
  if (nvs_open("fanstate", NVS_READONLY, &h) == ESP_OK) {
    nvs_get_u8(h, "on", &on);
    nvs_get_u8(h, "pct", &pct);
    nvs_close(h);
  }
  savedOn = on;
  savedPct = pct;
  if (!on || !pct) return;
  Log.printf("restoring fan state: ON %u%%\n", pct);
  Fan.setSpeedPercent(pct);
  fanSetOn(true);
  applyFan(MatterFan::FAN_MODE_HIGH, pct);  // in case the setters did not run the change callback
}

// ---- device identity shown in Home (Basic Information) ----
// Names, serial and versions are free text. Vendor/product ID stay the test IDs (0xFFF1/0x8000): they must
// match the device attestation certificate, and a mismatch makes every controller refuse to pair.
class FanInstanceInfo
    : public chip::DeviceLayer::Internal::GenericDeviceInstanceInfoProvider<chip::DeviceLayer::Internal::ESP32Config> {
 public:
  FanInstanceInfo()
      : GenericDeviceInstanceInfoProvider(
            static_cast<chip::DeviceLayer::Internal::GenericConfigurationManagerImpl<chip::DeviceLayer::Internal::ESP32Config> &>(
                chip::DeviceLayer::ConfigurationManagerImpl::GetDefaultInstance())) {}
  CHIP_ERROR GetVendorName(char *buf, size_t n) override { return put(buf, n, "Atomberg"); }
  CHIP_ERROR GetProductName(char *buf, size_t n) override { return put(buf, n, "Renesa Prime Ceiling Fan"); }
  CHIP_ERROR GetProductLabel(char *buf, size_t n) override { return put(buf, n, "Renesa Prime"); }
  CHIP_ERROR GetPartNumber(char *buf, size_t n) override { return put(buf, n, "RENESA-PRIME-MATTER"); }
  CHIP_ERROR GetProductURL(char *buf, size_t n) override { return put(buf, n, ""); }
  CHIP_ERROR GetHardwareVersionString(char *buf, size_t n) override { return put(buf, n, "ESP32-C3 + MCF8316C"); }
  CHIP_ERROR GetHardwareVersion(uint16_t &v) override { v = 1; return CHIP_NO_ERROR; }
  CHIP_ERROR GetSerialNumber(char *buf, size_t n) override {  // unique per fan: the WiFi MAC
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (n < 13) return CHIP_ERROR_BUFFER_TOO_SMALL;
    snprintf(buf, n, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return CHIP_NO_ERROR;
  }

 private:
  static CHIP_ERROR put(char *buf, size_t n, const char *v) {
    if (strlen(v) + 1 > n) return CHIP_ERROR_BUFFER_TOO_SMALL;
    strcpy(buf, v);
    return CHIP_NO_ERROR;
  }
};
static FanInstanceInfo fanInstanceInfo;

// Apple Home builds every accessory's information page (Manufacturer, Model, Serial Number, Firmware) from
// Basic Information, and treats a missing Serial Number as a failed read: the tile shows "No Response" even
// though commands go through (seen 2026-11-16: speed changes worked, Home said No Response, and homed logged
// "could not find cached attribute values for ... BasicInformation SerialNumber"). SerialNumber is optional
// in Matter, so the Arduino node leaves it out and the fan answered UNSUPPORTED_ATTRIBUTE. Add it before
// Matter starts; its value is served by fanInstanceInfo (the WiFi MAC), the stored copy matches.
static void addSerialNumberAttribute() {
  using namespace esp_matter;
  node_t *node = node::get();
  endpoint_t *root = node ? endpoint::get(node, 0) : nullptr;
  cluster_t *info = root ? cluster::get(root, chip::app::Clusters::BasicInformation::Id) : nullptr;
  if (!info) {
    Log.println("serial number: Basic Information cluster not found");
    return;
  }
  if (attribute::get(info, chip::app::Clusters::BasicInformation::Attributes::SerialNumber::Id)) return;
  static char serial[13];
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(serial, sizeof serial, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  attribute_t *a = cluster::basic_information::attribute::create_serial_number(info, serial, strlen(serial));
  Log.printf("serial number attribute %s (%s)\n", a ? "added" : "FAILED", serial);
}

// ---- serial terminal (also used by tools/c3flash.py, fanterm.py and matter_creds.py) ----
static String serialLine;

static void handleLine(String cmd) {
  cmd.trim();
  if (cmd.isEmpty()) return;
  if (cmd == "help") {
    Log.println("info | matter | devhost [ip|off] | pair | factory set/show/clear | mcf read <addr> | mcf telem | telem on/off | telem burst [s] | mcf stop | decommission | reboot | dl");
  } else if (cmd == "info") {
    // Matter drives WiFi through esp_netif directly, so WiFi.localIP() stays 0.0.0.0; ask the netif.
    esp_netif_ip_info_t ip = {};
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta) esp_netif_get_ip_info(sta, &ip);
    wifi_ap_record_t ap = {};
    bool linked = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    Log.printf("wifi=%s ssid=%s rssi=%d ip=" IPSTR " commissioned=%d fan=%s %u%% motor=%.1f%% (target %.1f%%) heap=%u min_heap=%u motor_restores=%lu wifi_ps=%d\n",
                  linked ? "up" : "down", linked ? (const char *)ap.ssid : "-", linked ? ap.rssi : 0, IP2STR(&ip.ip),
                  Matter.isDeviceCommissioned(),
                  Fan.getOnOff() ? "ON" : "OFF", Fan.getSpeedPercent(), motorPct, motorTarget,
                  ESP.getFreeHeap(), ESP.getMinFreeHeap(), (unsigned long)motorRestores, ({ wifi_ps_type_t ps = WIFI_PS_NONE; esp_wifi_get_ps(&ps); (int)ps; }));
  } else if (cmd == "devhost" || cmd.startsWith("devhost ")) {
    devHostCommand(cmd.length() > 8 ? cmd.substring(8) : String());
  } else if (cmd == "matter") {
    printMatterStatus();
  } else if (cmd == "pair") {
    printPairingCodes();
  } else if (cmd.startsWith("factory")) {
    cmdFactory(cmd.substring(7).c_str() + (cmd.length() > 7 ? 1 : 0));
  } else if (cmd.startsWith("mcf read ")) {
    uint16_t reg = strtoul(cmd.c_str() + 9, nullptr, 0);
    uint32_t v;
    if (mcfRead32(reg, v)) Log.printf("MCF 0x%03X = 0x%08lX\n", reg, v);
    else Log.printf("MCF 0x%03X = READ FAILED\n", reg);
  } else if (cmd.startsWith("telem burst")) {
    telemetryBurst(cmd.length() > 12 ? cmd.substring(12).toInt() : 4);
  } else if (cmd == "mcf telem") {
    mcfTelemetry();
  } else if (cmd == "telem on" || cmd == "telem off") {
    telemetryOn = cmd == "telem on";
    Log.printf("telemetry %s\n", telemetryOn ? "on" : "off");
  } else if (cmd == "mcf stop") {
    Fan.setOnOff(false);
    setMotor(0);
  } else if (cmd == "decommission") {
    Log.println("decommissioning");
    Matter.decommission();
  } else if (cmd == "reboot") {
    ESP.restart();
  } else if (cmd == "dl") {
    // The dev kit can't pulse our EN, so ask the ROM for download mode on the next reset.
    setMotor(0);
    Log.println("entering download mode");
    delay(100);
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
  } else {
    Log.println("unknown command, try 'help'");
  }
}

// Commands from the WiFi dev link. Spoofing the dev host's IP is easy on a LAN, so only read-only
// commands (plus stopping the motor) are accepted from there; everything that changes the fan
// (devhost <ip>, factory, decommission, reboot, dl, pair) needs the serial cable.
static void handleNetworkLine(String cmd) {
  cmd.trim();
  static const char *const allowed[] = {"help", "info", "matter", "devhost", "mcf telem", "telem on", "telem off", "mcf stop"};
  bool ok = cmd.startsWith("telem burst") || cmd.startsWith("mcf read ");
  for (const char *a : allowed) ok |= cmd == a;
  if (ok) handleLine(cmd);
  else Log.printf("refused over network (serial only): %s\n", cmd.c_str());
}

void setup() {
  Serial.begin(115200);
  Log.printf("\natomberg_matter build %s %s\n", __DATE__, __TIME__);
#ifdef ROLLBACK_TEST
  abort();  // deliberately broken build, to prove OTA rollback works
#endif
  for (uint8_t p : ROWS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  for (uint8_t p : COLS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  heapDiagBegin();
  i2cLock = xSemaphoreCreateMutex();
  Wire.begin(MCF_SDA, MCF_SCL, 100000);
  Wire.setTimeOut(50);
  setMotor(0);  // never start spinning on boot
  devlinkBegin([] { setMotor(0); });  // OTA stops the motor before flashing
  // No update while pairing is still in progress: the reboot would drop the half-finished pairing.
  devUpdateAllowed = [] {
    return Matter.isDeviceCommissioned() && !chip::Server::GetInstance().GetFailSafeContext().IsFailSafeArmed();
  };

  const esp_timer_create_args_t ledTimerArgs = {.callback = ledTick, .name = "led"};
  esp_timer_handle_t ledTimer;
  esp_timer_create(&ledTimerArgs, &ledTimer);
  esp_timer_start_periodic(ledTimer, 2000);

  remoteQueue = xQueueCreate(8, 1);
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(IR_PIN, irEdge, FALLING);

  Fan.begin(0, MatterFan::FAN_MODE_OFF, MatterFan::FAN_MODE_SEQ_OFF_HIGH);
  // Keep on/off and speed consistent, as in the MatterFan example.
  Fan.onChangeSpeedPercent([](uint8_t pct) {
    if (pct == MatterFan::OFF_SPEED && Fan.getMode() != MatterFan::FAN_MODE_OFF) return Fan.setOnOff(false, Fan.ATTR_SET);
    if (pct > MatterFan::OFF_SPEED && Fan.getMode() == MatterFan::FAN_MODE_OFF) return fanSetOn(true, Fan.ATTR_SET);
    return true;
  });
  Fan.onChangeMode([](MatterFan::FanMode_t mode) {
    if (Fan.getSpeedPercent() == MatterFan::OFF_SPEED && mode != MatterFan::FAN_MODE_OFF)
      return Fan.setSpeedPercent(100 / 6, Fan.ATTR_SET);  // turning on from 0% starts at speed 1
    return true;
  });
  Fan.onChange(applyFan);
  addSerialNumberAttribute();  // the node exists once the fan endpoint does; Matter starts below

  ensureFactoryData();
  Matter.onEvent(onMatterEvent);
  Matter.begin();
  chip::DeviceLayer::SetDeviceInstanceInfoProvider(&fanInstanceInfo);  // read at pairing + on every read
  if (!Matter.isDeviceCommissioned()) {
    dropStaleWifi();
    Log.println("\natomberg_matter: not commissioned, pair over Bluetooth");
    printPairingCodes();
  } else {
    Fan.updateAccessory();  // restore last state from Matter storage
    releaseBle();
    restoreFanState();
  }
  Log.printf("heap after Matter start: %u\n", ESP.getFreeHeap());
}

void loop() {
  static bool wasCommissioned = true;
  static uint32_t pairedAt = 0;
  static uint32_t lastStep = 0;
  static uint8_t step = 0;
  bool commissioned = Matter.isDeviceCommissioned();
  // Slow LED chase while waiting to be paired (the remote still works; its speed shows instead while on).
  if (!commissioned && !Fan.getOnOff() && ledMode == LED_SPEED && powerBurst <= 1 && millis() - lastStep > 300) {
    lastStep = millis();
    setLeds(&SPEED_LED[step++ % 6], 1);
  } else if (commissioned && !wasCommissioned) {
    showLeds(Fan.getOnOff(), Fan.getSpeedPercent());
    pairedAt = millis();
  }
  wasCommissioned = commissioned;
  // Free BLE once the phone has moved to WiFi (or, as a fallback, a minute after the fabric is added).
  if (bleLinkClosed || (commissioned && pairedAt && millis() - pairedAt > 60000)) {
    bleLinkClosed = false;
    pairedAt = 0;
    releaseBle();
  }
  // Not at once: a factory reset (decommission, removal from Home) wipes pairing data and WiFi credentials
  // on the Matter task and then restarts by itself. Restarting under it left the WiFi credentials behind.
  static uint32_t rebootForBleAt = 0;
  if (restartAfterWifiCleanup) {
    Log.println("restarting clean for pairing");
    delay(200);
    ESP.restart();
  }
  if (rebootForBle && !rebootForBleAt) rebootForBleAt = millis() | 1;
  if (rebootForBleAt && millis() - rebootForBleAt > 5000) {
    Log.println("not paired and bluetooth is off: rebooting to advertise again");
    delay(200);
    ESP.restart();
  }

  uint8_t cmd;
  while (xQueueReceive(remoteQueue, &cmd, 0) == pdTRUE) handleRemote(cmd);

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleLine(serialLine); serialLine = ""; }
    else if (c != '\r' && serialLine.length() < 320) serialLine += c;  // factory set lines are ~200 chars
  }
  devPollCommands(handleNetworkLine);
  motorRampTick();
  motorGuardTick();
  telemetryTick();
  heapDiagTick();
  fanStateTick();
  fanModeTick();
  wifiPowerTick();
  pairingWindowTick();
  routerSolicitTick();
  mdnsIpv6OnlyTick();
  checkPowerBurstTimeout();
  delay(10);
}
