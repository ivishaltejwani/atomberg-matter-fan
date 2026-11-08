// Bench firmware for the Atomberg fan board (ESP32-C3-MINI-1):
// WiFi + ArduinoOTA + telnet terminal for testing LEDs, IR and the MCF8316C pins.
// Arduino-ESP32 core 3.x, board "ESP32C3 Dev Module", USB CDC On Boot: Disabled.

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Preferences.h>
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"

static const char *HOSTNAME = "atomberg-fan";

// ---- LED matrix: LED lights when its row AND column are HIGH ----
static const uint8_t ROWS[2] = {5, 6};
static const uint8_t COLS[3] = {1, 4, 10};
struct Led { uint8_t d, row, col; };
static const Led LEDS[] = {
  {9, 0, 0}, {14, 0, 1}, {12, 0, 2},   // row GPIO5
  {8, 1, 0}, {13, 1, 1}, {11, 1, 2},   // row GPIO6
};
static volatile uint8_t ledMask[2] = {0, 0};  // per row, bit = column

static void setLeds(const uint8_t *ds, size_t n) {
  uint8_t m[2] = {0, 0};
  for (size_t i = 0; i < n; i++)
    for (const Led &l : LEDS)
      if (l.d == ds[i]) m[l.row] |= 1 << l.col;
  ledMask[0] = m[0];
  ledMask[1] = m[1];
}

// ---- fan state driven by the remote ----
// Speed n lights speedLed[n-1]. Default guessed from the PCB photo (clockwise around U7).
static uint8_t speedLed[6] = {12, 11, 9, 8, 14, 13};
static Preferences prefs;
static volatile bool fanOn = false;
static volatile uint8_t fanSpeed = 1;
static volatile bool ledDisplay = true;  // LED button blanks the indicator, fan state unchanged

static void showFan() {
  if (fanOn && ledDisplay) setLeds(&speedLed[fanSpeed - 1], 1);
  else setLeds(nullptr, 0);
}

// Atomberg remote: NEC, 16-bit address 0xF300.
static const uint16_t REMOTE_ADDR = 0xF300;
struct Button { uint8_t cmd; const char *name; int8_t speed; };
static const Button BUTTONS[] = {
  {0x91, "power", 0}, {0x8B, "1", 1}, {0x90, "2", 2}, {0x8A, "3", 3}, {0x93, "4", 4},
  {0x88, "5", 5}, {0x8F, "6", 6}, {0x96, "timer", -1}, {0x97, "led", -2}, {0x8E, "sleep", -1},
};

static void postMsg(const char *fmt, ...);

static void onRemote(uint32_t v) {
  if ((v & 0xFFFF) != REMOTE_ADDR) return;
  uint8_t cmd = v >> 16;
  for (const Button &b : BUTTONS) {
    if (b.cmd != cmd) continue;
    if (b.speed > 0) { fanSpeed = b.speed; fanOn = true; }
    else if (b.speed == 0) fanOn = !fanOn;
    else if (b.speed == -2) ledDisplay = !ledDisplay;
    showFan();
    postMsg("REMOTE: %s -> fan %s, speed %u, display %s", b.name, fanOn ? "ON" : "OFF", fanSpeed,
            ledDisplay ? "on" : "off");
    return;
  }
  postMsg("REMOTE: unknown cmd 0x%02X", cmd);
}

static void ledTask(void *) {
  for (uint8_t r = 0;; r ^= 1) {
    digitalWrite(ROWS[0], LOW);
    digitalWrite(ROWS[1], LOW);
    for (int c = 0; c < 3; c++) digitalWrite(COLS[c], (ledMask[r] >> c) & 1);
    if (ledMask[r]) digitalWrite(ROWS[r], HIGH);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ---- IR receiver on GPIO3 (active-low output) ----
static const int IR_PIN = 3;
static const size_t IR_MAX = 96;
static QueueHandle_t msgQueue;
static volatile bool irPrint = true;
static volatile bool irVerbose = false;  // raw NEC frames and repeats

static bool near(uint32_t v, uint32_t t) { return v > t * 7 / 10 && v < t * 13 / 10; }

static void postMsg(const char *fmt, ...) {
  char *buf = (char *)malloc(512);
  if (!buf) return;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, 512, fmt, ap);
  va_end(ap);
  if (xQueueSend(msgQueue, &buf, 0) != pdTRUE) free(buf);
}

static void irTask(void *) {
  static rmt_data_t s[IR_MAX];
  uint32_t frame = 0;
  for (;;) {
    size_t n = IR_MAX;
    // No timeout: in core 3.x a timed-out rmtRead leaves the RX channel disabled.
    if (!rmtRead(IR_PIN, s, &n, RMT_WAIT_FOR_EVER) || n < 2 || !irPrint) continue;
    frame++;
    if (s[0].level0 == 0 && near(s[0].duration0, 9000) && near(s[0].duration1, 2250)) {
      if (irVerbose) postMsg("IR #%lu NEC repeat", frame);
      continue;
    }
    bool nec = s[0].level0 == 0 && near(s[0].duration0, 9000) && near(s[0].duration1, 4500) && n >= 33;
    uint32_t v = 0;
    for (int i = 0; nec && i < 32; i++) {
      if (!near(s[1 + i].duration0, 560)) nec = false;
      else if (near(s[1 + i].duration1, 1690)) v |= 1UL << i;
      else if (!near(s[1 + i].duration1, 560)) nec = false;
    }
    if (nec) {
      uint8_t a = v, na = v >> 8, c = v >> 16, nc = v >> 24;
      bool ok = (uint8_t)(c ^ nc) == 0xFF;
      if (irVerbose)
        postMsg("IR #%lu NEC addr=0x%02X/0x%02X cmd=0x%02X/0x%02X raw=0x%08lX%s", frame, a, na, c, nc, v,
                ok ? "" : " (cmd check FAILED)");
      if (ok) onRemote(v);
      continue;
    }
    String raw;
    for (size_t i = 0; i < n; i++)
      raw += String(s[i].level0 ? '+' : '-') + s[i].duration0 + "/" + s[i].duration1 + " ";
    postMsg("IR #%lu UNKNOWN symbols=%u: %s", frame, (unsigned)n, raw.c_str());
  }
}

// ---- terminal ----
static WiFiServer server(23);
static WiFiClient client;
static String line, serialLine;

static void out(const String &s) {
  Serial.println(s);
  if (client && client.connected()) client.println(s);
}

// MCF8316C side and spare pins; LED and IR pins are owned by their tasks.
static const uint8_t FREE_PINS[] = {0, 2, 7, 8, 18, 19};
static bool isFree(int p) {
  for (uint8_t f : FREE_PINS) if (f == p) return true;
  return false;
}

static void cmdScan() {
  for (uint8_t p : FREE_PINS) {
    pinMode(p, INPUT_PULLDOWN); delay(5); int dn = digitalRead(p);
    pinMode(p, INPUT_PULLUP);   delay(5); int up = digitalRead(p);
    pinMode(p, INPUT);
    out("GPIO" + String(p) + " pd=" + dn + " pu=" + up + "  " +
        (dn && up ? "held HIGH" : !dn && !up ? "held LOW" : "floating"));
  }
}

static uint32_t heldPins = 0;  // free pins driven by 'gpio set'; i2c all leaves them alone

static int i2cScanPair(uint8_t sda, uint8_t scl, bool quiet) {
  Wire.begin(sda, scl, 100000);
  Wire.setTimeOut(20);
  uint8_t acks[127];
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) acks[found++] = a;
  }
  Wire.end();
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  String pair = "SDA=GPIO" + String(sda) + " SCL=GPIO" + scl;
  if (found > 8) {
    if (!quiet) out(pair + ": " + found + " ACKs, SDA stuck low (not a bus)");
    return 0;
  }
  for (int i = 0; i < found; i++) out(pair + " ACK at 0x" + String(acks[i], HEX));
  if (!found && !quiet) out(pair + ": no devices");
  return found;
}

// "i2c scan" = GPIO2/8 both ways, "i2c scan <sda> <scl>", "i2c all" = every ordered pair of free pins.
static void cmdI2c(const String &rest) {
  int sda = -1, scl = -1;
  if (rest == "all") {
    int total = 0;
    for (uint8_t a : FREE_PINS)
      for (uint8_t b : FREE_PINS)
        if (a != b && !(heldPins >> a & 1) && !(heldPins >> b & 1)) total += i2cScanPair(a, b, true);
    out("i2c all: " + String(total) + " ACK(s)");
  } else if (sscanf(rest.c_str(), "scan %d %d", &sda, &scl) == 2 && isFree(sda) && isFree(scl) && sda != scl) {
    i2cScanPair(sda, scl, false);
  } else {
    i2cScanPair(2, 8, false);
    i2cScanPair(8, 2, false);
  }
}

// ---- MCF8316C (U5), read-only ----
// I2C target 0x01, SDA=GPIO19, SCL=GPIO18. 24-bit control word: CW23 read, CW22 CRC_EN,
// CW21-20 DLEN (01b = 32-bit), CW19-12 section/page (0), CW11-0 address. Data comes back LSB first.
static const uint8_t MCF_ADDR = 0x01, MCF_SDA = 19, MCF_SCL = 18;
static const uint8_t WAKE_PIN = 0;  // SPEED/WAKE: the MCF only answers I2C while this is high

static bool mcfRead32(uint16_t reg, uint32_t &val) {
  uint32_t cw = (1UL << 23) | (1UL << 20) | (reg & 0xFFF);
  Wire.beginTransmission(MCF_ADDR);
  Wire.write(cw >> 16);
  Wire.write(cw >> 8);
  Wire.write(cw);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom(MCF_ADDR, (uint8_t)4) != 4) return false;
  val = 0;
  for (int i = 0; i < 4; i++) val |= (uint32_t)Wire.read() << (8 * i);
  return true;
}

static bool mcfWrite32(uint16_t reg, uint32_t val) {
  uint32_t cw = (1UL << 20) | (reg & 0xFFF);  // write, 32-bit, no CRC
  Wire.beginTransmission(MCF_ADDR);
  Wire.write(cw >> 16);
  Wire.write(cw >> 8);
  Wire.write(cw);
  for (int i = 0; i < 4; i++) Wire.write(val >> (8 * i));
  return Wire.endTransmission() == 0;
}

// ALGO_DEBUG1 (0xEC): bit31 OVERRIDE, bits30-16 DIGITAL_SPEED_CTRL (32767 = 100%), lower bits left 0.
static const uint16_t MCF_ALGO_DEBUG1 = 0xEC;
static const float SPEED_MAX_PCT = 10.0f;     // bench safety caps
static const uint32_t SPEED_MAX_MS = 2000;

static bool mcfSetSpeed(float pct) {
  uint32_t raw = (uint32_t)(pct / 100.0f * 32767.0f) & 0x7FFF;
  return mcfWrite32(MCF_ALGO_DEBUG1, (1UL << 31) | (raw << 16));
}

static const char *mcfName(uint16_t r) {
  switch (r) {
    case 0x80: return "ISD_CONFIG";       case 0x82: return "REV_DRIVE_CONFIG";
    case 0x84: return "MOTOR_STARTUP1";   case 0x86: return "MOTOR_STARTUP2";
    case 0x88: return "CLOSED_LOOP1";     case 0x8A: return "CLOSED_LOOP2";
    case 0x8C: return "CLOSED_LOOP3";     case 0x8E: return "CLOSED_LOOP4";
    case 0x90: return "FAULT_CONFIG1";    case 0x92: return "FAULT_CONFIG2";
    case 0x94: return "REF_PROFILES1";    case 0x96: return "REF_PROFILES2";
    case 0x98: return "REF_PROFILES3";    case 0x9A: return "REF_PROFILES4";
    case 0x9C: return "REF_PROFILES5";    case 0x9E: return "REF_PROFILES6";
    case 0xA0: return "INT_ALGO_1";       case 0xA2: return "INT_ALGO_2";
    case 0xA4: return "PIN_CONFIG";       case 0xA6: return "DEVICE_CONFIG1";
    case 0xA8: return "DEVICE_CONFIG2";   case 0xAA: return "PERI_CONFIG1";
    case 0xAC: return "GD_CONFIG1";       case 0xAE: return "GD_CONFIG2";
    case 0xE0: return "GATE_DRIVER_FAULT_STATUS"; case 0xE2: return "CONTROLLER_FAULT_STATUS";
    case 0xE4: return "ALGO_STATUS";      case 0xE6: return "MTR_PARAMS";
    case 0xE8: return "ALGO_STATUS_MPET";
    default: return "";
  }
}

static void mcfPrint(uint16_t reg) {
  uint32_t v;
  char buf[80];
  if (mcfRead32(reg, v)) snprintf(buf, sizeof buf, "MCF 0x%03X = 0x%08lX  %s", reg, v, mcfName(reg));
  else snprintf(buf, sizeof buf, "MCF 0x%03X = READ FAILED  %s", reg, mcfName(reg));
  out(buf);
}

static void cmdMcf(const String &rest) {
  Wire.begin(MCF_SDA, MCF_SCL, 100000);
  Wire.setTimeOut(50);
  if (rest == "dump") {
    out("MCF dump begin (EEPROM shadow 0x80-0xAE, status 0xE0-0xE8)");
    for (uint16_t r = 0x80; r <= 0xAE; r += 2) mcfPrint(r);
    for (uint16_t r = 0xE0; r <= 0xE8; r += 2) mcfPrint(r);
    out("MCF dump end");
  } else if (rest.startsWith("read ")) {
    mcfPrint(strtoul(rest.c_str() + 5, nullptr, 0));
  } else if (rest == "stop") {
    out(mcfSetSpeed(0) ? "MCF speed 0 (stop)" : "MCF stop write FAILED");
  } else if (rest.startsWith("speed ")) {
    float pct = -1;
    long ms = -1;
    sscanf(rest.c_str() + 6, "%f %ld", &pct, &ms);
    if (pct < 0 || pct > SPEED_MAX_PCT || ms < 0 || ms > (long)SPEED_MAX_MS) {
      out("refused: mcf speed <0-10 %> <0-2000 ms>");
    } else if (!mcfSetSpeed(pct)) {
      out("MCF speed write FAILED");
    } else {
      out("MCF speed " + String(pct, 1) + "% for " + ms + " ms");
      delay(ms);
      bool ok = mcfSetSpeed(0);
      out(ok ? "MCF speed 0 (stop)" : "MCF STOP WRITE FAILED - cut 24 V");
      delay(200);
      for (uint16_t r = 0xE0; r <= 0xE4; r += 2) mcfPrint(r);
    }
  } else {
    out("usage: mcf dump | read <addr> | speed <pct> <ms> | stop");
  }
  Wire.end();
  pinMode(MCF_SDA, INPUT);
  pinMode(MCF_SCL, INPUT);
}

static void cmdHelp() {
  out("commands:");
  out("  info                      IP, RSSI, uptime, heap");
  out("  led all | off | <D#...>   e.g. 'led 8 13' (D8 D9 D11 D12 D13 D14)");
  out("  ir on | off | raw         remote handling on/off; 'raw' also prints NEC frames");
  out("  ledorder [D1 .. D6]       show or set which LED is speed 1..6 (saved)");
  out("  fan on | off | <1-6>      same as the remote");
  out("  scan                      pull-up/down read of free pins 0 2 7 8 18 19");
  out("  gpio read <n>             read a free pin");
  out("  gpio set <n> <0|1>        drive a free pin");
  out("  gpio in <n> [up|down]     release a free pin to input");
  out("  i2c scan [sda scl]        scan GPIO2/8 both ways, or the given pins");
  out("  i2c all                   scan every pair of free pins");
  out("  wake on | off             GPIO0 (MCF SPEED/WAKE) high / released");
  out("  mcf dump | read <addr>    read MCF8316C registers");
  out("  mcf speed <pct> <ms>      run at pct (max 10) for ms (max 2000), then stop");
  out("  mcf stop                  speed command 0");
  out("  reboot");
  out("  dl                      reboot into ROM download mode (for USB flashing)");
}

static void handleLine(String cmd) {
  cmd.trim();
  if (cmd.isEmpty()) return;
  int sp1 = cmd.indexOf(' ');
  String head = sp1 < 0 ? cmd : cmd.substring(0, sp1);
  String rest = sp1 < 0 ? "" : cmd.substring(sp1 + 1);
  rest.trim();

  if (head == "help") cmdHelp();
  else if (head == "info")
    out("ip=" + WiFi.localIP().toString() + " rssi=" + WiFi.RSSI() + "dBm uptime=" + millis() / 1000 +
        "s heap=" + ESP.getFreeHeap());
  else if (head == "led") {
    uint8_t ds[6]; size_t n = 0;
    if (rest == "all") { for (const Led &l : LEDS) ds[n++] = l.d; }
    else if (rest != "off") {
      while (rest.length() && n < 6) {
        int sp = rest.indexOf(' ');
        ds[n++] = (sp < 0 ? rest : rest.substring(0, sp)).toInt();
        rest = sp < 0 ? "" : rest.substring(sp + 1);
        rest.trim();
      }
    }
    setLeds(ds, n);
    out("ok");
  }
  else if (head == "ir") {
    irPrint = rest != "off";
    irVerbose = rest == "raw";
    out(!irPrint ? "IR off" : irVerbose ? "IR on, raw frames" : "IR on");
  }
  else if (head == "ledorder") {
    int d[6];
    if (rest.length() &&
        sscanf(rest.c_str(), "%d %d %d %d %d %d", &d[0], &d[1], &d[2], &d[3], &d[4], &d[5]) == 6) {
      for (int i = 0; i < 6; i++) speedLed[i] = d[i];
      prefs.putBytes("ledorder", speedLed, 6);
      showFan();
    }
    String s = "speed 1..6 -> ";
    for (uint8_t x : speedLed) s += "D" + String(x) + " ";
    out(s);
  }
  else if (head == "fan") {
    if (rest == "on") fanOn = true;
    else if (rest == "off") fanOn = false;
    else if (rest.toInt() >= 1 && rest.toInt() <= 6) { fanSpeed = rest.toInt(); fanOn = true; }
    showFan();
    out(String("fan ") + (fanOn ? "ON" : "OFF") + ", speed " + fanSpeed);
  }
  else if (head == "scan") cmdScan();
  else if (head == "i2c") cmdI2c(rest);
  else if (head == "mcf") cmdMcf(rest);
  else if (head == "wake") {
    if (rest == "on") {
      pinMode(WAKE_PIN, OUTPUT); digitalWrite(WAKE_PIN, HIGH); heldPins |= 1u << WAKE_PIN;
      out("GPIO0 (SPEED/WAKE) HIGH");
    } else {
      pinMode(WAKE_PIN, INPUT); heldPins &= ~(1u << WAKE_PIN);
      out("GPIO0 (SPEED/WAKE) released");
    }
  }
  else if (head == "gpio") {
    char op[8] = {0}, arg[8] = {0};
    int p = -1;
    sscanf(rest.c_str(), "%7s %d %7s", op, &p, arg);
    if (!isFree(p)) { out("pin must be one of 0 2 7 8 18 19"); return; }
    if (!strcmp(op, "read")) out("GPIO" + String(p) + "=" + digitalRead(p));
    else if (!strcmp(op, "set")) {
      pinMode(p, OUTPUT); digitalWrite(p, atoi(arg)); heldPins |= 1u << p; out("ok");
    }
    else if (!strcmp(op, "in")) {
      pinMode(p, !strcmp(arg, "up") ? INPUT_PULLUP : !strcmp(arg, "down") ? INPUT_PULLDOWN : INPUT);
      heldPins &= ~(1u << p);
      out("ok");
    }
    else out("usage: gpio read|set|in <n> ...");
  }
  else if (head == "reboot") { out("rebooting"); delay(100); ESP.restart(); }
  else if (head == "dl") {
    // The dev kit can't pulse our EN, so ask the ROM for download mode on the next reset.
    out("entering download mode");
    delay(100);
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
  }
  else out("unknown command, try 'help'");
}

void setup() {
  Serial.begin(115200);
  for (uint8_t p : ROWS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  for (uint8_t p : COLS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  for (uint8_t p : FREE_PINS) pinMode(p, INPUT);

  prefs.begin("fan_lab");
  prefs.getBytes("ledorder", speedLed, 6);
  msgQueue = xQueueCreate(32, sizeof(char *));
  xTaskCreate(ledTask, "led", 2048, nullptr, 2, nullptr);
  if (rmtInit(IR_PIN, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, 1000000)) {
    rmtSetRxMaxThreshold(IR_PIN, 12000);
    xTaskCreate(irTask, "ir", 4096, nullptr, 1, nullptr);
  } else {
    Serial.println("rmtInit failed");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("\nfan_lab: connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    static int i = 0;
    setLeds(&speedLed[i++ % 6], 1);  // chase in speed order while connecting
    Serial.print('.');
    delay(250);
  }
  Serial.printf("\nconnected, ip=%s\n", WiFi.localIP().toString().c_str());

  MDNS.begin(HOSTNAME);
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  server.begin();
  server.setNoDelay(true);

  uint8_t all[] = {8, 9, 11, 12, 13, 14};
  setLeds(all, 6);
  delay(2000);
  showFan();
}

void loop() {
  ArduinoOTA.handle();

  if (server.hasClient()) {
    if (client) client.stop();
    client = server.accept();
    client.println("=== atomberg-fan lab terminal, type 'help' ===");
  }
  while (client && client.available()) {
    char c = client.read();
    if (c == '\n') { handleLine(line); line = ""; }
    else if (c != '\r' && line.length() < 120) line += c;
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleLine(serialLine); serialLine = ""; }
    else if (c != '\r' && serialLine.length() < 120) serialLine += c;
  }

  char *msg;
  while (xQueueReceive(msgQueue, &msg, 0) == pdTRUE) {
    out(msg);
    free(msg);
  }

  if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  delay(5);
}
