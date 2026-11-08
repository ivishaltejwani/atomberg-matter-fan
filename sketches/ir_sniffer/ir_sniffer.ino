// IR sniffer for the Atomberg fan board: IR receiver (U7) output on GPIO3.
// Arduino-ESP32 core 3.x, board "ESP32C3 Dev Module", USB CDC On Boot: Disabled.
// Prints NEC frames decoded, anything else as raw mark/space timings (us).

#include <Arduino.h>

static const int IR_PIN = 3;
static const size_t MAX_SYMBOLS = 96;  // 2 RMT memory blocks on the C3

static rmt_data_t symbols[MAX_SYMBOLS];
static uint32_t frameNo = 0;

static bool near(uint32_t v, uint32_t target) {
  return v > target * 7 / 10 && v < target * 13 / 10;
}

// Receiver output is active-low: level 0 = carrier present (mark).
static bool decodeNec(const rmt_data_t *s, size_t n, uint32_t &value, bool &repeat) {
  repeat = false;
  if (n < 1 || s[0].level0 != 0 || !near(s[0].duration0, 9000)) return false;
  if (near(s[0].duration1, 2250)) { repeat = true; return true; }
  if (!near(s[0].duration1, 4500) || n < 33) return false;
  value = 0;
  for (int i = 0; i < 32; i++) {
    const rmt_data_t &b = s[1 + i];
    if (!near(b.duration0, 560)) return false;
    if (near(b.duration1, 1690)) value |= 1UL << i;  // LSB first
    else if (!near(b.duration1, 560)) return false;
  }
  return true;
}

static uint32_t rawHash(const rmt_data_t *s, size_t n) {
  // Quantise to 100 us so repeated presses of one button hash the same.
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) {
    h = (h ^ ((s[i].duration0 + 50) / 100)) * 16777619u;
    h = (h ^ ((s[i].duration1 + 50) / 100)) * 16777619u;
  }
  return h;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!rmtInit(IR_PIN, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, 1000000)) {  // 1 tick = 1 us
    Serial.println("rmtInit failed");
    while (true) delay(1000);
  }
  rmtSetRxMaxThreshold(IR_PIN, 12000);  // 12 ms of silence ends a frame
  Serial.println("\n=== IR sniffer on GPIO3: press each remote button once ===");
}

void loop() {
  size_t n = MAX_SYMBOLS;
  if (!rmtRead(IR_PIN, symbols, &n, 1000) || n < 2) return;

  frameNo++;
  uint32_t value;
  bool repeat;
  if (decodeNec(symbols, n, value, repeat)) {
    if (repeat) {
      Serial.printf("#%lu NEC repeat\n", frameNo);
    } else {
      uint8_t a = value, na = value >> 8, c = value >> 16, nc = value >> 24;
      Serial.printf("#%lu NEC addr=0x%02X ~addr=0x%02X cmd=0x%02X ~cmd=0x%02X raw=0x%08lX%s\n",
                    frameNo, a, na, c, nc, value,
                    (uint8_t)(c ^ nc) == 0xFF ? "" : "  (cmd check FAILED)");
    }
    return;
  }

  Serial.printf("#%lu UNKNOWN symbols=%u hash=0x%08lX\n  ", frameNo, (unsigned)n, rawHash(symbols, n));
  for (size_t i = 0; i < n; i++) {
    Serial.printf("%c%u/%u ", symbols[i].level0 ? '+' : '-', symbols[i].duration0, symbols[i].duration1);
    if (i % 8 == 7) Serial.print("\n  ");
  }
  Serial.println();
}
