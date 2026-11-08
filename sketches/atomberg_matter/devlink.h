// Dev link over WiFi, for a fan with no serial cable: OTA updates, log forwarding and a command line.
// Everything is started by the fan (pull), because the office network doesn't pass unicast from the
// wired Mac to WiFi clients. Talks to tools/devlink.py on a "dev host" stored in NVS ('devhost <ip>').
// No dev host set (the default, e.g. a fresh GitHub build) = nothing runs.
//
//   fan -> host:8070  HTTP  GET /fw.manifest (signed, 112 B), GET /fw.bin
//   fan -> host:8071  UDP   log lines + a heartbeat every 10 s (from fan port 8072)
//   host:8071 -> fan:8072 UDP  command lines (replies to the fan's own traffic)
//
// OTA security: the network is treated as hostile. The fan only installs an image whose manifest
//   "ATOMOTA1" | version u32 | length u32 | SHA-256 of fw.bin (32) | ECDSA P-256 r||s over the first 48 B
// verifies against the public key compiled in from ota_pubkey.h (made by 'tools/devlink.py keygen'; the
// private key never leaves the Mac). The version must be higher than the last one installed, which blocks
// replaying old signed builds and retrying a build that was rolled back. The downloaded image must match
// the signed length and SHA-256 before it can become the boot partition. No ota_pubkey.h = no OTA.
// Network commands are limited to read-only ones by the sketch (see handleNetworkLine).
//
// Crash safety: the bootloader has rollback enabled. A new image boots "pending verify" and is only
// marked valid after DEVLINK_HEALTHY_MS of uptime with WiFi up; a crash or reboot before that rolls back.
#pragma once
#include <Arduino.h>
#include <nvs.h>
#include "esp_app_desc.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/sha256.h"
#if __has_include("ota_pubkey.h")
#include "ota_pubkey.h"  // static const uint8_t OTA_PUBKEY[65]: uncompressed P-256 point
#define DEVLINK_SIGNED_OTA 1
#else
#define DEVLINK_SIGNED_OTA 0
#endif

static const uint16_t DEVLINK_HTTP_PORT = 8070, DEVLINK_LOG_PORT = 8071, DEVLINK_CMD_PORT = 8072;
static const uint32_t DEVLINK_CHECK_MS = 30000, DEVLINK_HEALTHY_MS = 60000;

static char devHost[40] = "";
static int devSock = -1;
static sockaddr_in devAddr = {};
static SemaphoreHandle_t devLock;

static bool devWifiUp() {
  wifi_ap_record_t ap;
  return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
}

static void devSend(const char *buf, size_t len) {
  if (devSock < 0 || !len) return;
  sendto(devSock, buf, len, 0, (sockaddr *)&devAddr, sizeof devAddr);
}

// Log text is only buffered here; devTask sends it. Logs can come from the WiFi/lwIP tasks themselves,
// and calling sendto() from those could deadlock.
static char devRing[2048];
static size_t devRingHead = 0, devRingLen = 0;  // guarded by devLock

static void devQueue(const char *data, size_t n) {
  if (devSock < 0 || !devLock || xSemaphoreTake(devLock, 0) != pdTRUE) return;
  for (size_t i = 0; i < n && devRingLen < sizeof devRing; i++, devRingLen++)
    devRing[(devRingHead + devRingLen) % sizeof devRing] = data[i];
  xSemaphoreGive(devLock);
}

static void devFlush() {
  static char out[512];
  for (;;) {
    size_t n = 0;
    xSemaphoreTake(devLock, portMAX_DELAY);
    while (n < sizeof out && devRingLen) {
      out[n++] = devRing[devRingHead];
      devRingHead = (devRingHead + 1) % sizeof devRing;
      devRingLen--;
    }
    xSemaphoreGive(devLock);
    if (!n) return;
    devSend(out, n);
  }
}

// Print that goes to Serial and, line by line, to the dev host. Used everywhere instead of Serial.
class NetLog : public Print {
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *data, size_t n) override {
    Serial.write(data, n);
    devQueue((const char *)data, n);
    return n;
  }
};
static NetLog Log;

// ESP-IDF / Matter logs (the "E (1234) chip[..]" lines) go to UART and the dev host too.
static vprintf_like_t devOrigVprintf;
static int devVprintf(const char *fmt, va_list args) {
  if (devSock >= 0) {
    char buf[128];  // small: this runs on whatever task logged
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(buf, sizeof buf, fmt, copy);
    va_end(copy);
    if (n > 0) devQueue(buf, min((size_t)n, sizeof buf - 1));
  }
  return devOrigVprintf(fmt, args);
}

static void devOpenSocket() {
  if (devSock >= 0 || !devHost[0]) return;
  int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return;
  sockaddr_in local = {};
  local.sin_family = AF_INET;
  local.sin_port = htons(DEVLINK_CMD_PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  int nb = 1;
  if (bind(s, (sockaddr *)&local, sizeof local) < 0 || lwip_ioctl(s, FIONBIO, &nb) < 0) { close(s); return; }
  devAddr.sin_family = AF_INET;
  devAddr.sin_port = htons(DEVLINK_LOG_PORT);
  inet_pton(AF_INET, devHost, &devAddr.sin_addr);
  devSock = s;
}

static void devAppSha(char out[65]) {
  const uint8_t *sha = esp_app_get_description()->app_elf_sha256;
  for (int i = 0; i < 32; i++) sprintf(out + 2 * i, "%02x", sha[i]);
}

// Minimal HTTP/1.0 GET over a plain socket (esp_http_client pulls in esp-tls: +130 KB of flash).
// Returns the socket positioned at the body and the Content-Length, or -1.
static int devHttpOpen(const char *path, int *contentLength) {
  int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0) return -1;
  timeval tv = {10, 0};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
  sockaddr_in a = devAddr;
  a.sin_port = htons(DEVLINK_HTTP_PORT);
  char req[128];
  int n = snprintf(req, sizeof req, "GET %s HTTP/1.0\r\nHost: %s\r\n\r\n", path, devHost);
  if (connect(s, (sockaddr *)&a, sizeof a) < 0 || send(s, req, n, 0) != n) { close(s); return -1; }
  // Read the header byte by byte up to the blank line (it is small).
  char hdr[512];
  int len = 0;
  while (len < (int)sizeof hdr - 1) {
    if (recv(s, hdr + len, 1, 0) != 1) { close(s); return -1; }
    len++;
    if (len >= 4 && memcmp(hdr + len - 4, "\r\n\r\n", 4) == 0) break;
  }
  hdr[len] = 0;
  const char *cl = strcasestr(hdr, "Content-Length:");
  if (strncmp(hdr, "HTTP/1.", 7) != 0 || atoi(hdr + 9) != 200 || !cl) { close(s); return -1; }
  *contentLength = atoi(cl + 15);
  return s;
}

static int devRecvAll(int s, char *buf, int want) {
  int got = 0;
  while (got < want) {
    int n = recv(s, buf + got, want - got, 0);
    if (n <= 0) break;
    got += n;
  }
  return got;
}

// GET http://devHost:8070<path> into buf (small responses only).
static int devHttpGetSmall(const char *path, char *buf, int size) {
  int total;
  int s = devHttpOpen(path, &total);
  if (s < 0) return -1;
  int n = devRecvAll(s, buf, min(total, size - 1));
  close(s);
  buf[n] = 0;
  return n;
}

static void (*devBeforeUpdate)() = nullptr;

static const size_t OTA_MANIFEST_BODY = 48, OTA_MANIFEST_SIZE = 112;

// Last manifest version installed (stored before rebooting into it, so a rolled-back build is never retried).
static uint32_t devOtaVersion() {
  nvs_handle_t h;
  uint32_t v = 0;
  if (nvs_open("devlink", NVS_READONLY, &h) == ESP_OK) {
    nvs_get_u32(h, "otaver", &v);
    nvs_close(h);
  }
  return v;
}

static void devSetOtaVersion(uint32_t v) {
  nvs_handle_t h;
  if (nvs_open("devlink", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_u32(h, "otaver", v);
    nvs_commit(h);
    nvs_close(h);
  }
}

#if DEVLINK_SIGNED_OTA
static bool devVerifyManifest(const uint8_t *m) {
  uint8_t hash[32];
  if (mbedtls_sha256(m, OTA_MANIFEST_BODY, hash, 0) != 0) return false;
  mbedtls_ecp_group grp;
  mbedtls_ecp_point q;
  mbedtls_mpi r, s;
  mbedtls_ecp_group_init(&grp);
  mbedtls_ecp_point_init(&q);
  mbedtls_mpi_init(&r);
  mbedtls_mpi_init(&s);
  bool ok = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1) == 0 &&
            mbedtls_ecp_point_read_binary(&grp, &q, OTA_PUBKEY, sizeof OTA_PUBKEY) == 0 &&
            mbedtls_mpi_read_binary(&r, m + OTA_MANIFEST_BODY, 32) == 0 &&
            mbedtls_mpi_read_binary(&s, m + OTA_MANIFEST_BODY + 32, 32) == 0 &&
            mbedtls_ecdsa_verify(&grp, hash, sizeof hash, &q, &r, &s) == 0;
  mbedtls_mpi_free(&r);
  mbedtls_mpi_free(&s);
  mbedtls_ecp_point_free(&q);
  mbedtls_ecp_group_free(&grp);
  return ok;
}
#endif

// Downloads fw.bin into the spare slot; it only becomes bootable if it matches the signed length and SHA-256.
// RETRY = network trouble, try again later. REJECT = the server sent something that doesn't match the signed
// manifest: never retry that manifest (a hostile server could otherwise wear the flash out with rewrites).
enum DevOtaResult { DEV_OTA_OK, DEV_OTA_RETRY, DEV_OTA_REJECT };

static DevOtaResult devDownloadAndFlash(uint32_t length, const uint8_t *sha256) {
  const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
  if (!next) { Log.println("ota: no OTA partition (old partition table?)"); return DEV_OTA_REJECT; }
  int total;
  int s = devHttpOpen("/fw.bin", &total);
  if (s < 0) { Log.println("ota: could not fetch fw.bin"); return DEV_OTA_RETRY; }
  esp_ota_handle_t ota = 0;
  if (total != (int)length || total > (int)next->size) {
    Log.printf("ota: REJECTED image: %d bytes, signed manifest says %lu (slot %lu)\n", total, (unsigned long)length,
               (unsigned long)next->size);
    close(s);
    return DEV_OTA_REJECT;
  }
  if (esp_ota_begin(next, total, &ota) != ESP_OK) {
    close(s);
    return DEV_OTA_RETRY;
  }
  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);
  static char chunk[1024];
  int got = 0, lastPct = -10;
  while (got < total) {
    int n = recv(s, chunk, min((int)sizeof chunk, total - got), 0);
    if (n <= 0 || esp_ota_write(ota, chunk, n) != ESP_OK) break;
    mbedtls_sha256_update(&sha, (const uint8_t *)chunk, n);
    got += n;
    int pct = got * 100LL / total;
    if (pct >= lastPct + 10) { Log.printf("ota: %d%%\n", pct); lastPct = pct; }
  }
  close(s);
  uint8_t digest[32];
  mbedtls_sha256_finish(&sha, digest);
  mbedtls_sha256_free(&sha);
  if (got != total) {
    esp_ota_abort(ota);
    Log.printf("ota: FAILED after %d of %d bytes\n", got, total);
    return DEV_OTA_RETRY;
  }
  if (memcmp(digest, sha256, 32) != 0) {
    esp_ota_abort(ota);
    Log.println("ota: REJECTED image, SHA-256 doesn't match the signed manifest");
    return DEV_OTA_REJECT;
  }
  // esp_ota_end also validates the image format before it can become the boot partition.
  if (esp_ota_end(ota) != ESP_OK || esp_ota_set_boot_partition(next) != ESP_OK) {
    Log.println("ota: REJECTED image: signed but not a valid app image");
    return DEV_OTA_REJECT;
  }
  return DEV_OTA_OK;
}

// One OTA check: fetch the manifest, verify it, install if it is a newer signed build.
static void devCheckForUpdate() {
#if DEVLINK_SIGNED_OTA
  static uint8_t last[OTA_MANIFEST_SIZE];
  static bool haveLast = false;
  static uint32_t pausedAt = 0;  // after a rejection, stop checking for an hour (see below)
  if (pausedAt && millis() - pausedAt < 3600000) return;
  pausedAt = 0;
  uint8_t m[OTA_MANIFEST_SIZE + 1];
  int n = devHttpGetSmall("/fw.manifest", (char *)m, sizeof m);
  if (n != (int)OTA_MANIFEST_SIZE || (haveLast && memcmp(m, last, OTA_MANIFEST_SIZE) == 0)) return;
  memcpy(last, m, OTA_MANIFEST_SIZE);
  haveLast = true;
  if (memcmp(m, "ATOMOTA1", 8) != 0 || !devVerifyManifest(m)) {
    Log.println("ota: REJECTED manifest: bad signature (not signed with this fan's key), pausing updates 1 h");
    pausedAt = millis() | 1;
    return;
  }
  uint32_t version, length;
  memcpy(&version, m + 8, 4);
  memcpy(&length, m + 12, 4);
  uint32_t installed = devOtaVersion();
  if (version <= installed) return;  // same build, an older one (replay/downgrade), or one that was rolled back
  Log.printf("ota: signed build v%lu (installed v%lu), updating\n", (unsigned long)version, (unsigned long)installed);
  if (devBeforeUpdate) devBeforeUpdate();
  DevOtaResult r = devDownloadAndFlash(length, m + 16);
  if (r == DEV_OTA_OK) {
    devSetOtaVersion(version);
    Log.println("ota: done, rebooting");
    devFlush();
    delay(300);
    esp_restart();
  }
  if (r == DEV_OTA_RETRY) haveLast = false;  // network trouble: try this manifest again next time
  // Something served doesn't match what was signed. Remembering only the last manifest isn't enough (a
  // server could alternate two), so pause: bounds flash rewrites to one per hour whatever is served.
  if (r == DEV_OTA_REJECT) {
    Log.println("ota: pausing updates for 1 h");
    pausedAt = millis() | 1;
  }
#endif
}

static void devTask(void *) {
  uint32_t lastCheck = 0, lastBeat = 0;
  bool confirmed = false;
  char mySha[65];
  devAppSha(mySha);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
    devFlush();
    bool up = devWifiUp();
    if (up) devOpenSocket();

    // Rollback guard: confirm this image once it has run healthily for a while.
    if (!confirmed && up && millis() > DEVLINK_HEALTHY_MS) {
      confirmed = true;
      esp_ota_img_states_t st;
      if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        Log.println("ota: new firmware confirmed");
      }
    }
    if (!up || devSock < 0) continue;

    if (millis() - lastBeat > 10000) {
      lastBeat = millis();
      Log.printf("hb sha=%.8s up=%lus heap=%lu ota=v%lu%s\n", mySha, millis() / 1000, (unsigned long)ESP.getFreeHeap(),
                 (unsigned long)devOtaVersion(), DEVLINK_SIGNED_OTA ? "" : " (ota disabled: no ota_pubkey.h)");
    }
    // Don't update an unconfirmed image (a rollback would undo it anyway).
    if (!confirmed || millis() - lastCheck < DEVLINK_CHECK_MS) continue;
    lastCheck = millis();
    devCheckForUpdate();
  }
}

// Commands from the dev host, fed to the same handler as the serial terminal. Call from loop().
static void devPollCommands(void (*onLine)(String)) {
  if (devSock < 0) return;
  static char buf[330];
  sockaddr_in from;
  socklen_t fl = sizeof from;
  int n = recvfrom(devSock, buf, sizeof buf - 1, 0, (sockaddr *)&from, &fl);
  if (n <= 0 || from.sin_addr.s_addr != devAddr.sin_addr.s_addr) return;
  buf[n] = 0;
  for (char *line = strtok(buf, "\r\n"); line; line = strtok(nullptr, "\r\n")) onLine(String(line));
}

static void devLoadHost() {
  nvs_handle_t h;
  size_t len = sizeof devHost;
  if (nvs_open("devlink", NVS_READONLY, &h) == ESP_OK) {
    if (nvs_get_str(h, "host", devHost, &len) != ESP_OK) devHost[0] = 0;
    nvs_close(h);
  }
}

// 'devhost <ipv4>' / 'devhost off' / 'devhost'
static void devHostCommand(const String &arg) {
  if (arg.length()) {
    in_addr test;
    bool off = arg == "off";
    if (!off && inet_pton(AF_INET, arg.c_str(), &test) != 1) { Log.println("usage: devhost <ipv4> | off"); return; }
    nvs_handle_t h;
    if (nvs_open("devlink", NVS_READWRITE, &h) == ESP_OK) {
      if (off) nvs_erase_key(h, "host");
      else nvs_set_str(h, "host", arg.c_str());
      nvs_commit(h);
      nvs_close(h);
    }
    Log.printf("dev host %s, rebooting to apply\n", off ? "cleared" : arg.c_str());
    delay(200);
    esp_restart();
  }
  char sha[65];
  devAppSha(sha);
  const esp_partition_t *run = esp_ota_get_running_partition();
  esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(run, &st);
  Log.printf("dev host: %s  link=%s  running %s (state %d)  sha=%.16s  ota=%s v%lu\n", devHost[0] ? devHost : "(none)",
             devSock >= 0 ? "open" : "closed", run ? run->label : "?", (int)st, sha,
             DEVLINK_SIGNED_OTA ? "signed" : "disabled", (unsigned long)devOtaVersion());
}

static void devlinkBegin(void (*beforeUpdate)()) {
  devBeforeUpdate = beforeUpdate;
  devLock = xSemaphoreCreateMutex();
  devLoadHost();
  // Even with no dev host, keep the rollback guard: confirms the image after a healthy minute.
  devOrigVprintf = esp_log_set_vprintf(devVprintf);
  xTaskCreate(devTask, "devlink", 4096, nullptr, 1, nullptr);
}

// Keep the bootloader's rollback armed: Arduino would otherwise mark every new image valid at boot.
extern "C" bool verifyRollbackLater() { return true; }
