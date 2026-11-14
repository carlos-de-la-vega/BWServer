// RTC-scheduled relay with deep sleep. All times are UTC.
//   DS3231 alarm 1 (INT/SQW -> GPIO34) wakes the ESP32 at the next time in config.txt.
//   config.txt  -> activation time(s) UTC, duration, days
//   log.txt     -> timestamped events (UTC)
//   secrets.h   -> WiFi credentials for NTP: #define WIFI_SSID "..." / #define WIFI_PASS "..."
// After a power-on or reset (not after a scheduled wake) the board stays awake in two phases:
//   1. NTP window, NTP_WINDOW_MS: blue LED blinks 1 s on / 1 s off; PB3 runs an NTP sync (station mode, secrets.h).
//   2. Hotspot, AP_DURATION_MS: access point AP_SSID / AP_PASS with an HTTP server (http://192.168.4.1):
//        /set_time=HH:MM (UTC)  -> replaces time_on in config.txt
//        /set_duration=SECONDS  -> replaces duration_s in config.txt
//        /status                -> shows the RTC time (UTC) and the current config values
//        /start_mission         -> closes the hotspot at once and sleeps (skips the remaining time), shows the next activation
//      The green LED blinks 1 s on / 1 s off while the hotspot waits for a command. A valid command blinks it once
//      and stops the blinking; after CMD_GRACE_MS without another command the hotspot closes.
//      With no command the hotspot closes after AP_DURATION_MS and config.txt is left as it was.
//      When the hotspot closes: blue, green, blue, green (1 s each), then the board sleeps.
// Blue LED (active HIGH): blink in the NTP window, on during an NTP sync and 1.5 s after a good one; one 30 ms flash
// every heartbeat_s (config.txt) while asleep = armed and waiting. GPIO0 is a strapping pin, so it is held LOW during sleep.
// Buttons: PB1 manual relay (on/off), PB2 print status, PB3 NTP sync (only in the NTP window).
// WiFi is only ever switched on by PB3, the NTP serial command or the hotspot phase, never on a scheduled wake.
// Serial (115200): SET YYYY-MM-DD HH:MM:SS (UTC) | STATUS | RELOAD | LOG | NTP | RELAY ON | RELAY OFF
//                  CFG HH:MM[,HH:MM...] SECONDS [DAYS] | WIFICFG SSID PASSWORD | SLEEP | AWAKE
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <sys/time.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define RTC_ADDR 0x68
#define RTC_INT_PIN 34           // DS3231 INT/SQW, open drain, low on alarm
#define SD_CS 5

#define RELAY_PIN 13
#define RELAY_ON HIGH            // level at which the relay conducts
#define PB_1 25
#define PB_2 26
#define PB_3 27
#define LED_R 32                 // error (SD / RTC / NTP)
#define LED_G 33                 // relay indicator: steady during the pulse (TEST_LED_MODE 1) or 0.5 s flashes on switch-on/off (0)
#define LED_B 0                  // heartbeat / NTP in progress; active HIGH on this board (popup-buoy's is active LOW)
#define LED_B_ON HIGH

#define CONFIG_FILE "/config.txt"
#define LOG_FILE "/log.txt"
#define SECRETS_FILE "/secrets.h"
#define MAX_TIMES 8

#define NTP_WINDOW_MS 30000UL       // after power-on: time to press PB3, blue LED blinking
#define AP_DURATION_MS 300000UL     // then: hotspot open this long
#define AP_SSID "BW_Server"
#define AP_PASS "server123"         // WPA2 needs 8..63 characters ("server" is too short)
#define CMD_GRACE_MS 10000UL        // after a valid HTTP command, sleep this long after the last one
#define CMD_LED_MS 500              // green blink confirming a command
#define NTP_WIFI_TIMEOUT_MS 20000UL
#define NTP_REPLY_TIMEOUT_MS 10000UL
#define NTP_MIN_EPOCH 1700000000L   // anything below this means SNTP has not answered yet
#define NO_CLOCK_RETRY_S 3600       // sleep length when there is no usable clock or schedule
#define WAKE_FALLBACK_S 5           // timer wake this long after the RTC alarm, in case INT never fires
#define HEARTBEAT_FLASH_MS 30
#define RELAY_LED_FLASH_MS 500
#define TEST_LED_MODE 1             // 1: green on for the whole pulse + blue end sequence (bench test); 0: 0.5 s green flashes

struct Config {
  int times[MAX_TIMES];          // minutes since midnight, UTC
  int nTimes = 0;
  uint32_t durationS = 300;
  uint32_t heartbeatS = 60;      // blue LED flash period while asleep; 0 = no flash
  bool days[8] = {false, true, true, true, true, true, true, true};  // 1=Monday..7=Sunday, UTC
} cfg;

struct DateTime { int y, mo, d, h, mi, s, dow; };

RTC_DATA_ATTR long lastTriggerKey = -1;   // minute (epoch/60) already served; survives deep sleep
RTC_DATA_ATTR bool hbWake = false;        // next timer wake is only a LED flash, not a real wake
RTC_DATA_ATTR uint32_t hbPeriodS = 0;
RTC_DATA_ATTR uint32_t hbRemainingS = 0;  // seconds of the fallback timer still to sleep after the flash

bool sdOk = false, rtcOk = false, rtcLostPower = false;
bool relayOn = false, stayAwake = false;
uint32_t relayStartMs = 0, relayDurMs = 0;
uint32_t awakeUntil = 0;
DateTime now_;

uint8_t dec2bcd(uint8_t v) { return (v / 10) << 4 | (v % 10); }
uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)RTC_ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void writeReg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(reg);
  Wire.write(v);
  Wire.endTransmission();
}

bool readTime(DateTime &t) {
  uint8_t r[7], st;
  if (!readRegs(0x00, r, 7) || !readRegs(0x0F, &st, 1)) return false;
  rtcLostPower = st & 0x80;
  t.s = bcd2dec(r[0] & 0x7F); t.mi = bcd2dec(r[1]); t.h = bcd2dec(r[2] & 0x3F);
  t.dow = r[3] & 0x07; t.d = bcd2dec(r[4]); t.mo = bcd2dec(r[5] & 0x1F); t.y = 2000 + bcd2dec(r[6]);
  return true;
}

void setTime(int y, int mo, int d, int h, int mi, int s) {
  static const int tb[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  int yy = mo < 3 ? y - 1 : y;
  int dow = (yy + yy / 4 - yy / 100 + yy / 400 + tb[mo - 1] + d) % 7;
  if (dow == 0) dow = 7;
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.write(dec2bcd(s)); Wire.write(dec2bcd(mi)); Wire.write(dec2bcd(h));
  Wire.write(dow); Wire.write(dec2bcd(d)); Wire.write(dec2bcd(mo)); Wire.write(dec2bcd(y - 2000));
  Wire.endTransmission();
  uint8_t st;
  if (readRegs(0x0F, &st, 1)) writeReg(0x0F, st & 0x7F);  // clear OSF
}

time_t toEpoch(const DateTime &t) {      // TZ is UTC0, so mktime() is a plain UTC conversion
  struct tm tm = {};
  tm.tm_year = t.y - 1900; tm.tm_mon = t.mo - 1; tm.tm_mday = t.d;
  tm.tm_hour = t.h; tm.tm_min = t.mi; tm.tm_sec = t.s;
  return mktime(&tm);
}

void setTimeEpoch(time_t e) {
  struct tm tm;
  gmtime_r(&e, &tm);
  setTime(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}

String isoUtc(time_t e) {
  struct tm tm;
  gmtime_r(&e, &tm);
  char b[24];
  snprintf(b, sizeof b, "%04d-%02d-%02dT%02d:%02d:%02dZ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  return String(b);
}

bool clockValid() { return rtcOk && !rtcLostPower; }

void logEvent(const String &msg) {
  String line = (clockValid() ? isoUtc(toEpoch(now_)) : String("CLOCK-INVALID")) + " " + msg;
  Serial.println(line);
  if (!sdOk) return;
  File f = SD.open(LOG_FILE, FILE_APPEND);
  if (!f) { sdOk = false; return; }
  f.println(line);
  f.close();
}

bool wokeFromSleep = false;   // scheduled wake: no NTP window and no hotspot
WebServer server(80);
bool apActive = false, hotspotStarted = false, cmdSeen = false, missionStart = false;
uint32_t ntpWindowEnd = 0, apEnd = 0;

void stopHotspot();

bool mountSD() {
  SD.end();
  sdOk = SD.begin(SD_CS);
  return sdOk;
}

bool writeConfig(const String &times, uint32_t durationS, const String &days) {
  SD.remove(CONFIG_FILE);
  File f = SD.open(CONFIG_FILE, FILE_WRITE);
  if (!f) return false;
  f.println("# Relay configuration (all times UTC)");
  f.println("# time_on: one or more HH:MM separated by commas (max 8)");
  f.println("time_on=" + times);
  f.println("# duration_s: seconds the relay conducts");
  f.println("duration_s=" + String(durationS));
  f.println("# days: active weekdays (UTC), 1=Monday ... 7=Sunday");
  f.println("days=" + days);
  f.println("# heartbeat_s: blue LED flash period while asleep, 0 = off (each flash costs a short wake)");
  f.println("heartbeat_s=" + String(cfg.heartbeatS));
  f.close();
  return true;
}

void parseTimes(String v) {
  cfg.nTimes = 0;
  while (v.length() && cfg.nTimes < MAX_TIMES) {
    int c = v.indexOf(',');
    String p = c < 0 ? v : v.substring(0, c);
    v = c < 0 ? "" : v.substring(c + 1);
    int h, m;
    if (sscanf(p.c_str(), "%d:%d", &h, &m) == 2 && h >= 0 && h < 24 && m >= 0 && m < 60)
      cfg.times[cfg.nTimes++] = h * 60 + m;
  }
}

void loadConfig(bool verbose = true) {
  cfg = Config();
  if (!sdOk) return;
  if (!SD.exists(CONFIG_FILE)) { writeConfig("08:00", 300, "1234567"); logEvent("CONFIG created with defaults"); }
  File f = SD.open(CONFIG_FILE);
  if (!f) { logEvent("ERROR opening config.txt"); return; }
  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    if (!l.length() || l[0] == '#') continue;
    int e = l.indexOf('=');
    if (e < 0) continue;
    String k = l.substring(0, e), v = l.substring(e + 1);
    k.trim(); v.trim();
    if (k == "time_on") parseTimes(v);
    else if (k == "duration_s") cfg.durationS = max(1L, v.toInt());
    else if (k == "heartbeat_s") cfg.heartbeatS = max(0L, v.toInt());
    else if (k == "days") {
      for (int i = 1; i <= 7; i++) cfg.days[i] = v.indexOf((char)('0' + i)) >= 0;
    }
  }
  f.close();
  if (!cfg.nTimes) logEvent("WARNING config has no valid time_on");
  if (!verbose) return;
  String ts;
  for (int i = 0; i < cfg.nTimes; i++) { char b[8]; snprintf(b, sizeof b, "%02d:%02d ", cfg.times[i] / 60, cfg.times[i] % 60); ts += b; }
  String ds;
  for (int i = 1; i <= 7; i++) if (cfg.days[i]) ds += (char)('0' + i);
  logEvent("CONFIG time_on(UTC)=" + ts + "duration_s=" + String(cfg.durationS) + " days=" + ds + " heartbeat_s=" + String(cfg.heartbeatS));
}

// ---- relay -----------------------------------------------------------------------------------

void relaySet(bool on, const char *reason, uint32_t durS = 0) {
  relayOn = on;
  digitalWrite(RELAY_PIN, on ? RELAY_ON : !RELAY_ON);
#if TEST_LED_MODE
  digitalWrite(LED_G, on);                          // green stays on for the whole pulse
#else
  digitalWrite(LED_G, HIGH);                        // short flash on every relay change, not on for the whole pulse
  delay(RELAY_LED_FLASH_MS);
  digitalWrite(LED_G, LOW);
#endif
  if (on) { relayStartMs = millis(); relayDurMs = durS * 1000UL; }
  logEvent(String("RELAY ") + (on ? "ON" : "OFF") + " (" + reason + ")" + (on ? " duration_s=" + String(durS) : ""));
#if TEST_LED_MODE
  if (!on) {                                        // end of pulse: blue on 1 s, off 1 s, on 1 s, then off
    digitalWrite(LED_B, LED_B_ON); delay(1000);
    digitalWrite(LED_B, !LED_B_ON); delay(1000);
    digitalWrite(LED_B, LED_B_ON); delay(1000);
    digitalWrite(LED_B, !LED_B_ON);
  }
#endif
}

// ---- schedule --------------------------------------------------------------------------------

// Earliest configured activation strictly after nowT (+2 s margin), looking up to 8 days ahead; 0 if none.
time_t nextOccurrence(time_t nowT) {
  if (!cfg.nTimes) return 0;
  time_t best = 0, midnight = nowT - nowT % 86400L;
  for (int d = 0; d < 9; d++) {
    time_t base = midnight + d * 86400L;
    struct tm tm;
    gmtime_r(&base, &tm);
    if (!cfg.days[tm.tm_wday == 0 ? 7 : tm.tm_wday]) continue;
    for (int i = 0; i < cfg.nTimes; i++) {
      time_t t = base + cfg.times[i] * 60L;
      if (t > nowT + 2 && (!best || t < best)) best = t;
    }
  }
  return best;
}

bool armAlarm(time_t t) {
  struct tm tm;
  gmtime_r(&t, &tm);
  uint8_t st;
  if (!readRegs(0x0F, &st, 1)) return false;
  writeReg(0x0F, st & ~0x03);                       // release INT before arming
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x07);                                 // alarm 1: match date + hours + minutes + seconds
  Wire.write(dec2bcd(tm.tm_sec)); Wire.write(dec2bcd(tm.tm_min)); Wire.write(dec2bcd(tm.tm_hour)); Wire.write(dec2bcd(tm.tm_mday));
  if (Wire.endTransmission() != 0) return false;
  writeReg(0x0E, 0x05);                             // INTCN=1, A1IE=1
  return true;
}

void checkSchedule() {
  if (!clockValid()) return;
  time_t e = toEpoch(now_);
  struct tm tm;
  gmtime_r(&e, &tm);
  if (!cfg.days[tm.tm_wday == 0 ? 7 : tm.tm_wday]) return;
  int minute = now_.h * 60 + now_.mi;
  long key = (long)(e / 60);
  for (int i = 0; i < cfg.nTimes; i++) {
    if (cfg.times[i] == minute && key != lastTriggerKey) {
      lastTriggerKey = key;
      if (!relayOn) relaySet(true, "SCHEDULE", cfg.durationS);
      else logEvent("WARNING activation time reached while relay already on");
    }
  }
}

void goToSleep() {
  rtcOk = readTime(now_);
  uint64_t sleepS = NO_CLOCK_RETRY_S;
  bool useAlarm = false;
  if (!clockValid()) {
    logEvent("SLEEP clock invalid (use NTP/SET); retry in " + String(NO_CLOCK_RETRY_S) + " s");
  } else {
    time_t nowT = toEpoch(now_), next = nextOccurrence(nowT);
    if (!next) logEvent("SLEEP no schedule; retry in " + String(NO_CLOCK_RETRY_S) + " s");
    else if (!armAlarm(next)) logEvent("ERROR could not arm RTC alarm; retry in " + String(NO_CLOCK_RETRY_S) + " s");
    else {
      useAlarm = true;
      sleepS = (uint64_t)(next - nowT) + WAKE_FALLBACK_S;
      logEvent("SLEEP until " + isoUtc(next) + " (in " + String((long)(next - nowT)) + " s)");
    }
  }
  // With a heartbeat, the fallback timer is cut into short sleeps; each one wakes only to flash the LED (see setup()).
  uint64_t timerS = sleepS;
  hbWake = false;
  if (useAlarm && cfg.heartbeatS > 0 && sleepS > cfg.heartbeatS + 2) {
    hbWake = true; hbPeriodS = cfg.heartbeatS; hbRemainingS = sleepS - cfg.heartbeatS; timerS = cfg.heartbeatS;
  }
  SD.end();
  WiFi.mode(WIFI_OFF);
  digitalWrite(RELAY_PIN, !RELAY_ON);
  gpio_hold_en((gpio_num_t)RELAY_PIN);              // keep the relay pad off while the chip sleeps
  gpio_deep_sleep_hold_en();
  digitalWrite(LED_G, LOW); digitalWrite(LED_R, LOW); digitalWrite(LED_B, !LED_B_ON);
  gpio_hold_en((gpio_num_t)LED_B);                  // keep the blue LED off while asleep
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  if (useAlarm) esp_sleep_enable_ext0_wakeup((gpio_num_t)RTC_INT_PIN, 0);
  esp_sleep_enable_timer_wakeup(timerS * 1000000ULL);
  Serial.flush();
  esp_deep_sleep_start();
}

// ---- WiFi / NTP ------------------------------------------------------------------------------

String quotedValue(const String &line, const char *key) {
  String k = String("#define ") + key;
  if (!line.startsWith(k)) return "";
  int a = line.indexOf('"'), b = line.indexOf('"', a + 1);
  return (a >= 0 && b > a) ? line.substring(a + 1, b) : "";
}

bool readSecrets(String &ssid, String &pass) {
  ssid = ""; pass = "";
  if (!sdOk) return false;
  File f = SD.open(SECRETS_FILE);
  if (!f) return false;
  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    String v = quotedValue(l, "WIFI_SSID");
    if (v.length()) ssid = v;
    v = quotedValue(l, "WIFI_PASS");
    if (v.length()) pass = v;
  }
  f.close();
  return ssid.length();
}

bool ntpSync(const char *why) {
  if (apActive) stopHotspot();                      // NTP needs station mode; this closes the hotspot
  String ssid, pass;
  if (!readSecrets(ssid, pass)) { logEvent("NTP skipped: no usable secrets.h on SD"); return false; }
  logEvent(String("NTP start (") + why + ") ssid=" + ssid);
  digitalWrite(LED_B, LED_B_ON);                    // blue on while the NTP sync is running
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < NTP_WIFI_TIMEOUT_MS) delay(200);
  bool ok = false;
  time_t t = 0;
  if (WiFi.status() == WL_CONNECTED) {
    struct timeval zero = {0, 0};
    settimeofday(&zero, nullptr);                   // forget any earlier sync so we wait for a fresh answer
    configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
    t0 = millis();
    while (millis() - t0 < NTP_REPLY_TIMEOUT_MS) {
      t = time(nullptr);
      if (t > NTP_MIN_EPOCH) { ok = true; break; }
      delay(200);
    }
    if (!ok) logEvent("NTP FAILED: WiFi ok but no reply from NTP servers");
  } else {
    logEvent("NTP FAILED: WiFi did not connect (status=" + String((int)WiFi.status()) + ")");
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  digitalWrite(LED_B, !LED_B_ON);
  if (!ok) { digitalWrite(LED_R, HIGH); delay(1500); return false; }

  bool hadClock = readTime(now_) && !rtcLostPower;
  long before = hadClock ? (long)(t - toEpoch(now_)) : 0;
  setTimeEpoch(t);
  rtcOk = readTime(now_);
  logEvent(String("NTP OK: RTC set to UTC") + (hadClock ? ", correction " + String(before) + " s" : ", clock had been lost"));
  digitalWrite(LED_B, LED_B_ON); delay(1500); digitalWrite(LED_B, !LED_B_ON);
  return true;
}

// ---- hotspot + HTTP --------------------------------------------------------------------------

String timesString() {
  String ts;
  for (int i = 0; i < cfg.nTimes; i++) { char b[8]; snprintf(b, sizeof b, "%02d:%02d", cfg.times[i] / 60, cfg.times[i] % 60); if (i) ts += ","; ts += b; }
  return ts;
}

String daysString() {
  String ds;
  for (int i = 1; i <= 7; i++) if (cfg.days[i]) ds += (char)('0' + i);
  return ds;
}

void sendText(int code, const String &s) { server.send(code, "text/plain", s + "\n"); }

// A valid command was applied: confirm with one green blink, then sleep CMD_GRACE_MS after the last command.
void commandApplied() {
  digitalWrite(LED_G, HIGH); delay(CMD_LED_MS); digitalWrite(LED_G, LOW);
  cmdSeen = true;
  awakeUntil = millis() + CMD_GRACE_MS;
  apEnd = awakeUntil;
}

// Serial only: these run on the WiFi task, so no SD access here.
void onApEvent(arduino_event_id_t event, arduino_event_info_t info) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) Serial.println("HOTSPOT station connected");
  else if (event == ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED) Serial.println("HOTSPOT station got IP " + IPAddress(info.wifi_ap_staipassigned.ip.addr).toString());
  else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) Serial.println("HOTSPOT station disconnected");
}

void handleHttp() {
  String u = server.uri();
  u.replace("%3A", ":"); u.replace("%3a", ":");
  logEvent("HTTP " + u + " from " + server.client().remoteIP().toString());
  int h, m;
  unsigned dur;
  if (u == "/") {
    sendText(200, String(AP_SSID) + "\n/set_time=HH:MM (UTC)\n/set_duration=SECONDS\n/status\n/start_mission (close the hotspot now and sleep)");
  } else if (u == "/status") {
    rtcOk = readTime(now_);
    String rtcStr = !rtcOk ? String("ERROR") : isoUtc(toEpoch(now_)) + (rtcLostPower ? " (INVALID, RTC lost power)" : "");
    sendText(200, "rtc_time(UTC)=" + rtcStr + "\ntime_on(UTC)=" + timesString() + "\nduration_s=" + String(cfg.durationS) + "\ndays=" + daysString());
  } else if (u.startsWith("/set_time=")) {
    if (sscanf(u.c_str(), "/set_time=%d:%d", &h, &m) != 2 || h < 0 || h > 23 || m < 0 || m > 59) {
      sendText(400, "ERROR use /set_time=HH:MM (UTC)");
      return;
    }
    char b[8];
    snprintf(b, sizeof b, "%02d:%02d", h, m);
    if (!sdOk || !writeConfig(b, cfg.durationS, daysString())) { sendText(500, "ERROR cannot write config.txt"); return; }
    loadConfig();
    logEvent(String("HOTSPOT set_time=") + b + " applied");
    sendText(200, String("OK time_on(UTC)=") + b);
    commandApplied();
  } else if (u.startsWith("/set_duration=")) {
    if (sscanf(u.c_str(), "/set_duration=%u", &dur) != 1 || dur < 1 || dur > 86400) {
      sendText(400, "ERROR use /set_duration=SECONDS (1..86400)");
      return;
    }
    if (!sdOk || !writeConfig(timesString(), dur, daysString())) { sendText(500, "ERROR cannot write config.txt"); return; }
    loadConfig();
    logEvent("HOTSPOT set_duration=" + String(dur) + " applied");
    sendText(200, "OK duration_s=" + String(dur));
    commandApplied();
  } else if (u == "/start_mission") {
    rtcOk = readTime(now_);
    time_t next = clockValid() ? nextOccurrence(toEpoch(now_)) : 0;
    logEvent("HOTSPOT start_mission received");
    sendText(200, String("OK hotspot closing, going to sleep\nnext_activation(UTC)=") + (next ? isoUtc(next) : String("NONE")) +
                  (clockValid() ? "" : "\nWARNING RTC clock invalid, the relay will not fire until the clock is set"));
    delay(300);                                     // let the reply reach the phone before the AP goes down
    missionStart = true;
    apEnd = millis();                               // skip the rest of the hotspot time and the command grace
    awakeUntil = millis();
  } else {
    sendText(404, "ERROR unknown command");
  }
}

void startHotspot() {
  WiFi.onEvent(onApEvent);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 2)) {
    logEvent("HOTSPOT FAILED to start (password must be 8..63 characters)");
    WiFi.mode(WIFI_OFF);
    awakeUntil = millis() + 60000UL;                // leave a minute for the serial port, then sleep
    return;
  }
  server.onNotFound(handleHttp);
  server.begin();
  apActive = true;
  apEnd = millis() + AP_DURATION_MS;
  awakeUntil = apEnd;
  logEvent(String("HOTSPOT start ssid=") + AP_SSID + " ip=" + WiFi.softAPIP().toString() + " open " + String(AP_DURATION_MS / 1000) + " s");
}

void stopHotspot() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  apActive = false;
  if (!relayOn) digitalWrite(LED_G, LOW);
  logEvent("HOTSPOT stop");
}

// Hotspot phase over: blue, green, blue, green, 1 s each, so it is clear the board is about to sleep.
void endOfHotspotSequence() {
  const int seq[4] = {LED_B, LED_G, LED_B, LED_G};
  for (int i = 0; i < 4; i++) {
    bool blue = seq[i] == LED_B;
    digitalWrite(seq[i], blue ? LED_B_ON : HIGH);
    delay(1000);
    digitalWrite(seq[i], blue ? !LED_B_ON : LOW);
  }
}

// ---- status / serial / buttons ---------------------------------------------------------------

void printStatus() {
  Serial.printf("Status: RTC=%s SD=%s relay=%s OSF=%d\n", rtcOk ? "ok" : "ERROR", sdOk ? "ok" : "ERROR", relayOn ? "ON" : "off", rtcLostPower);
  if (rtcOk) {
    Serial.println("Time: " + isoUtc(toEpoch(now_)) + (rtcLostPower ? " (INVALID)" : ""));
    if (clockValid()) {
      time_t n = nextOccurrence(toEpoch(now_));
      Serial.println(n ? "Next activation: " + isoUtc(n) : String("Next activation: none"));
    }
  }
  if (relayOn) Serial.printf("%lu s left\n", (relayDurMs - (millis() - relayStartMs)) / 1000);
}

void handleSerial() {
  if (!Serial.available()) return;
  String l = Serial.readStringUntil('\n');
  l.trim();
  int y, mo, d, h, mi, s;
  char times[64], days[16], ssid[64], pass[64];
  unsigned dur;
  if (sscanf(l.c_str(), "SET %d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) == 6) {
    setTime(y, mo, d, h, mi, s);
    rtcOk = readTime(now_);
    logEvent("RTC time set over serial (UTC)");
  } else if (l.startsWith("CFG ")) {
    strcpy(days, "1234567");
    if (sscanf(l.c_str(), "CFG %63s %u %15s", times, &dur, days) >= 2 && sdOk && writeConfig(times, dur, days)) loadConfig();
    else Serial.println("CFG failed (usage: CFG HH:MM[,HH:MM...] SECONDS [DAYS])");
  } else if (l.startsWith("WIFICFG ")) {
    if (sscanf(l.c_str(), "WIFICFG %63s %63s", ssid, pass) == 2 && sdOk) {
      SD.remove(SECRETS_FILE);
      File f = SD.open(SECRETS_FILE, FILE_WRITE);
      if (f) {
        f.printf("#define WIFI_SSID \"%s\"\n#define WIFI_PASS \"%s\"\n", ssid, pass);
        f.close();
        logEvent(String("WiFi credentials saved for ") + ssid);
      } else Serial.println("WIFICFG failed: cannot write secrets.h");
    } else Serial.println("WIFICFG failed (usage: WIFICFG SSID PASSWORD)");
  } else if (l == "STATUS") printStatus();
  else if (l == "NTP") ntpSync("serial");
  else if (l == "LOG") {
    File f = sdOk ? SD.open(LOG_FILE) : File();
    if (!f) { Serial.println("LOG not available"); return; }
    while (f.available()) Serial.write(f.read());
    f.close();
    Serial.println("--- end of log ---");
  } else if (l == "RELAY ON") { if (!relayOn) relaySet(true, "SERIAL", cfg.durationS); }
  else if (l == "RELAY OFF") { if (relayOn) relaySet(false, "SERIAL"); }
  else if (l == "RELOAD") { if (!sdOk) mountSD(); loadConfig(); }
  else if (l == "AWAKE") { stayAwake = true; Serial.println("Staying awake (send SLEEP to resume)"); }
  else if (l == "SLEEP") { stayAwake = false; awakeUntil = 0; }
}

void handleButtons() {
  static bool last[3] = {true, true, true};
  static uint32_t lastChange[3];
  const int pins[3] = {PB_1, PB_2, PB_3};
  for (int i = 0; i < 3; i++) {
    bool s = digitalRead(pins[i]);
    if (s != last[i] && millis() - lastChange[i] > 50) {
      last[i] = s; lastChange[i] = millis();
      if (s) continue;  // press edge only
      if (i == 0) { if (relayOn) relaySet(false, "MANUAL"); else relaySet(true, "MANUAL", cfg.durationS); }
      if (i == 1) printStatus();
      if (i == 2) {
        if (millis() < ntpWindowEnd) { ntpSync("button"); ntpWindowEnd = 0; }   // done: go on to the hotspot
        else Serial.println("PB3 ignored: NTP only in the first " + String(NTP_WINDOW_MS / 1000) + " s after power-on");
      }
    }
  }
}

// ---- main ------------------------------------------------------------------------------------

void setup() {
  if (hbWake && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    // Heartbeat wake: flash the LED and go straight back to sleep. No I2C, SD or serial, and the
    // relay pad stays held. The RTC alarm is still armed and ext0 is re-enabled below.
    gpio_hold_dis((gpio_num_t)LED_B);
    pinMode(LED_B, OUTPUT);
    digitalWrite(LED_B, LED_B_ON);
    delay(HEARTBEAT_FLASH_MS);
    digitalWrite(LED_B, !LED_B_ON);
    gpio_hold_en((gpio_num_t)LED_B);                // LED off while asleep
    uint32_t next;
    if (hbRemainingS > hbPeriodS + 2) { hbRemainingS -= hbPeriodS; next = hbPeriodS; }
    else { next = hbRemainingS > 0 ? hbRemainingS : 1; hbWake = false; }   // last stretch: next wake is a real one
    gpio_deep_sleep_hold_en();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)RTC_INT_PIN, 0);
    esp_sleep_enable_timer_wakeup((uint64_t)next * 1000000ULL);
    esp_deep_sleep_start();
  }
  gpio_hold_dis((gpio_num_t)RELAY_PIN);             // release the pads held during deep sleep
  gpio_hold_dis((gpio_num_t)LED_B);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, !RELAY_ON);               // idle: not conducting
  pinMode(PB_1, INPUT_PULLUP); pinMode(PB_2, INPUT_PULLUP); pinMode(PB_3, INPUT_PULLUP);
  pinMode(RTC_INT_PIN, INPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW); digitalWrite(LED_G, LOW); digitalWrite(LED_B, !LED_B_ON);
  setenv("TZ", "UTC0", 1);
  tzset();
  Serial.begin(115200);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool fromSleep = cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_TIMER;
  wokeFromSleep = fromSleep;
  if (!fromSleep) delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  rtcOk = readTime(now_);
  uint8_t st;
  if (readRegs(0x0F, &st, 1)) writeReg(0x0F, st & ~0x03);   // clear alarm flags so INT releases
  mountSD();

  static const char *resetNames[] = {"UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT", "WDT", "DEEPSLEEP", "BROWNOUT", "SDIO"};
  int rr = (int)esp_reset_reason();
  logEvent(String(fromSleep ? "WAKE " : "BOOT ") + "reset=" + (rr >= 0 && rr <= 10 ? resetNames[rr] : "?") +
           " cause=" + (cause == ESP_SLEEP_WAKEUP_EXT0 ? "RTC_ALARM" : cause == ESP_SLEEP_WAKEUP_TIMER ? "TIMER" : "NONE") +
           " rtc=" + (rtcOk ? "ok" : "ERROR") + " sd=" + (sdOk ? "ok" : "ERROR") + (rtcLostPower ? " RTC_LOST_POWER(OSF)" : ""));
  loadConfig(!fromSleep);

  if (!fromSleep) {
    ntpWindowEnd = millis() + NTP_WINDOW_MS;        // phase 1: PB3 = NTP, then phase 2: hotspot (see loop)
    awakeUntil = ntpWindowEnd + AP_DURATION_MS;
  } else {
    awakeUntil = 0;
  }
  checkSchedule();                                  // serve the activation before anything can put us back to sleep
}

void loop() {
  static uint32_t lastTick, lastSdRetry;
  handleSerial();
  handleButtons();

  if (relayOn && millis() - relayStartMs >= relayDurMs) relaySet(false, "DURATION ENDED");

  uint32_t ms = millis();
  if (!wokeFromSleep) {
    if (ms < ntpWindowEnd) {
      digitalWrite(LED_B, ((ms / 1000) % 2 == 0) ? LED_B_ON : !LED_B_ON);   // blink 1 s on / 1 s off = NTP window
    } else if (!hotspotStarted) {
      digitalWrite(LED_B, !LED_B_ON);
      hotspotStarted = true;
      startHotspot();
    }
    if (apActive) {
      server.handleClient();
      if (!cmdSeen && !relayOn) digitalWrite(LED_G, ((ms / 1000) % 2 == 0) ? HIGH : LOW);   // green blinks 1 s / 1 s while waiting for a command
      if (millis() >= apEnd) {                      // fresh clock read, same as the sleep check below
        logEvent(missionStart ? "HOTSPOT closed by start_mission" : cmdSeen ? "HOTSPOT closed after command" : "HOTSPOT timeout, config unchanged");
        stopHotspot();
        endOfHotspotSequence();
      }
    }
  }

  bool maintenance = stayAwake || millis() < awakeUntil;
  if (millis() - lastTick >= 1000) {
    lastTick = millis();
    rtcOk = readTime(now_);
    checkSchedule();
  }
  if (!sdOk && millis() - lastSdRetry > 10000) { lastSdRetry = millis(); if (mountSD()) { logEvent("SD recovered"); loadConfig(); } }
  digitalWrite(LED_R, !(rtcOk && sdOk) || rtcLostPower);

  if (!relayOn && !maintenance && !apActive) goToSleep();   // never sleep with the hotspot still up: close it (and run its LED sequence) first
}
