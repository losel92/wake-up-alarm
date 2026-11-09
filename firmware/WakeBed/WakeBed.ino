// WakeBed: bed-presence alarm for the M5Stack AtomS3 Lite
//
// Hardware (same as bed_sensor_test.ino):
//   - Weight Unit (HX711) on the Atom's own Grove port  -> HX_DOUT / HX_SCK
//   - Passive buzzer on PortABC Port B                  -> BUZZER_PIN
//   - Built-in button (GPIO 41) and RGB LED (GPIO 35)
//
// Control: Bluetooth LE. Open the WakeBed control page in Chrome (Android or
// Windows), press Connect, pick "WakeBed". The same commands also work over
// USB in the Serial Monitor (115200 baud), e.g. "status", "tare", "test".
//
// Libraries (Arduino Library Manager):
//   - "HX711 Arduino Library" by Bogdan Necula
//   - "NimBLE-Arduino" by h2zero (version 2.x)
//
// Alarm behaviour:
//   At the alarm time, if the bed is occupied, a wake session starts.
//   During the session the buzzer rings whenever the bed is occupied and is
//   silent while it's empty. The session ends once the bed has been empty for
//   `clr` minutes in a row (getting back in resets that countdown and rings
//   again), or after `cap` minutes in total. If the bed is empty at the alarm
//   time, the alarm is skipped.

#include <Arduino.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <time.h>
#include <sys/time.h>
#include "HX711.h"

// ---------------- Pins (keep whatever worked in bed_sensor_test) -------------
const int HX_DOUT    = 1;
const int HX_SCK     = 2;
const int BUZZER_PIN = 7;    // Port B: 7, or 8 if 7 stays silent
const int BUTTON_PIN = 41;   // AtomS3 built-in button (active LOW)
const int LED_PIN    = 35;   // AtomS3 Lite RGB LED; set to -1 to disable

// Local time zone (Switzerland, with daylight saving)
const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

// ---------------- BLE UUIDs (must match the web page) ------------------------
#define SVC_UUID  "6e7a0001-3c5b-4f0e-9a51-7b2d1c0e4a10"
#define CFG_UUID  "6e7a0002-3c5b-4f0e-9a51-7b2d1c0e4a10"  // read/write settings
#define LIVE_UUID "6e7a0003-3c5b-4f0e-9a51-7b2d1c0e4a10"  // notify sensor data
#define CMD_UUID  "6e7a0004-3c5b-4f0e-9a51-7b2d1c0e4a10"  // write commands
#define EVT_UUID  "6e7a0005-3c5b-4f0e-9a51-7b2d1c0e4a10"  // notify event text

// ---------------- Settings (saved in flash) ----------------------------------
struct Config {
  bool     en   = false;  // alarm enabled
  uint8_t  ah   = 7;      // alarm hour (local)
  uint8_t  am   = 30;     // alarm minute
  uint8_t  days = 0x7F;   // bit0 = Monday ... bit6 = Sunday
  long     on   = 50000;  // |delta| above this -> occupied
  long     off  = 25000;  // |delta| below this -> empty (hysteresis)
  uint16_t clr  = 5;      // minutes empty in a row that end the session
  uint16_t cap  = 45;     // max session length, minutes
  uint8_t  vol  = 30;     // starting buzzer duty, 1..128 (128 = loudest)
  uint16_t ramp = 5;      // minutes to ramp up to full volume (0 = no ramp)
  uint16_t freq = 2700;   // buzzer frequency, Hz
  bool     btn  = true;   // allow a 3 s button hold to stop a session
} cfg;

Preferences prefs;
HX711 scale;

// ---------------- Runtime state ----------------------------------------------
enum State { IDLE, RINGING_SESSION };
State state = IDLE;

long  emptyLevel = 0;
bool  haveEmpty  = false;
long  rawFilt    = 0;
long  delta      = 0;
bool  occupied   = false;
bool  sensorOk   = false;

unsigned long occChangeSince = 0;  // debounce timer
const unsigned long DEBOUNCE_MS = 800;

unsigned long sessionStartMs = 0;
unsigned long emptySinceMs   = 0;  // 0 = currently occupied
int           lastFiredYday  = -1;
int           lastFiredMin   = -1;
unsigned long testRingUntil  = 0;
unsigned long simAlarmAt     = 0;  // millis() of a simulated alarm, 0 = none

bool bleConnected = false;
NimBLECharacteristic *cfgChr, *liveChr, *evtChr;
QueueHandle_t cmdQueue;
struct CmdMsg { char s[96]; };

// ---------------- Helpers ----------------------------------------------------
bool timeValid() { return time(nullptr) > 1700000000; }

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  if (LED_PIN < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(LED_PIN, r, g, b);
#else
  neopixelWrite(LED_PIN, r, g, b);
#endif
}

void buzzerInit() {
  if (BUZZER_PIN < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BUZZER_PIN, cfg.freq, 8);
#else
  ledcSetup(0, cfg.freq, 8);
  ledcAttachPin(BUZZER_PIN, 0);
#endif
}

void buzzerSet(uint8_t duty) {
  if (BUZZER_PIN < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(BUZZER_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

void buzzerFreq(uint16_t f) {
  if (BUZZER_PIN < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcChangeFrequency(BUZZER_PIN, f, 8);
#else
  ledcChangeFrequency(0, f, 8);
#endif
}

void event(const String& msg) {
  String line = msg;
  if (timeValid()) {
    char ts[10];
    time_t now = time(nullptr);
    strftime(ts, sizeof ts, "%H:%M:%S", localtime(&now));
    line = String(ts) + " " + msg;
  }
  Serial.println(line);
  if (bleConnected && evtChr) {
    evtChr->setValue(line.substring(0, 180).c_str());
    evtChr->notify();
  }
}

String configString() {
  char buf[200];
  snprintf(buf, sizeof buf,
    "{\"en\":%d,\"ah\":%u,\"am\":%u,\"days\":%u,\"on\":%ld,\"off\":%ld,"
    "\"clr\":%u,\"cap\":%u,\"vol\":%u,\"ramp\":%u,\"freq\":%u,\"btn\":%d}",
    cfg.en, cfg.ah, cfg.am, cfg.days, cfg.on, cfg.off,
    cfg.clr, cfg.cap, cfg.vol, cfg.ramp, cfg.freq, cfg.btn);
  return String(buf);
}

void publishConfig() {
  if (cfgChr) cfgChr->setValue(configString().c_str());
}

void saveConfig() {
  prefs.putBytes("cfg", &cfg, sizeof cfg);
  publishConfig();
}

void loadConfig() {
  if (prefs.getBytesLength("cfg") == sizeof cfg) prefs.getBytes("cfg", &cfg, sizeof cfg);
  if (prefs.isKey("empty")) { emptyLevel = prefs.getLong("empty"); haveEmpty = true; }
}

// key=value; returns true if the key was known
bool setKey(const String& k, long v) {
  if      (k == "en")   cfg.en   = v != 0;
  else if (k == "ah")   cfg.ah   = constrain(v, 0, 23);
  else if (k == "am")   cfg.am   = constrain(v, 0, 59);
  else if (k == "days") cfg.days = v & 0x7F;
  else if (k == "on")   cfg.on   = max(1L, v);
  else if (k == "off")  cfg.off  = max(0L, v);
  else if (k == "clr")  cfg.clr  = constrain(v, 1, 120);
  else if (k == "cap")  cfg.cap  = constrain(v, 1, 240);
  else if (k == "vol")  cfg.vol  = constrain(v, 1, 128);
  else if (k == "ramp") cfg.ramp = constrain(v, 0, 60);
  else if (k == "freq") { cfg.freq = constrain(v, 200, 8000); buzzerFreq(cfg.freq); }
  else if (k == "btn")  cfg.btn  = v != 0;
  else return false;
  if (cfg.off >= cfg.on) cfg.off = cfg.on / 2;
  return true;
}

void tare() {
  if (!sensorOk) { event("Tare failed: sensor not ready"); return; }
  emptyLevel = rawFilt;
  haveEmpty = true;
  prefs.putLong("empty", emptyLevel);
  event("Tared: empty level = " + String(emptyLevel));
}

void startSession(const char* why) {
  if (!occupied) {
    event(String("Alarm (") + why + ") skipped: bed empty");
    return;
  }
  state = RINGING_SESSION;
  sessionStartMs = millis();
  emptySinceMs = 0;
  event(String("Alarm (") + why + "): ringing");
}

void endSession(const char* why) {
  if (state != RINGING_SESSION) return;
  state = IDLE;
  buzzerSet(0);
  prefs.putLong("empty", emptyLevel);   // keep the drift-tracked empty level
  event(String("Session ended: ") + why);
}

String statusLine() {
  String s = "state=" + String(state == IDLE ? "idle" : "session");
  s += " occupied=" + String(occupied) + " delta=" + String(delta);
  s += " empty=" + String(emptyLevel) + " ble=" + String(bleConnected);
  if (timeValid()) {
    char ts[24]; time_t now = time(nullptr);
    strftime(ts, sizeof ts, "%a %H:%M:%S", localtime(&now));
    s += " time=" + String(ts);
  } else s += " time=unset";
  s += " cfg=" + configString();
  return s;
}

// Commands from BLE or Serial. Several may be joined with ';'
void handleCommand(String c) {
  c.trim();
  if (c.length() == 0) return;
  int semi = c.indexOf(';');
  if (semi >= 0) { handleCommand(c.substring(0, semi)); handleCommand(c.substring(semi + 1)); return; }

  int eq = c.indexOf('=');
  String k = eq >= 0 ? c.substring(0, eq) : c;
  String v = eq >= 0 ? c.substring(eq + 1) : "";
  k.trim(); v.trim();

  if (k == "time") {                      // time=<unix seconds, UTC>
    struct timeval tv = { (time_t)strtoll(v.c_str(), nullptr, 10), 0 };
    settimeofday(&tv, nullptr);
    event("Time set");
  } else if (k == "alarm") {              // alarm=HH:MM
    int h = v.substring(0, 2).toInt(), m = v.substring(3, 5).toInt();
    setKey("ah", h); setKey("am", m); saveConfig();
    event("Alarm time " + v);
  } else if (k == "tare")   { tare();
  } else if (k == "test")   { testRingUntil = millis() + (v.length() ? v.toInt() : 3) * 1000UL; event("Test ring");
  } else if (k == "sim")    { simAlarmAt = millis() + (v.length() ? v.toInt() : 15) * 1000UL;
                              event("Simulated alarm in " + String((simAlarmAt - millis()) / 1000) + " s");
  } else if (k == "stop")   { simAlarmAt = 0; testRingUntil = 0; buzzerSet(0); endSession("stopped remotely");
  } else if (k == "status") { event(statusLine());
  } else if (k == "reboot") { event("Rebooting"); delay(200); ESP.restart();
  } else if (setKey(k, v.toInt())) { saveConfig(); event("Set " + k + "=" + v);
  } else event("Unknown command: " + c);
  publishConfig();
}

// ---------------- BLE --------------------------------------------------------
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    bleConnected = true;
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    bleConnected = false;
    NimBLEDevice::startAdvertising();
  }
};

class CmdCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    CmdMsg m{};
    std::string v = c->getValue();
    strncpy(m.s, v.c_str(), sizeof(m.s) - 1);
    xQueueSend(cmdQueue, &m, 0);   // handled in loop(), not in the BLE task
  }
};

void bleInit() {
  NimBLEDevice::init("WakeBed");
  NimBLEDevice::setMTU(247);
  NimBLEServer* srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  NimBLEService* svc = srv->createService(SVC_UUID);

  cfgChr  = svc->createCharacteristic(CFG_UUID,  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  liveChr = svc->createCharacteristic(LIVE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* cmdChr =
            svc->createCharacteristic(CMD_UUID,  NIMBLE_PROPERTY::WRITE);
  evtChr  = svc->createCharacteristic(EVT_UUID,  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  CmdCB* cb = new CmdCB();
  cmdChr->setCallbacks(cb);
  cfgChr->setCallbacks(cb);      // writing "key=value" to config works too
  publishConfig();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setName("WakeBed");
  adv->enableScanResponse(true);
  adv->start();
}

// ---------------- Sensor -----------------------------------------------------
void readSensor() {
  if (!scale.is_ready()) return;
  long r = scale.read();
  static long buf[4]; static int n = 0, i = 0;
  buf[i] = r; i = (i + 1) % 4; if (n < 4) n++;
  long s = 0; for (int j = 0; j < n; j++) s += buf[j];
  rawFilt = s / n;
  if (!sensorOk) { sensorOk = true; event("Sensor ready"); }

  if (!haveEmpty && n == 4) { tare(); }   // first boot only; later boots reuse the saved level
  delta = rawFilt - emptyLevel;
  long load = labs(delta);

  bool want = occupied ? (load > cfg.off) : (load > cfg.on);
  if (want != occupied) {
    if (occChangeSince == 0) occChangeSince = millis();
    if (millis() - occChangeSince >= DEBOUNCE_MS) {
      occupied = want; occChangeSince = 0;
      event(occupied ? "Bed occupied" : "Bed empty");
    }
  } else {
    occChangeSince = 0;
  }

  // Follow slow drift of the empty level while the bed is clearly empty
  if (!occupied && load < cfg.off / 2) emptyLevel += (rawFilt - emptyLevel) / 256;
}

// ---------------- Alarm + buzzer ---------------------------------------------
void checkAlarm() {
  if (simAlarmAt && (long)(millis() - simAlarmAt) >= 0) {
    simAlarmAt = 0;
    if (state == IDLE) startSession("simulated");
  }
  if (!cfg.en || !timeValid() || state != IDLE) return;
  time_t now = time(nullptr);
  struct tm t; localtime_r(&now, &t);
  int dayBit = (t.tm_wday + 6) % 7;                    // Monday = 0
  if (!(cfg.days & (1 << dayBit))) return;
  if (t.tm_hour == cfg.ah && t.tm_min == cfg.am &&
      !(t.tm_yday == lastFiredYday && t.tm_min == lastFiredMin)) {
    lastFiredYday = t.tm_yday; lastFiredMin = t.tm_min;
    startSession("scheduled");
  }
}

void runSession() {
  if (state != RINGING_SESSION) return;
  unsigned long now = millis();
  if (now - sessionStartMs >= cfg.cap * 60000UL) { endSession("time cap reached"); return; }
  if (occupied) {
    if (emptySinceMs) event("Back in bed: ringing again");
    emptySinceMs = 0;
  } else {
    if (!emptySinceMs) { emptySinceMs = now; event("Out of bed: " + String(cfg.clr) + " min countdown"); }
    if (now - emptySinceMs >= cfg.clr * 60000UL) endSession("bed empty long enough");
  }
}

void driveBuzzer() {
  unsigned long now = millis();
  bool ring = (state == RINGING_SESSION && occupied) || now < testRingUntil;
  if (!ring) { buzzerSet(0); return; }

  // Pattern: 4 beeps of 100 ms, then 600 ms pause (1.4 s cycle)
  unsigned long ph = now % 1400;
  bool on = ph < 800 && (ph % 200) < 100;

  uint8_t duty = cfg.vol;
  if (state == RINGING_SESSION && cfg.ramp > 0) {
    float f = (now - sessionStartMs) / (cfg.ramp * 60000.0f);
    if (f > 1) f = 1;
    duty = cfg.vol + (uint8_t)((128 - cfg.vol) * f);
  }
  buzzerSet(on ? duty : 0);
}

void checkButton() {
  static unsigned long downSince = 0;
  bool down = digitalRead(BUTTON_PIN) == LOW;
  if (down && !downSince) downSince = millis();
  if (!down) downSince = 0;
  if (down && downSince && millis() - downSince > 3000) {
    downSince = millis() + 600000;   // don't retrigger while still held
    if (state == RINGING_SESSION && cfg.btn) endSession("button held");
    else if (state == IDLE) { testRingUntil = millis() + 300; event("Button OK"); }
  }
}

void driveLed() {
  unsigned long now = millis();
  bool blink = (now / 500) % 2;
  if (state == RINGING_SESSION)  setLed(blink ? 40 : 0, 0, 0);       // red flash
  else if (!sensorOk)            setLed(blink ? 20 : 0, 8, 0);       // orange: sensor missing
  else if (!timeValid())         setLed(blink ? 12 : 0, 0, 12);      // purple: time not set
  else if (bleConnected)         setLed(0, 0, 15);                   // blue: phone connected
  else                           setLed(0, cfg.en ? 6 : 1, 0);       // green: armed / dim: off
}

// ---------------- Main -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  setenv("TZ", TZ_INFO, 1); tzset();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  prefs.begin("wake", false);
  loadConfig();

  scale.begin(HX_DOUT, HX_SCK);
  buzzerInit();
  buzzerSet(0);

  cmdQueue = xQueueCreate(8, sizeof(CmdMsg));
  bleInit();
  event("WakeBed started" + String(haveEmpty ? " (saved empty level)" : " (will tare: keep bed empty)"));
}

void loop() {
  CmdMsg m;
  while (xQueueReceive(cmdQueue, &m, 0) == pdTRUE) handleCommand(String(m.s));

  static String serialLine;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(serialLine); serialLine = ""; }
    else if (serialLine.length() < 90) serialLine += ch;
  }

  readSensor();
  checkAlarm();
  runSession();
  driveBuzzer();
  checkButton();
  driveLed();

  // Live data ~5 Hz: delta,occupied,state,raw,secondsEmpty,sessionSeconds,epoch
  static unsigned long lastLive = 0;
  if (millis() - lastLive >= 200) {
    lastLive = millis();
    char buf[96];
    unsigned long se = (state == RINGING_SESSION && emptySinceMs) ? (millis() - emptySinceMs) / 1000 : 0;
    unsigned long ss = state == RINGING_SESSION ? (millis() - sessionStartMs) / 1000 : 0;
    snprintf(buf, sizeof buf, "%ld,%d,%d,%ld,%lu,%lu,%ld",
             delta, occupied, (int)state, rawFilt, se, ss, (long)(timeValid() ? time(nullptr) : 0));
    liveChr->setValue(buf);
    if (bleConnected) liveChr->notify();
  }
  delay(5);
}
