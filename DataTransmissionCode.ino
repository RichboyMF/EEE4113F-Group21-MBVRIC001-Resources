/*
 * DataTransmissionCode.ino
 * MARIS Ocean Buoy - EEE4113F Group 21
 *
 * Non-blocking state-machine rewrite.
 *
 * Boot determines initial state:
 *   GPIO21 HIGH → SETUP mode  (keypad selects which setup)
 *   GPIO21 LOW  → LIVE mode   (no keypad - fully autonomous)
 *
 * ── SETUP MODE ────────────────────────────────────────────────────
 * Keypad selects one of three procedures:
 *
 *   'B' → Short Distance Setup
 *           NRF24 bulk transfer only. Verifies SD data extraction.
 *
 *   'A' → Long Distance Setup
 *           GSM cycle (press '1') → GSM + command/interval check
 *           (press '2') → Done. Verifies GSM / Firebase comms.
 *
 *   '6' → Full Setup  (tests both paths end-to-end)
 *           GSM cycle 1 (press '1') → GSM cycle 2 (press '2')
 *           → NRF24 bulk transfer (press '3')
 *           → final GSM cycle (press 'A') → Done
 *
 * Post-setup menu (shown after any setup path completes):
 *   '4' → return to setup mode selection (re-run any mode)
 *   '5' → launch Live mode without rebooting
 *
 * ── LIVE MODE ─────────────────────────────────────────────────────
 * Fully autonomous - NO keypad.
 *
 * Transmission mode (g_tx_mode) is set at boot from LIVE_MODE_PIN:
 *   HIGH → LONG  (GSM/Iridium)
 *   LOW  → SHORT (NRF24 bulk transfer)
 *
 * The integrated housing subsystem can override g_tx_mode at any
 * time by writing to it directly (shared global), or via Firebase
 * ctrl ("tx_mode": "long" / "short") read on each GSM cycle.
 *
 * isSurfaced() is provided by the housing subsystem - this file
 * declares it as a weak stub; the integrated build overrides it.
 *
 * Pause / resume is handled via:
 *   1. Firebase ctrl command "stop" / "start"  (existing mechanism)
 *   2. PAUSE_PIN (GPIO) - INPUT_PULLUP, active LOW (physical switch)
 *
 * Mode switching via Firebase ctrl command:
 *   "mode_long"     → g_tx_mode = TX_LONG  (switch to GSM/Iridium)
 *   "mode_short"    → g_tx_mode = TX_SHORT (switch to NRF24 bulk)
 *   "surface_hold"  → stay surfaced, enter LIVE_SEARCHING for NRF24
 *                     extraction; saves current mode and restores on
 *                     resume so operator only needs "start" to resume.
 *   Buoy picks up on next GSM surface cycle and ACKs status="done".
 *
 * ── EXTRACTION LIFECYCLE ──────────────────────────────────────────
 * Triggered by Firebase ctrl command "surface_hold":
 *
 *   ① LIVE_IDLE / LIVE_TRANSMIT  (normal GSM cycling)
 *         operator sends: surface_hold
 *
 *   ② LIVE_SEARCHING  (buoy stays surfaced)
 *         GSM ping every tx_interval  → map stays live, operator
 *                                        can locate buoy
 *         NRF24 probe every 10s       → fast-fail handshake
 *                                        (short ACK window on
 *                                         PKT_START only; data
 *                                         packets use full timeout)
 *         when receiver in range      → runBulkTransfer()
 *
 *   ③ POST-TRANSFER  (auto, no operator input needed)
 *         PUT buoy/status → {event:"extraction_complete", ...}
 *         SD.remove(sensor_log.txt)   ← bulk data cleared
 *         surface_log.txt KEPT        ← last GPS fix retained
 *         g_tx_mode restored          ← mode before surface_hold
 *         → LIVE_PAUSED
 *
 *   ④ Operator confirms on dashboard, sends "start"
 *
 *   ⑤ LIVE_IDLE  → normal diving resumes in restored mode
 *
 * ── SENSOR / SURFACE LOG SCHEMA ───────────────────────────────────
 * Owned by the sensing subsystem. Comms reads last line of each.
 *
 * sensor_log.txt  (continuous, appended while submerged and surfaced)
 *   {"ts":1706436000,"t":10.5,"p":1013.0,
 *    "ax":-0.12,"ay":0.98,"az":9.81,
 *    "gx":0.02,"gy":-0.01,"gz":0.00}
 *
 * surface_log.txt  (written on each surface event by sensing/GPS)
 *   {"ts":1706436000,"la":-31.9758,"ln":11.5809,"b":85}
 *
 * Transmitted payload (comms merges both - ~170 bytes, fits Iridium):
 *   {"ts":1706436000,"la":-31.9758,"ln":11.5809,
 *    "t":10.5,"p":1013.0,"b":85,
 *    "ax":-0.12,"ay":0.98,"az":9.81,
 *    "gx":0.02,"gy":-0.01,"gz":0.00}
 *
 * ── GSM / IRIDIUM NOTE ────────────────────────────────────────────
 * SIM800L is a PoC stand-in for the Iridium 9603.
 * Iridium SBD max payload: 340 bytes uplink / 270 bytes downlink.
 * buildSBDPayload() produces the compact payload used in production.
 * buildHTTPPayload() produces the verbose payload used in PoC/debug.
 */

#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <RF24.h>
#include "Keypad.h"

// ── Pins ──────────────────────────────────────────────────────────
#define SETUP_GATE_PIN  21    // INPUT_PULLUP: HIGH = setup, LOW = live
#define LIVE_MODE_PIN   22    // INPUT_PULLUP: HIGH = LONG, LOW = SHORT
#define PAUSE_PIN       23    // INPUT_PULLUP: LOW = paused (physical switch)
#define LED_PIN          8
#define SD_SCK          12
#define SD_MISO         13
#define SD_MOSI         11
#define SD_CS           10
#define NRF_CE           4
#define NRF_CSN          5
#define NRF_SCK          6
#define NRF_MOSI         7
#define NRF_MISO        15

// ── Transmission mode ─────────────────────────────────────────────
enum TxMode { TX_LONG, TX_SHORT };
TxMode g_tx_mode      = TX_LONG;   // default; set at boot from LIVE_MODE_PIN
TxMode g_pre_hold_mode = TX_LONG;  // saved on surface_hold, restored on resume

// ── isSurfaced ────────────────────────────────────────────────────
// Weak stub - the integrated housing subsystem overrides this.
// Returns true when the buoy has surfaced and is ready to transmit.
// g_surface_hold overrides this in LIVE_SEARCHING - buoy stays up
// regardless of what the housing subsystem reports.
__attribute__((weak)) bool isSurfaced() { return false; }

// ── Key scanning (setup mode only) ───────────────────────────────
#define KEY_HOLD_MS 80UL

void waitForKey(char target, unsigned long holdMs = KEY_HOLD_MS) {
  unsigned long heldSince = 0;
  bool counting = false;
  unsigned long lastPrint = millis();
  while (true) {
    char k = keypadScan();
    if (k == target) {
      if (!counting) { counting = true; heldSince = millis(); }
      else if (millis() - heldSince >= holdMs) { keypadWaitRelease(); return; }
    } else {
      counting = false;
    }
    if (millis() - lastPrint >= 3000) {
      Serial.printf("[KP] Waiting for key '%c'...\n", target);
      lastPrint = millis();
    }
    delay(10);
  }
}

// ── State machine ─────────────────────────────────────────────────
enum AppState {
  // ── Short Distance Setup ──────────────────────────────────────
  SDS_NRF_WAIT,
  SDS_NRF_RUN,
  SDS_DONE,

  // ── Long Distance Setup ───────────────────────────────────────
  LDS_S1_WAIT,
  LDS_S1_RUN,
  LDS_S2_WAIT,
  LDS_S2_RUN,
  LDS_DONE,

  // ── Full Setup ────────────────────────────────────────────────
  FS_S1_WAIT,
  FS_S1_RUN,
  FS_S2_WAIT,
  FS_S2_RUN,
  FS_S3_WAIT,
  FS_S3_RUN,
  FS_GSM_REINIT,
  FS_S4_WAIT,
  FS_S4_RUN,
  FS_DONE,

  // ── Post-completion menu ──────────────────────────────────────
  SETUP_MENU,

  // ── Live ──────────────────────────────────────────────────────
  LIVE_IDLE,        // waiting for isSurfaced()
  LIVE_TRANSMIT,    // executing runTransmission(g_tx_mode)
  LIVE_SEARCHING,   // surface_hold active: GSM pings + NRF24 probe loop
  LIVE_PAUSED,      // paused via Firebase stop or PAUSE_PIN
};

AppState      g_state;
bool          g_nrf_ready    = false;
bool          g_gsm_ready    = false;
bool          g_cycle_active = true;   // controlled by Firebase start/stop
bool          g_surface_hold = false;  // set by "surface_hold" command
bool          g_prompt_said  = false;
bool          g_sim_silent   = false;

// ── SD card (HSPI) ────────────────────────────────────────────────
SPIClass sdSpi(HSPI);
bool g_sd_ok = false;

const char* SENSOR_FILE  = "/sensor_log.txt";
const char* SURFACE_FILE = "/surface_log.txt";
const char* LOG_FILE     = "/sender_log.txt";
const char* STATE_FILE   = "/state.json";

// ── NRF24 (VSPI) ─────────────────────────────────────────────────
RF24 radio(NRF_CE, NRF_CSN);
const byte NRF_ADDR[6] = "00001";

#define ACK_TIMEOUT      3000UL   // ms - full timeout for data packets
#define ACK_PROBE_TIMEOUT  800UL  // ms - short timeout for PKT_START handshake only
#define ACK_PROBE_RETRIES    3    // retries for PKT_START (fast-fail, no receiver = ~2.4s max)
#define PKT_START    1
#define PKT_DATA     0
#define PKT_LINE_END 3
#define PKT_END      2
#define PKT_FILE     4

struct Packet {
  uint8_t  type;
  uint16_t lineNum;
  uint8_t  chunkIdx;
  char     data[28];
};

// ── SIM800 / Iridium (UART1) ──────────────────────────────────────
// SIM800L is a PoC stand-in for the Iridium 9603.
// HTTP calls map to SBD calls in production.
HardwareSerial sim800(1);

const String FIREBASE_HOST   = "https://esp32-gps-firebase-9a2b8-default-rtdb.firebaseio.com";
const String FIREBASE_SECRET = "Kc7VAThdjgqty4ENZnauQ0QXp3qlum4myuo5h6xj";

float  g_airtime     = -1.0;
String g_data_str    = "";
int    g_tx_interval = 30;

// ── Cached sensor values ──────────────────────────────────────────
struct SensorReading {
  // surface_log fields
  float  la      = -31.975801f;
  float  ln      =  11.580919f;
  int    battery =  85;
  String surf_ts = "";

  // sensor_log fields
  float  t       =   0.0f;   // temperature
  float  p       = 1013.0f;  // pressure
  float  ax      =   0.0f;
  float  ay      =   0.0f;
  float  az      =   9.81f;
  float  gx      =   0.0f;
  float  gy      =   0.0f;
  float  gz      =   0.0f;
  String sens_ts = "";

  bool valid    = false;
} g_reading;

// ── Persistent state counters ─────────────────────────────────────
int g_gsm_cycles      = 0;
int g_nrf_extractions = 0;
int g_sim_cycle       = 0;

// ── NRF packet counters (reset each transfer) ─────────────────────
uint32_t nrf_failedPackets = 0;
uint32_t nrf_totalPackets  = 0;

// ══════════════════════════════════════════════════════════════════
//  SIMULATION MODE
// ══════════════════════════════════════════════════════════════════
#define SIMULATE_SENSORS

#define SIM_BATCH_SIZE          1000
#define SIM_SAMPLE_INTERVAL_MIN   30
#define SIM_GAP_HOURS              6

#ifdef SIMULATE_SENSORS
static float sim_lat    = -31.975801f;
static float sim_lng    =  11.580919f;
static float sim_batt   =  85.0f;
static float sim_depth  =   0.0f;       // internal only - not transmitted
static float sim_max_depth = 38.0f;
static bool  sim_diving = true;

// Simulated IMU state
static float sim_ax = 0.0f, sim_ay = 0.0f, sim_az = 9.81f;
static float sim_gx = 0.0f, sim_gy = 0.0f, sim_gz = 0.0f;

String simTimestamp() {
  const int dpm[] = {31,29,31,30,31,30,31,31,30,31,30,31};
  int totalMins = 600 + g_sim_cycle * SIM_SAMPLE_INTERVAL_MIN;
  int dayOfYear = 27 + totalMins / 1440;
  int timeMin   = totalMins % 1440;
  int year = 2024, month = 0;
  while (dayOfYear >= dpm[month]) {
    dayOfYear -= dpm[month];
    month++;
    if (month >= 12) { month = 0; year++; }
  }
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:00",
           year, month+1, dayOfYear+1, timeMin/60, timeMin%60);
  return String(buf);
}

void simulateIMU() {
  float waveNoise = sim_diving ? 0.05f : 0.30f;
  sim_ax = (float)random(-10, 10) / 100.0f * waveNoise;
  sim_ay = (float)random(-10, 10) / 100.0f * waveNoise;
  sim_az = 9.81f + (float)random(-5, 5) / 100.0f * waveNoise;
  sim_gx = (float)random(-5, 5) / 1000.0f;
  sim_gy = (float)random(-5, 5) / 1000.0f;
  sim_gz = (float)random(-5, 5) / 1000.0f;
}

void simulateAndWrite() {
  if (!g_sd_ok) return;
  g_sim_cycle++;

  float step = 3.0f + (float)random(-5, 10) / 10.0f;
  if (sim_diving) {
    sim_depth += step;
    if (sim_depth >= sim_max_depth) { sim_depth = sim_max_depth; sim_diving = false; }
  } else {
    sim_depth -= step;
    if (sim_depth <= 0.0f) {
      sim_depth = 0.0f; sim_diving = true;
      sim_max_depth = 15.0f + (float)random(0, 60);
    }
  }

  float dlng = -(0.020f + (float)random(0, 30) / 1000.0f);
  float dlat  = (dlng * -0.361f) + (float)random(-50, 80) / 1000.0f;
  sim_lat += dlat;
  sim_lng += dlng;

  float temp = 10.0f - (sim_depth * 0.05f) + (float)random(-5, 5) / 10.0f;
  float pres = 1013.0f + (sim_depth * 9.87f) + (float)random(-5, 5) / 10.0f;
  sim_batt -= 0.15f;
  if (sim_batt < 0) sim_batt = 0;

  simulateIMU();

  String ts = simTimestamp();

  File sf = SD.open(SENSOR_FILE, FILE_APPEND);
  if (sf) {
    sf.printf("{\"ts\":\"%s\",\"t\":%.1f,\"p\":%.1f,"
              "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
              "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}\n",
              ts.c_str(), temp, pres,
              sim_ax, sim_ay, sim_az,
              sim_gx, sim_gy, sim_gz);
    sf.close();
  }

  File svf = SD.open(SURFACE_FILE, FILE_WRITE);
  if (svf) {
    svf.printf("{\"ts\":\"%s\",\"la\":%.6f,\"ln\":%.6f,\"b\":%d}\n",
               ts.c_str(), sim_lat, sim_lng, (int)sim_batt);
    svf.close();
  }

  if (!g_sim_silent)
    Serial.printf("[SIM] cycle=%d lat=%.5f lng=%.5f batt=%.0f%% "
                  "ax=%.2f ay=%.2f az=%.2f\n",
                  g_sim_cycle, sim_lat, sim_lng, sim_batt,
                  sim_ax, sim_ay, sim_az);
}

void forceSurface() {
  sim_depth = 0.0f; sim_diving = true;
  simulateIMU();
  if (!g_sd_ok) return;
  String ts   = simTimestamp();
  float temp  = 10.0f + (float)random(-5, 5) / 10.0f;
  float pres  = 1013.0f + (float)random(-5, 5) / 10.0f;

  File svf = SD.open(SURFACE_FILE, FILE_WRITE);
  if (svf) {
    svf.printf("{\"ts\":\"%s\",\"la\":%.6f,\"ln\":%.6f,\"b\":%d}\n",
               ts.c_str(), sim_lat, sim_lng, (int)sim_batt);
    svf.close();
  }
  File sf = SD.open(SENSOR_FILE, FILE_APPEND);
  if (sf) {
    sf.printf("{\"ts\":\"%s\",\"t\":%.1f,\"p\":%.1f,"
              "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
              "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}\n",
              ts.c_str(), temp, pres,
              sim_ax, sim_ay, sim_az,
              sim_gx, sim_gy, sim_gz);
    sf.close();
  }
}
#endif // SIMULATE_SENSORS

// ══════════════════════════════════════════════════════════════════
//  FIRST-FLASH SD SETUP
// ══════════════════════════════════════════════════════════════════
#define FF_GEN_ENTRIES    1000
#define FF_INTERVAL_MIN     30
#define FF_STATE_INIT  "{\"gsm_cycles\":0,\"nrf_extractions\":0,\"sim_cycle\":0}"

static int  ff_start_year;
static int  ff_start_doy;
static int  ff_start_min;
static char ff_now_ts[24];

static void ffParseDate(int &y, int &mo, int &d) {
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mstr[4] = { __DATE__[0], __DATE__[1], __DATE__[2], 0 };
  mo = 1;
  for (int i = 0; i < 12; i++)
    if (strncmp(mstr, months + i*3, 3) == 0) { mo = i+1; break; }
  d = (__DATE__[4]==' ') ? (__DATE__[5]-'0')
                         : ((__DATE__[4]-'0')*10 + (__DATE__[5]-'0'));
  y = (__DATE__[7]-'0')*1000 + (__DATE__[8]-'0')*100
    + (__DATE__[9]-'0')*10   + (__DATE__[10]-'0');
}
static void ffParseTime(int &h, int &mi) {
  h  = (__TIME__[0]-'0')*10 + (__TIME__[1]-'0');
  mi = (__TIME__[3]-'0')*10 + (__TIME__[4]-'0');
}
static bool ffIsLeap(int y) { return (y%4==0)&&(y%100!=0||y%400==0); }

static void ffInitBaseTime() {
  int cy, cm, cd, ch, cmi;
  ffParseDate(cy, cm, cd);
  ffParseTime(ch, cmi);
  int dpm[] = {31, ffIsLeap(cy)?29:28, 31,30,31,30,31,31,30,31,30,31};
  int doy = cd - 1;
  for (int i = 0; i < cm-1; i++) doy += dpm[i];
  snprintf(ff_now_ts, sizeof(ff_now_ts), "%04d-%02d-%02dT%02d:%02d:00", cy, cm, cd, ch, cmi);
  int nowMins   = doy*1440 + ch*60 + cmi;
  int startMins = nowMins - (FF_GEN_ENTRIES-1)*FF_INTERVAL_MIN;
  if (startMins < 0) { cy--; startMins += (ffIsLeap(cy)?366:365)*1440; }
  ff_start_year = cy;
  ff_start_doy  = startMins / 1440;
  ff_start_min  = startMins % 1440;
}

static String ffGenTimestamp(int cycle) {
  int year = ff_start_year;
  bool leap = ffIsLeap(year);
  int dpm[] = {31, leap?29:28, 31,30,31,30,31,31,30,31,30,31};
  int totalMins = ff_start_min + cycle*FF_INTERVAL_MIN;
  int doy       = ff_start_doy + totalMins/1440;
  int timeMin   = totalMins % 1440;
  int month = 0;
  while (doy >= dpm[month]) {
    doy -= dpm[month++];
    if (month >= 12) { month=0; year++; leap=ffIsLeap(year); dpm[1]=leap?29:28; }
  }
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:00",
           year, month+1, doy+1, timeMin/60, timeMin%60);
  return String(buf);
}

static void ffGenerateSensorLog() {
  File f = SD.open(SENSOR_FILE, FILE_WRITE);
  if (!f) { Serial.println("[FF] ERR: cannot open sensor_log.txt"); return; }
  float depth     = 10.0f;
  float max_depth = 10.0f + (float)random(0, 65);
  bool  diving    = true;
  float ax, ay, az = 9.81f, gx, gy, gz;
  for (int i = 0; i < FF_GEN_ENTRIES; i++) {
    float temp = 10.5f - depth*0.05f + (float)random(-3,  3)/10.0f;
    float pres = 1013.0f + depth*9.87f + (float)random(-20,20)/10.0f;
    float wn = (depth < 1.0f) ? 0.30f : 0.05f;
    ax = (float)random(-10,10)/100.0f * wn;
    ay = (float)random(-10,10)/100.0f * wn;
    az = 9.81f + (float)random(-5,5)/100.0f * wn;
    gx = (float)random(-5,5)/1000.0f;
    gy = (float)random(-5,5)/1000.0f;
    gz = (float)random(-5,5)/1000.0f;
    String ts = ffGenTimestamp(i);
    f.printf("{\"ts\":\"%s\",\"t\":%.1f,\"p\":%.1f,"
             "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
             "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}\n",
             ts.c_str(), temp, pres, ax, ay, az, gx, gy, gz);
    float step = 3.0f + (float)random(0, 50)/10.0f;
    if (diving) {
      depth += step;
      if (depth >= max_depth) { depth = max_depth; diving = false; }
    } else {
      depth -= step * 1.3f;
      if (depth <= 0.0f) {
        depth = 0.0f; diving = true;
        max_depth = 10.0f + (float)random(0, 65);
      }
    }
    if ((i+1)%100==0) Serial.printf("[FF]  %d/%d\n", i+1, FF_GEN_ENTRIES);
  }
  f.close();
}

void runFirstFlashSetup() {
  Serial.println("\n[FF] ===== First Flash SD Setup =====");
  ffInitBaseTime();
  Serial.printf("[FF] Compile time : %s\n", ff_now_ts);
  const char* toRemove[] = { SENSOR_FILE, SURFACE_FILE, STATE_FILE };
  for (int i = 0; i < 3; i++) if (SD.exists(toRemove[i])) SD.remove(toRemove[i]);
  Serial.printf("[FF] Writing %d sensor entries...\n", FF_GEN_ENTRIES);
  ffGenerateSensorLog();
  Serial.println("[FF] sensor_log.txt done");
  File stf = SD.open(STATE_FILE, FILE_WRITE);
  if (stf) { stf.println(FF_STATE_INIT); stf.close(); Serial.println("[FF] state.json done"); }
  else       Serial.println("[FF] ERR: cannot write state.json");
  Serial.println("[FF] ===== First Flash Setup Complete =====\n");
}

// ── SD helpers ────────────────────────────────────────────────────

float jsonFloat(const String& json, const char* key) {
  String search = String("\"") + key + "\":";
  int idx = json.indexOf(search);
  if (idx == -1) return 0.0f;
  int s = idx + search.length();
  while (s < (int)json.length() && json[s] == ' ') s++;
  int e = s;
  while (e < (int)json.length() && (isDigit(json[e]) || json[e] == '-' || json[e] == '.')) e++;
  return json.substring(s, e).toFloat();
}
int  jsonInt(const String& json, const char* key) { return (int)jsonFloat(json, key); }
long jsonLong(const String& json, const char* key) {
  String search = String("\"") + key + "\":";
  int idx = json.indexOf(search);
  if (idx == -1) return 0L;
  int s = idx + search.length();
  while (s < (int)json.length() && json[s] == ' ') s++;
  int e = s;
  while (e < (int)json.length() && (isDigit(json[e]) || json[e] == '-')) e++;
  return json.substring(s, e).toInt();
}

String jsonString(const String& json, const char* key) {
  String search = String("\"") + key + "\":\"";
  int idx = json.indexOf(search);
  if (idx == -1) return "";
  int s = idx + search.length();
  int e = json.indexOf('"', s);
  if (e == -1) return "";
  return json.substring(s, e);
}

String readLastLine(const char* filename) {
  File f = SD.open(filename, FILE_READ);
  if (!f) return "";
  long size = f.size();
  if (size == 0) { f.close(); return ""; }
  long pos = size - 1;
  while (pos > 0) { f.seek(pos); if (f.read() != '\n' && f.peek() != '\r') break; pos--; }
  long lineEnd = pos;
  while (pos > 0) { f.seek(pos - 1); if (f.read() == '\n') break; pos--; }
  f.seek(pos);
  String line = "";
  while (f.available() && f.position() <= (unsigned long)lineEnd + 1) {
    char c = f.read();
    if (c == '\n' || c == '\r') break;
    line += c;
  }
  f.close();
  return line;
}

void refreshFromSD() {
  if (!g_sd_ok) return;

  String sLine = readLastLine(SENSOR_FILE);
  if (sLine.length() > 2) {
    g_reading.t       = jsonFloat(sLine, "t");
    g_reading.p       = jsonFloat(sLine, "p");
    g_reading.ax      = jsonFloat(sLine, "ax");
    g_reading.ay      = jsonFloat(sLine, "ay");
    g_reading.az      = jsonFloat(sLine, "az");
    g_reading.gx      = jsonFloat(sLine, "gx");
    g_reading.gy      = jsonFloat(sLine, "gy");
    g_reading.gz      = jsonFloat(sLine, "gz");
    g_reading.sens_ts = jsonString(sLine, "ts");
    Serial.printf("[SD] Sensor: t=%.1f p=%.1f ax=%.2f ay=%.2f az=%.2f\n",
                  g_reading.t, g_reading.p,
                  g_reading.ax, g_reading.ay, g_reading.az);
  } else {
    Serial.println("[SD] WARN: sensor_log.txt empty or missing");
  }

  String sfLine = readLastLine(SURFACE_FILE);
  if (sfLine.length() > 2) {
    g_reading.la      = jsonFloat(sfLine, "la");
    g_reading.ln      = jsonFloat(sfLine, "ln");
    g_reading.battery = jsonInt   (sfLine, "b");
    g_reading.surf_ts = jsonString(sfLine, "ts");
    Serial.printf("[SD] Surface: la=%.5f ln=%.5f b=%d%%\n",
                  g_reading.la, g_reading.ln, g_reading.battery);
  } else {
    Serial.println("[SD] surface_log absent - will be created on first surface event");
  }
  g_reading.valid = true;
}

void saveState() {
  if (!g_sd_ok) return;
  File f = SD.open(STATE_FILE, FILE_WRITE);
  if (!f) { Serial.println("[STATE] Write failed"); return; }
  f.printf("{\"gsm_cycles\":%d,\"nrf_extractions\":%d,\"sim_cycle\":%d}\n",
           g_gsm_cycles, g_nrf_extractions, g_sim_cycle);
  f.close();
  Serial.printf("[STATE] Saved: gsm=%d nrf=%d sim_cycle=%d\n",
                g_gsm_cycles, g_nrf_extractions, g_sim_cycle);
}

void loadState() {
  File f = SD.open(STATE_FILE, FILE_READ);
  if (!f) { Serial.println("[STATE] No state file - starting fresh"); return; }
  String s = "";
  while (f.available()) s += (char)f.read();
  f.close();
  g_gsm_cycles      = jsonInt(s, "gsm_cycles");
  g_nrf_extractions = jsonInt(s, "nrf_extractions");
  g_sim_cycle       = jsonInt(s, "sim_cycle");
  Serial.printf("[STATE] Loaded: gsm=%d nrf=%d sim_cycle=%d\n",
                g_gsm_cycles, g_nrf_extractions, g_sim_cycle);
}

// ══════════════════════════════════════════════════════════════════
//  PAYLOAD BUILDERS
// ══════════════════════════════════════════════════════════════════

String bestTimestamp() {
  if (g_reading.surf_ts.length() > 0) return g_reading.surf_ts;
  if (g_reading.sens_ts.length() > 0) return g_reading.sens_ts;
  return "";
}

String buildSBDPayload() {
  String ts = bestTimestamp();
  String j = "{";
  if (ts.length() > 0) j += "\"ts\":\"" + ts + "\",";
  j += "\"la\":"  + String(g_reading.la,  4)  + ",";
  j += "\"ln\":"  + String(g_reading.ln,  4)  + ",";
  j += "\"b\":"   + String(g_reading.battery);
  j += "}";
  return j;
}

String buildHTTPPayload(const String& networkTs) {
  String ts = bestTimestamp();
  String j = "{";
  if (ts.length() > 0) j += "\"ts\":\"" + ts + "\",";
  j += "\"la\":"  + String(g_reading.la,  6)  + ",";
  j += "\"ln\":"  + String(g_reading.ln,  6)  + ",";
  j += "\"b\":"   + String(g_reading.battery) + ",";
  if (g_airtime >= 0)
    j += "\"airtime\":" + String(g_airtime, 2) + ",";
  if (g_data_str.length() > 0)
    j += "\"data_mb\":\"" + g_data_str + "\",";
  j += "\"network_ts\":\"" + networkTs + "\"";
  j += "}";
  return j;
}

// ══════════════════════════════════════════════════════════════════
//  GSM / FIREBASE
// ══════════════════════════════════════════════════════════════════

void flushSerial() { delay(200); while (sim800.available()) sim800.read(); }

String simRead(int waitMs = 1500) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)waitMs) delay(50);
  String r = "";
  while (sim800.available()) r += (char)sim800.read();
  return r;
}

void safeHttpTerm() {
  sim800.println("AT+HTTPSTATUS"); delay(1000);
  String s = ""; while (sim800.available()) s += (char)sim800.read();
  if (s.indexOf("IDLE") == -1) { sim800.println("AT+HTTPTERM"); delay(2000); while (sim800.available()) sim800.read(); }
}

void waitForNetwork() {
  Serial.println("[NET] Waiting for MTN...");
  for (int i = 0; i < 20; i++) {
    sim800.println("AT+COPS?"); delay(1000);
    String r = ""; while (sim800.available()) r += (char)sim800.read();
    if (r.indexOf("Mobile Telephone Network") != -1) { Serial.println("[NET] Registered on MTN"); return; }
    delay(2000);
  }
  Serial.println("[NET] WARNING: timeout");
}

int checkSignal(int minCSQ = 16, int maxAttempts = 10) {
  Serial.println("[SIG] Checking...");
  for (int i = 0; i < maxAttempts; i++) {
    sim800.println("AT+CSQ"); delay(1000);
    String r = ""; while (sim800.available()) r += (char)sim800.read();
    int idx = r.indexOf("+CSQ: ");
    if (idx == -1) { delay(2000); continue; }
    int s = idx + 6, comma = r.indexOf(',', s);
    if (comma == -1) { delay(2000); continue; }
    int csq = r.substring(s, comma).toInt();
    int dbm = (csq != 99) ? -113 + csq * 2 : 0;
    String q = (csq == 99 || csq == 0) ? "NO SIGNAL"
             : (csq <= 15) ? "MARGINAL" : (csq <= 20) ? "FAIR" : "EXCELLENT";
    Serial.printf("[SIG] CSQ:%d | %ddBm | %s\n", csq, dbm, q.c_str());
    if (csq != 99 && csq >= minCSQ) return csq;
    Serial.printf("[SIG] Too weak (need >=%d), retrying...\n", minCSQ);
    delay(5000);
  }
  Serial.println("[SIG] WARNING: Could not get sufficient signal");
  return 0;
}

int readCSQ() {
  sim800.println("AT+CSQ"); delay(1000);
  String r = ""; while (sim800.available()) r += (char)sim800.read();
  int idx = r.indexOf("+CSQ:");
  if (idx == -1) return 99;
  int s = idx + 5;
  while (s < (int)r.length() && r[s] == ' ') s++;
  int comma = r.indexOf(',', s);
  return (comma == -1) ? 99 : r.substring(s, comma).toInt();
}

bool openGPRS() {
  Serial.println("[GPRS] Opening bearer...");
  sim800.println("AT+SAPBR=0,1"); delay(3000); flushSerial();
  sim800.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\""); simRead();
  sim800.println("AT+SAPBR=3,1,\"APN\",\"internet\"");  simRead();
  Serial.println("[GPRS] Connecting...");
  sim800.println("AT+SAPBR=1,1"); delay(10000); flushSerial();
  sim800.println("AT+SAPBR=2,1"); delay(3000);
  String ip = ""; while (sim800.available()) ip += (char)sim800.read();
  if (ip.indexOf("0.0.0.0") != -1 || ip.indexOf("SAPBR") == -1) {
    Serial.println("[GPRS] FAILED - no IP"); return false;
  }
  int q1 = ip.indexOf('"'), q2 = ip.lastIndexOf('"');
  Serial.println("[GPRS] Connected - IP: " + (q1 != -1 && q2 > q1 ? ip.substring(q1+1, q2) : "?"));
  return true;
}

bool checkGPRS() {
  sim800.println("AT+SAPBR=2,1"); delay(2000);
  String r = ""; while (sim800.available()) r += (char)sim800.read();
  return r.indexOf("1,1") != -1 && r.indexOf("0.0.0.0") == -1;
}

bool ensureGPRS() {
  if (checkGPRS()) return true;
  Serial.println("[GPRS] Bearer dropped - reconnecting...");
  safeHttpTerm();
  return openGPRS();
}

void syncNTP() {
  Serial.println("[NTP] Syncing...");
  sim800.println("AT+CNTP=\"pool.ntp.org\",8"); simRead(2000);
  sim800.println("AT+CNTP"); simRead(5000);
  Serial.println("[NTP] Done");
}

String getNetworkTime() {
  sim800.println("AT+CCLK?"); delay(1000);
  String r = ""; while (sim800.available()) r += (char)sim800.read();
  int q1 = r.indexOf('"'), q2 = r.lastIndexOf('"');
  if (q1 == -1 || q2 <= q1 || (q2 - q1) < 18) return "1970-01-01T00:00:00";
  String raw = r.substring(q1 + 1, q2);
  return "20" + raw.substring(0,2) + "-" + raw.substring(3,5) + "-"
              + raw.substring(6,8) + "T" + raw.substring(9,17);
}

void checkBalance() {
  Serial.println("[BAL] Checking MTN balance...");
  sim800.println("AT+CUSD=1,\"*141#\"");
  String resp = ""; unsigned long t = millis();
  while (millis() - t < 15000) {
    if (sim800.available()) resp += (char)sim800.read();
  }
  sim800.println("AT+CUSD=2"); simRead(1000);
  g_airtime = -1.0f;
  int ai = resp.indexOf("Airtime:R");
  if (ai != -1) {
    int s = ai + 9, e = resp.indexOf('\n', s);
    if (e == -1) e = resp.indexOf('"', s);
    if (e != -1) { String v = resp.substring(s, e); v.trim(); g_airtime = v.toFloat(); }
  }
  g_data_str = "";
  int di = resp.indexOf("Data:");
  if (di != -1) {
    int s = di + 5, e = resp.indexOf('\n', s);
    if (e == -1) e = resp.indexOf('"', s);
    if (e != -1) { g_data_str = resp.substring(s, e); g_data_str.trim(); }
  }
  Serial.printf("[BAL] Airtime: R%.2f | Data: %s\n",
                g_airtime, g_data_str.length() > 0 ? g_data_str.c_str() : "none");
}

void httpSetup(const String& url) {
  flushSerial(); safeHttpTerm();
  sim800.println("AT+HTTPINIT");                           simRead(2000);
  sim800.println("AT+HTTPSSL=1");                          simRead(1000);
  sim800.println("AT+HTTPPARA=\"CID\",1");                 simRead(1000);
  sim800.println("AT+HTTPPARA=\"URL\",\"" + url + "\"");   simRead(2000);
  sim800.println("AT+HTTPPARA=\"REDIR\",1");               simRead(1000);
}

String waitHTTPAction(int timeoutMs = 60000) {
  String resp = ""; unsigned long start = millis();
  while (millis() - start < (unsigned long)timeoutMs) {
    if (sim800.available()) {
      char c = sim800.read(); resp += c;
      if (resp.indexOf("+HTTPACTION:") != -1 && resp.endsWith("\n")) break;
    }
  }
  Serial.printf("[HTTP] Response in %lums\n", millis() - start);
  return resp;
}

String readUntilSilence(int silenceMs = 1500) {
  String r = ""; unsigned long last = millis();
  while (millis() - last < (unsigned long)silenceMs) {
    while (sim800.available()) { char c = sim800.read(); r += c; last = millis(); }
  }
  return r;
}

bool httpPost(const String& url, const String& json) {
  Serial.println("[HTTP] POST " + url.substring(url.lastIndexOf('/'), url.indexOf('?')));
  if (!ensureGPRS()) { Serial.println("[HTTP] POST skipped - no GPRS"); return false; }
  httpSetup(url);
  sim800.println("AT+HTTPPARA=\"CONTENT\",\"application/json\""); simRead(1000);
  sim800.println("AT+HTTPDATA=" + String(json.length()) + ",10000");  simRead(3000);
  sim800.print(json); simRead(4000);
  flushSerial();
  sim800.println("AT+HTTPACTION=1"); simRead(2000);
  String resp = waitHTTPAction();
  bool ok = resp.indexOf(",200,") != -1 || resp.indexOf(",204,") != -1;
  Serial.println(ok ? "[HTTP] POST OK" : "[HTTP] POST FAILED - " + resp);
  safeHttpTerm(); flushSerial();
  return ok;
}

String httpGet(const String& url) {
  Serial.println("[HTTP] GET " + url.substring(url.lastIndexOf('/'), url.indexOf('?')));
  if (!ensureGPRS()) { Serial.println("[HTTP] GET skipped - no GPRS"); return ""; }
  httpSetup(url);
  flushSerial();
  sim800.println("AT+HTTPACTION=0"); simRead(2000);
  String actionResp = waitHTTPAction();
  String data = "";
  if (actionResp.indexOf(",200,") != -1) {
    int lc = actionResp.lastIndexOf(',');
    int bodyLen = (lc != -1) ? actionResp.substring(lc + 1).toInt() : 0;
    sim800.println("AT+HTTPREAD");
    delay(max(2000, bodyLen * 2));
    String raw = readUntilSilence(1000);
    int hdrEnd = raw.indexOf('\n');
    data = (hdrEnd != -1) ? raw.substring(hdrEnd + 1) : raw;
    data.trim();
  } else {
    Serial.println("[HTTP] GET FAILED - " + actionResp);
  }
  safeHttpTerm(); flushSerial();
  return data;
}

#ifdef SIMULATE_SENSORS
void fetchLatestPositionFromFirebase() {
  Serial.println("[SIM] Fetching latest GPS from Firebase...");
  String body = httpGet(FIREBASE_HOST + "/buoy/history.json?auth=" + FIREBASE_SECRET
                        + "&orderBy=%22%24key%22&limitToLast=1");
  if (body.length() < 5) { Serial.println("[SIM] No position data found"); return; }
  float lat = jsonFloat(body, "la");
  float lng = jsonFloat(body, "ln");
  if (lat != 0.0f || lng != 0.0f) {
    sim_lat = lat; sim_lng = lng;
    Serial.printf("[SIM] Starting position from Firebase: %.6f, %.6f\n", sim_lat, sim_lng);
  }
}
#endif

bool appendHistory(const String& json) {
  return httpPost(FIREBASE_HOST + "/buoy/history.json?auth=" + FIREBASE_SECRET, json);
}

void writeCommandStatus(const String& status) {
  httpPost(FIREBASE_HOST + "/buoy/ctrl/command/status.json?auth=" + FIREBASE_SECRET
           + "&x-http-method-override=PUT", "\"" + status + "\"");
}

// readCtrl - reads Firebase ctrl node.
// Updates: g_tx_interval, g_cycle_active, g_tx_mode, g_surface_hold
//
// Firebase ctrl fields:
//   "tx_interval":  <int seconds>
//   "tx_mode":      "long" | "short"
//   "command": { "cmd": "start"|"stop"|"mode_long"|"mode_short"|"surface_hold",
//                "status": "pending" }
//
// surface_hold lifecycle:
//   1. Operator sends "surface_hold" via Firebase
//   2. readCtrl() sets g_surface_hold=true, saves g_pre_hold_mode, switches TX_SHORT
//   3. LIVE_SEARCHING alternates GSM pings and NRF24 probes until transfer succeeds
//   4. On success: g_tx_mode restored to g_pre_hold_mode, g_surface_hold cleared,
//      buoy moves to LIVE_PAUSED
//   5. Operator sends "start" → LIVE_IDLE in restored mode
bool readCtrl() {
  String body = httpGet(FIREBASE_HOST + "/buoy/ctrl.json?auth=" + FIREBASE_SECRET);
  if (body.length() < 3 || body.indexOf("null") != -1) {
    Serial.println("[CTRL] No data"); return false;
  }

  // tx_interval
  int idx = body.indexOf("\"tx_interval\":");
  if (idx != -1) {
    int s = idx + 14;
    while (s < (int)body.length() && body[s] == ' ') s++;
    int e = s;
    while (e < (int)body.length() && (isDigit(body[e]) || body[e] == '-')) e++;
    if (e > s) {
      int val = body.substring(s, e).toInt();
      g_tx_interval = (val == 0) ? 0 : max(val, 30);
    }
    Serial.printf("[CFG] tx_interval=%ds\n", g_tx_interval);
  }

  // command
  if (body.indexOf("\"status\":\"pending\"") == -1) {
    Serial.println("[CMD] No pending command"); return false;
  }
  int ci = body.indexOf("\"cmd\":\"");
  if (ci == -1) return false;
  int cs = ci + 7, ce = body.indexOf('"', cs);
  if (ce == -1) return false;
  String cmd = body.substring(cs, ce);
  Serial.println("[CMD] Received: " + cmd);
  writeCommandStatus("processing");

  if (cmd == "start") {
    g_cycle_active = true;
    Serial.println("[CMD] START - transmission resumed");

  } else if (cmd == "stop") {
    g_cycle_active = false;
    if (g_surface_hold) {
      g_surface_hold = false;
      g_tx_mode      = g_pre_hold_mode;
      Serial.println("[CMD] STOP - extraction aborted, mode restored");
    }
#ifdef SIMULATE_SENSORS
    forceSurface();
#endif
    Serial.println("[CMD] STOP - transmission suppressed");

  } else if (cmd == "mode_long") {
    g_tx_mode = TX_LONG;
    Serial.println("[CMD] MODE → LONG (GSM/Iridium)");

  } else if (cmd == "mode_short") {
    g_tx_mode = TX_SHORT;
    Serial.println("[CMD] MODE → SHORT (NRF24 bulk)");

  } else if (cmd == "surface_hold") {
    // Save current mode so it can be restored after extraction.
    // Buoy stays surfaced in LIVE_SEARCHING until transfer succeeds,
    // then returns to this mode when operator sends "start".
    g_pre_hold_mode = g_tx_mode;
    g_surface_hold  = true;
    g_tx_mode       = TX_SHORT;
    Serial.printf("[CMD] SURFACE_HOLD - staying surfaced for NRF24 extraction\n");
    Serial.printf("[CMD] Pre-hold mode saved: %s - will restore on resume\n",
                  g_pre_hold_mode == TX_LONG ? "LONG" : "SHORT");
    // Transition to LIVE_SEARCHING on next loop iteration.
    // If already in LIVE_IDLE or LIVE_TRANSMIT the state will
    // pick this up; if in LIVE_PAUSED it stays paused until start.
    if (g_state == LIVE_IDLE || g_state == LIVE_TRANSMIT) {
      g_state = LIVE_SEARCHING;
    }
  }

  writeCommandStatus("done");
  return false;
}

// ══════════════════════════════════════════════════════════════════
//  NRF24 BULK TRANSFER
// ══════════════════════════════════════════════════════════════════

void logToSD(const char* level, const char* msg) {
  File f = SD.open(LOG_FILE, FILE_APPEND);
  if (f) {
    f.print("["); f.print(millis()); f.print("ms] [");
    f.print(level); f.print("] "); f.println(msg);
    f.close();
  }
  Serial.print("["); Serial.print(millis());
  Serial.print("ms] ["); Serial.print(level);
  Serial.print("] "); Serial.println(msg);
}

void logToSDf(const char* level, const char* fmt, ...) {
  char buf[128]; va_list args;
  va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
  logToSD(level, buf);
}

// waitForACK - listens for 0xAA ACK byte from receiver.
// timeoutMs: how long to wait. Use ACK_PROBE_TIMEOUT for the initial
// PKT_START handshake (fast-fail when no receiver in range).
// Use ACK_TIMEOUT (default) for all data packets.
bool waitForACK(unsigned long timeoutMs = ACK_TIMEOUT) {
  radio.startListening();
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (radio.available()) {
      uint8_t ack; radio.read(&ack, 1);
      if (ack == 0xAA) return true;
    }
    delay(2);
  }
  return false;
}

// sendWithRetry - sends a packet and waits for ACK.
// ackTimeoutMs / maxRetries: caller controls per-packet timeout.
// PKT_START uses ACK_PROBE_TIMEOUT + ACK_PROBE_RETRIES (fast-fail).
// All data packets use ACK_TIMEOUT + default retries (reliable).
// runBulkTransfer() does NOT increment g_nrf_extractions - caller's responsibility.
bool sendWithRetry(Packet& pkt, int maxRetries = 5,
                   unsigned long ackTimeoutMs = ACK_TIMEOUT) {
  nrf_totalPackets++;
  for (int i = 0; i < maxRetries; i++) {
    radio.stopListening(); delay(10);
    radio.write(&pkt, sizeof(pkt)); delay(5);
    if (waitForACK(ackTimeoutMs)) return true;
    logToSDf("WARN", "Retry %d line=%u chunk=%u", i+1, pkt.lineNum, pkt.chunkIdx);
    delay(200 + (i * 100));
  }
  nrf_failedPackets++;
  logToSDf("ERROR", "PACKET_FAILED line=%u chunk=%u", pkt.lineNum, pkt.chunkIdx);
  return false;
}

bool sendFileHeader(const char* name) {
  Packet pkt;
  pkt.type = PKT_FILE; pkt.lineNum = 0; pkt.chunkIdx = 0;
  memset(pkt.data, 0, sizeof(pkt.data));
  strncpy(pkt.data, name, sizeof(pkt.data) - 1);
  logToSDf("INFO", "FILE header: %s", name);
  return sendWithRetry(pkt);
}

bool sendLine(uint16_t lineNum, const char* line) {
  int len         = strlen(line);
  int chunkSize   = sizeof(((Packet*)0)->data) - 1;
  int totalChunks = max(1, (len + chunkSize - 1) / chunkSize);
  for (int c = 0; c < totalChunks; c++) {
    Packet pkt;
    pkt.type = PKT_DATA; pkt.lineNum = lineNum; pkt.chunkIdx = c;
    memset(pkt.data, 0, sizeof(pkt.data));
    strncpy(pkt.data, line + c * chunkSize, chunkSize);
    if (!sendWithRetry(pkt)) return false;
  }
  Packet endPkt;
  endPkt.type = PKT_LINE_END; endPkt.lineNum = lineNum; endPkt.chunkIdx = totalChunks;
  memset(endPkt.data, 0, sizeof(endPkt.data));
  return sendWithRetry(endPkt);
}

int sendFile(const char* filename, uint16_t lineNumOffset) {
  File f = SD.open(filename, FILE_READ);
  if (!f) { logToSDf("WARN", "File not found: %s", filename); return 0; }
  uint16_t total = 0;
  while (f.available()) { if (f.read() == '\n') total++; }
  f.seek(0);
  logToSDf("INFO", "%s: %u lines", filename, total);
  uint16_t lineNum = lineNumOffset;
  char buf[256] = ""; int idx = 0;
  while (f.available()) {
    char c = f.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        buf[idx] = '\0'; lineNum++;
        Serial.printf("[NRF] %s line %u/%u\n", filename, lineNum - lineNumOffset, total);
        if (!sendLine(lineNum, buf)) logToSDf("ERROR", "LINE_FAILED line=%u", lineNum);
        idx = 0; memset(buf, 0, sizeof(buf));
      }
    } else if (idx < (int)sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
  f.close();
  return lineNum - lineNumOffset;
}

// runBulkTransfer - executes a complete NRF24 bulk transfer.
//
// PKT_START handshake uses ACK_PROBE_TIMEOUT (800ms) and
// ACK_PROBE_RETRIES (3) for fast-fail when no receiver is present.
// Worst-case blocking time on probe: ~2.4s (vs 15s with full timeout).
// This keeps the LIVE_SEARCHING GSM ping timer from drifting badly.
//
// All subsequent data packets use full ACK_TIMEOUT (3000ms).
//
// Returns true on successful transfer (PKT_END sent and ack'd).
// Does NOT increment g_nrf_extractions - caller's responsibility.
bool runBulkTransfer() {
  nrf_failedPackets = 0; nrf_totalPackets = 0;
  uint16_t sensorLines = 0, surfaceLines = 0;
  File f = SD.open(SENSOR_FILE,  FILE_READ);
  if (f) { while (f.available()) { if (f.read() == '\n') sensorLines++;  } f.close(); }
  f      = SD.open(SURFACE_FILE, FILE_READ);
  if (f) { while (f.available()) { if (f.read() == '\n') surfaceLines++; } f.close(); }
  uint16_t total = sensorLines + surfaceLines;
  logToSDf("INFO", "Transfer start: sensor=%u surface=%u total=%u",
           sensorLines, surfaceLines, total);

  // PKT_START: fast-fail handshake - short ACK window + fewer retries.
  // If no receiver responds within ~2.4s total we bail without sending data.
  Packet startPkt;
  startPkt.type = PKT_START; startPkt.lineNum = 0; startPkt.chunkIdx = 0;
  memset(startPkt.data, 0, sizeof(startPkt.data));
  snprintf(startPkt.data, sizeof(startPkt.data), "%u", total);
  if (!sendWithRetry(startPkt, ACK_PROBE_RETRIES, ACK_PROBE_TIMEOUT)) {
    logToSD("INFO", "No receiver in range - will retry next cycle");
    return false;
  }
  logToSD("INFO", "START ack'd - beginning transfer");

  if (!sendFileHeader("sensors")) { logToSD("ERROR", "FILE header failed"); return false; }
  int sentSensor = sendFile(SENSOR_FILE, 0);
  if (!sendFileHeader("surface")) { logToSD("ERROR", "FILE header failed"); return false; }
  int sentSurface = sendFile(SURFACE_FILE, (uint16_t)sentSensor);

  Packet endPkt;
  endPkt.type = PKT_END; endPkt.lineNum = (uint16_t)(sentSensor + sentSurface); endPkt.chunkIdx = 0;
  memset(endPkt.data, 0, sizeof(endPkt.data));
  sendWithRetry(endPkt);

  logToSDf("INFO", "Done: sensor=%d surface=%d pkts=%u failed=%u",
           sentSensor, sentSurface, nrf_totalPackets, nrf_failedPackets);
  for (int i = 0; i < 5; i++) { digitalWrite(LED_PIN, HIGH); delay(200); digitalWrite(LED_PIN, LOW); delay(200); }
  return true;
}

// ── Sim batch seeding ─────────────────────────────────────────────
void seedSimBatch(int count) {
  Serial.printf("[SIM] Seeding %d entries - first 3 shown, then hundreds\n", count);
  g_sim_silent = true;
  for (int i = 0; i < count; i++) {
    simulateAndWrite();
    if (i < 3) {
      Serial.printf("[SIM] cycle=%d lat=%.5f lng=%.5f batt=%.0f%% ax=%.2f ay=%.2f az=%.2f\n",
                    g_sim_cycle, sim_lat, sim_lng, sim_batt,
                    sim_ax, sim_ay, sim_az);
      if (i == 2) Serial.println("[SIM] ...");
    } else if ((i + 1) % 100 == 0) {
      Serial.printf("[SIM]   %d / %d  (cycle=%d)\n", i + 1, count, g_sim_cycle);
    }
  }
  g_sim_silent = false;
  Serial.printf("[SIM] Seeding complete - %d entries written, sim_cycle=%d\n", count, g_sim_cycle);
}

// ── Hardware init ─────────────────────────────────────────────────
bool initNRF() {
  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
  if (!radio.begin(&SPI)) { logToSD("ERROR", "NRF24 not found"); return false; }
  radio.openWritingPipe(NRF_ADDR);
  radio.openReadingPipe(1, NRF_ADDR);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 15);
  g_nrf_ready = true;
  logToSD("INFO", "NRF24 ready");
  return true;
}

bool initGSM() {
  sim800.setRxBufferSize(2048);
  sim800.begin(9600, SERIAL_8N1, 16, 17);
  delay(8000);
  sim800.println("AT");         simRead();
  sim800.println("AT+CFUN=1"); simRead();
  sim800.println("AT+CMEE=2"); simRead();
  waitForNetwork();
  int csq = checkSignal(16, 10);
  if (csq == 0) { Serial.println("[GSM] Signal too weak"); return false; }
  if (!openGPRS()) { Serial.println("[GSM] GPRS failed"); return false; }
  syncNTP();
  g_gsm_ready = true;
  Serial.println("[GSM] Ready");
  return true;
}

// ══════════════════════════════════════════════════════════════════
//  GSM CYCLE  (long distance transmission)
// ══════════════════════════════════════════════════════════════════

bool runGSMCycle() {
  Serial.println("\n========== GSM Cycle ==========");
  Serial.printf("[CYCLE] cycle_active=%s  tx_mode=%s  surface_hold=%s\n",
                g_cycle_active ? "YES" : "NO",
                g_tx_mode == TX_LONG ? "LONG" : "SHORT",
                g_surface_hold ? "YES" : "NO");
#ifdef SIMULATE_SENSORS
  if (g_cycle_active) simulateAndWrite();
#endif
  if (!g_gsm_ready && !initGSM()) { Serial.println("[CYCLE] GSM init failed"); return false; }
  if (!ensureGPRS())               { Serial.println("[CYCLE] No GPRS");        return false; }
  readCtrl();
  if (g_tx_interval == 0) return true;
  refreshFromSD();
  checkBalance();
  String networkTs = getNetworkTime();
  String payload   = buildHTTPPayload(networkTs);
  Serial.println("[DATA] Payload: " + payload);
  Serial.printf("[DATA] Payload size: %d bytes (Iridium limit: 340)\n", payload.length());
  bool ok = appendHistory(payload);
  if (ok) { g_gsm_cycles++; saveState(); }
  Serial.printf("[CYCLE] Done. gsm_cycles=%d\n\n", g_gsm_cycles);
  return ok;
}

// ══════════════════════════════════════════════════════════════════
//  TRANSMISSION DISPATCHER
//  Called by Live mode on each surface event.
// ══════════════════════════════════════════════════════════════════

void runTransmission() {
  Serial.printf("\n[TX] Surface event - mode=%s\n",
                g_tx_mode == TX_LONG ? "LONG (GSM)" : "SHORT (NRF24)");
  if (g_tx_mode == TX_LONG) {
    runGSMCycle();
  } else {
    if (!g_nrf_ready) initNRF();
    if (runBulkTransfer()) {
      // runBulkTransfer() does not increment g_nrf_extractions - done here
      g_nrf_extractions++;
      saveState();
      if (g_sd_ok) {
        SD.remove(SENSOR_FILE);
        // surface_log intentionally kept - holds last known GPS fix
        Serial.println("[SD] sensor_log cleared, surface_log retained");
      }
#ifdef SIMULATE_SENSORS
      g_sim_cycle += (SIM_GAP_HOURS * 60) / SIM_SAMPLE_INTERVAL_MIN;
      Serial.printf("[SIM] Jumped %d cycles (%dh gap)\n",
                    (SIM_GAP_HOURS * 60) / SIM_SAMPLE_INTERVAL_MIN, SIM_GAP_HOURS);
      seedSimBatch(SIM_BATCH_SIZE);
#endif
    }
  }
}

// ── Pause check (Live mode) ───────────────────────────────────────
bool livePaused() {
  return !g_cycle_active || (digitalRead(PAUSE_PIN) == LOW);
}

// ══════════════════════════════════════════════════════════════════
//  SETUP MODE SELECTION
// ══════════════════════════════════════════════════════════════════

void enterSetupMode(char sel) {
  g_nrf_ready   = false;
  g_prompt_said = false;
  if (sel == 'B') {
    Serial.println("[MENU] SHORT DISTANCE SETUP selected");
    g_state = SDS_NRF_WAIT;
  } else if (sel == 'A') {
    Serial.println("[MENU] LONG DISTANCE SETUP selected");
    if (!g_gsm_ready) {
      initGSM();
#ifdef SIMULATE_SENSORS
      if (g_gsm_ready) fetchLatestPositionFromFirebase();
#endif
    }
    g_state = LDS_S1_WAIT;
  } else {
    Serial.println("[MENU] FULL SETUP selected");
    if (!g_gsm_ready) {
      initGSM();
#ifdef SIMULATE_SENSORS
      if (g_gsm_ready) fetchLatestPositionFromFirebase();
#endif
    }
    g_state = FS_S1_WAIT;
  }
}

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));

  pinMode(SETUP_GATE_PIN, INPUT_PULLUP);
  pinMode(LIVE_MODE_PIN,  INPUT_PULLUP);
  pinMode(PAUSE_PIN,      INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  bool isSetup = (digitalRead(SETUP_GATE_PIN) == HIGH);

  Serial.println("\n===== MARIS Buoy V2 =====");
  Serial.printf("[BOOT] Mode: %s\n", isSetup ? "SETUP" : "LIVE");

  // ── SD card ────────────────────────────────────────────────────
  sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS, sdSpi)) { g_sd_ok = true; Serial.println("[SD] Card ready"); }
  else                          Serial.println("[SD] ERROR: card failed");

  if (g_sd_ok && (!SD.exists(STATE_FILE) || isSetup)) {
    if (isSetup) Serial.println("[FF] Setup mode - forced SD reinit");
    runFirstFlashSetup();
  }

  loadState();

  if (!isSetup) {
    // ── LIVE mode ─────────────────────────────────────────────────
    g_tx_mode      = (digitalRead(LIVE_MODE_PIN) == HIGH) ? TX_LONG : TX_SHORT;
    g_pre_hold_mode = g_tx_mode;  // initialise pre-hold to boot mode
    Serial.printf("[MODE] LIVE - tx_mode=%s  (LIVE_MODE_PIN=%s)\n",
                  g_tx_mode == TX_LONG ? "LONG" : "SHORT",
                  g_tx_mode == TX_LONG ? "HIGH" : "LOW");
    Serial.println("[MODE] Pause: Firebase stop/start or PAUSE_PIN LOW");
    Serial.println("[MODE] isSurfaced() provided by housing subsystem");
    Serial.println("[MODE] Extraction: Firebase surface_hold → LIVE_SEARCHING");

    if (g_tx_mode == TX_LONG) {
      initGSM();
#ifdef SIMULATE_SENSORS
      if (g_gsm_ready) fetchLatestPositionFromFirebase();
#endif
    }
    g_state = LIVE_IDLE;

  } else {
    // ── SETUP mode ────────────────────────────────────────────────
    keypadBegin();
    Serial.println("[BOOT] Select setup mode:");
    Serial.println("  'A' = Long Distance Setup  (GSM / Firebase verification)");
    Serial.println("  'B' = Short Distance Setup (NRF24 bulk transfer)");
    Serial.println("  '6' = Full Setup           (GSM + NRF24 end-to-end test)");
    char sel = 0;
    while (sel == 0) {
      char k = keypadScan();
      if (k == 'A' || k == 'B' || k == '6') { sel = k; keypadWaitRelease(); }
      delay(50);
    }
    enterSetupMode(sel);
  }

  // ── Seed simulation data ───────────────────────────────────────
#ifdef SIMULATE_SENSORS
  {
    File chk = SD.open(SENSOR_FILE, FILE_READ);
    bool hasData = chk && chk.size() > 0;
    if (chk) chk.close();
    if (!hasData) {
      seedSimBatch(SIM_BATCH_SIZE);
    } else {
      Serial.println("[SIM] Existing sensor_log found - using SD data");
    }
  }
  {
    File svchk = SD.open(SURFACE_FILE, FILE_READ);
    bool hasSurface = svchk && svchk.size() > 0;
    if (svchk) svchk.close();
    if (!hasSurface) {
      Serial.println("[SIM] No surface_log - creating initial surface fix");
      forceSurface();
    }
  }
#endif

  refreshFromSD();
}

// ══════════════════════════════════════════════════════════════════
//  LOOP - state machine
// ══════════════════════════════════════════════════════════════════

void loop() {
  switch (g_state) {

    // ════════════════════════════════════════════════════════════
    //  SHORT DISTANCE SETUP
    // ════════════════════════════════════════════════════════════

    case SDS_NRF_WAIT: {
      if (!g_prompt_said) {
        Serial.println("[SHORT DIST SETUP] Connect the NRF24 receiver, then press 'B'.");
        g_prompt_said = true;
      }
      waitForKey('B');
      Serial.println("[SHORT DIST SETUP] Starting NRF24 bulk transfer...");
      g_state = SDS_NRF_RUN;
      break;
    }

    case SDS_NRF_RUN:
      if (!g_nrf_ready) initNRF();
      if (runBulkTransfer()) {
        // runBulkTransfer() does not increment g_nrf_extractions - done here
        g_nrf_extractions++;
        saveState();
        Serial.printf("[SHORT DIST SETUP] Transfer complete - nrf_extractions=%d\n", g_nrf_extractions);
      }
      Serial.println("[SHORT DIST SETUP] Done.");
      g_state = SDS_DONE;
      break;

    case SDS_DONE:
      Serial.println("\n[MENU] ===== Setup complete =====");
      Serial.println("[MENU] '4' = Return to Setup menu  |  '5' = Launch Live mode");
      Serial.flush();
      g_state = SETUP_MENU;
      break;

    // ════════════════════════════════════════════════════════════
    //  LONG DISTANCE SETUP
    // ════════════════════════════════════════════════════════════

    case LDS_S1_WAIT: {
      if (!g_prompt_said) {
        Serial.println("[LONG DIST SETUP] Step 1 - press '1' to run first GSM cycle.");
        g_prompt_said = true;
      }
      waitForKey('1');
      Serial.println("[LONG DIST SETUP] Running GSM cycle...");
      g_state = LDS_S1_RUN;
      break;
    }

    case LDS_S1_RUN:
      if (runGSMCycle())
        Serial.printf("[LONG DIST SETUP] Step 1 complete - gsm_cycles=%d\n", g_gsm_cycles);
      Serial.println("[LONG DIST SETUP] Step 2 - set tx_interval=40 in Firebase, then press '2'.");
      g_state = LDS_S2_WAIT;
      break;

    case LDS_S2_WAIT:
      waitForKey('2');
      Serial.println("[LONG DIST SETUP] Running GSM cycle + command check...");
      g_state = LDS_S2_RUN;
      break;

    case LDS_S2_RUN:
      if (runGSMCycle())
        Serial.printf("[LONG DIST SETUP] Step 2 complete - gsm_cycles=%d  tx_interval=%ds\n",
                      g_gsm_cycles, g_tx_interval);
      Serial.println("[LONG DIST SETUP] Done - GSM / Firebase verified.");
      g_state = LDS_DONE;
      break;

    case LDS_DONE:
      Serial.println("\n[MENU] ===== Setup complete =====");
      Serial.println("[MENU] '4' = Return to Setup menu  |  '5' = Launch Live mode");
      Serial.flush();
      g_state = SETUP_MENU;
      break;

    // ════════════════════════════════════════════════════════════
    //  FULL SETUP
    // ════════════════════════════════════════════════════════════

    case FS_S1_WAIT: {
      if (!g_prompt_said) {
        Serial.println("[FULL SETUP] Step 1 - press '1' to run GSM cycle 1.");
        g_prompt_said = true;
      }
      waitForKey('1');
      Serial.println("[FULL SETUP] Running GSM cycle 1...");
      g_state = FS_S1_RUN;
      break;
    }

    case FS_S1_RUN:
      if (runGSMCycle())
        Serial.printf("[FULL SETUP] Step 1 complete - gsm_cycles=%d\n", g_gsm_cycles);
      Serial.println("[FULL SETUP] Step 2 - set tx_interval=40 in Firebase, then press '2'.");
      g_state = FS_S2_WAIT;
      break;

    case FS_S2_WAIT:
      waitForKey('2');
      Serial.println("[FULL SETUP] Running GSM cycle 2 + command check...");
      g_state = FS_S2_RUN;
      break;

    case FS_S2_RUN:
      if (runGSMCycle())
        Serial.printf("[FULL SETUP] Step 2 complete - gsm_cycles=%d  tx_interval=%ds\n",
                      g_gsm_cycles, g_tx_interval);
      Serial.println("[FULL SETUP] Step 3 - connect NRF24 receiver, then press '3'.");
      g_state = FS_S3_WAIT;
      break;

    case FS_S3_WAIT:
      waitForKey('3');
      Serial.println("[FULL SETUP] Running NRF24 bulk transfer...");
      g_state = FS_S3_RUN;
      break;

    case FS_S3_RUN:
      if (!g_nrf_ready) initNRF();
      if (runBulkTransfer()) {
        // runBulkTransfer() does not increment g_nrf_extractions - done here
        g_nrf_extractions++;
        saveState();
        Serial.printf("[FULL SETUP] Step 3 complete - nrf_extractions=%d\n", g_nrf_extractions);
      }
      Serial.println("[FULL SETUP] Re-establishing GSM after NRF24 SPI usage...");
      g_state = FS_GSM_REINIT;
      break;

    case FS_GSM_REINIT:
      if (!ensureGPRS()) { g_gsm_ready = false; initGSM(); }
#ifdef SIMULATE_SENSORS
      if (g_gsm_ready) fetchLatestPositionFromFirebase();
#endif
      Serial.println("[FULL SETUP] Step 4 - set tx_interval=30 in Firebase, then press 'A'.");
      g_state = FS_S4_WAIT;
      break;

    case FS_S4_WAIT:
      waitForKey('A');
      Serial.println("[FULL SETUP] Running final GSM cycle...");
      g_state = FS_S4_RUN;
      break;

    case FS_S4_RUN:
      if (runGSMCycle())
        Serial.printf("[FULL SETUP] Step 4 complete - gsm_cycles=%d  tx_interval=%ds\n",
                      g_gsm_cycles, g_tx_interval);
      Serial.println("[FULL SETUP] Done - GSM and NRF24 verified end-to-end.");
      g_state = FS_DONE;
      break;

    case FS_DONE:
      Serial.println("\n[MENU] ===== Setup complete =====");
      Serial.println("[MENU] '4' = Return to Setup menu  |  '5' = Launch Live mode");
      Serial.flush();
      g_state = SETUP_MENU;
      break;

    // ════════════════════════════════════════════════════════════
    //  SETUP MENU
    // ════════════════════════════════════════════════════════════

    case SETUP_MENU: {
      static unsigned long menuReminder = 0;
      if (millis() - menuReminder >= 5000) {
        Serial.println("[MENU] Waiting - '4' = Setup menu  |  '5' = Live mode");
        menuReminder = millis();
      }
      char k = keypadScan();
      if (k == '4') {
        keypadWaitRelease();
        Serial.println("\n[MENU] Returning to setup mode selection...");
        Serial.println("  'A' = Long Distance Setup");
        Serial.println("  'B' = Short Distance Setup");
        Serial.println("  '6' = Full Setup");
        char sel = 0;
        while (sel == 0) {
          char sk = keypadScan();
          if (sk == 'A' || sk == 'B' || sk == '6') { sel = sk; keypadWaitRelease(); }
          delay(50);
        }
        enterSetupMode(sel);
      } else if (k == '5') {
        keypadWaitRelease();
        Serial.println("\n[MENU] Launching Live mode...");
        g_tx_mode       = (digitalRead(LIVE_MODE_PIN) == HIGH) ? TX_LONG : TX_SHORT;
        g_pre_hold_mode = g_tx_mode;
        Serial.printf("[LIVE] tx_mode=%s\n",
                      g_tx_mode == TX_LONG ? "LONG (GSM)" : "SHORT (NRF24)");
        if (g_tx_mode == TX_LONG && !g_gsm_ready) {
          initGSM();
#ifdef SIMULATE_SENSORS
          if (g_gsm_ready) fetchLatestPositionFromFirebase();
#endif
        }
        g_state = LIVE_IDLE;
      } else {
        delay(100);
      }
      break;
    }

    // ════════════════════════════════════════════════════════════
    //  LIVE MODE
    //
    //  Three normal states:
    //    LIVE_IDLE      - polling isSurfaced() and livePaused()
    //    LIVE_TRANSMIT  - runTransmission(g_tx_mode)
    //    LIVE_PAUSED    - paused via Firebase stop or PAUSE_PIN LOW
    //
    //  Extraction state (triggered by Firebase "surface_hold"):
    //    LIVE_SEARCHING - buoy stays surfaced; alternates GSM position
    //                     pings (every tx_interval) with NRF24 probe
    //                     attempts (every 10s). Transfers when receiver
    //                     comes in range. Moves to LIVE_PAUSED on
    //                     success; operator sends "start" to resume.
    // ════════════════════════════════════════════════════════════

    case LIVE_IDLE: {
      if (livePaused()) {
        Serial.println("[LIVE] Paused - Firebase stop or PAUSE_PIN active");
        g_state = LIVE_PAUSED;
        break;
      }
      // surface_hold can arrive via readCtrl() during a GSM cycle and
      // transition us directly to LIVE_SEARCHING. Check it explicitly
      // here too in case it was set while we were in LIVE_IDLE.
      if (g_surface_hold) {
        Serial.println("[LIVE] surface_hold active - entering LIVE_SEARCHING");
        g_state = LIVE_SEARCHING;
        break;
      }
      if (isSurfaced()) {
        Serial.printf("[LIVE] Surface detected - tx_mode=%s\n",
                      g_tx_mode == TX_LONG ? "LONG" : "SHORT");
        g_state = LIVE_TRANSMIT;
      } else {
        delay(500);
      }
      break;
    }

    case LIVE_TRANSMIT:
      runTransmission();
      g_state = LIVE_IDLE;
      break;

    // ════════════════════════════════════════════════════════════
    //  LIVE_SEARCHING
    //
    //  Entered when operator sends Firebase "surface_hold" command.
    //  g_surface_hold is true; buoy treats itself as always surfaced.
    //
    //  Two interleaved timers (non-blocking):
    //    GSM ping  - every tx_interval seconds
    //      Keeps map updated so operator can locate buoy.
    //      Gracefully skipped if GSM unavailable (buoy keeps searching).
    //      readCtrl() is called inside runGSMCycle() - picks up "start"
    //      or a cancel if operator aborts the extraction.
    //
    //    NRF24 probe - every NRF_SEARCH_INTERVAL_MS (10s)
    //      runBulkTransfer() with short PKT_START ACK window (800ms,
    //      3 retries → max ~2.4s block). Returns false immediately if
    //      no receiver responds. When receiver comes in range the full
    //      transfer runs to completion.
    //
    //  On successful transfer:
    //    - PUT buoy/status → {event:"extraction_complete", ...}
    //    - SD.remove(SENSOR_FILE)   ← bulk data cleared
    //    - surface_log.txt KEPT     ← last GPS fix retained, not bulk data
    //    - g_tx_mode restored to g_pre_hold_mode
    //    - g_surface_hold cleared
    //    - g_cycle_active = false   ← operator must send "start" to resume
    //    - → LIVE_PAUSED
    // ════════════════════════════════════════════════════════════

    case LIVE_SEARCHING: {
      #define NRF_SEARCH_INTERVAL_MS  10000UL   // probe receiver every 10s

      static unsigned long lastGSMPing    = 0;
      static unsigned long lastNRFProbe   = 0;
      static bool          searchTimersInit = false;

      // Initialise timers on first entry so we don't immediately fire both.
      // Stagger: GSM ping fires first (after tx_interval), NRF probe fires
      // sooner so we start looking for the receiver quickly.
      if (!searchTimersInit) {
        lastGSMPing      = millis();
        lastNRFProbe     = millis() - (NRF_SEARCH_INTERVAL_MS - 2000UL); // first probe in 2s
        searchTimersInit = true;
        Serial.printf("[SEARCH] Entered LIVE_SEARCHING - GSM ping every %ds, NRF probe every %lus\n",
                      g_tx_interval, NRF_SEARCH_INTERVAL_MS / 1000UL);
        Serial.println("[SEARCH] Bring receiver within NRF24 range to trigger transfer.");
      }

      // If surface_hold was cleared externally (shouldn't normally happen
      // but guard anyway), exit back to LIVE_IDLE.
      if (!g_surface_hold) {
        Serial.println("[SEARCH] surface_hold cleared externally - returning to LIVE_IDLE");
        searchTimersInit = false;
        g_state = LIVE_IDLE;
        break;
      }

      unsigned long now = millis();

      // ── GSM position ping ────────────────────────────────────
      if (now - lastGSMPing >= (unsigned long)g_tx_interval * 1000UL) {
        Serial.println("[SEARCH] GSM position ping...");
        if (g_gsm_ready || initGSM()) {
          runGSMCycle();
          // readCtrl() is called inside runGSMCycle().
          // If operator sent "start" during the ping, g_cycle_active is
          // now true but g_surface_hold is still set - that means the
          // operator wants to abort the extraction and resume normally.
          // We check for that here.
          if (!g_surface_hold) {
            Serial.println("[SEARCH] surface_hold cleared by Firebase - aborting extraction, resuming");
            searchTimersInit = false;
            g_state = LIVE_IDLE;
            break;
          }
        } else {
          Serial.println("[SEARCH] GSM unavailable - skipping ping, staying surfaced");
        }
        lastGSMPing = millis();
      }

      // ── NRF24 probe ──────────────────────────────────────────
      if (now - lastNRFProbe >= NRF_SEARCH_INTERVAL_MS) {
        Serial.println("[SEARCH] NRF24 probe...");
        if (!g_nrf_ready) initNRF();
        if (runBulkTransfer()) {
          // ── Transfer succeeded ────────────────────────────────
          Serial.println("[SEARCH] Transfer complete - running post-extraction cleanup");

          // Write extraction ACK to Firebase so operator/dashboard
          // knows the data was received successfully.
          if (g_gsm_ready || initGSM()) {
            String networkTs  = getNetworkTime();
            String ackPayload = "{\"event\":\"extraction_complete\","
                                "\"ts\":\"" + networkTs + "\","
                                "\"nrf_extractions\":" + String(g_nrf_extractions + 1) + ","
                                "\"mode_restored\":\"" +
                                (g_pre_hold_mode == TX_LONG ? "long" : "short") + "\"}";
            httpPost(FIREBASE_HOST + "/buoy/status.json?auth=" + FIREBASE_SECRET
                     + "&x-http-method-override=PUT", ackPayload);
          } else {
            Serial.println("[SEARCH] GSM unavailable - extraction ACK not sent to Firebase");
          }

          // Clear sensor log - the receiver now has this data.
          // surface_log is intentionally kept: it holds the last GPS fix
          // and is small; deleting it would leave the buoy position-blind
          // until the next surface event writes a new entry.
          if (g_sd_ok) {
            SD.remove(SENSOR_FILE);
            Serial.println("[SEARCH] sensor_log cleared, surface_log retained");
          }

#ifdef SIMULATE_SENSORS
          // Advance sim clock and seed next batch (mirrors runTransmission SHORT path)
          g_sim_cycle += (SIM_GAP_HOURS * 60) / SIM_SAMPLE_INTERVAL_MIN;
          Serial.printf("[SIM] Jumped %d cycles (%dh gap)\n",
                        (SIM_GAP_HOURS * 60) / SIM_SAMPLE_INTERVAL_MIN, SIM_GAP_HOURS);
          seedSimBatch(SIM_BATCH_SIZE);
#endif

          // runBulkTransfer() does not increment g_nrf_extractions - done here.
          // This is the only increment in the LIVE_SEARCHING path.
          g_nrf_extractions++;
          saveState();

          // Restore the transmission mode that was active before surface_hold.
          // Operator only needs to send "start" to resume - no mode command needed.
          g_tx_mode      = g_pre_hold_mode;
          g_surface_hold = false;
          g_cycle_active = false;   // wait for explicit operator "start"

          Serial.printf("[SEARCH] Mode restored to %s - awaiting operator resume (send 'start')\n",
                        g_tx_mode == TX_LONG ? "LONG" : "SHORT");

          searchTimersInit = false;
          g_state = LIVE_PAUSED;

        } else {
          Serial.println("[SEARCH] No receiver in range - will retry");
          lastNRFProbe = millis();
        }
      }

      delay(100);  // yield - don't spin-lock the CPU between timer checks
      break;
    }

    case LIVE_PAUSED: {
      if (!livePaused()) {
        Serial.println("[LIVE] Resumed");
        g_state = LIVE_IDLE;
      } else {
        static unsigned long lastPauseCheck = 0;
        if (g_tx_mode == TX_LONG && millis() - lastPauseCheck > 60000UL) {
          if (g_gsm_ready || initGSM()) readCtrl();
          lastPauseCheck = millis();
        }
        delay(500);
      }
      break;
    }
  }
}
