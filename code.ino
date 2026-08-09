/* ===========================================================================
 *  HCHO-Guard  |  Formaldehyde / Formalin Detection Node
 *  MS1100 VOC-HCHO sensor  +  ESP32 DevKit v1  ->  Supabase REST
 *  ---------------------------------------------------------------------------
 *  Author : Md. Mehedi Hasan  (232-15-497), Daffodil International University
 *  Board  : ESP32 Dev Module   |  Arduino-ESP32 core 2.0.x or 3.x
 *
 *  LIBRARIES (Library Manager)
 *    - ArduinoJson            by Benoit Blanchon   (v6.x)
 *    - Adafruit SSD1306 + Adafruit GFX             (only if USE_OLED)
 *    - DHT sensor library     by Adafruit          (only if USE_DHT)
 *
 *  ---------------------------------------------------------------------------
 *  WIRING
 *    MS1100 module          ESP32
 *      VCC   ---------------  5V (VIN)        <- module heater needs 5V
 *      GND   ---------------  GND             <- COMMON GROUND, mandatory
 *      AOUT  --- R1 10k ----  GPIO34 (ADC1_6)
 *                     |
 *                     R2 10k --- GND          <- divider: AOUT max 5V -> 2.5V
 *      DOUT  ---------------  GPIO35 (optional, module's own comparator)
 *
 *    Buzzer (active)  GPIO25 -> +, GND -> -
 *    Status LED       GPIO2  (onboard blue)
 *    Alarm LED (red)  GPIO26 -> 220R -> LED -> GND
 *    DHT22 (optional) GPIO27 data, 4k7 pull-up to 3V3
 *    OLED  (optional) SDA GPIO21, SCL GPIO22, addr 0x3C
 *
 *  IMPORTANT: GPIO34/35 are input-only ADC1 pins. Never use ADC2 pins
 *  (GPIO0/2/4/12-15/25-27) for analogRead while WiFi is on - ADC2 is
 *  owned by the radio and returns garbage.
 *
 *  ---------------------------------------------------------------------------
 *  FIRST-RUN PROCEDURE
 *    1. Burn-in the MS1100 for 24-48 h powered in clean air (one time, new
 *       sensor). Skipping this gives drifting, useless ppm numbers.
 *    2. Flash, open Serial Monitor @115200.
 *    3. Let it warm up (WARMUP_MS, 180 s) in clean outdoor air.
 *    4. Type  cal  + Enter  -> R0 is measured and stored in NVS flash.
 *    5. Verify with a reference: 40% formalin bottle cap opened 20 cm away
 *       should push ppm up sharply and recover within ~60 s.
 *
 *  SERIAL COMMANDS:  cal | info | raw | wipe | send
 * ===========================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

// ---------------------------------------------------------------------------
// OPTIONAL PERIPHERALS  -> set to 1 to enable
// ---------------------------------------------------------------------------
#define USE_DHT   0        // DHT22 temperature / humidity compensation
#define USE_OLED  0        // 0.96" SSD1306 local display

// ===========================================================================
//  1. USER CONFIGURATION
// ===========================================================================
const char* WIFI_SSID     = "Me";
const char* WIFI_PASSWORD = "mehedi113";

// ---- Supabase --------------------------------------------------------------
const char* SUPABASE_URL  = "https://veiaafecvwthgkplofog.supabase.co";
const char* SUPABASE_ANON =
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InZlaWFhZmVjdnd0aGdrcGxvZm9nIiwicm9sZSI6ImFub24i"
  "LCJpYXQiOjE3ODYyNTQ0NjUsImV4cCI6MjEwMTgzMDQ2NX0."
  "l4gkPoFPtjVTHdT0jdKcGmPFW1IKbgrPJLtcMsejwuc";

/*  >>> READ THIS ONCE <<<
 *  DEVICE_ID below must be BYTE-FOR-BYTE identical to devices.device_id in
 *  Supabase and to the device you select in the dashboard. A silent mismatch
 *  here (node-01 vs node-02) means rows insert fine but nothing ever renders.
 */
#define DEVICE_ID   "hcho-node-01"
#define FW_VERSION  "1.0.0"

// ---- Timing ----------------------------------------------------------------
const uint32_t WARMUP_MS       = 180000;   // 3 min sensor warm-up before valid data
const uint32_t SAMPLE_MS       = 2000;     // local read + display cadence
const uint32_t UPLOAD_MS       = 30000;    // push to Supabase every 30 s
const uint32_t ALARM_UPLOAD_MS = 5000;     // faster push while in alarm state

// ---- Sensor front-end ------------------------------------------------------
const int   PIN_AOUT      = 34;
const int   PIN_DOUT      = 35;
const int   PIN_BUZZER    = 25;
const int   PIN_LED_STAT  = 2;
const int   PIN_LED_ALARM = 26;
const int   PIN_DHT       = 27;

const float ADC_VREF      = 3.30f;   // ESP32 reference after 11 dB attenuation
const int   ADC_MAX       = 4095;

/*  SUPPLY MODE - your 4-pin module is VCC / GND / DOUT / AOUT.
 *  1 = powered from 5V (VIN). AOUT can swing to ~5V, so the 10k/10k divider
 *      on AOUT is REQUIRED or GPIO34 gets damaged.
 *  0 = powered from 3V3. AOUT never exceeds 3.3V, so wire AOUT straight to
 *      GPIO34 with no divider. Only use this if the module actually heats
 *      and responds at 3V3 - many MS1100 boards need the full 5V heater.
 */
#define SENSOR_5V   1

#if SENSOR_5V
  const float VC_VOLT      = 5.0f;   // sensor supply rail
  const float DIVIDER_GAIN = 2.0f;   // 10k/10k divider -> multiply back up
#else
  const float VC_VOLT      = 3.30f;
  const float DIVIDER_GAIN = 1.0f;   // AOUT wired direct, no divider
#endif

/*  Load resistor already fitted on the module, between AOUT and GND.
 *  Find it on the board: usually a 10k (marked 103) or 1k (marked 102)
 *  next to the sensor can. The blue potentiometer is NOT this resistor -
 *  it only sets the DOUT comparator trip point.
 */
const float RL_KOHM       = 10.0f;

/*  MS1100 response curve, Rs/R0 -> ppm (power law: ppm = A * ratio^B)
 *  These constants come from digitising the HCHO curve in the MS1100
 *  datasheet. Refine them for YOUR unit with a reference instrument - the
 *  shape is right, the absolute scale is sensor-specific.
 */
const float CURVE_A = 0.6103f;
const float CURVE_B = -2.2926f;

/*  Rs/R0 measured in CLEAN AIR.
 *  R0 is a reference resistance, not simply the resistance you happen to
 *  measure in fresh air. Storing raw clean-air Rs as R0 would make ratio = 1
 *  in clean air, and the curve above returns 0.61 ppm at ratio 1 - a
 *  permanent false alarm. Anchoring at 4.4 puts clean air at roughly
 *  0.02 ppm, the WHO limit at ratio ~2.4, and 0.3 ppm at ratio ~1.4.
 *
 *  Refit this together with CURVE_A/CURVE_B if you calibrate against a
 *  reference instrument.
 */
const float CLEAN_AIR_RATIO = 4.4f;

// ---- Alarm thresholds (ppm) ------------------------------------------------
const float PPM_MODERATE = 0.048f;   // 60% of WHO
const float PPM_WARN     = 0.080f;   // WHO 30-min guideline, 0.1 mg/m3
const float PPM_ALARM    = 0.300f;   // clear formalin contamination

// ---- Offline buffer --------------------------------------------------------
#define QUEUE_LEN  48                // ~24 min of buffered data at 30 s

// ===========================================================================
//  2. GLOBALS
// ===========================================================================
Preferences prefs;

#if USE_DHT
  #include <DHT.h>
  DHT dht(PIN_DHT, DHT22);
#endif

#if USE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 oled(128, 64, &Wire, -1);
#endif

struct Sample {
  time_t  ts;
  int     adc;
  float   mv;
  float   rs;
  float   ratio;
  float   ppm;
  float   mg;
  float   tempC;
  float   rh;
  int8_t  level;      // 0 good 1 moderate 2 warn 3 alarm
};

Sample   queueBuf[QUEUE_LEN];
uint8_t  qHead = 0, qCount = 0;

float    R0            = 0.0f;       // clean-air resistance, from NVS
bool     calibrated    = false;
bool     warmedUp      = false;
bool     forceUpload   = false;
uint32_t bootMs        = 0;
uint32_t tSample       = 0;
uint32_t tUpload       = 0;
Sample   last;
float    ppmEma        = 0.0f;       // exponential moving average
const float EMA_ALPHA  = 0.25f;

const char* LEVEL_NAME[4] = { "good", "moderate", "warn", "alarm" };

// forward declarations
void  calibrateR0();
void  registerDevice();
void  patchDeviceR0();
bool  uploadBatch(const Sample& s);
void  enqueue(const Sample& s);
float readSensorMv();
float mvToRs(float mv);

// ===========================================================================
//  3. SENSOR FRONT-END
// ===========================================================================

// Oversampled ADC read -> millivolts at the sensor output (divider undone)
float readSensorMv() {
  const int N = 64;
  uint32_t acc = 0;
  for (int i = 0; i < N; i++) { acc += analogRead(PIN_AOUT); delayMicroseconds(200); }
  float counts = (float)acc / N;
  last.adc = (int)counts;
  return (counts / ADC_MAX) * ADC_VREF * DIVIDER_GAIN * 1000.0f;
}

// Sensing resistance from the divider formed by Rs and RL
float mvToRs(float mv) {
  float v = mv / 1000.0f;
  if (v <= 0.01f) v = 0.01f;
  if (v >= VC_VOLT - 0.01f) v = VC_VOLT - 0.01f;
  return ((VC_VOLT - v) / v) * RL_KOHM;
}

// Temperature / humidity correction factor for the Rs/R0 ratio.
// Reference point is 20 C / 65 %RH. Second-order fit, clamped for safety.
float envCorrection(float tempC, float rh) {
  if (isnan(tempC) || isnan(rh)) return 1.0f;
  float f = 1.0f + 0.0092f * (20.0f - tempC) + 0.0018f * (65.0f - rh);
  return constrain(f, 0.75f, 1.35f);
}

float ratioToPpm(float ratio) {
  if (ratio <= 0.0f) return 0.0f;
  float ppm = CURVE_A * powf(ratio, CURVE_B);
  if (isnan(ppm) || isinf(ppm)) return 0.0f;
  return constrain(ppm, 0.0f, 30.0f);
}

int8_t classify(float ppm) {
  if (ppm >= PPM_ALARM)    return 3;
  if (ppm >= PPM_WARN)     return 2;
  if (ppm >= PPM_MODERATE) return 1;
  return 0;
}

// Clean-air calibration: average Rs over ~30 s and store as R0
void calibrateR0() {
  Serial.println(F("\n[CAL] Clean-air calibration - keep the node in fresh air."));
  Serial.println(F("[CAL] Sampling for 30 s ..."));
  float acc = 0; int n = 0;
  float first = 0, lastRs = 0;
  for (int i = 0; i < 30; i++) {
    float rs = mvToRs(readSensorMv());
    if (i == 0) first = rs;
    lastRs = rs;
    acc += rs; n++;
    Serial.printf("  %2d/30  Rs = %.2f kOhm\n", i + 1, rs);
    digitalWrite(PIN_LED_STAT, !digitalRead(PIN_LED_STAT));
    delay(1000);
  }
  float rsAir = acc / n;

  // A sensor that is still warming up drifts steadily upward. Calibrating on
  // a moving value bakes the drift into every future reading.
  float drift = fabsf(lastRs - first) / rsAir * 100.0f;
  if (drift > 3.0f) {
    Serial.printf("[CAL] WARNING: Rs moved %.1f%% during calibration.\n", drift);
    Serial.println(F("[CAL] The sensor has not settled. Wait 10-20 min and run 'cal' again."));
  }

  R0 = rsAir / CLEAN_AIR_RATIO;
  calibrated = true;
  prefs.begin("hcho", false);
  prefs.putFloat("r0", R0);
  prefs.end();
  Serial.printf("[CAL] Clean-air Rs = %.2f kOhm\n", rsAir);
  Serial.printf("[CAL] Done. R0 = %.2f kOhm stored in flash (anchor %.1f).\n\n", R0, CLEAN_AIR_RATIO);

  // report the new R0 back to the devices table
  patchDeviceR0();
}

// ===========================================================================
//  4. NETWORK
// ===========================================================================
void wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("[WiFi] connecting to %s ", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400); Serial.print('.');
  }
  Serial.println(WiFi.status() == WL_CONNECTED
                 ? "  connected: " + WiFi.localIP().toString()
                 : "  FAILED (will retry, data is buffered)");
}

void syncTime() {
  // Asia/Dhaka = UTC+6, no DST
  configTime(6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print(F("[NTP] syncing "));
  uint32_t t0 = millis();
  while (time(nullptr) < 1700000000 && millis() - t0 < 15000) { delay(300); Serial.print('.'); }
  Serial.println(time(nullptr) > 1700000000 ? F("  ok") : F("  timeout"));
}

// ISO-8601 with the +06:00 offset so Postgres stores the correct instant
String isoTime(time_t t) {
  struct tm tmv;
  localtime_r(&t, &tmv);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S+06:00", &tmv);
  return String(buf);
}

// Register / refresh this node in devices (upsert on primary key)
void registerDevice() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/devices?on_conflict=device_id";
  if (!http.begin(client, url)) return;
  http.addHeader("apikey", SUPABASE_ANON);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "resolution=merge-duplicates,return=minimal");

  StaticJsonDocument<384> doc;
  doc["device_id"]    = DEVICE_ID;
  doc["sensor_model"] = "MS1100";
  doc["firmware"]     = FW_VERSION;
  doc["warn_ppm"]     = PPM_WARN;
  doc["alarm_ppm"]    = PPM_ALARM;
  if (calibrated) doc["r0_clean_air"] = R0;
  String body; serializeJson(doc, body);

  int code = http.POST(body);
  Serial.printf("[SB ] device upsert -> %d\n", code);
  http.end();
}

void patchDeviceR0() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/devices?device_id=eq." + DEVICE_ID;
  if (!http.begin(client, url)) return;
  http.addHeader("apikey", SUPABASE_ANON);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");
  String body = String("{\"r0_clean_air\":") + String(R0, 3) + "}";
  int code = http.sendRequest("PATCH", body);
  Serial.printf("[SB ] R0 patch -> %d\n", code);
  http.end();
}

// Build one JSON object for a sample
void sampleToJson(const Sample& s, JsonObject o) {
  o["device_id"]     = DEVICE_ID;
  o["recorded_at"]   = isoTime(s.ts);
  o["adc_raw"]       = s.adc;
  o["voltage_mv"]    = roundf(s.mv * 10) / 10.0f;
  o["rs_kohm"]       = roundf(s.rs * 100) / 100.0f;
  o["ratio"]         = roundf(s.ratio * 1000) / 1000.0f;
  o["hcho_ppm"]      = roundf(s.ppm * 10000) / 10000.0f;
  o["hcho_mg_m3"]    = roundf(s.mg * 10000) / 10000.0f;
  o["level"]         = LEVEL_NAME[s.level];
  if (!isnan(s.tempC)) o["temperature_c"] = s.tempC;
  if (!isnan(s.rh))    o["humidity_pct"]  = s.rh;
  o["rssi"]          = WiFi.RSSI();
  o["uptime_s"]      = (uint32_t)(millis() / 1000);
  o["fw"]            = FW_VERSION;
  o["is_calibrated"] = calibrated;
}

// POST the current sample plus everything queued, in one batch request
bool uploadBatch(const Sample& now) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/readings";
  if (!http.begin(client, url)) return false;
  http.addHeader("apikey", SUPABASE_ANON);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");
  http.setTimeout(12000);

  DynamicJsonDocument doc(16384);
  JsonArray arr = doc.to<JsonArray>();

  // oldest queued rows first, then the live one
  for (uint8_t i = 0; i < qCount; i++) {
    uint8_t idx = (qHead + QUEUE_LEN - qCount + i) % QUEUE_LEN;
    sampleToJson(queueBuf[idx], arr.createNestedObject());
  }
  sampleToJson(now, arr.createNestedObject());

  String body; serializeJson(doc, body);
  int code = http.POST(body);
  String resp = (code > 0 && code != 201) ? http.getString() : "";
  http.end();

  if (code == 201 || code == 200) {
    Serial.printf("[SB ] %u row(s) uploaded  (queue cleared)\n", arr.size());
    qCount = 0;
    return true;
  }
  Serial.printf("[SB ] upload failed http=%d %s\n", code, resp.c_str());
  return false;
}

void enqueue(const Sample& s) {
  queueBuf[qHead] = s;
  qHead = (qHead + 1) % QUEUE_LEN;
  if (qCount < QUEUE_LEN) qCount++;
  Serial.printf("[BUF] offline, queued %u/%u\n", qCount, QUEUE_LEN);
}

// ===========================================================================
//  5. LOCAL FEEDBACK
// ===========================================================================
void updateIndicators(int8_t level) {
  static uint32_t tBeep = 0;
  static bool     beepOn = false;

  digitalWrite(PIN_LED_ALARM, level >= 2 ? HIGH : LOW);

  if (level == 3) {                       // alarm: fast 200 ms chirp
    if (millis() - tBeep > 200) { beepOn = !beepOn; digitalWrite(PIN_BUZZER, beepOn); tBeep = millis(); }
  } else if (level == 2) {                // warn: slow 900 ms chirp
    if (millis() - tBeep > 900) { beepOn = !beepOn; digitalWrite(PIN_BUZZER, beepOn); tBeep = millis(); }
  } else {
    digitalWrite(PIN_BUZZER, LOW); beepOn = false;
  }
}

#if USE_OLED
void drawOled(const Sample& s, bool warm) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(F("HCHO-Guard  "));
  oled.print(WiFi.status() == WL_CONNECTED ? F("WiFi") : F("----"));

  if (!warm) {
    oled.setCursor(0, 24);
    oled.setTextSize(1);
    oled.print(F("Warming up..."));
    oled.setCursor(0, 40);
    uint32_t left = (WARMUP_MS - (millis() - bootMs)) / 1000;
    oled.printf("%lu s left", (unsigned long)left);
  } else {
    oled.setTextSize(3);
    oled.setCursor(0, 20);
    oled.printf("%.3f", s.ppm);
    oled.setTextSize(1);
    oled.setCursor(96, 34); oled.print(F("ppm"));
    oled.setCursor(0, 54);
    oled.printf("%.3f mg/m3  %s", s.mg, LEVEL_NAME[s.level]);
  }
  oled.display();
}
#endif

// ===========================================================================
//  6. SERIAL CONSOLE
// ===========================================================================
void printInfo() {
  Serial.println(F("\n---------------- HCHO-Guard ----------------"));
  Serial.printf("Device      : %s  (fw %s)\n", DEVICE_ID, FW_VERSION);
  Serial.printf("WiFi        : %s  RSSI %d dBm\n",
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "offline",
                WiFi.RSSI());
  Serial.printf("Calibrated  : %s   R0 = %.2f kOhm\n", calibrated ? "yes" : "NO - run 'cal'", R0);
  Serial.printf("Warmed up   : %s\n", warmedUp ? "yes" : "no");
  Serial.printf("Last ppm    : %.4f ppm  (%.4f mg/m3, %s)\n", last.ppm, last.mg, LEVEL_NAME[last.level]);
  Serial.printf("Queue       : %u/%u rows\n", qCount, QUEUE_LEN);
  Serial.println(F("Commands    : cal | info | raw | wipe | send"));
  Serial.println(F("--------------------------------------------\n"));
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); cmd.toLowerCase();

  if (cmd == "cal")       calibrateR0();
  else if (cmd == "info") printInfo();
  else if (cmd == "raw") {
    for (int i = 0; i < 20; i++) {
      float mv = readSensorMv();
      Serial.printf("adc=%4d  mv=%7.1f  Rs=%7.2f  ratio=%6.3f\n",
                    last.adc, mv, mvToRs(mv), R0 > 0 ? mvToRs(mv) / R0 : 0);
      delay(500);
    }
  }
  else if (cmd == "wipe") {
    prefs.begin("hcho", false); prefs.clear(); prefs.end();
    R0 = 0; calibrated = false;
    Serial.println(F("[NVS] calibration erased."));
  }
  else if (cmd == "send") {
    tUpload = 0;
    forceUpload = true;
    Serial.println(F("[SB ] forcing one upload, ignoring warm-up gate..."));
  }
  else if (cmd.length())  Serial.println(F("? try: cal | info | raw | wipe | send"));
}

// ===========================================================================
//  7. SETUP / LOOP
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n\n=== HCHO-Guard : MS1100 + ESP32 ==="));

  pinMode(PIN_DOUT, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);      digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LED_STAT, OUTPUT);
  pinMode(PIN_LED_ALARM, OUTPUT);   digitalWrite(PIN_LED_ALARM, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_AOUT, ADC_11db);   // full 0-3.3 V span

#if USE_DHT
  dht.begin();
#endif
#if USE_OLED
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println(F("[OLED] not found"));
  oled.clearDisplay(); oled.display();
#endif

  prefs.begin("hcho", true);
  R0 = prefs.getFloat("r0", 0.0f);
  prefs.end();
  calibrated = (R0 > 0.1f);
  Serial.printf("[NVS] R0 = %.2f kOhm  (%s)\n", R0,
                calibrated ? "loaded" : "not calibrated - run 'cal' after warm-up");

  wifiConnect();
  syncTime();
  registerDevice();

  bootMs  = millis();
  tSample = millis();
  tUpload = millis();
  last.tempC = NAN; last.rh = NAN;
  printInfo();
  Serial.printf("[RUN] warming up for %lu s ...\n", (unsigned long)(WARMUP_MS / 1000));
}

void loop() {
  handleSerial();

  if (!warmedUp && millis() - bootMs >= WARMUP_MS) {
    warmedUp = true;
    Serial.println(F("[RUN] warm-up complete, readings are now valid."));
  }

  // ---------------- sample ------------------------------------------------
  if (millis() - tSample >= SAMPLE_MS) {
    tSample = millis();

    float mv = readSensorMv();
    float rs = mvToRs(mv);

#if USE_DHT
    float t = dht.readTemperature();
    float h = dht.readHumidity();
#else
    float t = NAN, h = NAN;
#endif

    float ratio = (R0 > 0.1f) ? (rs / R0) * envCorrection(t, h) : 0.0f;
    float ppm   = calibrated ? ratioToPpm(ratio) : 0.0f;

    // smooth out MEMS noise before classifying
    ppmEma = (ppmEma <= 0) ? ppm : (EMA_ALPHA * ppm + (1 - EMA_ALPHA) * ppmEma);

    last.ts    = time(nullptr);
    last.mv    = mv;
    last.rs    = rs;
    last.ratio = ratio;
    last.ppm   = ppmEma;
    last.mg    = ppmEma * 1.228f;      // HCHO, MW 30.03, 25 C 1 atm
    last.tempC = t;
    last.rh    = h;
    last.level = warmedUp && calibrated ? classify(ppmEma) : 0;

    updateIndicators(last.level);
    digitalWrite(PIN_LED_STAT, WiFi.status() == WL_CONNECTED);

#if USE_OLED
    drawOled(last, warmedUp);
#endif

    // DOUT is the module's onboard comparator, trip point set by the blue pot.
    // Most boards pull it LOW when gas exceeds the pot setting.
    bool doutTrip = (digitalRead(PIN_DOUT) == LOW);

    Serial.printf("[%s] %.4f ppm | %.4f mg/m3 | Rs %.1fk | ratio %.3f | %s | DOUT %s%s\n",
                  warmedUp ? "OK " : "WRM", last.ppm, last.mg, rs, ratio,
                  LEVEL_NAME[last.level], doutTrip ? "TRIP" : "idle",
                  calibrated ? "" : "  (uncalibrated)");
  }

  // ---------------- upload -------------------------------------------------
  uint32_t period = (last.level == 3) ? ALARM_UPLOAD_MS : UPLOAD_MS;
  if (millis() - tUpload >= period) {
    tUpload = millis();

    if (!warmedUp && !forceUpload) {
      Serial.println(F("[SB ] skipped, still warming up."));
    } else {
      if (WiFi.status() != WL_CONNECTED) wifiConnect();
      if (!uploadBatch(last)) enqueue(last);
      forceUpload = false;
    }
  }
}
