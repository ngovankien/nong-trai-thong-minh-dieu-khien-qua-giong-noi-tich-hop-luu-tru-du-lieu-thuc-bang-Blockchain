#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DHT.h>

/*
  BAN NANG CAP NAY CO:
  1) Ket noi WiFi chac hon va tu reconnect khi rot mang
  2) Relay de dao muc kich cho de cau hinh
  3) Cam bien dat duoc loc nhieu bang nhieu mau + lay trung vi/trung binh
  4) JSON /data co them thong tin chu thich de web hien thi ro hon
  5) Them flow sensor, canh bao khong co nuoc va manual mode bat/tat bom
  6) Toi uu theo hieu chuan cam bien dat (% am) va he so xung luu luong
  7) Chong giat bat/tat bom: xac nhan nhieu lan, gioi han chu ky bat/tat, auto quyet dinh theo nhip

  CACH HIEU CHUAN NHANH:
  - De cam bien dat kho hoan toan, doc soilRaw => gan vao SOIL_RAW_DRY
  - Dat cam bien vao nuoc/rat am, doc soilRaw => gan vao SOIL_RAW_WET
  - Cho 1 lit nuoc chay qua flow sensor, dem tong pulses => gan vao FLOW_PULSES_PER_LITER

  LUU Y:
  - Ban nay dang sua theo huong: DHT D4 + relay ACTIVE LOW
  - Neu bam bat bom ma relay hoat dong nguoc, doi RELAY_ACTIVE_LOW
  - Neu cam bien dat van nhay manh, kiem tra lai day AO, GND va nguon module
*/

// =======================
// WIFI
// =======================
const char* WIFI_SSID = "resroom";
const char* WIFI_PASS = "6666888822";

// =======================
// CHÂN KẾT NỐI
// =======================
#define DHTPIN D4   // doi sang D4 de test vi DHT11 tren D2 dang khong on dinh
#define DHTTYPE DHT11

#define RELAY_PIN D1
#define SOIL_PIN  A0
#define FLOW_PIN  D5

// Relay thường active LOW

// =======================
// NGUONG VA HIEU CHUAN
// =======================
// Nguong raw de debug/hien thi
int SOIL_DRY_THRESHOLD_RAW = 700;   // dat kho
int SOIL_WET_STOP_RAW      = 550;   // dat du am de dung bom

// Hieu chuan cam bien dat thuc te
int SOIL_RAW_DRY = 900;   // gia tri khi dat kho
int SOIL_RAW_WET = 350;   // gia tri khi dat rat am / ngam nuoc

// Dieu khien theo % do am dat
int SOIL_START_PUMP_PERCENT = 30;   // <= muc nay thi bat bom
int SOIL_STOP_PUMP_PERCENT  = 65;   // >= muc nay thi tat bom

// Hieu chuan cam bien luu luong
float FLOW_PULSES_PER_LITER = 450.0;
int NO_WATER_CONFIRM_SECONDS = 3;
float FLOW_SMOOTH_ALPHA = 0.25;   // cang nho thi cang mem
float FLOW_MIN_VALID_LPM = 0.03;  // duoi muc nay xem nhu 0
float FLOW_MAX_VALID_LPM = 12.0;  // tren muc nay xem nhu xung nhieu/gia

unsigned long DHT_READ_INTERVAL = 3000;

// Relay de dao muc kich de cau hinh de hon
// Neu relay cua ban kich muc cao thi doi true thanh false.
const bool RELAY_ACTIVE_LOW = true;   // sua lai vi relay dang bi nguoc: bat thanh tat, tat thanh bat

// Loc nhieu cho cam bien dat
const int SOIL_FILTER_SAMPLES = 7;
const int SOIL_FILTER_DELAY_MS = 12;

// =======================
// BIẾN TOÀN CỤC
// =======================
ESP8266WebServer server(80);
DHT dht(DHTPIN, DHTTYPE);

float temperatureC = 0.0;
float humidity = 0.0;
float lastValidTempC = 0.0;
float lastValidHumidity = 0.0;
bool dhtValid = false;
int soilRaw = 0;

bool pumpOn = false;
bool autoThresholdOn = false;
bool soilDryAlert = false;
bool noWaterAlert = false;   // tạm để OFF vì chưa dùng flow sensor
bool dosingActive = false;

float dosingTargetMl = 0.0;

float flowLMin = 0.0;
float flowLMinSmooth = 0.0;
float cycleMl = 0.0;
float totalMl = 0.0;
int soilPercent = 0;

volatile unsigned long flowPulseCount = 0;
unsigned long lastFlowCalcMs = 0;
int noWaterZeroPulseSeconds = 0;
bool manualMode = false;

unsigned long dosingStartMs = 0;
unsigned long dosingDurationMs = 0;

unsigned long lastDhtReadMs = 0;
unsigned long lastDebugMs = 0;
unsigned long lastPumpChangeMs = 0;
unsigned long lastAutoDecisionMs = 0;
unsigned long manualHoldUntilMs = 0;
int soilLowConfirmCount = 0;
int soilHighConfirmCount = 0;
const unsigned long MIN_PUMP_ON_MS = 5000;
const unsigned long MIN_PUMP_OFF_MS = 3000;
const unsigned long AUTO_DECISION_INTERVAL_MS = 1500;
const unsigned long MANUAL_HOLD_MS = 15000;
const int SOIL_CONFIRM_COUNT = 3;

// =======================
// FLOW SENSOR
// =======================
void IRAM_ATTR onFlowPulse() {
  flowPulseCount++;
}

// =======================
// HÀM PHỤ
// =======================
String onOff(bool v) {
  return v ? "ON" : "OFF";
}

int calcSoilPercent(int rawValue) {
  if (SOIL_RAW_DRY == SOIL_RAW_WET) return 0;

  long pct = map(rawValue, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}

void writeRelay(bool on) {
  int activeLevel = RELAY_ACTIVE_LOW ? LOW : HIGH;
  int inactiveLevel = RELAY_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(RELAY_PIN, on ? activeLevel : inactiveLevel);
}

void setPump(bool on) {
  if (pumpOn == on) return;

  unsigned long now = millis();

  // dang tat ma muon bat => phai tat toi thieu MIN_PUMP_OFF_MS
  if (on && !pumpOn) {
    if (now - lastPumpChangeMs < MIN_PUMP_OFF_MS) return;
  }

  // dang bat ma muon tat => phai bat toi thieu MIN_PUMP_ON_MS
  if (!on && pumpOn) {
    if (now - lastPumpChangeMs < MIN_PUMP_ON_MS) return;
  }

  pumpOn = on;
  writeRelay(on);
  lastPumpChangeMs = now;

  if (!on) {
    flowLMin = 0.0;
    flowLMinSmooth = 0.0;
  }
}

void updateDHT() {
  if (millis() < 4000) return;
  if (millis() - lastDhtReadMs < DHT_READ_INTERVAL) return;
  lastDhtReadMs = millis();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  bool valid = true;
  if (isnan(h) || isnan(t)) valid = false;
  if (t < 0 || t > 60) valid = false;
  if (h < 0 || h > 100) valid = false;

  if (valid) {
    humidity = h;
    temperatureC = t;
    lastValidHumidity = h;
    lastValidTempC = t;
    dhtValid = true;
  } else {
    if (dhtValid) {
      humidity = lastValidHumidity;
      temperatureC = lastValidTempC;
    } else {
      humidity = 0.0;
      temperatureC = 0.0;
    }
    Serial.println("DHT11 read invalid -> kiem tra day DATA/VCC/GND, dang giu gia tri hop le gan nhat");
  }
}

int readSoilFiltered() {
  int values[SOIL_FILTER_SAMPLES];
  for (int i = 0; i < SOIL_FILTER_SAMPLES; i++) {
    values[i] = analogRead(SOIL_PIN);
    delay(SOIL_FILTER_DELAY_MS);
  }

  // sap xep tang dan de lay trung vi
  for (int i = 0; i < SOIL_FILTER_SAMPLES - 1; i++) {
    for (int j = i + 1; j < SOIL_FILTER_SAMPLES; j++) {
      if (values[j] < values[i]) {
        int tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }

  // ket hop trung vi + trung binh 3 gia tri giua de giam nhieu
  int mid = SOIL_FILTER_SAMPLES / 2;
  int avgMid = (values[mid - 1] + values[mid] + values[mid + 1]) / 3;
  return avgMid;
}

void updateSoil() {
  soilRaw = readSoilFiltered();
  soilPercent = calcSoilPercent(soilRaw);

  soilDryAlert = (soilRaw >= SOIL_DRY_THRESHOLD_RAW);
  if (soilPercent <= SOIL_START_PUMP_PERCENT) {
    soilDryAlert = true;
  }
  if (soilPercent >= SOIL_STOP_PUMP_PERCENT) {
    soilDryAlert = false;
  }
}

void updateFlow() {
  unsigned long now = millis();
  if (now - lastFlowCalcMs < 1000) return;

  noInterrupts();
  unsigned long pulses = flowPulseCount;
  flowPulseCount = 0;
  interrupts();

  float seconds = (now - lastFlowCalcMs) / 1000.0;
  lastFlowCalcMs = now;

  float liters = pulses / FLOW_PULSES_PER_LITER;
  float instantFlowLMin = (seconds > 0) ? (liters * 60.0 / seconds) : 0.0;

  // chan gia tri bat thuong do nhieu xung / rung tin hieu
  if (instantFlowLMin < FLOW_MIN_VALID_LPM) instantFlowLMin = 0.0;
  if (instantFlowLMin > FLOW_MAX_VALID_LPM) instantFlowLMin = 0.0;

  // loc mem de hien thi on dinh hon
  flowLMinSmooth = (FLOW_SMOOTH_ALPHA * instantFlowLMin) + ((1.0 - FLOW_SMOOTH_ALPHA) * flowLMinSmooth);

  // neu dong chay thuc te da ve 0 thi ep ve 0 nhanh de tranh treo so le
  if (instantFlowLMin == 0.0 && flowLMinSmooth < 0.05) {
    flowLMinSmooth = 0.0;
  }

  flowLMin = flowLMinSmooth;

  float mlThisPeriod = liters * 1000.0;
  if (pumpOn) {
    cycleMl += mlThisPeriod;
    totalMl += mlThisPeriod;
  }

  if (pumpOn && pulses == 0) {
    noWaterZeroPulseSeconds++;
  } else {
    noWaterZeroPulseSeconds = 0;
  }

  noWaterAlert = (pumpOn && noWaterZeroPulseSeconds >= NO_WATER_CONFIRM_SECONDS);
}

// =======================
// LOGIC GIỐNG BẢN UNO
// =======================
void handleAutoThresholdLogic() {
  if (millis() - lastAutoDecisionMs < AUTO_DECISION_INTERVAL_MS) return;
  lastAutoDecisionMs = millis();

  // Dang dieu khien tay thi auto khong chen vao
  if (manualMode) return;

  // Chua bat auto
  if (!autoThresholdOn) return;

  // Dang tuoi dinh luong
  if (dosingActive) return;

  if (soilPercent <= SOIL_START_PUMP_PERCENT) {
    soilLowConfirmCount++;
    soilHighConfirmCount = 0;
  } else if (soilPercent >= SOIL_STOP_PUMP_PERCENT) {
    soilHighConfirmCount++;
    soilLowConfirmCount = 0;
  } else {
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
  }

  if (soilLowConfirmCount >= SOIL_CONFIRM_COUNT) {
    setPump(true);
    soilDryAlert = true;
    soilLowConfirmCount = 0;
  }

  if (soilHighConfirmCount >= SOIL_CONFIRM_COUNT) {
    setPump(false);
    soilDryAlert = false;
    soilHighConfirmCount = 0;
  }
}

// mô phỏng tưới định lượng bằng thời gian
// để hành vi gần giống bản Uno cũ
void startDose(float targetMl) {
  dosingActive = true;
  manualMode = false;
  dosingTargetMl = targetMl;
  autoThresholdOn = false;
  cycleMl = 0.0;
  flowLMin = 0.0;
  flowLMinSmooth = 0.0;
  soilLowConfirmCount = 0;
  soilHighConfirmCount = 0;

  if (targetMl <= 20.0) {
    dosingDurationMs = 3000;
  } else {
    dosingDurationMs = 12000;
  }

  dosingStartMs = millis();
  setPump(true);
}

void handleDoseLogic() {
  if (!dosingActive) return;

  if (millis() - dosingStartMs >= dosingDurationMs) {
    dosingActive = false;
    setPump(false);
  }
}
String buildDataJson() {
  String json = "{";
  json += "\"temp\":\"" + String(temperatureC, 1) + "\",";
  json += "\"hum\":\"" + String(humidity, 1) + "\",";
  json += "\"dht_status\":\"" + String(dhtValid ? "OK" : "INVALID") + "\",";
  json += "\"soil\":\"" + String(soilRaw) + "\",";
  json += "\"soil_percent\":\"" + String(soilPercent) + "\",";
  json += "\"pump\":\"" + onOff(pumpOn) + "\",";
  json += "\"flow\":\"" + String(flowLMin, 2) + "\",";
  json += "\"cycle_ml\":\"" + String(cycleMl, 2) + "\",";
  json += "\"total_ml\":\"" + String(totalMl, 2) + "\",";
  json += "\"auto_threshold\":\"" + onOff(autoThresholdOn) + "\",";
  json += "\"soil_dry_alert\":\"" + onOff(soilDryAlert) + "\",";
  json += "\"no_water_alert\":\"" + onOff(noWaterAlert) + "\",";
  json += "\"dosing_active\":\"" + onOff(dosingActive) + "\",";
  json += "\"manual_mode\":\"" + onOff(manualMode) + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"wifi_status\":\"" + String(WiFi.status()) + "\",";
  json += "\"relay_mode\":\"" + String(RELAY_ACTIVE_LOW ? "ACTIVE_LOW" : "ACTIVE_HIGH") + "\",";
  json += "\"soil_filter\":\"" + String(SOIL_FILTER_SAMPLES) + "_samples_median_avg\",";
  json += "\"soil_calibration\":\"DRY=" + String(SOIL_RAW_DRY) + ";WET=" + String(SOIL_RAW_WET) + "\",";
  json += "\"soil_control\":\"START=" + String(SOIL_START_PUMP_PERCENT) + "%;STOP=" + String(SOIL_STOP_PUMP_PERCENT) + "%\",";
  json += "\"flow_calibration\":\"" + String(FLOW_PULSES_PER_LITER, 1) + "_pulses_per_liter\",";
  json += "\"flow_smoothing\":\"" + String(FLOW_SMOOTH_ALPHA, 2) + "_ema\",";
  json += "\"pump_guard\":\"ON>=" + String(MIN_PUMP_ON_MS) + "ms;OFF>=" + String(MIN_PUMP_OFF_MS) + "ms\",";
  json += "\"auto_guard\":\"confirm=" + String(SOIL_CONFIRM_COUNT) + ";interval=" + String(AUTO_DECISION_INTERVAL_MS) + "ms\",";
  json += "\"note\":\"ESP8266 toi uu theo hieu chuan thuc te: WiFi reconnect, relay configurable, soil percent calibration, flow calibration\"";
  json += "}";
  return json;
}

// =======================
// API GIỐNG BẢN UNO + WEB
// =======================
void handleRoot() {
  server.send(200, "text/plain", "ESP8266 Smart Farm API OK");
}

void handleData() {
  server.send(200, "application/json", buildDataJson());
}

void sendOk(String msg) {
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"" + msg + "\"}");
}

void sendBad(String msg) {
  server.send(400, "application/json", "{\"ok\":false,\"message\":\"" + msg + "\"}");
}

void handleCommand() {
  if (!server.hasArg("cmd")) {
    sendBad("missing cmd");
    return;
  }

  String cmd = server.arg("cmd");
  cmd.trim();

  if (cmd == "PUMP_ON") {
    manualMode = true;
    manualHoldUntilMs = millis() + MANUAL_HOLD_MS;
    dosingActive = false;
    autoThresholdOn = false;
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
    setPump(true);
    sendOk("PUMP_ON");
    return;
  }

  if (cmd == "PUMP_OFF") {
    manualMode = true;
    manualHoldUntilMs = millis() + MANUAL_HOLD_MS;
    dosingActive = false;
    autoThresholdOn = false;
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
    setPump(false);
    sendOk("PUMP_OFF");
    return;
  }

  if (cmd == "AUTO_WATER_20") {
    manualMode = false;
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
    startDose(20.0);
    sendOk("AUTO_WATER_20");
    return;
  }

  if (cmd == "AUTO_WATER_200") {
    manualMode = false;
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
    startDose(200.0);
    sendOk("AUTO_WATER_200");
    return;
  }

  if (cmd == "AUTO_THRESHOLD_ON") {
    manualMode = false;
    dosingActive = false;
    autoThresholdOn = true;
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
    sendOk("AUTO_THRESHOLD_ON");
    return;
  }

  if (cmd == "AUTO_THRESHOLD_OFF") {
    manualMode = false;
    autoThresholdOn = false;
    dosingActive = false;
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
    setPump(false);
    sendOk("AUTO_THRESHOLD_OFF");
    return;
  }

  if (cmd == "STOP_AUTO") {
    manualMode = false;
    autoThresholdOn = false;
    dosingActive = false;
    soilLowConfirmCount = 0;
    soilHighConfirmCount = 0;
    setPump(false);
    sendOk("STOP_AUTO");
    return;
  }

  if (cmd == "READ_ALL") {
    server.send(200, "application/json", buildDataJson());
    return;
  }

  sendBad("unknown cmd");
}

void setupRoutes() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/command", handleCommand);
  server.begin();
  Serial.println("HTTP server started");
}

// =======================
// WIFI
// =======================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(1000);

  Serial.println("====================================");
  Serial.print("Dang ket noi toi SSID: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 60) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  Serial.println();
  Serial.print("WiFi status = ");
  Serial.println(WiFi.status());

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Khong vao duoc WiFi luc khoi dong.");
    Serial.println("He thong van chay local va se tu reconnect trong loop.");
  }
  Serial.println("====================================");
}

void ensureWiFiConnected() {
  static unsigned long lastReconnectAttempt = 0;
  if (WiFi.status() == WL_CONNECTED) return;

  if (millis() - lastReconnectAttempt < 10000) return;
  lastReconnectAttempt = millis();

  Serial.println("WiFi bi mat. Dang thu reconnect...");
  WiFi.disconnect();
  delay(200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("BOOT OK");
  Serial.print("Reset reason: ");
  Serial.println(ESP.getResetReason());

  pinMode(RELAY_PIN, OUTPUT);
  writeRelay(false);

  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), onFlowPulse, FALLING);
  lastFlowCalcMs = millis();

  dht.begin();
  delay(2000);

  connectWiFi();
  setupRoutes();

  lastDhtReadMs = 0;
}

// =======================
// LOOP
// =======================
void loop() {
  ensureWiFiConnected();
  server.handleClient();
  yield();

  updateDHT();
  updateSoil();
  updateFlow();

  // Dang manual mode thi giu quyen dieu khien tay.
  // Chi khi nguoi dung bam AUTO_THRESHOLD_ON moi tra quyen cho auto.
  if (manualMode && millis() > manualHoldUntilMs && !autoThresholdOn) {
    // giu nguyen manual, khong tu tra quyen
  }

  handleDoseLogic();
  handleAutoThresholdLogic();

  if (millis() - lastDebugMs > 3000) {
    lastDebugMs = millis();
    Serial.print("WiFi status = ");
    Serial.print(WiFi.status());
    Serial.print(" | IP = ");
    Serial.print(WiFi.localIP());
    Serial.print(" | SoilRaw = ");
    Serial.print(soilRaw);
    Serial.print(" | Soil% = ");
    Serial.print(soilPercent);
    Serial.print(" | Temp = ");
    Serial.print(temperatureC, 1);
    Serial.print(" | Hum = ");
    Serial.print(humidity, 1);
    Serial.print(" | DHT = ");
    Serial.print(dhtValid ? "OK" : "INVALID");
    Serial.print(" | Pump = ");
    Serial.print(onOff(pumpOn));
    Serial.print(" | Flow = ");
    Serial.print(flowLMin, 2);
    Serial.print(" | Manual = ");
    Serial.print(onOff(manualMode));
    Serial.print(" | Auto = ");
    Serial.print(onOff(autoThresholdOn));
    Serial.print(" | LowC = ");
    Serial.print(soilLowConfirmCount);
    Serial.print(" | HighC = ");
    Serial.print(soilHighConfirmCount);
    Serial.print(" | RelayMode = ");
    Serial.println(RELAY_ACTIVE_LOW ? "ACTIVE_LOW" : "ACTIVE_HIGH");
  }

  delay(1);
}