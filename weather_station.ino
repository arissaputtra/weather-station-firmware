/*
  ============================================================
  Weather Station - ESP32
  ============================================================
  CATATAN PIN: komentar file asli bilang "ESP32-C3", tapi PIN_BATTERY_ADC=11
  cuma valid ADC di ESP32-S3 (ADC1 C3 cuma sampai GPIO4). Pin dibiarkan
  sesuai yang sudah kamu set - cek ulang mapping-nya pas pindah ke hardware final.

  FITUR BARU DI VERSI INI:
  1. Kalibrasi baterai empiris (bukan cuma rasio divider teoritis lagi).
  2. Baca sensor tiap READ_INTERVAL_SEC detik, dirata-rata tiap AVERAGE_WINDOW_SEC detik.
  3. Kirim ke Sheets tiap SEND_INTERVAL_MIN menit, sekali kirim = banyak data (batch/array).
  4. WiFi radio OFF di luar jendela kirim (hemat daya), nyala WIFI_LEAD_TIME_SEC
     detik sebelum jadwal kirim. Relay 2 nyala RELAY2_LEAD_TIME_MIN menit sebelum kirim.
  5. Kalau WiFi gak ketemu pas jadwal kirim, data TIDAK dibuang - digabung
     dengan siklus kirim berikutnya (semua data lama+baru dikirim sekaligus).

  SEMUA ANGKA DI BAWAH INI (interval baca, rata-rata, kirim, lead time)
  BISA DIUBAH BEBAS - tinggal ganti konstanta di bagian "PENGATURAN WAKTU".
  ============================================================
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

// ---------------- WiFi & Google Sheets ----------------
const char* WIFI_SSID     = "Testing IoT";
const char* WIFI_PASSWORD = "12345678";
const char* GAS_URL = "https://script.google.com/macros/s/AKfycbxcdF5NI9K33YufzSAEqrRSuIck2Fw2CBREPauTBzRBM9T84nHGqgyVEU2iCCeuF-ma/exec";

// ---------------- OTA Update via GitHub ----------------
const char* FIRMWARE_VERSION  = "1.0.0"; 
const char* OTA_VERSION_URL   = "https://raw.githubusercontent.com/USERNAME/REPO/main/firmware/version.txt";
const char* OTA_FIRMWARE_URL  = "https://raw.githubusercontent.com/USERNAME/REPO/main/firmware/weather_station.bin";
const unsigned long OTA_CHECK_INTERVAL_HOURS = 24; 
const unsigned long OTA_CHECK_INTERVAL_MS = OTA_CHECK_INTERVAL_HOURS * 3600UL * 1000UL;

// ---------------- Pin Mapping ----------------
#define PIN_BATTERY_ADC 11
#define PIN_SOIL_ADC    7
#define PIN_DS18B20     4
#define PIN_DS18B20V2   17   // <--- PIN UNTUK DS18B20 KEDUA
#define PIN_RELAY1      5
#define PIN_RELAY2      6
#define PIN_SDA         8
#define PIN_SCL         9

// ---------------- PENGATURAN WAKTU ----------------
const unsigned long READ_INTERVAL_SEC    = 5;   
const unsigned long AVERAGE_WINDOW_SEC   = 30;  
const unsigned long SEND_INTERVAL_MIN    = 5;   
const unsigned long RELAY2_LEAD_TIME_MIN = 1;   
const unsigned long WIFI_LEAD_TIME_SEC   = 30;  
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000; 
const unsigned long HTTP_TIMEOUT_MS = 20000; 

// ---------------- Hasil turunan ----------------
const unsigned long READ_INTERVAL_MS    = READ_INTERVAL_SEC * 1000UL;
const unsigned long AVERAGE_WINDOW_MS   = AVERAGE_WINDOW_SEC * 1000UL;
const int           AVERAGE_COUNT       = AVERAGE_WINDOW_SEC / READ_INTERVAL_SEC;    
const unsigned long SEND_INTERVAL_MS    = SEND_INTERVAL_MIN * 60UL * 1000UL;
const unsigned long RELAY2_LEAD_TIME_MS = RELAY2_LEAD_TIME_MIN * 60UL * 1000UL;
const unsigned long WIFI_LEAD_TIME_MS   = WIFI_LEAD_TIME_SEC * 1000UL;
const int           BATCH_SIZE          = SEND_INTERVAL_MS / AVERAGE_WINDOW_MS;      
const int           MAX_BUFFERED_RECORDS = BATCH_SIZE * 3; 

// ---------------- Kalibrasi Soil Moisture ----------------
int SOIL_AIR_VALUE   = 3300;  
int SOIL_WATER_VALUE = 2470;  
bool CALIBRATION_MODE = false;

// ---------------- Kalibrasi Baterai ----------------
const float BATTERY_CALIBRATION_FACTOR = 1.3737;
const float BATTERY_MAX_VOLTAGE = 4.127; 
const float BATTERY_MIN_VOLTAGE = 3.0;   

// ---------------- Relay 1 Hysteresis ----------------
const int RELAY1_ON_THRESHOLD  = 40;
const int RELAY1_OFF_THRESHOLD = 60;
bool relay1State = false;
bool relay2State = false; 

// ---------------- Objek Sensor ----------------
// Dipisah jadi 2 objek OneWire agar tidak error
OneWire oneWire1(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire1);

OneWire oneWire2(PIN_DS18B20V2);
DallasTemperature ds18b20V2(&oneWire2);

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
BH1750 lightMeter;

bool ahtOk = false;
bool bmpOk = false;
bool bh1750Ok = false;

// ---------------- Data sensor instan ----------------
struct SensorData {
  float tempDS18B20   = NAN;
  float tempDS18B20V2 = NAN; // <--- Variabel sensor 2
  float tempAHT       = NAN;
  float humidityAHT   = NAN;
  float pressureBMP   = NAN;
  float luxBH1750     = NAN;
  int   soilRaw        = 0;
  int   soilPercent    = 0;
  float batteryVoltage = 0;
  int   batteryPercent = 0;
} data;

// ---------------- Satu titik data hasil rata-rata ----------------
struct AveragedRecord {
  float tempDS18B20;
  float tempDS18B20V2; // <--- Variabel rata-rata sensor 2
  float tempAHT;
  float humidityAHT;
  float pressureBMP;
  float luxBH1750;
  int   soilRaw;
  int   soilPercent;
  float batteryVoltage;
  int   batteryPercent;
  bool  relay1;
  bool  relay2;
  unsigned long recordMillis;
};

AveragedRecord recordBuffer[MAX_BUFFERED_RECORDS];
int recordCount = 0;

// ---------------- Akumulator untuk rata-rata berjalan ----------------
float accTempDS18B20 = 0, accTempDS18B20V2 = 0, accTempAHT = 0, accHumidityAHT = 0, accPressureBMP = 0, accLuxBH1750 = 0;
long  accSoilRaw = 0;
float accBatteryVoltage = 0;
int   cntTempDS18B20 = 0, cntTempDS18B20V2 = 0, cntTempAHT = 0, cntHumidityAHT = 0, cntPressureBMP = 0, cntLuxBH1750 = 0;
int   sampleCount = 0; 

// ---------------- Timer & state ----------------
unsigned long lastReadMillis   = 0;
unsigned long nextSendMillis   = 0;
bool relay2ActivatedForCycle   = false;
bool wifiActivatedForCycle     = false;
unsigned long lastOtaCheckMillis = 0;
bool otaCheckedOnce = false;

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY2, OUTPUT);
  digitalWrite(PIN_RELAY1, LOW);
  digitalWrite(PIN_RELAY2, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOIL_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  ds18b20.begin();
  ds18b20V2.begin(); // <--- Inisialisasi sensor 2

  ahtOk = aht.begin(&Wire);
  if (!ahtOk) Serial.println("WARNING: AHT20 tidak terdeteksi!");

  bmpOk = bmp.begin(0x76);
  if (!bmpOk) bmpOk = bmp.begin(0x77);
  if (!bmpOk) Serial.println("WARNING: BMP280 tidak terdeteksi di 0x76 maupun 0x77!");

  bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  if (!bh1750Ok) Serial.println("WARNING: BH1750 tidak terdeteksi!");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  nextSendMillis = millis() + SEND_INTERVAL_MS;

  Serial.println("=== Weather Station Ready ===");
  Serial.print("Baca tiap "); Serial.print(READ_INTERVAL_SEC); Serial.println(" detik");
  Serial.print("Rata-rata tiap "); Serial.print(AVERAGE_WINDOW_SEC); Serial.print(" detik ("); Serial.print(AVERAGE_COUNT); Serial.println(" sampel/titik)");
  Serial.print("Kirim tiap "); Serial.print(SEND_INTERVAL_MIN); Serial.print(" menit ("); Serial.print(BATCH_SIZE); Serial.println(" titik data/kirim)");

  if (CALIBRATION_MODE) Serial.println("MODE KALIBRASI SOIL AKTIF.");
}

// ============================================================
void loop() {
  unsigned long now = millis();

  if (now - lastReadMillis >= READ_INTERVAL_MS) {
    lastReadMillis = now;
    readAllSensors();
    if (!CALIBRATION_MODE) {
      controlRelay1();
      accumulateSample();
      printSensorData();

      if (sampleCount >= AVERAGE_COUNT) {
        finalizeAverageRecord(now);
      }
    } else {
      printSensorData();
    }
  }

  if (CALIBRATION_MODE) return; 

  long msUntilSend = (long)(nextSendMillis - now);

  if (!relay2ActivatedForCycle && msUntilSend <= (long)RELAY2_LEAD_TIME_MS) {
    relay2State = true;
    digitalWrite(PIN_RELAY2, HIGH);
    relay2ActivatedForCycle = true;
    Serial.println(">> Relay 2 ON (bersiap kirim data)");
  }

  if (!wifiActivatedForCycle && msUntilSend <= (long)WIFI_LEAD_TIME_MS) {
    Serial.println(">> Menyalakan radio WiFi (bersiap kirim data)...");
    connectWiFi();
    wifiActivatedForCycle = true;
  }

  if (msUntilSend <= 0) {
    bool sent = false;
    if (WiFi.status() == WL_CONNECTED) {
      sent = sendBatchToGoogleSheets(now);

      bool timeForOtaCheck = (!otaCheckedOnce) || (now - lastOtaCheckMillis >= OTA_CHECK_INTERVAL_MS);
      if (timeForOtaCheck) {
        lastOtaCheckMillis = now;
        otaCheckedOnce = true;
        checkForOTAUpdate(); 
      }
    } else {
      Serial.println(">> WiFi tidak ditemukan saat jadwal kirim. Data ditahan, digabung ke siklus berikutnya.");
    }

    relay2State = false;
    digitalWrite(PIN_RELAY2, LOW);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println(">> Relay 2 OFF, radio WiFi dimatikan lagi.");

    if (sent) recordCount = 0; 
    
    relay2ActivatedForCycle = false;
    wifiActivatedForCycle   = false;
    nextSendMillis = now + SEND_INTERVAL_MS; 
  }
}

// ============================================================
void accumulateSample() {
  if (!isnan(data.tempDS18B20))   { accTempDS18B20 += data.tempDS18B20; cntTempDS18B20++; }
  if (!isnan(data.tempDS18B20V2)) { accTempDS18B20V2 += data.tempDS18B20V2; cntTempDS18B20V2++; } // <--- Akumulasi sensor 2
  if (!isnan(data.tempAHT))       { accTempAHT     += data.tempAHT;     cntTempAHT++; }
  if (!isnan(data.humidityAHT))   { accHumidityAHT += data.humidityAHT; cntHumidityAHT++; }
  if (!isnan(data.pressureBMP))   { accPressureBMP += data.pressureBMP; cntPressureBMP++; }
  if (!isnan(data.luxBH1750))     { accLuxBH1750   += data.luxBH1750;   cntLuxBH1750++; }

  accSoilRaw        += data.soilRaw;
  accBatteryVoltage += data.batteryVoltage;
  sampleCount++;
}

// ============================================================
void finalizeAverageRecord(unsigned long now) {
  if (recordCount >= MAX_BUFFERED_RECORDS) {
    Serial.println("!! Buffer data penuh (WiFi kemungkinan mati lama) - titik data ini dilewati.");
  } else {
    AveragedRecord rec;
    rec.tempDS18B20   = cntTempDS18B20   > 0 ? accTempDS18B20   / cntTempDS18B20   : NAN;
    rec.tempDS18B20V2 = cntTempDS18B20V2 > 0 ? accTempDS18B20V2 / cntTempDS18B20V2 : NAN; // <--- Rata-rata sensor 2
    rec.tempAHT       = cntTempAHT       > 0 ? accTempAHT       / cntTempAHT       : NAN;
    rec.humidityAHT   = cntHumidityAHT   > 0 ? accHumidityAHT   / cntHumidityAHT   : NAN;
    rec.pressureBMP   = cntPressureBMP   > 0 ? accPressureBMP   / cntPressureBMP   : NAN;
    rec.luxBH1750     = cntLuxBH1750     > 0 ? accLuxBH1750     / cntLuxBH1750     : NAN;

    int avgSoilRaw = accSoilRaw / sampleCount;
    rec.soilRaw = avgSoilRaw;
    int pct = map(avgSoilRaw, SOIL_AIR_VALUE, SOIL_WATER_VALUE, 0, 100);
    rec.soilPercent = constrain(pct, 0, 100);

    float avgBatteryVoltage = accBatteryVoltage / sampleCount;
    rec.batteryVoltage = avgBatteryVoltage;
    int battPct = (int)((avgBatteryVoltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0);
    rec.batteryPercent = constrain(battPct, 0, 100);

    rec.relay1 = relay1State;
    rec.relay2 = relay2State;
    rec.recordMillis = now;

    recordBuffer[recordCount] = rec;
    recordCount++;

    Serial.print(">> Titik data rata-rata #"); Serial.print(recordCount);
    Serial.print(" tersimpan (soil "); Serial.print(rec.soilPercent);
    Serial.print("%, batt "); Serial.print(rec.batteryVoltage); Serial.println("V)");
  }

  // Reset semua akumulator
  accTempDS18B20 = accTempDS18B20V2 = accTempAHT = accHumidityAHT = accPressureBMP = accLuxBH1750 = 0;
  accSoilRaw = accBatteryVoltage = 0;
  cntTempDS18B20 = cntTempDS18B20V2 = cntTempAHT = cntHumidityAHT = cntPressureBMP = cntLuxBH1750 = 0;
  sampleCount = 0;
}

// ============================================================
void readAllSensors() {
  ds18b20.requestTemperatures(); 
  float t1 = ds18b20.getTempCByIndex(0);
  data.tempDS18B20 = (t1 == DEVICE_DISCONNECTED_C) ? NAN : t1;

  ds18b20V2.requestTemperatures(); // <--- Baca sensor 2
  float t2 = ds18b20V2.getTempCByIndex(0);
  data.tempDS18B20V2 = (t2 == DEVICE_DISCONNECTED_C) ? NAN : t2;

  if (ahtOk) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    data.tempAHT     = temp.temperature;
    data.humidityAHT = humidity.relative_humidity;
  }

  if (bmpOk) {
    data.pressureBMP = bmp.readPressure() / 100.0F;
  }

  if (bh1750Ok) {
    data.luxBH1750 = lightMeter.readLightLevel();
  }

  // --- Soil Moisture ---
  data.soilRaw = analogRead(PIN_SOIL_ADC);
  if (CALIBRATION_MODE) {
    static int rawMin = 4095, rawMax = 0;
    if (data.soilRaw < rawMin) rawMin = data.soilRaw;
    if (data.soilRaw > rawMax) rawMax = data.soilRaw;
    Serial.print("Soil Raw: "); Serial.print(data.soilRaw);
    Serial.print(" | Min: "); Serial.print(rawMin);
    Serial.print(" | Max: "); Serial.println(rawMax);
  } else {
    int pct = map(data.soilRaw, SOIL_AIR_VALUE, SOIL_WATER_VALUE, 0, 100);
    data.soilPercent = constrain(pct, 0, 100);
  }

  // --- Battery Voltage ---
  int batMv = analogReadMilliVolts(PIN_BATTERY_ADC);
  float measuredAdcVoltage = batMv / 1000.0;
  data.batteryVoltage = measuredAdcVoltage * BATTERY_CALIBRATION_FACTOR;

  int battPct = (int)((data.batteryVoltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0);
  data.batteryPercent = constrain(battPct, 0, 100);
}

// ============================================================
void controlRelay1() {
  if (!relay1State && data.soilPercent <= RELAY1_ON_THRESHOLD) {
    relay1State = true;
    digitalWrite(PIN_RELAY1, HIGH);
    Serial.println(">> Relay 1 ON (soil kering, mulai irigasi)");
  } else if (relay1State && data.soilPercent >= RELAY1_OFF_THRESHOLD) {
    relay1State = false;
    digitalWrite(PIN_RELAY1, LOW);
    Serial.println(">> Relay 1 OFF (soil sudah cukup basah)");
  }
}

// ============================================================
void printSensorData() {
  Serial.println("---- Sensor Data (instan) ----");
  Serial.print("DS18B20 Temp   : "); Serial.print(data.tempDS18B20); Serial.println(" C");
  Serial.print("DS18B20V2 Temp : "); Serial.print(data.tempDS18B20V2); Serial.println(" C"); // <--- Print sensor 2
  Serial.print("AHT20 Temp     : "); Serial.print(data.tempAHT); Serial.println(" C");
  Serial.print("AHT20 Humidity : "); Serial.print(data.humidityAHT); Serial.println(" %");
  Serial.print("BMP280 Pressure: "); Serial.print(data.pressureBMP); Serial.println(" hPa");
  Serial.print("BH1750 Lux     : "); Serial.print(data.luxBH1750); Serial.println(" lx");
  Serial.print("Soil Moisture  : "); Serial.print(data.soilPercent); Serial.print(" % (raw "); Serial.print(data.soilRaw); Serial.println(")");
  Serial.print("Battery        : "); Serial.print(data.batteryVoltage); Serial.print(" V ("); Serial.print(data.batteryPercent); Serial.println(" %)");
  Serial.print("Relay1 (irigasi): "); Serial.println(relay1State ? "ON" : "OFF");
  Serial.print("Relay2 (jadwal) : "); Serial.println(relay2State ? "ON" : "OFF");
  Serial.println("-------------------------------");
}

// ============================================================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Menghubungkan ke WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Terhubung!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" Gagal konek WiFi.");
  }
}

// ============================================================
String recordToJson(const AveragedRecord &r, unsigned long nowMillis) {
  float secondsAgo = (nowMillis - r.recordMillis) / 1000.0;

  String json = "{";
  json += "\"seconds_ago\":" + String(secondsAgo, 1) + ",";
  json += "\"temp_ds18b20\":" + String(r.tempDS18B20, 2) + ",";
  json += "\"temp_ds18b20_v2\":" + String(r.tempDS18B20V2, 2) + ","; // <--- Label JSON diperbaiki agar tidak nabrak
  json += "\"temp_aht\":" + String(r.tempAHT, 2) + ",";
  json += "\"humidity_aht\":" + String(r.humidityAHT, 2) + ",";
  json += "\"pressure_bmp\":" + String(r.pressureBMP, 2) + ",";
  json += "\"lux_bh1750\":" + String(r.luxBH1750, 2) + ",";
  json += "\"soil_raw\":" + String(r.soilRaw) + ",";
  json += "\"soil_percent\":" + String(r.soilPercent) + ",";
  json += "\"battery_voltage\":" + String(r.batteryVoltage, 3) + ",";
  json += "\"battery_percent\":" + String(r.batteryPercent) + ",";
  json += "\"relay1\":" + String(r.relay1 ? 1 : 0) + ",";
  json += "\"relay2\":" + String(r.relay2 ? 1 : 0);
  json += "}";
  return json;
}

// ============================================================
bool sendBatchToGoogleSheets(unsigned long nowMillis) {
  if (recordCount == 0) {
    Serial.println(">> Tidak ada data untuk dikirim.");
    return true; 
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, GAS_URL);
  http.addHeader("Content-Type", "application/json");
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  http.setTimeout(HTTP_TIMEOUT_MS);

  String payload = "{\"records\":[";
  for (int i = 0; i < recordCount; i++) {
    if (i > 0) payload += ",";
    payload += recordToJson(recordBuffer[i], nowMillis);
  }
  payload += "]}";

  Serial.print(">> Mengirim "); Serial.print(recordCount); Serial.println(" titik data ke Google Sheets...");

  int httpCode = http.POST(payload);
  bool success = false;

  if (httpCode == HTTP_CODE_FOUND || httpCode == 301 || httpCode == 302 || httpCode == 307) {
    success = true;
    String redirectUrl = http.getLocation();
    http.end();
    client.stop();

    if (redirectUrl.length() > 0) {
      WiFiClientSecure client2;
      client2.setInsecure();
      HTTPClient http2;
      http2.begin(client2, redirectUrl);
      http2.setTimeout(HTTP_TIMEOUT_MS);
      int httpCode2 = http2.GET();
      Serial.print("Konfirmasi dari Sheets, HTTP code: ");
      Serial.println(httpCode2);
      if (httpCode2 > 0) Serial.println(http2.getString());
      http2.end();
      client2.stop();
    }
    return success;
  }

  if (httpCode == 200) {
    Serial.println("Kirim ke Sheets, HTTP code: 200");
    Serial.println(http.getString());
    success = true;
  } else if (httpCode > 0) {
    Serial.print("Kirim ke Sheets GAGAL, HTTP code: ");
    Serial.println(httpCode);
    Serial.println(http.getString());
  } else {
    Serial.print("Gagal kirim, error koneksi: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
  client.stop();
  return success;
}

// ============================================================
void checkForOTAUpdate() {
  Serial.println(">> Cek update firmware dari GitHub...");

  WiFiClientSecure otaClient;
  otaClient.setInsecure(); 

  HTTPClient http;
  http.begin(otaClient, OTA_VERSION_URL);
  http.setTimeout(HTTP_TIMEOUT_MS);
  int code = http.GET();

  if (code == 200) {
    String remoteVersion = http.getString();
    remoteVersion.trim();
    http.end();

    Serial.print("Versi firmware terpasang : "); Serial.println(FIRMWARE_VERSION);
    Serial.print("Versi firmware di GitHub  : "); Serial.println(remoteVersion);

    if (remoteVersion.length() > 0 && remoteVersion != String(FIRMWARE_VERSION)) {
      Serial.println(">> Versi baru ditemukan, mulai download & flash OTA...");
      performOTAUpdate();
    } else {
      Serial.println(">> Firmware sudah versi terbaru, tidak ada yang perlu diupdate.");
    }
  } else {
    Serial.print(">> Gagal ambil version.txt, HTTP code: ");
    Serial.println(code);
    http.end();
  }
}

// ============================================================
void performOTAUpdate() {
  WiFiClientSecure otaClient;
  otaClient.setInsecure();

  httpUpdate.rebootOnUpdate(true); 

  Serial.println(">> Downloading firmware dari GitHub, jangan cabut daya...");
  t_httpUpdate_return result = httpUpdate.update(otaClient, OTA_FIRMWARE_URL);

  switch (result) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(">> OTA GAGAL. Error (%d): %s\n",
                     httpUpdate.getLastError(),
                     httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(">> Server bilang tidak ada update (padahal version.txt beda - cek konsistensi rilis).");
      break;
    case HTTP_UPDATE_OK:
      Serial.println(">> OTA SUKSES! Restart ke firmware baru...");
      break; 
  }
}