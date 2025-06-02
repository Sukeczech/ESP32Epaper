#include <WiFi.h>
#include <WiFiClientSecure.h> // Pro HTTPS spojení
#include <HTTPClient.h>      // Pro stahování dat z TMEP a odesílání na Windy
#include <ArduinoJson.h>     // Pro parsování JSON z TMEP
#include <WiFiUdp.h>
#include <NTPClient.h>
// #include <WiFiManager.h> // Odebráno
#include <time.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <stdarg.h>
#include <math.h> // Pro sqrt, pow a NAN

// --- VERZE FIRMWARU ---
const char* FIRMWARE_VERSION = "2.0.7 (Intervals & Median N=3 Update)"; // Aktualizovaná verze

// --- Přihlašovací údaje k WiFi ---
const char* WIFI_SSID = "MS";
const char* WIFI_PASSWORD = "9110014100";

// --- Přidané definice pro WiFi anténu ---
#define WIFI_ENABLE 3      // GPIO pin pro povolení WiFi modulu (RF switch)
#define WIFI_ANT_CONFIG 14 // GPIO pin pro konfiguraci antény (výběr externí antény)

// --- Nastavení Srážkoměru (TMEP) ---
const char* RAIN_SERVER_ADDRESS = "f7yknf-892ymj.tmep.cz";
const char* RAIN_SERVER_PATH = "/index.php";
const char* RAIN_DATA_PARAM_NAME = "rain";
const char* RAIN_MONTH_PARAM_NAME = "rainmonth";
const char* RAIN_YEAR_PARAM_NAME = "rainyear";
const char* RSSI_PARAM_NAME = "rssi";      // Společný název parametru
const char* VOLTAGE_PARAM_NAME = "voltage";  // Společný název parametru

// --- Nastavení Větroměru (TMEP) ---
const char* WIND_SERVER_ADDRESS = "h4rprp-uayb4y.tmep.cz";
const char* WIND_SERVER_PATH = "/index.php";
const char* WIND_SPEED_PARAM_NAME = "windspeed";
const char* WIND_GUST_PARAM_NAME = "windspeedmax";
const char* WIND_CV_PARAM_NAME = "windcv";

// --- Nastavení pro stahování dat z TMEP (BME280) ---
const char* TMEP_JSON_URL = "https://tmep.cz/vystup-json.php?id=8510&export_key=e5ifu4co3a&extended=1"; // DEFINICE JE ZDE

// --- Nastavení pro odesílání odvozených dat (TMEP) ---
const char* DERIVED_DATA_SERVER_ADDRESS = "m38bm7-a6w3mt.tmep.cz";
const char* DERIVED_DATA_SERVER_PATH = "/index.php";
const char* WINDCHILL_PARAM_NAME = "WindChill";
const char* DEWPOINT_PARAM_NAME = "DewPoint";
const char* HEATINDEX_PARAM_NAME = "HeatIndex";

// --- Nastavení pro Windy.com ---
const char* WINDY_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJjaSI6MTQ4NjgyODMsImlhdCI6MTc0NzkxMTMxOH0.NbDQjJFI-Ha_ztuc1CI-DG48I4HIkg4MGAbnggnFS7A";
const char* WINDY_SERVER_ADDRESS = "stations.windy.com";
const char* WINDY_SERVER_PATH_PREFIX = "/pws/update/";
const char* WINDY_PARAM_TIMESTAMP = "ts";
const char* WINDY_PARAM_TEMP = "temp";       // teplota ve stupních Celsia
const char* WINDY_PARAM_RH = "rh";           // relativní vlhkost v %
const char* WINDY_PARAM_MBAR = "mbar";       // tlak v hPa (milibarech)
const char* WINDY_PARAM_WIND = "wind";       // rychlost větru v m/s
const char* WINDY_PARAM_GUST = "gust";       // náraz větru v m/s
const char* WINDY_PARAM_DEWPOINT = "dewpoint"; // rosný bod ve stupních Celsia
const char* WINDY_PARAM_PRECIP = "precip";     // srážky za poslední hodinu v mm


const int TMEP_SERVER_PORT = 80;
const int WINDY_SERVER_PORT = 443; // HTTPS
const int MAX_RETRIES = 3;

// Dešťový senzor
const int RAIN_SENSOR_PIN = 1;
const float RAINFALL_PER_IMPULSE_MM = 0.2794f; // mm srážek na jeden impuls

// Větrný senzor
const int WIND_SENSOR_PIN = 2;
const float WIND_MPS_PER_PULSE_PER_SEC = 0.0875f; // m/s na (impuls/sekundu)

// Měření baterie
#define BATTERY_ADC_PIN 0
#define BATTERY_CALIBRATION_MULTIPLIER (0.00207286F) // Kalibrační konstanta pro převod ADC na napětí
const int NUM_ADC_SAMPLES = 10; // Počet vzorků ADC pro průměrování

const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 0; // Budeme používat TZ string, takže UTC
const int NTP_UPDATE_INTERVAL_MS = 60000; // Aktualizace NTP každou minutu

// Interval odesílání dat (5 minut a 10 sekund)
const long SEND_INTERVAL_MS = (5 * 60 + 10) * 1000L; // 310000 ms

// Interval stahování dat z BME280 (2 minuty)
const unsigned long BME280_FETCH_INTERVAL_MS = 2 * 60 * 1000L; // 120000 ms
unsigned long lastBME280FetchTime = 0;

// Globální proměnné pro data z BME280 (TMEP)
float bme280_temperature = NAN;
float bme280_humidity = NAN;
float bme280_pressure = NAN;

// Globální proměnné pro vypočítané hodnoty
float calculated_wind_chill = NAN;
float calculated_dew_point = NAN;
float calculated_heat_index = NAN;

// Globální proměnné pro počítání odfiltrovaných debounce událostí
volatile unsigned long rainDebounceFilteredCount = 0;
volatile unsigned long windDebounceFilteredCount = 0;
volatile unsigned long windGustsFilteredByRatio = 0; 

// Pro výpočet srážek pro Windy
unsigned long lastSuccessfulWindySendTime = 0;
const int HOURLY_RAIN_BUFFER_SIZE = 60; 
float hourlyRainMinuteBuffer[HOURLY_RAIN_BUFFER_SIZE]; 
int hourlyRainBufferIndex = 0; 
volatile unsigned int currentMinuteRainPulses = 0; 
unsigned long lastMinuteTimestampForHourlyRain = 0; 


Preferences preferences;
const char* PREFS_NAMESPACE = "rainSensor";
const char* PREFS_KEY_MONTH_RAIN = "monthRain";
const char* PREFS_KEY_LAST_RESET_MONTH = "lastRstMon";
const char* PREFS_KEY_LAST_RESET_YEAR_FOR_MONTHLY = "lastRstYrM";
const char* PREFS_KEY_YEAR_RAIN = "yearRain";
const char* PREFS_KEY_YEAR_FOR_ACCUMULATION = "yearForAcc";
const char* PREFS_KEY_DEBOUNCE_RAIN = "debounceRain";
const char* PREFS_KEY_DEBOUNCE_WIND = "debounceWind";
const char* PREFS_KEY_GUST_RATIO = "gustRatio"; 
const char* PREFS_KEY_MIN_AVG_GUST = "minAvgGust"; 
const char* PREFS_KEY_WIND_AVG_EMA_ALPHA = "windAvgAlpha"; 


const unsigned long DEBOUNCE_DELAY_MS_DEFAULT = 800; 
const unsigned long WIND_PULSE_DEBOUNCE_US_DEFAULT = 2000; 
const float DEFAULT_WIND_AVG_EMA_ALPHA = 0.8f; 

volatile unsigned long debounceDelayMs = DEBOUNCE_DELAY_MS_DEFAULT;
volatile unsigned long windPulseDebounceUs = WIND_PULSE_DEBOUNCE_US_DEFAULT;


volatile unsigned long lastRainTime_ms = 0;
volatile unsigned int rainCount = 0; 
volatile float totalRainfallMm = 0.0f; 
volatile float totalRainfallMonthMm = 0.0f;
volatile float totalRainfallYearMm = 0.0f;
volatile unsigned int lastDisplayedRainCount = 0; 
int lastResetMonth = -1; 
int lastResetYearForMonthly = -1; 
int lastKnownYearForYearlyReset = -1; 

volatile unsigned long windPulseCount = 0; 
volatile unsigned long lastWindPulseTime_us = 0; 
unsigned long lastWindPulseSnapshotFor10minAvg = 0; 
float windSpeedAvgValue = 0.0f; 

const unsigned long GUST_SAMPLE_INTERVAL_MS = 3000; 
unsigned long lastGustSampleTime = 0; 
unsigned long pulses_at_last_gust_sample = 0; 
float currentMaxWindGust = 0.0f; 

// --- Parametry pro filtrování nárazů větru (nastavitelné přes Telnet) ---
float maxGustToAvgWindRatio = 3.0f; 
float minAvgWindForRatioCheckMps = 1.0f; 
float windAvgEmaAlpha = DEFAULT_WIND_AVG_EMA_ALPHA; 

// --- Proměnné pro mediánový filtr ---
const int MEDIAN_WINDOW_SIZE = 3; // Změněno na 3
float speedIntervalBuffer[MEDIAN_WINDOW_SIZE];
int speedIntervalBufferIndex = 0;
int speedIntervalSamplesCollected = 0;
volatile unsigned long windGustsMedianFilterActivity = 0; 

const int MAX_WIND_SAMPLES = (SEND_INTERVAL_MS / GUST_SAMPLE_INTERVAL_MS); 
float windSpeedSamples[MAX_WIND_SAMPLES]; 
int windSampleCount = 0; 
float windSpeedCV = 0.0f; 

float currentBatteryVoltage = 0.0f; 

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, GMT_OFFSET_SEC, NTP_UPDATE_INTERVAL_MS);
WiFiServer telnetServer(23);
WiFiClient telnetClient;
int lastMidnightCheckDayOfMonth = 0; 
unsigned long lastSendDataTime = 0; 
unsigned long lastSendAttemptTime = 0; 

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// --- Deklarace funkcí pro výpočty a odesílání ---
void calculateWindChill(float temperatureCelsius, float windSpeedMps);
void calculateDewPoint(float temperatureCelsius, float relativeHumidityPercent);
void calculateHeatIndex(float temperatureCelsius, float relativeHumidityPercent);
void updateAllDerivedWeatherData();
bool sendDerivedDataToServer(); 
bool sendDataToWindy(); 
void updateHourlyRainBuffer(); 

// Pomocná funkce pro formátování času z NTP
const char* formatLocalTime(const struct tm* ptm, char* buffer, size_t bufferSize) {
    if (ptm == nullptr) {
        snprintf(buffer, bufferSize, "(Time N/A)");
        return buffer;
    }
    snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d:%02d",
             ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
             ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    return buffer;
}

// Funkce pro výpis na Serial a Telnet (zkráceno pro přehlednost, beze změn)
void printToAll(const char* msg) { Serial.print(msg); if (telnetClient && telnetClient.connected()) telnetClient.print(msg); }
void printlnToAll(const char* msg) { Serial.println(msg); if (telnetClient && telnetClient.connected()) telnetClient.println(msg); }
void printToAll(const __FlashStringHelper* msg) { Serial.print(msg); if (telnetClient && telnetClient.connected()) telnetClient.print(msg); }
void printlnToAll(const __FlashStringHelper* msg) { Serial.println(msg); if (telnetClient && telnetClient.connected()) telnetClient.println(msg); }
void printToAll(const String& msg) { Serial.print(msg); if (telnetClient && telnetClient.connected()) telnetClient.print(msg); }
void printlnToAll(const String& msg) { Serial.println(msg); if (telnetClient && telnetClient.connected()) telnetClient.println(msg); }
void printToAll(char msg) { Serial.print(msg); if (telnetClient && telnetClient.connected()) telnetClient.print(msg); }
void printlnToAll(char msg) { Serial.println(msg); if (telnetClient && telnetClient.connected()) telnetClient.println(msg); }
void printToAll(int msg, int base = DEC) { Serial.print(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.print(msg, base); }
void printlnToAll(int msg, int base = DEC) { Serial.println(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.println(msg, base); }
void printToAll(long msg, int base = DEC) { Serial.print(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.print(msg, base); }
void printlnToAll(long msg, int base = DEC) { Serial.println(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.println(msg, base); }
void printToAll(unsigned int msg, int base = DEC) { Serial.print(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.print(msg, base); }
void printlnToAll(unsigned int msg, int base = DEC) { Serial.println(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.println(msg, base); }
void printToAll(unsigned long msg, int base = DEC) { Serial.print(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.print(msg, base); }
void printlnToAll(unsigned long msg, int base = DEC) { Serial.println(msg, base); if (telnetClient && telnetClient.connected()) telnetClient.println(msg, base); }
void printToAll(double msg, int prec = 2) { Serial.print(msg, prec); if (telnetClient && telnetClient.connected()) telnetClient.print(msg, prec); }
void printlnToAll(double msg, int prec = 2) { Serial.println(msg, prec); if (telnetClient && telnetClient.connected()) telnetClient.println(msg, prec); }
void printToAll(float msg, int prec = 2) { printToAll((double)msg, prec); }
void printlnToAll(float msg, int prec = 2) { printlnToAll((double)msg, prec); }
void printToAll(const IPAddress& ip) { Serial.print(ip); if (telnetClient && telnetClient.connected()) telnetClient.print(ip); }
void printlnToAll(const IPAddress& ip) { Serial.println(ip); if (telnetClient && telnetClient.connected()) telnetClient.println(ip); }
void printlnToAll() { Serial.println(); if (telnetClient && telnetClient.connected()) telnetClient.println(); }
void printfToAll(const char *format, ...) {
    char buffer[256];
    va_list args_serial;
    va_start(args_serial, format);
    vsnprintf(buffer, sizeof(buffer), format, args_serial);
    va_end(args_serial);
    Serial.print(buffer);
    if (telnetClient && telnetClient.connected()) { telnetClient.print(buffer); }
}

// Funkce pro čtení napětí baterie
float readBatteryVoltage() {
    uint32_t adc_reading_sum = 0;
    for (int i = 0; i < NUM_ADC_SAMPLES; i++) {
        adc_reading_sum += analogRead(BATTERY_ADC_PIN);
        delay(1); 
    }
    float avg_adc_reading = (float)adc_reading_sum / NUM_ADC_SAMPLES;
    float batteryVoltage = avg_adc_reading * BATTERY_CALIBRATION_MULTIPLIER;
    return batteryVoltage;
}

void printDeviceStatus(); 

// Uložení měsíčního úhrnu srážek do Preferences
void saveMonthRain() {
    preferences.begin(PREFS_NAMESPACE, false);
    preferences.putFloat(PREFS_KEY_MONTH_RAIN, totalRainfallMonthMm);
    preferences.end();
    printlnToAll(F("  Monthly rainfall saved."));
}

// Uložení data posledního měsíčního resetu
void saveLastResetDate(int month, int year) {
    preferences.begin(PREFS_NAMESPACE, false);
    preferences.putInt(PREFS_KEY_LAST_RESET_MONTH, month);
    preferences.putInt(PREFS_KEY_LAST_RESET_YEAR_FOR_MONTHLY, year);
    preferences.end();
}

// Uložení ročního úhrnu srážek
void saveYearRain() {
    preferences.begin(PREFS_NAMESPACE, false);
    preferences.putFloat(PREFS_KEY_YEAR_RAIN, totalRainfallYearMm);
    preferences.end();
    printlnToAll(F("  Yearly rainfall saved."));
}

// Uložení roku pro roční akumulaci
void saveLastKnownYearForYearlyReset(int year) {
    preferences.begin(PREFS_NAMESPACE, false);
    preferences.putInt(PREFS_KEY_YEAR_FOR_ACCUMULATION, year);
    preferences.end();
}

// Výpis aktuálního stavu zařízení
void printDeviceStatus() {
    printlnToAll(F("\n--- CURRENT DEVICE STATUS ---"));
    printToAll(F("  Firmware Version:    ")); printlnToAll(FIRMWARE_VERSION);

    char timeBuffer[30];
    if (timeClient.isTimeSet()) {
        time_t epochTime = timeClient.getEpochTime();
        struct tm *ptm = localtime(&epochTime);
        printToAll(F("  Local Time:          ")); printlnToAll(formatLocalTime(ptm, timeBuffer, sizeof(timeBuffer)));
    } else {
        printlnToAll(F("  Local Time:          N/A"));
    }

    float currentDayRain, currentMonthRain, currentYearRain;
    unsigned int currentRawImpulses;
    unsigned long currentRainDebounceFiltered, currentWindDebounceFiltered, currentWindGustsFilteredRatio; 

    portENTER_CRITICAL(&mux);
    currentDayRain = totalRainfallMm;
    currentMonthRain = totalRainfallMonthMm;
    currentYearRain = totalRainfallYearMm;
    currentRawImpulses = rainCount;
    currentRainDebounceFiltered = rainDebounceFilteredCount;
    currentWindDebounceFiltered = windDebounceFilteredCount;
    currentWindGustsFilteredRatio = windGustsFilteredByRatio; 
    // Počítadlo aktivity mediánového filtru se již explicitně nezískává pro výpis
    portEXIT_CRITICAL(&mux);

    printToAll(F("  Daily Rainfall:    ")); printToAll(currentDayRain, 4); printlnToAll(F(" mm"));
    printToAll(F("  Monthly Rainfall:  ")); printToAll(currentMonthRain, 4); printlnToAll(F(" mm"));
    printToAll(F("  Yearly Rainfall:   ")); printToAll(currentYearRain, 4); printlnToAll(F(" mm"));
    printToAll(F("  Raw Day Impulses:  ")); printlnToAll(currentRawImpulses);

    printToAll(F("  Wind Speed (avg):  ")); printToAll(windSpeedAvgValue, 2); printlnToAll(F(" m/s (EMA filtered)")); 
    printToAll(F("  Max Wind Gust:     ")); printToAll(currentMaxWindGust, 2); printlnToAll(F(" m/s")); 
    printToAll(F("  Wind Speed CV:     ")); printToAll(windSpeedCV, 1); printlnToAll(F(" %")); 

    printToAll(F("  BME280 Temp (TMEP):  ")); 
    if (isnan(bme280_temperature)) printToAll(F("N/A")); else printToAll(bme280_temperature, 1); 
    printlnToAll(F(" C"));
    
    printToAll(F("  BME280 Hum (TMEP):   ")); 
    if (isnan(bme280_humidity)) printToAll(F("N/A")); else printToAll(bme280_humidity, 1); 
    printlnToAll(F(" %"));

    printToAll(F("  BME280 Press (TMEP): ")); 
    if (isnan(bme280_pressure)) printToAll(F("N/A")); else printToAll(bme280_pressure, 1); 
    printlnToAll(F(" hPa"));

    printToAll(F("  Wind Chill:          "));
    if(isnan(calculated_wind_chill)) printToAll(F("N/A")); else printToAll(calculated_wind_chill, 1);
    printlnToAll(F(" C"));

    printToAll(F("  Dew Point:           "));
    if(isnan(calculated_dew_point)) printToAll(F("N/A")); else printToAll(calculated_dew_point, 1);
    printlnToAll(F(" C"));

    printToAll(F("  Heat Index:          "));
    if(isnan(calculated_heat_index)) printToAll(F("N/A")); else printToAll(calculated_heat_index, 1);
    printlnToAll(F(" C"));


    printToAll(F("  Battery Voltage:   ")); printToAll(currentBatteryVoltage, 2); printlnToAll(F(" V"));

    if (WiFi.status() == WL_CONNECTED) {
        printToAll(F("  WiFi SSID:           ")); printlnToAll(WiFi.SSID());
        printToAll(F("  IP Address:          ")); printlnToAll(WiFi.localIP());
        printToAll(F("  WiFi RSSI:           ")); printToAll(WiFi.RSSI()); printlnToAll(F(" dBm"));
    } else {
        printlnToAll(F("  WiFi Status:         DISCONNECTED"));
    }
    unsigned long timeSinceLastSend = (millis() - lastSendDataTime) / 1000UL;
    printToAll(F("  Time Since Last OK Send: ")); printToAll(timeSinceLastSend); printlnToAll(F(" sec"));
    
    unsigned long timeSinceLastBMEFetch = (millis() - lastBME280FetchTime) / 1000UL;
    printToAll(F("  Time Since Last BME Fetch: ")); 
    if (lastBME280FetchTime == 0 && isnan(bme280_temperature)) printToAll(F("Never/failed")); 
    else if (lastBME280FetchTime == 0) printToAll(F("Scheduled")); 
    else printToAll(timeSinceLastBMEFetch);
    printlnToAll(F(" sec"));

    unsigned long timeSinceLastWindySend = (millis() - lastSuccessfulWindySendTime) / 1000UL;
    printToAll(F("  Time Since Last Windy Send: "));
    if (lastSuccessfulWindySendTime == 0) printToAll(F("Never")); else printToAll(timeSinceLastWindySend);
    printlnToAll(F(" sec"));


    printToAll(F("  Midnight Check Day:  ")); printlnToAll(lastMidnightCheckDayOfMonth);
    printToAll(F("  Last Month Reset:    M=")); printToAll(lastResetMonth != -1 ? String(lastResetMonth + 1) : F("N/A")); printToAll(F("/Y=")); printlnToAll(lastResetYearForMonthly == -1 ? F("N/A") : String(lastResetYearForMonthly));
    printToAll(F("  Last Year Reset for: ")); printlnToAll(lastKnownYearForYearlyReset == -1 ? F("N/A") : String(lastKnownYearForYearlyReset));
    
    printToAll(F("  Rain Debounce:       ")); printToAll(debounceDelayMs); printlnToAll(F(" ms"));
    printToAll(F("  Wind Debounce:       ")); printToAll(windPulseDebounceUs); printlnToAll(F(" us"));
    printToAll(F("  Rain Debounce Filtered: ")); printlnToAll(currentRainDebounceFiltered);
    printToAll(F("  Wind Debounce Filtered: ")); printlnToAll(currentWindDebounceFiltered);
    // Počítadlo aktivity mediánového filtru se již nezobrazuje
    printToAll(F("  Wind Gusts Ratio Filtered: ")); printlnToAll(currentWindGustsFilteredRatio); 
    printToAll(F("  Wind Avg EMA Alpha:  ")); printToAll(windAvgEmaAlpha, 2); printlnToAll(); 
    printToAll(F("  Gust Filter Max Ratio: ")); printToAll(maxGustToAvgWindRatio, 1); printlnToAll();
    printToAll(F("  Gust Filter Min Avg Wind: ")); printToAll(minAvgWindForRatioCheckMps, 1); printlnToAll(F(" m/s"));


    printToAll(F("  Free Heap:           ")); printToAll(ESP.getFreeHeap()); printlnToAll(F(" B"));
    printToAll(F("  Min Free Heap Ever:  ")); printToAll(ESP.getMinFreeHeap()); printlnToAll(F(" B"));
    printlnToAll(F("-----------------------------"));
}

// Odeslání dat ze srážkoměru na TMEP server (beze změn)
bool sendRainDataToServer() {
    printlnToAll(F("\n--- [ACTION] SENDING RAIN DATA ---"));
    bool currentSendSuccess = false;
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client; 
        char urlBuffer[320]; 
        long rssi = WiFi.RSSI();
        float dayRain, monthRain, yearRain;
        float vbatSnapshot = currentBatteryVoltage; 
        portENTER_CRITICAL(&mux);
        dayRain = totalRainfallMm;
        monthRain = totalRainfallMonthMm;
        yearRain = totalRainfallYearMm;
        portEXIT_CRITICAL(&mux);
        int urlPathLength = snprintf(urlBuffer, sizeof(urlBuffer), "%s?%s=%.4f&%s=%.4f&%s=%.4f&%s=%ld&%s=%.2f",
                                     RAIN_SERVER_PATH, RAIN_DATA_PARAM_NAME, dayRain,
                                     RAIN_MONTH_PARAM_NAME, monthRain, RAIN_YEAR_PARAM_NAME, yearRain,
                                     RSSI_PARAM_NAME, rssi, VOLTAGE_PARAM_NAME, vbatSnapshot);
        if (urlPathLength < 0 || urlPathLength >= sizeof(urlBuffer)) {
            printlnToAll(F("  ERR: Rain URL too long!"));
            printlnToAll(F("--- END RAIN SEND (URL ERR) ---"));
            return false;
        } else {
            printToAll(F("  To Rain Server: ")); printlnToAll(RAIN_SERVER_ADDRESS);
            printToAll(F("  Rain URL Path: ")); printlnToAll(urlBuffer); 
            int retries = 0;
            while (retries < MAX_RETRIES && !currentSendSuccess) {
                printToAll(F("    Rain Send Attempt ")); printToAll(retries + 1); printlnToAll(F("..."));
                if (client.connect(RAIN_SERVER_ADDRESS, TMEP_SERVER_PORT)) {
                    printlnToAll(F("      Connected (Rain)."));
                    client.print(F("GET ")); client.print(urlBuffer); client.print(F(" HTTP/1.1\r\n"));
                    client.print(F("Host: ")); client.print(RAIN_SERVER_ADDRESS); client.print(F("\r\n"));
                    client.print(F("Connection: close\r\n\r\n"));
                    unsigned long responseStartTime = millis();
                    String httpStatusLine = ""; bool headersRead = false; bool emptyLineAfterHeaders = false;
                    while (client.connected() || client.available()) {
                        if (client.available()) {
                            String sLine = client.readStringUntil('\n');
                            if (!headersRead && sLine.startsWith(F("HTTP/"))) { 
                                httpStatusLine = sLine; httpStatusLine.trim(); headersRead = true;
                                printToAll(F("        HTTP Status (Rain): ")); printlnToAll(httpStatusLine);
                            }
                            if (telnetClient && telnetClient.connected() && headersRead && !httpStatusLine.isEmpty() && sLine != httpStatusLine) {
                                telnetClient.println(sLine);
                            }
                            if (sLine.length() <= 1 && headersRead) { emptyLineAfterHeaders = true; } 
                            if(emptyLineAfterHeaders && headersRead) break; 
                        }
                        if (millis() - responseStartTime > 7000) { printlnToAll(F("        Timeout (Rain).")); if(httpStatusLine.isEmpty()) httpStatusLine = F("Timeout"); break; } 
                        delay(10);
                    }
                    while(client.available()) client.read(); 
                    if (httpStatusLine.startsWith(F("HTTP/1.1 200 OK")) || httpStatusLine.startsWith(F("HTTP/1.0 200 OK"))) {
                        printlnToAll(F("      Rain data OK.")); currentSendSuccess = true;
                    } else {
                        printToAll(F("      Rain server err. Status: ")); printlnToAll(httpStatusLine.isEmpty() ? F("Unknown") : httpStatusLine);
                    }
                    client.stop();
                } else {
                    printlnToAll(F("      Rain server conn FAILED."));
                    retries++;
                    if (retries < MAX_RETRIES && !currentSendSuccess) { printlnToAll(F("      Retrying rain send 5s...")); delay(5000); }
                }
            }
        }
    } else {
        printlnToAll(F("  Skipped Rain Send: No WiFi."));
    }
    printlnToAll(F("--- END RAIN DATA SEND ---"));
    return currentSendSuccess;
}

// Odeslání dat z větroměru na TMEP server
bool sendWindDataToServer() {
    printlnToAll(F("\n--- [ACTION] SENDING WIND DATA ---"));
    bool currentSendSuccess = false;

    unsigned long currentTotalPulses_avg; 
    portENTER_CRITICAL(&mux);
    currentTotalPulses_avg = windPulseCount;
    portEXIT_CRITICAL(&mux);

    unsigned long pulsesForAvg = currentTotalPulses_avg - lastWindPulseSnapshotFor10minAvg; 
    float intervalSec = (float)SEND_INTERVAL_MS / 1000.0f;
    float current_raw_avg; 

    if (intervalSec > 0.0001f) { 
        current_raw_avg = ( (float)pulsesForAvg / intervalSec ) * WIND_MPS_PER_PULSE_PER_SEC;
    } else {
        current_raw_avg = 0.0f;
    }

    // Aplikace EMA filtru na windSpeedAvgValue
    if (isnan(windSpeedAvgValue)) { 
        windSpeedAvgValue = current_raw_avg;
    } else {
        windSpeedAvgValue = (windAvgEmaAlpha * current_raw_avg) + ((1.0f - windAvgEmaAlpha) * windSpeedAvgValue);
    }
    if (fabs(windSpeedAvgValue) < 0.001f) windSpeedAvgValue = 0.0f; 

    lastWindPulseSnapshotFor10minAvg = currentTotalPulses_avg; 

    if (windSampleCount >= 2) { 
        float sum = 0.0f;
        for (int i = 0; i < windSampleCount; i++) {
            sum += windSpeedSamples[i];
        }
        float mean = sum / windSampleCount;

        float sumSqDiff = 0.0f;
        for (int i = 0; i < windSampleCount; i++) {
            sumSqDiff += pow(windSpeedSamples[i] - mean, 2);
        }
        float stdDev = sqrt(sumSqDiff / windSampleCount);

        if (mean > 0.001f) { 
            windSpeedCV = (stdDev / mean) * 100.0f;
        } else {
            windSpeedCV = 0.0f;
        }
    } else {
        windSpeedCV = 0.0f; 
    }

    printToAll(F("  Raw Avg Wind (before EMA): ")); printToAll(current_raw_avg, 2); printlnToAll(F(" m/s"));
    printToAll(F("  EMA Avg Wind (")); printToAll(intervalSec, 1); printToAll(F("s): "));
    printToAll(windSpeedAvgValue, 2); printlnToAll(F(" m/s"));
    printToAll(F("  Max Gust:      ")); printToAll(currentMaxWindGust, 2); printlnToAll(F(" m/s"));
    printToAll(F("  Wind CV:       ")); printToAll(windSpeedCV, 1); printlnToAll(F(" %"));

    updateAllDerivedWeatherData(); 

    float vbatSnapshot = currentBatteryVoltage;
    bool urlFormattingError = false;

    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client; 
        char urlBuffer[380]; 
        long rssi = WiFi.RSSI();

        int urlPathLength = snprintf(urlBuffer, sizeof(urlBuffer), "%s?%s=%.2f&%s=%.2f&%s=%.1f&%s=%ld&%s=%.2f",
                                     WIND_SERVER_PATH,
                                     WIND_SPEED_PARAM_NAME, windSpeedAvgValue, 
                                     WIND_GUST_PARAM_NAME, currentMaxWindGust, 
                                     WIND_CV_PARAM_NAME, windSpeedCV,
                                     RSSI_PARAM_NAME, rssi,
                                     VOLTAGE_PARAM_NAME, vbatSnapshot);

        if (urlPathLength < 0 || urlPathLength >= sizeof(urlBuffer)) {
            printlnToAll(F("  ERR: Wind URL too long!"));
            urlFormattingError = true;
        } else {
            printToAll(F("  To Wind Server: ")); printlnToAll(WIND_SERVER_ADDRESS);
            printToAll(F("  Wind URL Path: ")); printlnToAll(urlBuffer);

            int retries = 0;
            while (retries < MAX_RETRIES && !currentSendSuccess) {
                 printToAll(F("    Wind Send Attempt ")); printToAll(retries + 1); printlnToAll(F("..."));
                if (client.connect(WIND_SERVER_ADDRESS, TMEP_SERVER_PORT)) {
                    printlnToAll(F("      Connected (Wind)."));
                    client.print(F("GET ")); client.print(urlBuffer); client.print(F(" HTTP/1.1\r\n"));
                    client.print(F("Host: ")); client.print(WIND_SERVER_ADDRESS); client.print(F("\r\n"));
                    client.print(F("Connection: close\r\n\r\n"));
                    
                    unsigned long responseStartTime = millis();
                    String httpStatusLine = ""; bool headersRead = false; bool emptyLineAfterHeaders = false;

                    while (client.connected() || client.available()) {
                        if (client.available()) {
                            String sLine = client.readStringUntil('\n');
                            if (!headersRead && sLine.startsWith(F("HTTP/"))) {
                                httpStatusLine = sLine; httpStatusLine.trim(); headersRead = true;
                                printToAll(F("        HTTP Status (Wind): ")); printlnToAll(httpStatusLine);
                            }
                            if (telnetClient && telnetClient.connected() && headersRead && !httpStatusLine.isEmpty() && sLine != httpStatusLine) {
                                telnetClient.println(sLine);
                            }
                            if (sLine.length() <= 1 && headersRead) { emptyLineAfterHeaders = true; }
                            if(emptyLineAfterHeaders && headersRead) break;
                        }
                        if (millis() - responseStartTime > 7000) { printlnToAll(F("        Timeout (Wind).")); if(httpStatusLine.isEmpty()) httpStatusLine = F("Timeout"); break; }
                        delay(10);
                    }
                    while(client.available()) client.read();

                    if (httpStatusLine.startsWith(F("HTTP/1.1 200 OK")) || httpStatusLine.startsWith(F("HTTP/1.0 200 OK"))) {
                        printlnToAll(F("      Wind data OK.")); currentSendSuccess = true;
                    } else {
                         printToAll(F("      Wind server err. Status: ")); printlnToAll(httpStatusLine.isEmpty() ? F("Unknown") : httpStatusLine);
                    }
                    client.stop();
                } else {
                    printlnToAll(F("      Wind server conn FAILED."));
                    retries++;
                    if (retries < MAX_RETRIES && !currentSendSuccess) { printlnToAll(F("      Retrying wind send 5s...")); delay(5000); }
                }
            }
        }
    } else {
        printlnToAll(F("  Skipped Wind Send: No WiFi."));
    }

    bool wifiWasConnectedDuringAttempt = (WiFi.status() == WL_CONNECTED && !urlFormattingError);

    currentMaxWindGust = 0.0f; 
    windSampleCount = 0; 

    if (urlFormattingError) {
        printlnToAll(F("  Gust/CV reset (URL err)."));
    } else if (!currentSendSuccess && wifiWasConnectedDuringAttempt) {
         printlnToAll(F("  Gust/CV reset (send fail)."));
    } else if (WiFi.status() != WL_CONNECTED) {
        printlnToAll(F("  Gust/CV reset (no WiFi)."));
    } else if (currentSendSuccess) {
        printlnToAll(F("  Gust/CV reset (next interval)."));
    }

    printlnToAll(F("--- END WIND DATA SEND ---"));
    return currentSendSuccess;
}

// Odeslání odvozených dat (beze změn)
bool sendDerivedDataToServer() {
    printlnToAll(F("\n--- [ACTION] SENDING DERIVED DATA (TMEP) ---"));
    bool currentSendSuccess = false;
    if (isnan(calculated_wind_chill) && isnan(calculated_dew_point) && isnan(calculated_heat_index)) {
        printlnToAll(F("  Skipped Derived Send: All NAN."));
        return false; 
    }
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client; 
        char urlBuffer[450]; 
        long rssi = WiFi.RSSI();
        float vbatSnapshot = currentBatteryVoltage;
        String params = "";
        if (!isnan(calculated_wind_chill)) { params += String(WINDCHILL_PARAM_NAME) + "=" + String(calculated_wind_chill, 1) + "&"; }
        if (!isnan(calculated_dew_point)) { params += String(DEWPOINT_PARAM_NAME) + "=" + String(calculated_dew_point, 1) + "&"; }
        if (!isnan(calculated_heat_index)) { params += String(HEATINDEX_PARAM_NAME) + "=" + String(calculated_heat_index, 1) + "&"; }
        params += String(RSSI_PARAM_NAME) + "=" + String(rssi) + "&";
        params += String(VOLTAGE_PARAM_NAME) + "=" + String(vbatSnapshot, 2);
        int urlPathLength = snprintf(urlBuffer, sizeof(urlBuffer), "%s?%s", DERIVED_DATA_SERVER_PATH, params.c_str());
        if (urlPathLength < 0 || urlPathLength >= sizeof(urlBuffer)) {
            printlnToAll(F("  ERR: Derived URL too long!"));
            printlnToAll(F("--- END DERIVED SEND (URL ERR) ---"));
            return false;
        } else {
            printToAll(F("  To Derived Server: ")); printlnToAll(DERIVED_DATA_SERVER_ADDRESS);
            printToAll(F("  Derived URL Path: ")); printlnToAll(urlBuffer);
            int retries = 0;
            while (retries < MAX_RETRIES && !currentSendSuccess) {
                printToAll(F("    Derived Send Attempt ")); printToAll(retries + 1); printlnToAll(F("..."));
                if (client.connect(DERIVED_DATA_SERVER_ADDRESS, TMEP_SERVER_PORT)) {
                    printlnToAll(F("      Connected (Derived)."));
                    client.print(F("GET ")); client.print(urlBuffer); client.print(F(" HTTP/1.1\r\n"));
                    client.print(F("Host: ")); client.print(DERIVED_DATA_SERVER_ADDRESS); client.print(F("\r\n"));
                    client.print(F("Connection: close\r\n\r\n"));
                    unsigned long responseStartTime = millis();
                    String httpStatusLine = ""; bool headersRead = false; bool emptyLineAfterHeaders = false;
                    while (client.connected() || client.available()) {
                        if (client.available()) {
                            String sLine = client.readStringUntil('\n');
                            if (!headersRead && sLine.startsWith(F("HTTP/"))) {
                                httpStatusLine = sLine; httpStatusLine.trim(); headersRead = true;
                                printToAll(F("        HTTP Status (Derived): ")); printlnToAll(httpStatusLine);
                            }
                            if (telnetClient && telnetClient.connected() && headersRead && !httpStatusLine.isEmpty() && sLine != httpStatusLine) {
                                telnetClient.println(sLine);
                            }
                             if (sLine.length() <= 1 && headersRead) { emptyLineAfterHeaders = true; }
                            if(emptyLineAfterHeaders && headersRead) break;
                        }
                        if (millis() - responseStartTime > 7000) { printlnToAll(F("        Timeout (Derived).")); if(httpStatusLine.isEmpty()) httpStatusLine = F("Timeout"); break; }
                        delay(10);
                    }
                    while(client.available()) client.read(); 
                    if (httpStatusLine.startsWith(F("HTTP/1.1 200 OK")) || httpStatusLine.startsWith(F("HTTP/1.0 200 OK"))) {
                        printlnToAll(F("      Derived data OK.")); currentSendSuccess = true;
                    } else {
                        printToAll(F("      Derived server err. Status: ")); printlnToAll(httpStatusLine.isEmpty() ? F("Unknown") : httpStatusLine);
                    }
                    client.stop();
                } else {
                    printlnToAll(F("      Derived server conn FAILED."));
                    retries++;
                    if (retries < MAX_RETRIES && !currentSendSuccess) { printlnToAll(F("      Retrying derived send 5s...")); delay(5000); }
                }
            }
        }
    } else {
        printlnToAll(F("  Skipped Derived Send: No WiFi."));
    }
    printlnToAll(F("--- END DERIVED DATA SEND (TMEP) ---"));
    return currentSendSuccess;
}

// Odeslání dat na Windy.com (beze změn)
bool sendDataToWindy() {
    printlnToAll(F("\n--- [ACTION] SENDING DATA TO WINDY.COM ---"));
    bool success = false;
    if (WiFi.status() != WL_CONNECTED) {
        printlnToAll(F("  Skipped Windy Send: No WiFi."));
        return false;
    }
    if (isnan(bme280_temperature) || isnan(bme280_humidity) || isnan(bme280_pressure) || 
        isnan(windSpeedAvgValue) || isnan(currentMaxWindGust) || isnan(calculated_dew_point)) { 
        printlnToAll(F("  Skipped Windy Send: Some required data is NAN."));
        printToAll(F("    Temp: ")); printToAll(bme280_temperature);
        printToAll(F(", Hum: ")); printToAll(bme280_humidity);
        printToAll(F(", Pres: ")); printToAll(bme280_pressure);
        printToAll(F(", Wind: ")); printToAll(windSpeedAvgValue); 
        printToAll(F(", Gust: ")); printToAll(currentMaxWindGust); 
        printToAll(F(", DewP: ")); printlnToAll(calculated_dew_point);
        return false;
    }
    float precipForWindyToSend = 0.0f; 
    for (int i = 0; i < HOURLY_RAIN_BUFFER_SIZE; i++) { precipForWindyToSend += hourlyRainMinuteBuffer[i]; }
    if (precipForWindyToSend < 0) precipForWindyToSend = 0.0f; 
    HTTPClient http;
    WiFiClientSecure clientSecure; 
    clientSecure.setInsecure(); 
    String urlPath = String(WINDY_SERVER_PATH_PREFIX) + String(WINDY_API_KEY);
    String queryParams = "?";
    queryParams += String(WINDY_PARAM_TIMESTAMP) + "=" + String(timeClient.getEpochTime());
    queryParams += "&stationId=0"; 
    queryParams += "&" + String(WINDY_PARAM_TEMP) + "=" + String(bme280_temperature, 1);
    queryParams += "&" + String(WINDY_PARAM_RH) + "=" + String(bme280_humidity, 1);
    queryParams += "&" + String(WINDY_PARAM_MBAR) + "=" + String(bme280_pressure, 1); 
    queryParams += "&" + String(WINDY_PARAM_WIND) + "=" + String(windSpeedAvgValue, 2); 
    queryParams += "&" + String(WINDY_PARAM_GUST) + "=" + String(currentMaxWindGust, 2); 
    queryParams += "&" + String(WINDY_PARAM_DEWPOINT) + "=" + String(calculated_dew_point, 1);
    queryParams += "&" + String(WINDY_PARAM_PRECIP) + "=" + String(precipForWindyToSend, 2); 
    printToAll(F("  To Windy Server: ")); printlnToAll(WINDY_SERVER_ADDRESS);
    printToAll(F("  Windy Full URL: https://")); printToAll(WINDY_SERVER_ADDRESS); printToAll(urlPath); printlnToAll(queryParams);
    if (http.begin(clientSecure, WINDY_SERVER_ADDRESS, WINDY_SERVER_PORT, urlPath + queryParams)) {
        http.setTimeout(10000); 
        int httpCode = http.GET();
        if (httpCode > 0) {
            String payload = http.getString();
            printToAll(F("    Windy Response Code: ")); printlnToAll(httpCode);
            printToAll(F("    Windy Response: ")); printlnToAll(payload); 
            if (httpCode == HTTP_CODE_OK) {
                StaticJsonDocument<512> doc; 
                DeserializationError error = deserializeJson(doc, payload);
                bool windyAcceptedByJson = false;
                if (!error) {
                    if (doc.containsKey(F("result")) && 
                        doc[F("result")].is<JsonObject>() &&
                        doc[F("result")].containsKey(F("0")) && 
                        doc[F("result")][F("0")].is<JsonObject>() &&
                        doc[F("result")][F("0")].containsKey(F("observations")) && 
                        doc[F("result")][F("0")][F("observations")].is<JsonArray>() &&
                        doc[F("result")][F("0")][F("observations")].as<JsonArray>().size() > 0 && 
                        doc[F("result")][F("0")][F("observations")][0].is<JsonObject>() &&
                        doc[F("result")][F("0")][F("observations")][0].containsKey(F("success")) &&
                        doc[F("result")][F("0")][F("observations")][0][F("success")].as<bool>() == true) {
                        windyAcceptedByJson = true;
                    }
                }
                if (windyAcceptedByJson) {       
                    printlnToAll(F("      Windy data accepted (JSON success:true)."));
                    success = true;
                } else if (payload.startsWith(F("SUCCESS"))) { 
                    printlnToAll(F("      Windy data accepted (Plain text SUCCESS)."));
                    success = true;
                }
                else {
                    printlnToAll(F("      Windy: HTTP 200 OK, but response does not indicate clear success."));
                }
                if (success) { lastSuccessfulWindySendTime = millis(); }
            } else {
                printToAll(F("      Windy server error. Code: ")); printToAll(httpCode); printToAll(F(", Payload: ")); printlnToAll(payload);
            }
        } else {
            printfToAll("    Windy HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
    } else {
        printlnToAll(F("    Windy HTTPClient.begin() failed."));
    }
    printlnToAll(F("--- END WINDY.COM DATA SEND ---"));
    return success;
}

// ISR (beze změn)
void IRAM_ATTR rainDetected() {
    unsigned long currentTime_ms = millis();
    if ((currentTime_ms - lastRainTime_ms) > debounceDelayMs) { 
        portENTER_CRITICAL_ISR(&mux);
        rainCount++; 
        currentMinuteRainPulses++; 
        totalRainfallMm += RAINFALL_PER_IMPULSE_MM;
        totalRainfallMonthMm += RAINFALL_PER_IMPULSE_MM;
        totalRainfallYearMm += RAINFALL_PER_IMPULSE_MM;
        lastRainTime_ms = currentTime_ms;
        portEXIT_CRITICAL_ISR(&mux);
    } else { 
        portENTER_CRITICAL_ISR(&mux);
        rainDebounceFilteredCount++;
        portEXIT_CRITICAL_ISR(&mux);
    }
}
void IRAM_ATTR windPulseDetected() {
    unsigned long currentTime_us = micros(); 
    if (((currentTime_us - lastWindPulseTime_us) > windPulseDebounceUs) || (lastWindPulseTime_us == 0)) { 
        portENTER_CRITICAL_ISR(&mux);
        windPulseCount++;
        lastWindPulseTime_us = currentTime_us;
        portEXIT_CRITICAL_ISR(&mux);
    } else { 
        portENTER_CRITICAL_ISR(&mux);
        windDebounceFilteredCount++;
        portEXIT_CRITICAL_ISR(&mux);
    }
}

// fetchBME280Data (beze změn)
void fetchBME280Data() {
    if (WiFi.status() != WL_CONNECTED) {
        printlnToAll(F("[BME FETCH] No WiFi."));
        bme280_temperature = NAN; bme280_humidity = NAN; bme280_pressure = NAN;
        updateAllDerivedWeatherData(); 
        return;
    }
    printlnToAll(F("\n--- [ACTION] FETCHING BME280 (TMEP) ---"));
    HTTPClient http;
    WiFiClientSecure clientSecure; 
    clientSecure.setInsecure(); 
    printToAll(F("  [BME FETCH] Connect: ")); printlnToAll(TMEP_JSON_URL);
    if (http.begin(clientSecure, TMEP_JSON_URL)) {
        printlnToAll(F("  [BME FETCH] HTTPClient.begin() OK."));
        http.setTimeout(15000); 
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); 
        int httpCode = http.GET();
        printToAll(F("  [BME FETCH] HTTP GET. Code: ")); printlnToAll(httpCode);
        if (httpCode > 0) {
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
                String payload = http.getString();
                StaticJsonDocument<2048> doc; 
                DeserializationError error = deserializeJson(doc, payload);
                if (error) {
                    printToAll(F("  [BME FETCH] JSON parse err: "));
                    printlnToAll(error.c_str());
                    bme280_temperature = NAN; bme280_humidity = NAN; bme280_pressure = NAN;
                } else {
                    printlnToAll(F("  [BME FETCH] JSON parse OK. Parsing values..."));
                    if (doc.containsKey(F("teplota")) && doc.containsKey(F("vlhkost")) && doc.containsKey(F("tlak"))) {
                        bme280_temperature = doc[F("teplota")].as<float>();
                        bme280_humidity = doc[F("vlhkost")].as<float>();
                        bme280_pressure = doc[F("tlak")].as<float>();
                        printlnToAll(F("  [BME FETCH] Data parsed OK."));
                        printToAll(F("    Temp: ")); if(isnan(bme280_temperature)) printToAll(F("N/A")); else printToAll(bme280_temperature, 1); printlnToAll(F(" C"));
                        printToAll(F("    Hum:  ")); if(isnan(bme280_humidity)) printToAll(F("N/A")); else printToAll(bme280_humidity, 1); printlnToAll(F(" %"));
                        printToAll(F("    Pres: ")); if(isnan(bme280_pressure)) printToAll(F("N/A")); else printToAll(bme280_pressure, 1); printlnToAll(F(" hPa"));
                    } else {
                        printlnToAll(F("  [BME FETCH] JSON err: Missing keys (teplota, vlhkost, tlak)."));
                        bme280_temperature = NAN; bme280_humidity = NAN; bme280_pressure = NAN;
                    }
                }
            } else {
                printfToAll("  [BME FETCH] HTTP GET fail, code: %d (%s)\n", httpCode, http.errorToString(httpCode).c_str());
                bme280_temperature = NAN; bme280_humidity = NAN; bme280_pressure = NAN;
            }
        } else {
            printfToAll("  [BME FETCH] HTTP GET fail, client err: %s\n", http.errorToString(httpCode).c_str());
            bme280_temperature = NAN; bme280_humidity = NAN; bme280_pressure = NAN;
        }
    } else {
        printlnToAll(F("  [BME FETCH] HTTPClient.begin() FAILED."));
        bme280_temperature = NAN; bme280_humidity = NAN; bme280_pressure = NAN;
    }
    http.end(); 
    lastBME280FetchTime = millis();
    updateAllDerivedWeatherData(); 
    printlnToAll(F("--- END BME280 FETCH ---"));
}

// Výpočtové funkce (beze změn)
void calculateWindChill(float temperatureCelsius, float windSpeedMps) {
    if (isnan(temperatureCelsius) || isnan(windSpeedMps)) { calculated_wind_chill = NAN; return; }
    float windSpeedKmh = windSpeedMps * 3.6f;
    if (temperatureCelsius <= 10.0f && windSpeedKmh >= 4.8f) {
        calculated_wind_chill = 13.12f + 0.6215f * temperatureCelsius - 11.37f * pow(windSpeedKmh, 0.16f) + 0.3965f * temperatureCelsius * pow(windSpeedKmh, 0.16f);
    } else { calculated_wind_chill = temperatureCelsius; }
}
void calculateDewPoint(float temperatureCelsius, float relativeHumidityPercent) {
    if (isnan(temperatureCelsius) || isnan(relativeHumidityPercent)) { calculated_dew_point = NAN; return; }
    const float a = 17.27f; const float b = 237.7f; 
    float gamma = (a * temperatureCelsius) / (b + temperatureCelsius) + log(relativeHumidityPercent / 100.0f);
    calculated_dew_point = (b * gamma) / (a - gamma);
}
void calculateHeatIndex(float temperatureCelsius, float relativeHumidityPercent) {
    if (isnan(temperatureCelsius) || isnan(relativeHumidityPercent)) { calculated_heat_index = NAN; return; }
    float T_F = (temperatureCelsius * 9.0f / 5.0f) + 32.0f; float RH = relativeHumidityPercent;
    if (T_F >= 80.0f && RH >= 40.0f) {
        float HI_F = -42.379f + 2.04901523f * T_F + 10.14333127f * RH - 0.22475541f * T_F * RH - 0.00683783f * T_F * T_F - 0.05481717f * RH * RH + 0.00122874f * T_F * T_F * RH + 0.00085282f * T_F * RH * RH - 0.00000199f * T_F * T_F * RH * RH;
        if (RH < 13.0f && T_F >= 80.0f && T_F <= 112.0f) { float adjustment = ((13.0f - RH) / 4.0f) * sqrt((17.0f - abs(T_F - 95.0f)) / 17.0f); HI_F -= adjustment; } 
        else if (RH > 85.0f && T_F >= 80.0f && T_F <= 87.0f) { float adjustment = ((RH - 85.0f) / 10.0f) * ((87.0f - T_F) / 5.0f); HI_F += adjustment; }
        if (HI_F < T_F) { HI_F = T_F; }
        calculated_heat_index = (HI_F - 32.0f) * 5.0f / 9.0f; 
    } else { calculated_heat_index = temperatureCelsius; }
}
void updateAllDerivedWeatherData() {
    printlnToAll(F("  [DERIVED] Updating..."));
    if (!isnan(bme280_temperature) && !isnan(bme280_humidity)) {
        calculateDewPoint(bme280_temperature, bme280_humidity);
        calculateHeatIndex(bme280_temperature, bme280_humidity);
    } else { calculated_dew_point = NAN; calculated_heat_index = NAN; }
    if (!isnan(bme280_temperature) && !isnan(windSpeedAvgValue)) { 
        calculateWindChill(bme280_temperature, windSpeedAvgValue); 
    } else { calculated_wind_chill = NAN; }
    printToAll(F("    Wind Chill: ")); if(isnan(calculated_wind_chill)) printToAll(F("N/A")); else printToAll(calculated_wind_chill, 1); printlnToAll(F(" C"));
    printToAll(F("    Dew Point:  ")); if(isnan(calculated_dew_point)) printToAll(F("N/A")); else printToAll(calculated_dew_point, 1); printlnToAll(F(" C"));
    printToAll(F("    Heat Index: ")); if(isnan(calculated_heat_index)) printToAll(F("N/A")); else printToAll(calculated_heat_index, 1); printlnToAll(F(" C"));
}

// updateHourlyRainBuffer (beze změn)
void updateHourlyRainBuffer() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastMinuteTimestampForHourlyRain >= 60000UL) { 
        unsigned long elapsedMinutes = (currentMillis - lastMinuteTimestampForHourlyRain) / 60000UL;
        portENTER_CRITICAL(&mux);
        float rainThisPassedMinute = currentMinuteRainPulses * RAINFALL_PER_IMPULSE_MM;
        currentMinuteRainPulses = 0; 
        portEXIT_CRITICAL(&mux);
        for (unsigned long m = 0; m < elapsedMinutes; m++) {
            hourlyRainMinuteBuffer[hourlyRainBufferIndex] = (m == 0) ? rainThisPassedMinute : 0.0f; 
            hourlyRainBufferIndex = (hourlyRainBufferIndex + 1) % HOURLY_RAIN_BUFFER_SIZE; 
        }
        lastMinuteTimestampForHourlyRain += elapsedMinutes * 60000UL; 
    }
}


// Obsluha Telnet spojení a příkazů
void handleTelnet() {
    if (telnetServer.hasClient()) {
        if (telnetClient && telnetClient.connected()) {
            WiFiClient newClient = telnetServer.available();
            newClient.println(F("Server busy. Try later."));
            newClient.stop();
        } else {
            telnetClient = telnetServer.available();
            if (telnetClient) {
                telnetClient.setNoDelay(true); 
                Serial.println(F("[TELNET] Client connected."));
                telnetClient.println(F("Welcome - ESP32 Rain & Wind Gauge"));
                telnetClient.print(F("Firmware: ")); telnetClient.println(FIRMWARE_VERSION);
                telnetClient.println(F("Type 'help' for commands."));
                currentBatteryVoltage = readBatteryVoltage(); 
                printDeviceStatus(); 
                telnetClient.print(F("> "));
            }
        }
    }

    if (telnetClient && telnetClient.connected()) {
        if (telnetClient.available()) {
            String commandLine = telnetClient.readStringUntil('\n');
            commandLine.trim(); 
            Serial.print(F("[TELNET] Cmd: '")); Serial.print(commandLine); Serial.println(F("'"));

            String command;
            String arg;
            int spaceIndex = commandLine.indexOf(' ');
            if (spaceIndex != -1) {
                command = commandLine.substring(0, spaceIndex);
                arg = commandLine.substring(spaceIndex + 1);
                arg.trim();
            } else {
                command = commandLine;
            }


            if (command.equalsIgnoreCase(F("help"))) {
                telnetClient.println(F("--- Available commands ---"));
                telnetClient.println(F("  help                 - Show this help"));
                telnetClient.println(F("  status               - Show status"));
                telnetClient.println(F("  uptime               - Show uptime"));
                telnetClient.println(F("  heap                 - Show free heap"));
                telnetClient.println(F("  senddata             - Manual data send (all servers)"));
                telnetClient.println(F("  sendwindy            - Manual data send to Windy.com")); 
                telnetClient.println(F("  fetchbme             - Manual BME280 fetch"));
                telnetClient.println(F("  resetalldata         - Reset all data"));
                telnetClient.println(F("  resetdebouncecounts  - Reset debounce & gust filter counts"));
                telnetClient.println(F("  setrainday <mm>      - Manually set today's total rainfall")); 
                telnetClient.println(F("  setrainmonth <val>   - Set monthly rain"));
                telnetClient.println(F("  setrainyear <val>    - Set yearly rain"));
                telnetClient.println(F("  getconfig            - Show debounce & gust filter config")); 
                telnetClient.println(F("  setraindebounce <ms> - Set rain debounce"));
                telnetClient.println(F("  setwinddebounce <us> - Set wind debounce"));
                telnetClient.println(F("  setgustratio <ratio> - Set max gust/avg wind ratio (e.g., 3.0)")); 
                telnetClient.println(F("  setmingustavg <mps>  - Set min avg wind for gust ratio check (e.g., 1.0)")); 
                telnetClient.println(F("  setwindavgalpha <alpha> - Set EMA alpha for wind avg (0.0-1.0)")); 
                telnetClient.println(F("  reboot               - Reboot device"));
                telnetClient.println(F("  quit / exit          - Disconnect Telnet"));
                telnetClient.println(F("--------------------------"));
            } else if (command.equalsIgnoreCase(F("status"))) {
                currentBatteryVoltage = readBatteryVoltage();
                printDeviceStatus();
            } else if (command.equalsIgnoreCase(F("uptime"))) {
                unsigned long now_ms = millis();
                int days = now_ms / (24UL * 60 * 60 * 1000); now_ms %= (24UL * 60 * 60 * 1000);
                int hours = now_ms / (60UL * 60 * 1000); now_ms %= (60UL * 60 * 1000);
                int minutes = now_ms / (60UL * 1000); now_ms %= (60UL * 1000);
                int seconds = now_ms / 1000UL;
                telnetClient.print(F("Uptime: "));
                telnetClient.print(days); telnetClient.print(F("d "));
                telnetClient.print(hours); telnetClient.print(F("h "));
                telnetClient.print(minutes); telnetClient.print(F("m "));
                telnetClient.print(seconds); telnetClient.println(F("s"));
            } else if (command.equalsIgnoreCase(F("heap"))) {
                telnetClient.print(F("Free Heap: ")); telnetClient.print(ESP.getFreeHeap()); telnetClient.println(F(" B"));
                telnetClient.print(F("Min Free Heap: ")); telnetClient.print(ESP.getMinFreeHeap()); telnetClient.println(F(" B"));
            } else if (command.equalsIgnoreCase(F("senddata"))) {
                telnetClient.println(F("Triggering manual data send (all)..."));
                currentBatteryVoltage = readBatteryVoltage();
                bool rainOK = sendRainDataToServer();
                bool windyOK = sendDataToWindy(); 
                bool windOK = sendWindDataToServer(); 
                bool derivedOK = false;
                if (!isnan(calculated_wind_chill) || !isnan(calculated_dew_point) || !isnan(calculated_heat_index)) { 
                    derivedOK = sendDerivedDataToServer();
                }
                if (rainOK || windOK || derivedOK || windyOK) { 
                    lastSendDataTime = millis(); 
                }
                telnetClient.println(F("--- Manual Data Send (all) Done ---"));
            } else if (command.equalsIgnoreCase(F("sendwindy"))) { 
                telnetClient.println(F("Triggering manual data send to Windy.com..."));
                currentBatteryVoltage = readBatteryVoltage(); 
                if (sendDataToWindy()) {
                }
                telnetClient.println(F("--- Manual Windy.com Data Send Done ---"));
            } else if (command.equalsIgnoreCase(F("fetchbme"))) {
                telnetClient.println(F("Triggering manual BME280 fetch..."));
                fetchBME280Data();
                printDeviceStatus(); 
                telnetClient.println(F("--- Manual BME280 Fetch Done ---"));
            } else if (command.equalsIgnoreCase(F("resetalldata"))) {
                telnetClient.println(F("!!! RESETTING ALL DATA !!!"));
                printlnToAll(F("--- DATA RESET VIA TELNET ---"));

                portENTER_CRITICAL(&mux);
                totalRainfallMm = 0.0f;
                rainCount = 0;
                totalRainfallMonthMm = 0.0f;
                totalRainfallYearMm = 0.0f;
                rainDebounceFilteredCount = 0; 
                windDebounceFilteredCount = 0; 
                windGustsFilteredByRatio = 0; 
                windGustsMedianFilterActivity = 0; 
                currentMinuteRainPulses = 0;
                for(int i=0; i<HOURLY_RAIN_BUFFER_SIZE; i++) { hourlyRainMinuteBuffer[i] = 0.0f; }
                hourlyRainBufferIndex = 0;

                windPulseCount = 0;
                lastWindPulseSnapshotFor10minAvg = 0;
                pulses_at_last_gust_sample = 0;
                lastWindPulseTime_us = 0;
                
                speedIntervalBufferIndex = 0;
                speedIntervalSamplesCollected = 0;
                for(int i=0; i<MEDIAN_WINDOW_SIZE; i++) { speedIntervalBuffer[i] = 0.0f; }

                portEXIT_CRITICAL(&mux);

                windSpeedAvgValue = 0.0f; 
                currentMaxWindGust = 0.0f; 
                windSampleCount = 0;
                windSpeedCV = 0.0f; 
                lastGustSampleTime = millis();


                if (timeClient.isTimeSet()) {
                    time_t epochTime = timeClient.getEpochTime();
                    struct tm *ptm = localtime(&epochTime);
                    if (ptm != nullptr) {
                        lastResetMonth = ptm->tm_mon;
                        lastResetYearForMonthly = ptm->tm_year + 1900;
                        lastKnownYearForYearlyReset = ptm->tm_year + 1900;
                        lastMidnightCheckDayOfMonth = ptm->tm_mday;
                    }
                } else { 
                    lastResetMonth = -1;
                    lastResetYearForMonthly = -1;
                    lastKnownYearForYearlyReset = -1;
                }

                saveMonthRain();
                saveLastResetDate(lastResetMonth, lastResetYearForMonthly);
                saveYearRain();
                saveLastKnownYearForYearlyReset(lastKnownYearForYearlyReset);

                lastDisplayedRainCount = 0;
                calculated_wind_chill = NAN;
                calculated_dew_point = NAN;
                calculated_heat_index = NAN;


                printlnToAll(F("All rain/wind data reset."));
                printlnToAll(F("Month/Year refs set to current (if time available)."));
                currentBatteryVoltage = readBatteryVoltage();
                printDeviceStatus(); 
                telnetClient.println(F("Data reset complete."));

            } else if (command.equalsIgnoreCase(F("resetdebouncecounts"))) { 
                printlnToAll(F("--- RESETTING DEBOUNCE & GUST FILTER COUNTS ---")); 
                telnetClient.println(F("Resetting debounce & gust filter counts...")); 
                portENTER_CRITICAL(&mux);
                rainDebounceFilteredCount = 0;
                windDebounceFilteredCount = 0;
                windGustsFilteredByRatio = 0; 
                windGustsMedianFilterActivity = 0; 
                portEXIT_CRITICAL(&mux);
                printlnToAll(F("Debounce & gust filter counts reset to 0.")); 
                telnetClient.println(F("Debounce & gust filter counts reset.")); 
                printDeviceStatus(); 
            } else if (command.equalsIgnoreCase(F("setrainday"))) {
                if (arg.length() > 0) {
                    float val = arg.toFloat();
                    if (val >= 0) { 
                        portENTER_CRITICAL(&mux);
                        totalRainfallMm = val;
                        rainCount = round(val / RAINFALL_PER_IMPULSE_MM); 
                        currentMinuteRainPulses = 0; 
                        for(int i=0; i<HOURLY_RAIN_BUFFER_SIZE; i++) { hourlyRainMinuteBuffer[i] = 0.0f; }
                        hourlyRainBufferIndex = 0; 
                        portEXIT_CRITICAL(&mux);
                        telnetClient.print(F("Daily rainfall set to: ")); telnetClient.print(val, 4); 
                        telnetClient.print(F(" mm (impulses: ")); telnetClient.print(rainCount); 
                        telnetClient.println(F("). Hourly rain buffer for Windy reset."));
                        printDeviceStatus();
                    } else {
                        telnetClient.println(F("Invalid value. Rainfall must be >= 0."));
                    }
                } else {
                    telnetClient.println(F("Usage: setrainday <value_mm>"));
                }
            } 
            else if (command.equalsIgnoreCase(F("setrainmonth"))) {
                if (arg.length() > 0) {
                    float val = arg.toFloat();
                    portENTER_CRITICAL(&mux);
                    totalRainfallMonthMm = val;
                    portEXIT_CRITICAL(&mux);
                    saveMonthRain();
                    telnetClient.print(F("Monthly rain set: ")); telnetClient.print(val, 4); telnetClient.println(F(" mm & saved."));
                    printDeviceStatus(); 
                } else {
                    telnetClient.println(F("Usage: setrainmonth <value>"));
                }
            } else if (command.equalsIgnoreCase(F("setrainyear"))) {
                 if (arg.length() > 0) {
                    float val = arg.toFloat();
                    portENTER_CRITICAL(&mux);
                    totalRainfallYearMm = val;
                    portEXIT_CRITICAL(&mux);
                    saveYearRain();
                    telnetClient.print(F("Yearly rain set: ")); telnetClient.print(val, 4); telnetClient.println(F(" mm & saved."));
                    printDeviceStatus(); 
                } else {
                    telnetClient.println(F("Usage: setrainyear <value>"));
                }
            } else if (command.equalsIgnoreCase(F("getconfig"))) {
                telnetClient.println(F("--- Sensor Debounce Config ---"));
                telnetClient.print(F("  Rain Debounce: "));
                telnetClient.print(debounceDelayMs);
                telnetClient.println(F(" ms"));
                telnetClient.print(F("  Wind Debounce: "));
                telnetClient.print(windPulseDebounceUs);
                telnetClient.println(F(" us"));
                telnetClient.println(F("--- Wind Average EMA Filter ---"));
                telnetClient.print(F("  Wind Avg EMA Alpha: ")); telnetClient.println(windAvgEmaAlpha, 2); 
                telnetClient.println(F("--- Gust Filter Config ---"));
                telnetClient.print(F("  Median Filter Window: ")); telnetClient.println(MEDIAN_WINDOW_SIZE); 
                telnetClient.print(F("  Max Gust/Avg Ratio: ")); telnetClient.println(maxGustToAvgWindRatio, 1);
                telnetClient.print(F("  Min Avg Wind for Filter: ")); telnetClient.print(minAvgWindForRatioCheckMps, 1); telnetClient.println(F(" m/s"));
                telnetClient.println(F("------------------------------"));
            } else if (command.equalsIgnoreCase(F("setraindebounce"))) {
                if (arg.length() > 0) {
                    long val = arg.toInt(); 
                    if (val > 0) { 
                        debounceDelayMs = val;
                        preferences.begin(PREFS_NAMESPACE, false);
                        preferences.putULong(PREFS_KEY_DEBOUNCE_RAIN, debounceDelayMs);
                        preferences.end();
                        telnetClient.print(F("Rain debounce set: "));
                        telnetClient.print(debounceDelayMs);
                        telnetClient.println(F(" ms & saved."));
                        printDeviceStatus(); 
                    } else {
                        telnetClient.println(F("Invalid value. Must be > 0."));
                    }
                } else {
                    telnetClient.println(F("Usage: setraindebounce <ms>"));
                }
            } else if (command.equalsIgnoreCase(F("setwinddebounce"))) {
                if (arg.length() > 0) {
                    long val = arg.toInt();
                    if (val > 0) { 
                        windPulseDebounceUs = val;
                        preferences.begin(PREFS_NAMESPACE, false);
                        preferences.putULong(PREFS_KEY_DEBOUNCE_WIND, windPulseDebounceUs);
                        preferences.end();
                        telnetClient.print(F("Wind debounce set: "));
                        telnetClient.print(windPulseDebounceUs);
                        telnetClient.println(F(" us & saved."));
                        printDeviceStatus(); 
                    } else {
                        telnetClient.println(F("Invalid value. Must be > 0."));
                    }
                } else {
                    telnetClient.println(F("Usage: setwinddebounce <us>"));
                }
            } else if (command.equalsIgnoreCase(F("setgustratio"))) { 
                if (arg.length() > 0) {
                    float val = arg.toFloat();
                    if (val > 0) { 
                        maxGustToAvgWindRatio = val;
                        preferences.begin(PREFS_NAMESPACE, false);
                        preferences.putFloat(PREFS_KEY_GUST_RATIO, maxGustToAvgWindRatio);
                        preferences.end();
                        telnetClient.print(F("Max Gust/Avg Ratio set to: "));
                        telnetClient.print(maxGustToAvgWindRatio, 1);
                        telnetClient.println(F(" & saved."));
                        printDeviceStatus(); 
                    } else {
                        telnetClient.println(F("Invalid value. Must be > 0."));
                    }
                } else {
                    telnetClient.println(F("Usage: setgustratio <value (e.g., 3.0)>"));
                }
            } else if (command.equalsIgnoreCase(F("setmingustavg"))) { 
                if (arg.length() > 0) {
                    float val = arg.toFloat();
                    if (val >= 0) { 
                        minAvgWindForRatioCheckMps = val;
                        preferences.begin(PREFS_NAMESPACE, false);
                        preferences.putFloat(PREFS_KEY_MIN_AVG_GUST, minAvgWindForRatioCheckMps);
                        preferences.end();
                        telnetClient.print(F("Min Avg Wind for Gust Ratio Check set to: "));
                        telnetClient.print(minAvgWindForRatioCheckMps, 1);
                        telnetClient.println(F(" m/s & saved."));
                        printDeviceStatus(); 
                    } else {
                        telnetClient.println(F("Invalid value. Must be >= 0."));
                    }
                } else {
                    telnetClient.println(F("Usage: setmingustavg <value_mps (e.g., 1.0)>"));
                }
            } else if (command.equalsIgnoreCase(F("setwindavgalpha"))) { 
                if (arg.length() > 0) {
                    float val = arg.toFloat();
                    if (val >= 0.0f && val <= 1.0f) { 
                        windAvgEmaAlpha = val;
                        preferences.begin(PREFS_NAMESPACE, false);
                        preferences.putFloat(PREFS_KEY_WIND_AVG_EMA_ALPHA, windAvgEmaAlpha);
                        preferences.end();
                        telnetClient.print(F("Wind Avg EMA Alpha set to: "));
                        telnetClient.print(windAvgEmaAlpha, 2);
                        telnetClient.println(F(" & saved."));
                        printDeviceStatus(); 
                    } else {
                        telnetClient.println(F("Invalid value. Alpha must be between 0.0 and 1.0."));
                    }
                } else {
                    telnetClient.println(F("Usage: setwindavgalpha <value (0.0-1.0)>"));
                }
            } else if (command.equalsIgnoreCase(F("reboot"))) {
                telnetClient.println(F("Rebooting device..."));
                Serial.println(F("[TELNET] Reboot cmd. Rebooting..."));
                delay(200);
                telnetClient.stop();
                ESP.restart();
            } else if (command.equalsIgnoreCase(F("quit")) || command.equalsIgnoreCase(F("exit"))) {
                telnetClient.println(F("Disconnecting. Bye!"));
                telnetClient.stop();
                Serial.println(F("[TELNET] Client disconnected."));
            } else if (command.length() > 0) { 
                telnetClient.print(F("Unknown cmd: '")); telnetClient.print(command); telnetClient.println(F("'"));
                telnetClient.println(F("Type 'help' for commands."));
            }

            if (telnetClient && telnetClient.connected()) {
                telnetClient.print(F("> "));
            }
        }
    } else { 
        if (telnetClient) { 
            telnetClient.stop(); 
        }
    }
}


void setup() {
    pinMode(WIFI_ENABLE, OUTPUT);
    digitalWrite(WIFI_ENABLE, LOW); 
    delay(100); 
    pinMode(WIFI_ANT_CONFIG, OUTPUT);
    digitalWrite(WIFI_ANT_CONFIG, HIGH); 

    Serial.begin(115200);
    unsigned long initMillis = millis();
    while (!Serial && (millis() - initMillis < 2000)); 

    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset(); 

    Serial.println(F("\n\n================================================"));
    Serial.println(F("  XIAO ESP32-C6 RAIN & WIND GAUGE - STARTING UP"));
    Serial.println(F("================================================"));
    Serial.print(F("  Firmware Version: ")); Serial.println(FIRMWARE_VERSION);
    Serial.print(F("  Build Timestamp:  ")); Serial.print(__DATE__); Serial.print(F(" ")); Serial.println(__TIME__);
    Serial.println(F("------------------------------------------------"));

    windSpeedAvgValue = 0.0f; 
    windSampleCount = 0;
    windSpeedCV = 0.0f; 
    for(int i=0; i<MAX_WIND_SAMPLES; ++i) windSpeedSamples[i] = 0.0f;
    lastWindPulseTime_us = 0; 

    speedIntervalBufferIndex = 0;
    speedIntervalSamplesCollected = 0;
    for(int i=0; i<MEDIAN_WINDOW_SIZE; i++) { speedIntervalBuffer[i] = 0.0f; }
    windGustsMedianFilterActivity = 0;


    bme280_temperature = NAN; 
    bme280_humidity = NAN;
    bme280_pressure = NAN;
    calculated_wind_chill = NAN;
    calculated_dew_point = NAN;
    calculated_heat_index = NAN;
    rainDebounceFilteredCount = 0; 
    windDebounceFilteredCount = 0;
    windGustsFilteredByRatio = 0; 
    lastSuccessfulWindySendTime = 0;
    for(int i=0; i<HOURLY_RAIN_BUFFER_SIZE; i++) { hourlyRainMinuteBuffer[i] = 0.0f; } 
    hourlyRainBufferIndex = 0;
    currentMinuteRainPulses = 0;
    lastMinuteTimestampForHourlyRain = millis(); 


    if (BME280_FETCH_INTERVAL_MS > 5000) {
      lastBME280FetchTime = millis() - BME280_FETCH_INTERVAL_MS + 5000;
    } else {
      lastBME280FetchTime = millis(); 
    }


    Serial.println(F("[ANTENNA CONFIG]"));
    Serial.print(F("  WIFI_ENABLE (Pin ")); Serial.print(WIFI_ENABLE); Serial.println(F(") set to LOW (RF switch active)"));
    Serial.print(F("  WIFI_ANT_CONFIG (Pin ")); Serial.print(WIFI_ANT_CONFIG); Serial.println(F(") set to HIGH (External Antenna)"));
    Serial.println(F("------------------------------------------------"));

    Serial.println(F("[ADC SETUP]"));
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db); 
    Serial.println(F("  ADC attenuation set for battery measurement on GPIO0."));
    Serial.println(F("------------------------------------------------"));

    Serial.println(F("[PREFERENCES LOAD]"));
    preferences.begin(PREFS_NAMESPACE, true); 
    totalRainfallMonthMm = preferences.getFloat(PREFS_KEY_MONTH_RAIN, 0.0f);
    lastResetMonth = preferences.getInt(PREFS_KEY_LAST_RESET_MONTH, -1); 
    lastResetYearForMonthly = preferences.getInt(PREFS_KEY_LAST_RESET_YEAR_FOR_MONTHLY, -1);
    totalRainfallYearMm = preferences.getFloat(PREFS_KEY_YEAR_RAIN, 0.0f);
    lastKnownYearForYearlyReset = preferences.getInt(PREFS_KEY_YEAR_FOR_ACCUMULATION, -1);
    
    debounceDelayMs = preferences.getULong(PREFS_KEY_DEBOUNCE_RAIN, DEBOUNCE_DELAY_MS_DEFAULT);
    windPulseDebounceUs = preferences.getULong(PREFS_KEY_DEBOUNCE_WIND, WIND_PULSE_DEBOUNCE_US_DEFAULT);
    
    maxGustToAvgWindRatio = preferences.getFloat(PREFS_KEY_GUST_RATIO, 3.0f); 
    minAvgWindForRatioCheckMps = preferences.getFloat(PREFS_KEY_MIN_AVG_GUST, 1.0f); 
    windAvgEmaAlpha = preferences.getFloat(PREFS_KEY_WIND_AVG_EMA_ALPHA, DEFAULT_WIND_AVG_EMA_ALPHA);
    
    preferences.end(); 

    Serial.print(F("  Monthly Rainfall from Prefs: ")); Serial.print(totalRainfallMonthMm, 4); Serial.println(F(" mm"));
    Serial.print(F("  Last Month Reset: M=")); Serial.print(lastResetMonth != -1 ? String(lastResetMonth + 1) : F("N/A")); Serial.print(F("/Y=")); Serial.println(lastResetYearForMonthly == -1 ? F("N/A") : String(lastResetYearForMonthly));
    Serial.print(F("  Yearly Rainfall from Prefs: ")); Serial.print(totalRainfallYearMm, 4); Serial.println(F(" mm"));
    Serial.print(F("  Year for Yearly Accumulation: ")); Serial.println(lastKnownYearForYearlyReset == -1 ? F("N/A") : String(lastKnownYearForYearlyReset));
    Serial.print(F("  Loaded Rain Debounce: ")); Serial.print(debounceDelayMs); Serial.println(F(" ms"));
    Serial.print(F("  Loaded Wind Debounce: ")); Serial.print(windPulseDebounceUs); Serial.println(F(" us"));
    Serial.print(F("  Loaded Wind Avg EMA Alpha: ")); Serial.println(windAvgEmaAlpha, 2); 
    Serial.print(F("  Loaded Max Gust/Avg Ratio: ")); Serial.println(maxGustToAvgWindRatio, 1);
    Serial.print(F("  Loaded Min Avg Wind for Gust Check: ")); Serial.print(minAvgWindForRatioCheckMps, 1); Serial.println(F(" m/s"));
    Serial.println(F("------------------------------------------------"));

    Serial.println(F("[DEVICE CONFIGURATION]"));
    Serial.print(F("  Rain Sensor Pin: GPIO ")); Serial.println(RAIN_SENSOR_PIN);
    Serial.print(F("  Wind Sensor Pin: GPIO ")); Serial.println(WIND_SENSOR_PIN);
    Serial.print(F("  Battery ADC Pin: GPIO ")); Serial.println(BATTERY_ADC_PIN);
    Serial.print(F("  Rainfall per Impulse: ")); Serial.print(RAINFALL_PER_IMPULSE_MM, 4); Serial.println(F(" mm"));
    Serial.print(F("  Wind m/s per (pulse/sec): ")); Serial.print(WIND_MPS_PER_PULSE_PER_SEC, 4); Serial.println(F(""));
    Serial.print(F("  Data Send Interval: ")); Serial.print(SEND_INTERVAL_MS / 1000UL); Serial.println(F(" seconds"));
    Serial.print(F("  Gust Sample Interval: ")); Serial.print(GUST_SAMPLE_INTERVAL_MS / 1000UL); Serial.println(F(" seconds"));
    Serial.print(F("  BME280 Fetch Interval: ")); Serial.print(BME280_FETCH_INTERVAL_MS / 1000UL); Serial.println(F(" seconds"));
    Serial.println(F("  NTP Client Offset: 0 (TZ string handles DST)"));
    Serial.println(F("------------------------------------------------"));

    pinMode(RAIN_SENSOR_PIN, INPUT_PULLUP); 
    pinMode(WIND_SENSOR_PIN, INPUT);      
    pinMode(BATTERY_ADC_PIN, INPUT);      

    Serial.println(F("[WIFI SETUP]"));
    Serial.print(F("  Connecting to WiFi SSID: ")); Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long wifiConnectStartTime = millis();
    const unsigned long WIFI_CONNECT_TIMEOUT_MS = 30000; 

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(F("."));
        if (millis() - wifiConnectStartTime > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println(F("\n  ERR: WiFi Connect failed/timed out."));
            Serial.println(F("  RESTARTING ESP in 5s..."));
            Serial.println(F("================================================"));
            delay(5000);
            ESP.restart();
        }
    }
    Serial.println(); 
    printlnToAll(F("  WiFi Connected!"));
    printToAll(F("  SSID: ")); printlnToAll(WiFi.SSID());
    printToAll(F("  IP Address: ")); printlnToAll(WiFi.localIP());
    printToAll(F("  Signal Strength (RSSI): ")); printToAll(WiFi.RSSI()); printlnToAll(F(" dBm"));
    printlnToAll(F("------------------------------------------------"));


    printlnToAll(F("[OTA SETUP]"));
    String ota_hostname = "xiao-srazkomer-vitr-" + String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF), HEX);
    ArduinoOTA.setHostname(ota_hostname.c_str());
    printToAll(F("  OTA Hostname: ")); printlnToAll(ota_hostname);
    ArduinoOTA.setPassword("9110014100"); 

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch"; else type = "filesystem"; 
            Serial.print(F("\n--- [OTA] Update Started for ")); Serial.print(type); Serial.println(F(" ---"));
            if (telnetClient && telnetClient.connected()) {
                telnetClient.println(F("!!! OTA update started. Telnet connection will be closed. !!!"));
                telnetClient.stop();
            }
            Serial.println(F("  Detaching sensor interrupts for OTA..."));
            detachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN));
            detachInterrupt(digitalPinToInterrupt(WIND_SENSOR_PIN));
        })
        .onEnd([]() {
            Serial.println(F("\n--- [OTA] Update Finished Successfully ---"));
            Serial.println(F("  Device will restart."));
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("  OTA Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.print(F("\n--- [OTA] Update Error ---"));
            Serial.printf("  Error Code [%u]: ", error);
            String errorMsgText;
            if (error == OTA_AUTH_ERROR) errorMsgText = F("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) errorMsgText = F("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) errorMsgText = F("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) errorMsgText = F("Receive Failed");
            else if (error == OTA_END_ERROR) errorMsgText = F("End Failed");
            else errorMsgText = F("Unknown Error");
            Serial.println(errorMsgText);

            printlnToAll(F("  Re-attaching sensor interrupts."));
            attachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN), rainDetected, FALLING);
            attachInterrupt(digitalPinToInterrupt(WIND_SENSOR_PIN), windPulseDetected, RISING);
            printlnToAll(F("--------------------------"));
        });

    ArduinoOTA.begin();
    printlnToAll(F("  OTA updates ready."));
    printlnToAll(F("------------------------------------------------"));

    Serial.println(F("[TELNET SERVER SETUP]"));
    telnetServer.begin();
    telnetServer.setNoDelay(true); 
    Serial.print(F("  Telnet server started. To connect: telnet "));
    if(WiFi.status() == WL_CONNECTED) Serial.print(WiFi.localIP().toString()); else Serial.print(F("<NO WIFI IP>"));
    Serial.println(F(" 23"));
    Serial.println(F("------------------------------------------------"));

    printlnToAll(F("[INTERRUPT ATTACHMENT]"));
    attachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN), rainDetected, FALLING); 
    printlnToAll(F("  Rain sensor interrupt attached."));
    attachInterrupt(digitalPinToInterrupt(WIND_SENSOR_PIN), windPulseDetected, RISING); 
    printlnToAll(F("  Wind sensor interrupt attached."));
    printlnToAll(F("------------------------------------------------"));

    printlnToAll(F("[NTP CLIENT INIT & TIME SYNC]"));
    timeClient.begin();
    printlnToAll(F("  Forcing initial time update (UTC)..."));
    char timeBuffer[30]; 

    if (timeClient.forceUpdate()) { 
        time_t epochTime = timeClient.getEpochTime(); 
        struct tm *ptm = localtime(&epochTime); 
        
        printToAll(F("  Current Local Time (NTP & TZ): ")); printlnToAll(formatLocalTime(ptm, timeBuffer, sizeof(timeBuffer)));

        if(ptm != nullptr) { 
            lastMidnightCheckDayOfMonth = ptm->tm_mday; 
            printToAll(F("  Day for daily reset init: ")); printlnToAll(lastMidnightCheckDayOfMonth);
            int currentMonthNTP = ptm->tm_mon; 
            int currentYearNTP = ptm->tm_year + 1900;

            if (lastResetMonth == -1 || lastResetYearForMonthly == -1 ) { 
                printlnToAll(F("    Monthly rain: First run/no last reset. Setting to current M/Y & resetting."));
                lastResetMonth = currentMonthNTP;
                lastResetYearForMonthly = currentYearNTP;
                portENTER_CRITICAL(&mux);
                totalRainfallMonthMm = 0.0f;
                portEXIT_CRITICAL(&mux);
                saveLastResetDate(currentMonthNTP, currentYearNTP);
                saveMonthRain();
            } else if (currentYearNTP != lastResetYearForMonthly || currentMonthNTP != lastResetMonth) { 
                printToAll(F("    Monthly rain: New M/Y DETECTED. (Curr: ")); printToAll(currentMonthNTP + 1); printToAll(F("/")); printToAll(currentYearNTP);
                printToAll(F(", Last: ")); printToAll(lastResetMonth + 1); printToAll(F("/")); printlnToAll(lastResetYearForMonthly);
                printlnToAll(F("    Resetting monthly rainfall."));
                portENTER_CRITICAL(&mux);
                totalRainfallMonthMm = 0.0f;
                portEXIT_CRITICAL(&mux);
                lastResetMonth = currentMonthNTP;
                lastResetYearForMonthly = currentYearNTP;
                saveMonthRain();
                saveLastResetDate(currentMonthNTP, currentYearNTP);
            } else { 
                printlnToAll(F("    Monthly rain: Current M/Y matches. Using stored value."));
            }

            if (lastKnownYearForYearlyReset == -1 ) { 
                printlnToAll(F("    Yearly rain: First run/no last reset. Setting to current Y & resetting."));
                lastKnownYearForYearlyReset = currentYearNTP;
                portENTER_CRITICAL(&mux);
                totalRainfallYearMm = 0.0f;
                portEXIT_CRITICAL(&mux);
                saveYearRain();
                saveLastKnownYearForYearlyReset(currentYearNTP);
            } else if (currentYearNTP != lastKnownYearForYearlyReset) { 
                printToAll(F("    Yearly rain: New Y DETECTED. (Curr: ")); printToAll(currentYearNTP);
                printToAll(F(", Last: ")); printlnToAll(lastKnownYearForYearlyReset);
                printlnToAll(F("    Resetting yearly rainfall."));
                portENTER_CRITICAL(&mux);
                totalRainfallYearMm = 0.0f;
                portEXIT_CRITICAL(&mux);
                lastKnownYearForYearlyReset = currentYearNTP;
                saveYearRain();
                saveLastKnownYearForYearlyReset(currentYearNTP);
            } else { 
                printlnToAll(F("    Yearly rain: Current Y matches. Using stored value."));
            }
        } else { 
             printlnToAll(F("  ERR: localtime() in setup failed! Resets deferred."));
        }
    } else { 
        printlnToAll(F("  ERR: NTP forceUpdate() failed. Time resets might be inaccurate."));
    }
    printlnToAll(F("------------------------------------------------"));

    lastSendDataTime = initMillis; 
    if (SEND_INTERVAL_MS > 5000) {
         lastSendAttemptTime = initMillis - SEND_INTERVAL_MS + 5000;
    } else {
         lastSendAttemptTime = initMillis;
    }

    portENTER_CRITICAL(&mux);
    lastWindPulseSnapshotFor10minAvg = windPulseCount; 
    pulses_at_last_gust_sample = windPulseCount; 
    portEXIT_CRITICAL(&mux);
    lastGustSampleTime = initMillis; 

    currentBatteryVoltage = readBatteryVoltage(); 

    printDeviceStatus(); 

    printlnToAll(F("================================================"));
    printlnToAll(F("                SETUP COMPLETE - ENTERING LOOP"));
    printlnToAll(F("================================================"));
}

// Pomocná funkce pro třídění (Bubble Sort) pro mediánový filtr
void bubbleSort(float arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

// Funkce pro výpočet a sledování nárazů větru a vzorků pro CV
void calculate_and_track_gust_and_samples() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastGustSampleTime >= GUST_SAMPLE_INTERVAL_MS) {
        unsigned long current_total_wind_pulses;

        portENTER_CRITICAL(&mux);
        current_total_wind_pulses = windPulseCount;
        portEXIT_CRITICAL(&mux);

        unsigned long pulses_in_this_interval = current_total_wind_pulses - pulses_at_last_gust_sample;
        unsigned long time_elapsed_ms = currentMillis - lastGustSampleTime;
        float raw_speed_this_interval = 0.0f;

        if (time_elapsed_ms > 0) { 
            raw_speed_this_interval = ((float)pulses_in_this_interval / ((float)time_elapsed_ms / 1000.0f)) * WIND_MPS_PER_PULSE_PER_SEC;
        }

        // === Začátek implementace mediánového filtru ===
        speedIntervalBuffer[speedIntervalBufferIndex] = raw_speed_this_interval;
        speedIntervalBufferIndex = (speedIntervalBufferIndex + 1) % MEDIAN_WINDOW_SIZE;
        if (speedIntervalSamplesCollected < MEDIAN_WINDOW_SIZE) {
            speedIntervalSamplesCollected++;
        }

        float speed_this_interval = raw_speed_this_interval; 

        if (speedIntervalSamplesCollected >= MEDIAN_WINDOW_SIZE) {
            float tempSortBuffer[MEDIAN_WINDOW_SIZE];
            for (int i = 0; i < MEDIAN_WINDOW_SIZE; i++) {
                tempSortBuffer[i] = speedIntervalBuffer[i];
            }
            bubbleSort(tempSortBuffer, MEDIAN_WINDOW_SIZE); 
            speed_this_interval = tempSortBuffer[MEDIAN_WINDOW_SIZE / 2]; 

            if (fabs(speed_this_interval - raw_speed_this_interval) > 0.01f) { 
                portENTER_CRITICAL(&mux);
                windGustsMedianFilterActivity++; 
                portEXIT_CRITICAL(&mux);
            }
        }
        // === Konec implementace mediánového filtru ===
            
        bool updateGust = true;
        // === Začátek filtru poměru nárazu k průměru ===
        if (windSpeedAvgValue > minAvgWindForRatioCheckMps && windSpeedAvgValue > 0.0001f ) { 
            if ((speed_this_interval / windSpeedAvgValue) > maxGustToAvgWindRatio) {
                updateGust = false;
                portENTER_CRITICAL(&mux);
                windGustsFilteredByRatio++;
                portEXIT_CRITICAL(&mux);
            }
        }
        // === Konec filtru poměru ===

        if (updateGust && speed_this_interval > currentMaxWindGust) {
            currentMaxWindGust = speed_this_interval; 
        }

        if (windSampleCount < MAX_WIND_SAMPLES) {
            windSpeedSamples[windSampleCount] = speed_this_interval; 
            windSampleCount++;
        } 

        pulses_at_last_gust_sample = current_total_wind_pulses;
        lastGustSampleTime = currentMillis;
    }
}


void loop() {
    ArduinoOTA.handle(); 
    handleTelnet();      
    calculate_and_track_gust_and_samples(); 
    updateHourlyRainBuffer(); 

    unsigned long currentMillisLoop = millis();
    char timeBuffer[30]; 

    if (WiFi.status() == WL_CONNECTED && (currentMillisLoop - lastBME280FetchTime >= BME280_FETCH_INTERVAL_MS || (lastBME280FetchTime < 5000 && millis() > 5000 && isnan(bme280_temperature)) ) ) { 
        fetchBME280Data();
    }


    unsigned int currentRainCountSnapshot;
    bool rainCountChanged = false;
    portENTER_CRITICAL(&mux);
    currentRainCountSnapshot = rainCount;
    portEXIT_CRITICAL(&mux);

    if (currentRainCountSnapshot != lastDisplayedRainCount) {
        rainCountChanged = true;
        printlnToAll(F("\n*****************************************"));
        printlnToAll(F("        *** RAIN EVENT DETECTED ***"));
        if(timeClient.isTimeSet()){
            time_t epochTime = timeClient.getEpochTime();
            struct tm* ptm = localtime(&epochTime);
            printToAll(F("  Time: ")); printlnToAll(formatLocalTime(ptm, timeBuffer, sizeof(timeBuffer)));
        } else {
            printlnToAll(F("  Time: (NTP N/A)"));
        }
        printToAll(F("  New daily impulses: ")); printToAll(currentRainCountSnapshot);
        printToAll(F(" (Prev: ")); printToAll(lastDisplayedRainCount); printlnToAll(F(")"));

        float currentDayRainfallSnapshot, currentMonthRainfallSnapshot, currentYearRainfallSnapshot;
        portENTER_CRITICAL(&mux);
        currentDayRainfallSnapshot = totalRainfallMm;
        currentMonthRainfallSnapshot = totalRainfallMonthMm;
        currentYearRainfallSnapshot = totalRainfallYearMm;
        portEXIT_CRITICAL(&mux);

        printToAll(F("  Daily Rain:   ")); printToAll(currentDayRainfallSnapshot, 4); printlnToAll(F(" mm"));
        printToAll(F("  Monthly Rain: ")); printToAll(currentMonthRainfallSnapshot, 4); printlnToAll(F(" mm"));
        printToAll(F("  Yearly Rain:  ")); printToAll(currentYearRainfallSnapshot, 4); printlnToAll(F(" mm"));

        currentBatteryVoltage = readBatteryVoltage(); 
        printDeviceStatus(); 
        printlnToAll(F("*****************************************"));
        lastDisplayedRainCount = currentRainCountSnapshot;

        if (telnetClient && telnetClient.connected()) { 
            telnetClient.print(F("> "));
        }
    }

    if (rainCountChanged) {
        float tempMonthRain, tempYearRain;
        portENTER_CRITICAL(&mux);
        tempMonthRain = totalRainfallMonthMm;
        tempYearRain = totalRainfallYearMm;
        portEXIT_CRITICAL(&mux);

        preferences.begin(PREFS_NAMESPACE, false);
        preferences.putFloat(PREFS_KEY_MONTH_RAIN, tempMonthRain);
        preferences.putFloat(PREFS_KEY_YEAR_RAIN, tempYearRain);
        preferences.end();
    }

    if (WiFi.status() == WL_CONNECTED) {
        static bool wifiWasDisconnected = true; 
        if (wifiWasDisconnected) {
            printlnToAll(F("\n--- WiFi Reconnected ---"));
            printlnToAll(F("  Forcing NTP update and BME fetch."));
            timeClient.forceUpdate();
            if (isnan(bme280_temperature)) fetchBME280Data(); 
            wifiWasDisconnected = false; 
            lastSendAttemptTime = currentMillisLoop - SEND_INTERVAL_MS + 10000; 
        }

        bool timeNeedsUpdate = timeClient.update(); 

        if (timeClient.isTimeSet()) { 
            time_t epochTime = timeClient.getEpochTime(); 
            struct tm *ptm = localtime(&epochTime);     

            if (ptm != nullptr) { 
                int currentDay = ptm->tm_mday;
                int currentMonth = ptm->tm_mon;       
                int currentYear = ptm->tm_year + 1900; 

                if (currentDay != lastMidnightCheckDayOfMonth && lastMidnightCheckDayOfMonth != 0) {
                    printlnToAll(F("\n-----------------------------------------"));
                    printlnToAll(F("    --- EVENT: MIDNIGHT DAILY RESET ---"));
                    printToAll(F("  Time: ")); printToAll(formatLocalTime(ptm, timeBuffer, sizeof(timeBuffer)));
                    printToAll(F(" (Day ")); printToAll(lastMidnightCheckDayOfMonth); printToAll(F(" -> ")); printToAll(currentDay); printlnToAll(F(")"));
                    printlnToAll(F("  Resetting DAILY rain counts.")); 

                    float previousDayTotalRain;
                    portENTER_CRITICAL(&mux);
                    previousDayTotalRain = totalRainfallMm;
                    rainCount = 0;
                    totalRainfallMm = 0.0f;
                    rainDebounceFilteredCount = 0; 
                    windDebounceFilteredCount = 0;
                    windGustsFilteredByRatio = 0; 
                    windGustsMedianFilterActivity = 0; 
                    portEXIT_CRITICAL(&mux);

                    printToAll(F("  Prev day total: ")); printToAll(previousDayTotalRain, 4); printlnToAll(F(" mm"));
                    lastMidnightCheckDayOfMonth = currentDay;
                    lastDisplayedRainCount = 0; 
                    printToAll(F("  Next daily reset: day ")); printlnToAll(lastMidnightCheckDayOfMonth);
                    printlnToAll(F("  Sending data post-daily-reset..."));
                    currentBatteryVoltage = readBatteryVoltage();
                    bool rainOK_DR = sendRainDataToServer(); 
                    bool windyOK_DR = sendDataToWindy();
                    bool windOK_DR = sendWindDataToServer();
                    bool derivedOK_DR = false;
                    if (!isnan(calculated_wind_chill) || !isnan(calculated_dew_point) || !isnan(calculated_heat_index)) {
                        derivedOK_DR = sendDerivedDataToServer();
                    }
                    if(rainOK_DR || windOK_DR || derivedOK_DR || windyOK_DR) lastSendDataTime = millis();

                    if (telnetClient && telnetClient.connected()) telnetClient.print(F("> "));
                    printlnToAll(F("-----------------------------------------"));
                } else if (lastMidnightCheckDayOfMonth == 0 && ptm->tm_year > (2000-1900) ) { 
                    lastMidnightCheckDayOfMonth = currentDay;
                    printlnToAll(F("\n--- INFO: DAILY RESET CHECK INIT ---"));
                    printToAll(F("  NTP time OK. Day for midnight check: ")); printlnToAll(lastMidnightCheckDayOfMonth);
                    printlnToAll(F("------------------------------------"));
                }

                if (lastResetMonth != -1 && (currentMonth != lastResetMonth || currentYear != lastResetYearForMonthly) ) {
                    printlnToAll(F("\n================================================"));
                    printlnToAll(F("    === EVENT: MONTH CHANGE - MONTHLY RESET ==="));
                    printToAll(F("  Time: ")); printToAll(formatLocalTime(ptm, timeBuffer, sizeof(timeBuffer)));
                    printToAll(F(" (M")); printToAll(lastResetMonth + 1); printToAll(F("/")); printToAll(lastResetYearForMonthly);
                    printToAll(F(" -> M")); printToAll(currentMonth + 1); printToAll(F("/")); printToAll(currentYear); printlnToAll(F(")"));

                    float previousMonthTotal;
                    portENTER_CRITICAL(&mux);
                    previousMonthTotal = totalRainfallMonthMm;
                    totalRainfallMonthMm = 0.0f; 
                    portEXIT_CRITICAL(&mux);

                    printToAll(F("  Prev month total (M")); printToAll(lastResetMonth + 1); printToAll(F("/")); printToAll(lastResetYearForMonthly);
                    printToAll(F("): ")); printToAll(previousMonthTotal, 4); printlnToAll(F(" mm"));
                    printlnToAll(F("  Resetting monthly rain."));

                    lastResetMonth = currentMonth;
                    lastResetYearForMonthly = currentYear;
                    saveMonthRain(); 
                    saveLastResetDate(currentMonth, currentYear); 

                    printToAll(F("  Next monthly reset: M")); printToAll(lastResetMonth + 1); printToAll(F("/")); printlnToAll(lastResetYearForMonthly);
                    printlnToAll(F("  Sending data post-monthly-reset..."));
                    currentBatteryVoltage = readBatteryVoltage();
                    bool rainOK_MR = sendRainDataToServer(); 
                    bool windyOK_MR = sendDataToWindy();
                    bool windOK_MR = sendWindDataToServer();
                    bool derivedOK_MR = false;
                    if (!isnan(calculated_wind_chill) || !isnan(calculated_dew_point) || !isnan(calculated_heat_index)) {
                        derivedOK_MR = sendDerivedDataToServer();
                    }
                    if(rainOK_MR || windOK_MR || derivedOK_MR || windyOK_MR) lastSendDataTime = millis();
                    if (telnetClient && telnetClient.connected()) telnetClient.print(F("> "));
                    printlnToAll(F("================================================"));
                }

                if (lastKnownYearForYearlyReset != -1 && currentYear != lastKnownYearForYearlyReset) {
                    printlnToAll(F("\n################################################"));
                    printlnToAll(F("    %%% EVENT: NEW YEAR - YEARLY RESET %%%"));
                    printToAll(F("  Time: ")); printToAll(formatLocalTime(ptm, timeBuffer, sizeof(timeBuffer)));
                    printToAll(F(" (Year ")); printToAll(lastKnownYearForYearlyReset);
                    printToAll(F(" -> ")); printToAll(currentYear); printlnToAll(F(")"));

                    float previousYearTotal;
                    portENTER_CRITICAL(&mux);
                    previousYearTotal = totalRainfallYearMm;
                    totalRainfallYearMm = 0.0f; 
                    portEXIT_CRITICAL(&mux);

                    printToAll(F("  Prev year total (")); printToAll(lastKnownYearForYearlyReset);
                    printToAll(F("): ")); printToAll(previousYearTotal, 4); printlnToAll(F(" mm"));
                    printlnToAll(F("  Resetting yearly rain."));

                    lastKnownYearForYearlyReset = currentYear;
                    saveYearRain(); 
                    saveLastKnownYearForYearlyReset(currentYear); 

                    printToAll(F("  Next yearly reset: Y")); printlnToAll(lastKnownYearForYearlyReset);
                    printlnToAll(F("  Sending data post-yearly-reset..."));
                    currentBatteryVoltage = readBatteryVoltage();
                    bool rainOK_YR = sendRainDataToServer(); 
                    bool windyOK_YR = sendDataToWindy();
                    bool windOK_YR = sendWindDataToServer();
                    bool derivedOK_YR = false;
                    if (!isnan(calculated_wind_chill) || !isnan(calculated_dew_point) || !isnan(calculated_heat_index)) {
                        derivedOK_YR = sendDerivedDataToServer();
                    }
                    if(rainOK_YR || windOK_YR || derivedOK_YR || windyOK_YR) lastSendDataTime = millis();
                    if (telnetClient && telnetClient.connected()) telnetClient.print(F("> "));
                    printlnToAll(F("################################################"));
                }

            } else { 
                printlnToAll(F("\n--- ERR: NTP localtime() fail ---"));
                printlnToAll(F("  Time resets may be affected."));
                printlnToAll(F("---------------------------------"));
            }
        } else { 
             static unsigned long lastNtpLogTime = 0;
             if (currentMillisLoop - lastNtpLogTime > 60000) { 
                 lastNtpLogTime = currentMillisLoop;
                 printlnToAll(F("\n--- NTP STATUS ---"));
                 printlnToAll(F("  Waiting for NTP time..."));
                 printlnToAll(F("------------------"));
             }
        }

        if (currentMillisLoop - lastSendAttemptTime >= SEND_INTERVAL_MS) {
            lastSendAttemptTime = currentMillisLoop; 

            printlnToAll(); 
            if(timeClient.isTimeSet()){
                time_t epochTime = timeClient.getEpochTime();
                struct tm* ptm = localtime(&epochTime);
                printToAll(formatLocalTime(ptm, timeBuffer, sizeof(timeBuffer)));
            } else {
                printToAll(F("(Time N/A)"));
            }
            printlnToAll(F(" - Scheduled data send."));

            currentBatteryVoltage = readBatteryVoltage(); 
            bool rainSendOK = sendRainDataToServer();
            bool windySendOK = sendDataToWindy();
            bool windSendOK = sendWindDataToServer(); 
            bool derivedSendOK = false;
            if (!isnan(calculated_wind_chill) || !isnan(calculated_dew_point) || !isnan(calculated_heat_index)) {
                 derivedSendOK = sendDerivedDataToServer();
            }

            if(rainSendOK || windSendOK || derivedSendOK || windySendOK) { 
                lastSendDataTime = currentMillisLoop; 
            }

            if (telnetClient && telnetClient.connected()) { 
                telnetClient.print(F("> "));
            }
        }
    } else { 
        static unsigned long lastWifiLogTime = 0;
        static unsigned long wifiDisconnectedSince = 0;
        const unsigned long WIFI_RECONNECT_ATTEMPT_INTERVAL = 5 * 60 * 1000; 
        const unsigned long WIFI_RESTART_AFTER_DISCONNECT_DURATION = 15 * 60 * 1000; 
        static unsigned long lastReconnectAttemptTime = 0;


        if (wifiDisconnectedSince == 0) { 
            wifiDisconnectedSince = currentMillisLoop;
            lastWifiLogTime = currentMillisLoop; 
            printlnToAll(F("\n--- WIFI ALERT ---"));
            printlnToAll(F("  WiFi DISCONNECTED. Will try reconnect."));
            printlnToAll(F("  Sending/NTP paused."));
            printlnToAll(F("--------------------"));
            if (telnetClient && telnetClient.connected()) {
                telnetClient.print(F("> "));
            }
        }
        
        if (currentMillisLoop - lastWifiLogTime >= 30000) { 
             lastWifiLogTime = currentMillisLoop;
             printlnToAll(F("\n--- WIFI ALERT (Still Disconnected) ---"));
             printToAll(F("  Disconnected for: ")); printToAll((currentMillisLoop - wifiDisconnectedSince)/1000); printlnToAll(F("s."));
             printlnToAll(F("---------------------------------------"));
             if (telnetClient && telnetClient.connected()) {
                 telnetClient.print(F("> "));
             }
        }

        if (currentMillisLoop - wifiDisconnectedSince > WIFI_RESTART_AFTER_DISCONNECT_DURATION) {
            printlnToAll(F("!!! WiFi disconnected >15min. Restarting device. !!!"));
            delay(1000);
            ESP.restart();
        } 
        else if (currentMillisLoop - wifiDisconnectedSince > WIFI_RECONNECT_ATTEMPT_INTERVAL) {
            if (currentMillisLoop - lastReconnectAttemptTime > WIFI_RECONNECT_ATTEMPT_INTERVAL) { 
                printlnToAll(F("--- WiFi disconnected >5min. Attempting reconnect... ---"));
                WiFi.reconnect();
                lastReconnectAttemptTime = currentMillisLoop;
            }
        }
        delay(1000); 
    }
    delay(10); 
}
