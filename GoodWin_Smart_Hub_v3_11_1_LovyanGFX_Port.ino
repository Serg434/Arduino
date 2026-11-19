#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Matter.h>
#include <esp_matter.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include <esp_log.h>
#include <stdarg.h>
#include <SPI.h>
#include <LovyanGFX.hpp>
#include <DHT.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <math.h>

// The selected Arduino target supplies the board profile. Runtime silicon
// identification below is read from the ESP32 itself.
#if defined(ARDUINO_BOARD)
  #define GOODWIN_BOARD_PROFILE ARDUINO_BOARD
#elif defined(ARDUINO_VARIANT)
  #define GOODWIN_BOARD_PROFILE ARDUINO_VARIANT
#elif defined(CONFIG_IDF_TARGET_ESP32)
  #define GOODWIN_BOARD_PROFILE "ESP32 generic target"
#else
  #define GOODWIN_BOARD_PROFILE "Unknown Arduino board profile"
#endif

#if __has_include(<esp32-hal-misc.h>)
  #include <esp32-hal-misc.h>
  #define GOODWIN_HAS_TEMPERATURE_READ 1
#else
  #define GOODWIN_HAS_TEMPERATURE_READ 0
#endif

static float goodwinReadChipTemperatureC() {
#if GOODWIN_HAS_TEMPERATURE_READ
  return temperatureRead();
#else
  return NAN;
#endif
}

static const char *goodwinFlashModeName(FlashMode_t mode) {
  switch (mode) {
    case FM_QIO: return "QIO";
    case FM_QOUT: return "QOUT";
    case FM_DIO: return "DIO";
    case FM_DOUT: return "DOUT";
    case FM_FAST_READ: return "FAST";
    case FM_SLOW_READ: return "SLOW";
    default: return "UNKNOWN";
  }
}

// ============================================================
// GoodWin Smart Hub v3.11.1-dev LovyanGFX port | DMA partial UI + buffered screensavers + resilient weather
// Matter node: LED dimmer + TFT dimmer + one climate endpoint with temperature and RH clusters.
// Climate endpoint advertises the standard Temperature Sensor device type and exposes
// Relative Humidity Measurement as an additional server cluster on the same endpoint.
// No second Humidity Sensor device type is advertised: Matter 1.5 forbids two unrelated
// Simple Application device types (Temperature Sensor + Humidity Sensor) on one endpoint.
// Matter QR commissions this single node once; the node exposes four endpoints.
// Preserves Smart Light Matter/MQTT/Web/Wi-Fi features; adds TFT, AM2302 and weather
// ============================================================

// ---------- Logging: 1 = enabled, 0 = disabled ----------
#define LOG_BOOT       1
#define LOG_WIFI       1
#define LOG_MATTER     1
#define LOG_BUTTON     1
#define LOG_EFFECT     1
#define LOG_WEB        1
#define LOG_RESET      1
#define LOG_STATUS     1

// Diagnostic ring buffer is allocated only while web diagnostics are explicitly enabled.
static constexpr uint8_t WEB_LOG_CAPACITY = 40;
static constexpr size_t WEB_LOG_LINE_SIZE = 160;
static char (*webLogLines)[WEB_LOG_LINE_SIZE] = nullptr;
static volatile bool webLogEnabled = false;
static uint32_t webLogWriteIndex = 0;
static portMUX_TYPE webLogMux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t previousEspLogVprintf = nullptr;
static DNSServer captiveDns;
static bool captiveDnsStarted = false;

void pushWebLog(const char *line) {
  if (!line) return;
  portENTER_CRITICAL(&webLogMux);
  if (!webLogEnabled || !webLogLines) { portEXIT_CRITICAL(&webLogMux); return; }
  const uint32_t slot = webLogWriteIndex % WEB_LOG_CAPACITY;
  strlcpy(webLogLines[slot], line, WEB_LOG_LINE_SIZE);
  webLogWriteIndex++;
  portEXIT_CRITICAL(&webLogMux);
}

void logPrintf(const char *tag, const char *fmt, ...) {
  char body[WEB_LOG_LINE_SIZE - 32];
  va_list args;
  va_start(args, fmt);
  vsnprintf(body, sizeof(body), fmt, args);
  va_end(args);
  char line[WEB_LOG_LINE_SIZE];
  snprintf(line, sizeof(line), "[%s] %s", tag, body);
  Serial.println(line);
  pushWebLog(line);
}

int webEspLogVprintf(const char *format, va_list args) {
  if (webLogEnabled) {
    va_list copy;
    va_copy(copy, args);
    char line[WEB_LOG_LINE_SIZE];
    vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    pushWebLog(line);
  }
  if (previousEspLogVprintf) return previousEspLogVprintf(format, args);
  return vprintf(format, args);
}

#define LOG_PRINT(tag, enabled, ...) do { if (enabled) logPrintf(tag, __VA_ARGS__); } while (0)
#define LOG_B(...)   LOG_PRINT("BOOT", LOG_BOOT, __VA_ARGS__)
#define LOG_W(...)   LOG_PRINT("WIFI", LOG_WIFI, __VA_ARGS__)
#define LOG_M(...)   LOG_PRINT("MATTER", LOG_MATTER, __VA_ARGS__)
#define LOG_BTN(...) LOG_PRINT("BUTTON", LOG_BUTTON, __VA_ARGS__)
#define LOG_E(...)   LOG_PRINT("EFFECT", LOG_EFFECT, __VA_ARGS__)
#define LOG_H(...)   LOG_PRINT("WEB", LOG_WEB, __VA_ARGS__)
#define LOG_R(...)   LOG_PRINT("RESET", LOG_RESET, __VA_ARGS__)
#define LOG_S(...)   LOG_PRINT("STATUS", LOG_STATUS, __VA_ARGS__)
#define LOG_ERR(...) LOG_PRINT("ERROR", 1, __VA_ARGS__)

// ---------- Hardware ----------
#ifndef BOOT_PIN
#define BOOT_PIN 0
#endif
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
static constexpr uint8_t LED_PIN = LED_BUILTIN;
static constexpr bool LED_ACTIVE_HIGH = true; // Set false if your LED is active-low.

// ---------- GoodWin Smart Hub hardware ----------
// Display 1: 1.8-inch SPI TFT 128x160, ST7735-compatible profile.
// Controller/profile must be confirmed on the physical module.
static constexpr int TFT1_SCK = 18;
static constexpr int TFT1_MOSI = 23;
static constexpr int TFT1_CS = 13;
static constexpr int TFT1_DC = 14;
static constexpr int TFT1_RST = 26;
static constexpr int TFT1_BL = 25;
// Display 2 is reserved now; leave physically disconnected and uninitialized.
static constexpr int TFT2_CS = 32;
static constexpr int TFT2_BL = 33;
static constexpr bool TFT2_ENABLED = false;
static constexpr int DHT_PIN = 27;
static constexpr uint8_t DHT_TYPE = DHT22; // AM2302
static constexpr uint32_t SENSOR_INTERVAL_MS = 2500;
static constexpr uint32_t WEATHER_INTERVAL_MS = 5UL * 60UL * 1000UL; // Refresh every 5 minutes
static constexpr float WEATHER_LAT = 56.807171f;
static constexpr float WEATHER_LON = 60.632042f;
static constexpr uint8_t TFT_BACKLIGHT_PERCENT = 85;

// LovyanGFX hardware driver: ST7735S over VSPI with DMA.
// This preserves the existing hubTFT drawing API while replacing the Adafruit driver.
class GoodWinTFT : public lgfx::LGFX_Device {
  lgfx::Bus_SPI _bus;
  lgfx::Panel_ST7735S _panel;
public:
  GoodWinTFT() {
    {
      auto cfg = _bus.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 20000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = 1;
      cfg.pin_sclk = TFT1_SCK;
      cfg.pin_mosi = TFT1_MOSI;
      cfg.pin_miso = -1;
      cfg.pin_dc = TFT1_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs = TFT1_CS;
      cfg.pin_rst = TFT1_RST;
      cfg.pin_busy = -1;
      cfg.panel_width = 128;
      cfg.panel_height = 160;
      cfg.memory_width = 128;
      cfg.memory_height = 160;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.readable = false;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};
GoodWinTFT hubTFT;
// Small native LovyanGFX RGB565 sprite for dynamic value patches: 114x26 = 5.8 KB.
// The full-screen 128x160 framebuffer would require 40 KB.
static constexpr int16_t HUB_VALUE_CANVAS_W = 114;
static constexpr int16_t HUB_VALUE_CANVAS_H = 26;
static lgfx::LGFX_Sprite hubValueCanvas(&hubTFT);
// Shared native LovyanGFX sprite: each saver is composed as a complete RGB565
// scene and transferred with pushSprite/DMA to eliminate clear-then-redraw flicker.
static lgfx::LGFX_Sprite *hubSaverCanvas = nullptr;
static int16_t hubSaverCanvasW = 0;
static int16_t hubSaverCanvasH = 0;
static volatile bool hubDisplayDirty = true; // full redraw: boot, layout or structural change
static volatile bool hubClockDirty = false;
static volatile bool hubSensorDirty = false;
static volatile bool hubWeatherDirty = false;
static volatile bool hubLightDirty = false;
DHT hubDHT(DHT_PIN, DHT_TYPE);
float indoorTempC = NAN;
float indoorHumidity = NAN;
float outdoorTempC = NAN;
float outdoorHumidity = NAN;
float outdoorPrecipMm = NAN;
float outdoorFeelsLikeC = NAN;
float outdoorPressureHpa = NAN;
float outdoorWindSpeedKmh = NAN;
float outdoorWindDirection = NAN;
float outdoorCloudCover = NAN;
float outdoorUvIndex = NAN;

static bool changedAtTenth(float a, float b) {
  if (isnan(a) || isnan(b)) return isnan(a) != isnan(b);
  return lroundf(a * 10.0f) != lroundf(b * 10.0f);
}
int outdoorPrecipProbability = -1;
int outdoorWeatherCode = -1;
bool weatherValid = false;
bool sensorValid = false;
uint32_t lastSensorReadAt = 0;
uint32_t lastTftDrawAt = 0;
uint8_t hubLayoutId = 0;
uint8_t hubSaverId = 0;
uint8_t hubOrientation = 0; // ST7735 rotation: 0..3 (portrait, landscape, inverted portrait, inverted landscape)
static constexpr uint8_t HUB_SAVER_COUNT = 22;
static uint16_t hubSaverIdleSeconds = 60;
static constexpr uint32_t HUB_SAVER_FRAME_MS = 40;
static constexpr uint32_t HUB_SAVER_COMPLEX_FRAME_MS = 100;
static uint32_t hubLastActivityAt = 0;
static uint32_t hubSaverLastFrameAt = 0;
static bool hubSaverActive = false;
static uint32_t hubSaverLastValueAt = 0;
static int16_t saverDotX = 0, saverDotY = 0, saverDotVX = 2, saverDotVY = 1;
static uint8_t saverDotRadius = 5;
static uint8_t saverStarX[18], saverStarY[18], saverStarPhase[18];
static uint8_t saverRainX[10], saverRainY[10];
static uint8_t saverSnowX[10], saverSnowY[10];
static uint8_t saverMatrixY[16];
static uint8_t saverMatrixSpeed[16];
static int16_t saverCloudX[2] = { -36, 44 };
static uint8_t saverPulseRadius = 18;
static int8_t saverPulseDirection = 1;
static float saverLastTemperature = NAN;
static float saverLastHumidity = NAN;
static constexpr uint8_t HUB_LAYOUT_COUNT = 42;
static bool hubForceFullRedraw = true;
bool hubDisplayEnabled = true;
bool displayMatterOn = true;
uint8_t displayBrightnessPercent = 85;
static bool displayPwmReady = false;
static bool displayPrefsPending = false;
static uint32_t displayPrefsChangedAt = 0;
static float lastMatterTemperature = NAN;
static float lastMatterHumidity = NAN;
static char weatherStatus[64] = "Ожидание";
static char weatherProvider[24] = "—";
static portMUX_TYPE weatherTextMux = portMUX_INITIALIZER_UNLOCKED;

static void setWeatherText(const char *status, const char *provider = nullptr) {
  bool changed = false;
  portENTER_CRITICAL(&weatherTextMux);
  if (status && strcmp(weatherStatus, status) != 0) {
    strlcpy(weatherStatus, status, sizeof(weatherStatus));
    changed = true;
  }
  if (provider && strcmp(weatherProvider, provider) != 0) {
    strlcpy(weatherProvider, provider, sizeof(weatherProvider));
    changed = true;
  }
  portEXIT_CRITICAL(&weatherTextMux);
  if (changed) hubWeatherDirty = true;
}

static void copyWeatherText(char *statusOut, size_t statusSize, char *providerOut, size_t providerSize) {
  portENTER_CRITICAL(&weatherTextMux);
  if (statusOut && statusSize) strlcpy(statusOut, weatherStatus, statusSize);
  if (providerOut && providerSize) strlcpy(providerOut, weatherProvider, providerSize);
  portEXIT_CRITICAL(&weatherTextMux);
}
static volatile uint8_t weatherSourceMode = 0; // 0=Auto, 1=Open-Meteo, 2=wttr.in, 3=MET Norway
static uint32_t weatherOpenMeteoLastAttemptAt = 0;
static uint32_t weatherWttrLastAttemptAt = 0;
static uint32_t weatherMetNoLastAttemptAt = 0;
static constexpr uint32_t WEATHER_WTTR_COOLDOWN_MS = 10UL * 60UL * 1000UL;
static constexpr uint32_t WEATHER_METNO_COOLDOWN_MS = 30UL * 60UL * 1000UL;
static uint8_t hubTestFpsTarget = 20;
static uint32_t hubTestLastFrameAt = 0;
static uint32_t hubTestFpsWindowAt = 0;
static uint32_t hubTestFrameCount = 0;
static uint32_t hubTestFrameNumber = 0;
static uint16_t hubTestMeasuredFps = 0;
static uint8_t hubTestPhase = 0;
static uint32_t hubTestPhaseAt = 0;
static uint32_t hubTestDrawUsLast = 0, hubTestDrawUsAvg = 0, hubTestDrawUsMax = 0;
static int16_t hubTestPreviousX = -1;
static bool hubTestFullToggle = false;
static bool hubProgressNeedsInit = true;
String timeText = "--:--";
String dateText = "--.--.----";
portMUX_TYPE hubDataMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t weatherTaskHandle = nullptr;
static const char *HUB_LAYOUT_NAMES[] = {
  "Dashboard", "Часы крупно", "Погода крупно", "Домашний климат", "Дом + улица", "Состояние света",
  "Aurora — погода PRO", "Тест TFT · FPS", "Orbit — neon telemetry", "Terminal — monochrome", "Dark Tech", "Amber", "Glass UI", "Cyber",
  "Компактный 1", "Компактный 2", "Крупные цифры 1", "Крупные цифры 2", "Погода карточками", "Климат карточками",
  "Свет + климат", "Свет + погода", "Системный статус", "Wi-Fi статус", "Matter статус", "MQTT статус",
  "Вертикальный 1", "Вертикальный 2", "Вертикальный 3", "Ночной", "Контрастный", "Монохром",
  "Погода тёмная", "Погода светлая", "Часы аналоговые", "Часы цифровые", "Инфо 1", "Инфо 2",
  "Инфо 3", "Инфо 4", "Пустой экран", "Тестовый"
};
static const char *HUB_SAVER_NAMES[22] = {
  "Нет", "Часы", "Плавающая точка", "Звёзды", "Дождь", "Снег", "Облака", "Температура",
  "Волна", "Пульс", "Северное сияние", "DNA · Double Helix", "Туманность", "Матрица",
  "Кометные следы", "Светлячки", "Радар", "Планетарные орбиты", "Эквалайзер", "Призматический тоннель",
  "Фрактальный цветок", "Звёздный разгон"
};

// ---------- Device identity (set before Matter.begin) ----------
static const char *DEVICE_NAME = "GoodWin Smart Hub";
static const char *DEVICE_VENDOR = "GoodWin";
static const char *DEVICE_PRODUCT = "GoodWin Smart Hub";
static const char *DEVICE_SERIAL = "GW-ESP32-001";

// ---------- Wi-Fi setup AP ----------
static const char *AP_NAME = "ESP-SETUP";
static const char *AP_PASSWORD = "esp32setup"; // Setup AP password, not home Wi-Fi.
static const char *MDNS_HOSTNAME = "goodwin-hub";
static const char *WIFI_HOSTNAME = "goodwin-smart-hub"; // DHCP hostname shown by most routers
// CHANGE THIS before enabling OTA on a real network.
static const char *OTA_PASSWORD = "CHANGE_ME_GW_OTA_2026";

// ---------- Home Assistant MQTT ----------
// MQTT settings are configured in the web interface and stored in Preferences.
static constexpr uint16_t MQTT_DEFAULT_PORT = 1883;
static const char *MQTT_DEFAULT_ROOT = "goodwin/GW-ESP32-001";

// ---------- Global objects ----------
MatterDimmableLight DimmableLight;       // GoodWin Lamp
MatterDimmableLight DisplayDimmer;        // GoodWin Display

// One Matter node exposes three application endpoints: lamp, display and climate.
// Climate is created natively so Temperature Measurement and Relative Humidity
// Measurement clusters can live on the same endpoint.
static uint16_t climateEndpointId = 0;
static bool climateEndpointReady = false;
Preferences netPrefs;
Preferences statePrefs;
WebServer server(80);
WiFiClient mqttNetwork;
PubSubClient mqttClient(mqttNetwork);

String wifiSSID;
String wifiPassword;
String pairingCode;
String pairingQR;

// MQTT runtime configuration, persisted in NVS.
String mqttHost, mqttUser, mqttPassword;
String mqttRoot = MQTT_DEFAULT_ROOT;
uint16_t mqttPort = MQTT_DEFAULT_PORT;
bool mqttEnabled = false;
String mqttLastError = "Не настроен";

bool matterStarted = false;
bool portalMode = false;
volatile bool lightOn = false;
volatile bool ledState = false;
volatile bool localMatterUpdate = false;
uint8_t brightnessPercent = 80;
uint16_t effectSpeedPercent = 100; // 25..300%; 100% is the default tempo.
static float smoothedBrightnessPercent = 80.0f;
static uint32_t lastBrightnessSmoothAt = 0;
static bool pwmReady = false;
static constexpr uint8_t PWM_RESOLUTION = 12;
static constexpr uint32_t PWM_FREQUENCY = 5000;
static constexpr uint16_t PWM_MAX = (1U << PWM_RESOLUTION) - 1;
static volatile bool mqttStateDirty = true;
bool resetting = false;
static bool restartPending = false;
static bool pendingFactoryReset = false;
static uint32_t restartAt = 0;
static bool stateDirty = false;
static uint32_t stateDirtyAt = 0;
static uint32_t effectStartedAt = 0;
static bool recoveryAPStarted = false;
static uint32_t lastWifiRetryAt = 0;
static uint32_t wifiConnectedSince = 0;
static bool bootButtonStable = true;
static bool bootButtonLastRaw = true;
static uint32_t bootButtonDebounceAt = 0;
static uint32_t bootButtonPressedAt = 0;
static bool bootButtonPressed = false;
static uint32_t portalLedLastToggle = 0;
static bool portalLedState = false;
static uint32_t lastPortalApRetryAt = 0;
static bool setupAPStarted = false;
static uint32_t portalLastStaAttemptAt = 0;
static uint32_t portalStaAttemptStartedAt = 0;
static bool portalStaAttemptInProgress = false;
static uint32_t portalWifiConnectedSince = 0;
static bool portalMatterStarted = false;

// ---------- Effects ----------
enum Effect : uint8_t {
  FX_SOLID = 0,
  FX_SLOW_BLINK,
  FX_FAST_BLINK,
  FX_PULSE,
  FX_BEACON,
  FX_DOUBLE_FLASH,
  FX_TRIPLE_FLASH,
  FX_STROBE_5HZ,
  FX_STROBE_8HZ,
  FX_STROBE_12HZ,
  FX_HEARTBEAT,
  FX_SOS,
  FX_ALARM,
  FX_CANDLE,
  FX_RANDOM,
  FX_EXTRA_15,
  FX_EXTRA_16,
  FX_EXTRA_17,
  FX_EXTRA_18,
  FX_EXTRA_19,
  FX_EXTRA_20,
  FX_EXTRA_21,
  FX_EXTRA_22,
  FX_EXTRA_23,
  FX_EXTRA_24,
  FX_EXTRA_25,
  FX_EXTRA_26,
  FX_EXTRA_27,
  FX_EXTRA_28,
  FX_EXTRA_29,
  FX_EXTRA_30,
  FX_EXTRA_31,
  FX_EXTRA_32,
  FX_EXTRA_33,
  FX_EXTRA_34,
  FX_EXTRA_35,
  FX_COUNT
};

struct EffectInfo {
  const char *name;
  const char *description;
  const char *icon;
};

const EffectInfo effects[FX_COUNT] = {
  {"Постоянный свет", "Непрерывное свечение", "☀"},
  {"Медленный импульс", "Длинный цикл включения и паузы", "◉"},
  {"Быстрое мигание", "Равномерные короткие вспышки", "ϟ"},
  {"Пульсация", "Короткая вспышка с длинной паузой", "♥"},
  {"Маяк", "Редкий сигнальный импульс", "◈"},
  {"Двойная вспышка", "Два импульса подряд", "✦"},
  {"Тройная вспышка", "Три импульса подряд", "✧"},
  {"Стробоскоп 5 Гц", "Пять циклов в секунду", "⚡"},
  {"Стробоскоп 8 Гц", "Восемь циклов в секунду", "⚡"},
  {"Стробоскоп 12 Гц", "Двенадцать циклов в секунду", "⚡"},
  {"Сердцебиение", "Двойной ритмичный импульс", "♡"},
  {"SOS", "Световой сигнал азбукой Морзе", "⌁"},
  {"Тревога", "Повторяющийся тревожный сигнал", "◉"},
  {"Свеча", "Неровное мерцание", "♨"},
  {"Случайный", "Случайные интервалы и состояния", "✳"},
  {"Маяк дальний", "Один яркий сигнал и длинная пауза", "◉"},
  {"Маяк двойной", "Две группы сигнальных импульсов", "◎"},
  {"Метроном", "Равномерный ритм с акцентом", "⌁"},
  {"Сканер", "Серия коротких импульсов с возвратом", "▥"},
  {"Пульс медленный", "Спокойный редкий пульс", "♡"},
  {"Пульс быстрый", "Ускоренный двойной пульс", "♥"},
  {"Уведомление", "Короткий двойной сигнал", "✉"},
  {"Входящий сигнал", "Три коротких импульса и пауза", "↗"},
  {"Выходящий сигнал", "Длинный импульс и два коротких", "↘"},
  {"Код Морзе: E", "Один короткий импульс", "·"},
  {"Код Морзе: T", "Один длинный импульс", "—"},
  {"Код Морзе: OK", "Сигнал OK азбукой Морзе", "✓"},
  {"Четыре вспышки", "Четыре коротких импульса", "✣"},
  {"Пять вспышек", "Пять коротких импульсов", "✥"},
  {"Серия 3+1", "Три быстрых и один длинный", "⚑"},
  {"Серия 1+3", "Один длинный и три быстрых", "⚐"},
  {"Ритм сирены", "Чередование коротких и длинных сигналов", "🚨"},
  {"Костёр", "Живое нерегулярное мерцание", "🔥"},
  {"Светлячок", "Редкие одиночные вспышки", "✦"},
  {"Звёздный дождь", "Нерегулярные короткие вспышки", "✧"},
  {"Тихий режим", "Очень редкие мягкие сигналы", "☾"}
};

uint8_t currentEffect = FX_SOLID;
uint32_t randomTimer = 0;
uint32_t randomInterval = 150;

// ---------- Function prototypes ----------
void factoryReset();
void startPortal();
bool startAccessPoint(bool recoveryMode);
void updatePortalIndicator();
void handleLogs();
void handleLogsClear();
void handleLogsStart();
void handleLogsStop();
void setupWebServer();
void startMatter();
void updateEffects();
void setEffect(uint8_t effect);
void setPower(bool state);
void nextEffect();
bool setLightState(bool state, uint8_t matterBrightness);
bool setDisplayState(bool state, uint8_t matterBrightness);
void writeDisplayBacklight();
void setBrightness(uint8_t percent);
uint8_t percentToMatter(uint8_t percent);
void handleRoot();
void handleApi();
void handleSystemMetrics();
void handleWeatherRefresh();
void handleWeatherSource();
void handleScanStart();
void handleScanResults();
void handleOn();
void handleOff();
void handleBrightness();
void handleEffect();
void handleWiFiSave();
void handleReset();
void handleWiFiReset();
void processBootButton();
void processWiFiRecovery();
bool connectSavedWiFi();
void setupMqtt();
void mqttLoop();
bool mqttConnectNow();
void loadMqttConfig();
void handleMqttConfigGet();
void handleMqttSave();
void handleMqttTest();
void publishMqttState();
void publishMqttDiscovery();
void publishMqttTelemetry();
void setEffectSpeed(uint16_t value);
void handleSpeed();
void setupSmartHubDisplay();
void updateSmartHub();
void drawSmartHub();
static void hubInvalidateValueCache();
void pushChangedTftFrame();
void updateHubClock();
void startWeatherTask();
void weatherTask(void *parameter);
void handleHubConfig();
void handleHubSaverStart();
void handleHubData();
void hubMarkActivity();

// ============================================================
// LED PWM + persistent state
// ============================================================
uint16_t gammaDuty(uint8_t percent) {
  if (percent == 0) return 0;
  if (percent > 100) percent = 100;
  // Gamma compensation makes the slider feel more linear to the eye.
  const float normalized = (float)percent / 100.0f;
  uint16_t duty = (uint16_t)lroundf(powf(normalized, 2.2f) * PWM_MAX);
  if (duty == 0) duty = 1;
  return duty;
}

void writeLEDLevel(uint8_t levelPercent) {
  if (levelPercent > 100) levelPercent = 100;
  ledState = levelPercent > 0;
  uint16_t duty = gammaDuty(levelPercent);
  if (!LED_ACTIVE_HIGH) duty = PWM_MAX - duty;
  if (pwmReady) ledcWrite(LED_PIN, duty);
  else digitalWrite(LED_PIN, (levelPercent > 0) == LED_ACTIVE_HIGH ? HIGH : LOW);
}

void writeLED(bool state) {
  writeLEDLevel(state ? getSmoothedBrightness() : 0);
}

uint8_t percentToMatter(uint8_t percent) {
  if (percent < 1) percent = 1;
  if (percent > 100) percent = 100;
  return (uint8_t)map(percent, 1, 100, 1, 254);
}

uint8_t matterToPercent(uint8_t value) {
  if (value < 1) value = 1;
  if (value > 254) value = 254;
  return (uint8_t)constrain((int)lroundf((float)value * 100.0f / 254.0f), 1, 100);
}

uint8_t getSmoothedBrightness() {
  return (uint8_t)constrain((int)lroundf(smoothedBrightnessPercent), 1, 100);
}

void updateBrightnessSmoothing() {
  const uint32_t now = millis();
  if (lastBrightnessSmoothAt == 0) lastBrightnessSmoothAt = now;
  const uint32_t dt = now - lastBrightnessSmoothAt;
  lastBrightnessSmoothAt = now;
  if (dt == 0) return;
  // Fast exponential easing (~18 ms). Matter reports the requested level immediately;
  // this short ramp only smooths the physical PWM output and avoids visible lag.
  const float alpha = 1.0f - expf(-(float)dt / 18.0f);
  smoothedBrightnessPercent += ((float)brightnessPercent - smoothedBrightnessPercent) * alpha;
  if (fabsf(smoothedBrightnessPercent - brightnessPercent) < 0.15f)
    smoothedBrightnessPercent = brightnessPercent;
}

void markStateDirty() {
  stateDirty = true;
  stateDirtyAt = millis();
}

void saveStateIfNeeded(bool force = false) {
  if (!stateDirty) return;
  if (!force && millis() - stateDirtyAt < 700) return;
  if (!statePrefs.begin("ESPstate", false)) {
    LOG_ERR("Cannot open state NVS namespace");
    return;
  }
  statePrefs.putBool("on", lightOn);
  statePrefs.putUChar("effect", currentEffect);
  statePrefs.putUChar("brightness", brightnessPercent);
  statePrefs.putUShort("speed", effectSpeedPercent);
  statePrefs.end();
  stateDirty = false;
}

// ============================================================
// Matter callbacks + power/effect control
// ============================================================
bool setLightState(bool state, uint8_t matterBrightness) {
  hubMarkActivity();
  const uint8_t requestedBrightness = matterToPercent(matterBrightness);
  const bool controllerChangedPower = (state != lightOn);
  const bool changed = controllerChangedPower || requestedBrightness != brightnessPercent;
  if (!changed) return true; // Ignore duplicate Matter reports; do not churn NVS/MQTT/logs.

  lightOn = state;
  brightnessPercent = requestedBrightness;
  hubLightDirty = true;

  // A Matter On/Off command selects steady light. A Matter brightness
  // command only changes the master level and preserves the local effect.
  if (!localMatterUpdate && controllerChangedPower) {
    currentEffect = FX_SOLID;
    effectStartedAt = millis();
  }

  markStateDirty();
  LOG_M("Controller state=%s brightness=%u%% effect=%s",
        state ? "ON" : "OFF", brightnessPercent,
        effects[currentEffect].name);
  publishMqttState();
  return true;
}

// Matter callback for the TFT backlight. GPIO25 must drive a suitable
// MOSFET/transistor stage; never connect the TFT LED pin directly to GPIO25.
bool setDisplayState(bool state, uint8_t matterBrightness) {
  const uint8_t requestedBrightness = matterToPercent(matterBrightness);
  const bool powerChanged = (displayMatterOn != state) || (hubDisplayEnabled != state);
  const bool brightnessChanged = (displayBrightnessPercent != requestedBrightness);
  if (!powerChanged && !brightnessChanged) return true;

  hubMarkActivity();
  displayMatterOn = state;
  hubDisplayEnabled = state;
  displayBrightnessPercent = requestedBrightness;

  // Backlight brightness is a hardware PWM change, not a reason to repaint pixels.
  // Only an actual display power transition requires a full redraw.
  if (powerChanged) hubDisplayDirty = true;
  writeDisplayBacklight();

  // Defer NVS writes until the Matter slider has been idle for 1.5 seconds.
  // This avoids writing flash for every 1% slider step.
  displayPrefsPending = true;
  displayPrefsChangedAt = millis();

  LOG_M("Display changed: state=%s brightness=%u%%",
        state ? "ON" : "OFF", displayBrightnessPercent);
  return true;
}

void writeDisplayBacklight() {
  const uint8_t level = displayMatterOn ? displayBrightnessPercent : 0;
  const uint8_t duty = (uint8_t)lroundf(powf((float)level / 100.0f, 2.2f) * 255.0f);
  if (displayPwmReady) ledcWrite(TFT1_BL, duty);
  else digitalWrite(TFT1_BL, level ? HIGH : LOW);
}

void setBrightness(uint8_t percent) {
  hubMarkActivity();
  if (percent < 1) percent = 1;
  if (percent > 100) percent = 100;
  brightnessPercent = percent;
  hubLightDirty = true;
  if (matterStarted) {
    localMatterUpdate = true;
    DimmableLight.setBrightness(percentToMatter(percent));
    localMatterUpdate = false;
  }
  markStateDirty();
  LOG_E("Brightness=%u%%", brightnessPercent);
  publishMqttState();
}

void setPower(bool state) {
  hubMarkActivity();
  hubLightDirty = true;
  if (matterStarted) {
    localMatterUpdate = true;
    DimmableLight.setOnOff(state);
    localMatterUpdate = false;
  }
  lightOn = state;
  currentEffect = FX_SOLID;
  effectStartedAt = millis();
  if (!state) writeLEDLevel(0);
  markStateDirty();
  LOG_E("Power=%s", state ? "ON" : "OFF");
  publishMqttState();
}

void setEffect(uint8_t effect) {
  hubMarkActivity();
  if (effect >= FX_COUNT) return;

  if (!lightOn && matterStarted) {
    localMatterUpdate = true;
    DimmableLight.setOnOff(true);
    localMatterUpdate = false;
  }

  lightOn = true;
  hubLightDirty = true;
  currentEffect = effect;
  effectStartedAt = millis();
  randomTimer = effectStartedAt;
  randomInterval = max(10UL, (uint32_t)random(60, 350) * 100UL / max((uint16_t)25, effectSpeedPercent));
  markStateDirty();
  LOG_E("Selected: %s (ID=%u)", effects[currentEffect].name, currentEffect);
  publishMqttState();
}

void nextEffect() {
  setEffect((currentEffect + 1) % FX_COUNT);
}

void setEffectSpeed(uint16_t value) {
  hubMarkActivity();
  effectSpeedPercent = constrain(value, 25, 300);
  effectStartedAt = millis();
  randomTimer = effectStartedAt;
  markStateDirty();
  LOG_E("Effect speed=%u%%", effectSpeedPercent);
  publishMqttState();
}

// ============================================================
// Non-blocking pattern engine
// ============================================================
void runPattern(const uint16_t *durations, const bool *states, uint8_t count) {
  uint32_t total = 0;
  for (uint8_t i = 0; i < count; ++i)
    total += max(10UL, (uint32_t)durations[i] * 100UL / max((uint16_t)25, effectSpeedPercent));
  if (total == 0) { writeLEDLevel(0); return; }

  uint32_t phase = (millis() - effectStartedAt) % total;
  for (uint8_t i = 0; i < count; ++i) {
    const uint32_t segment = max(10UL, (uint32_t)durations[i] * 100UL / max((uint16_t)25, effectSpeedPercent));
    if (phase < segment) {
      if (!states[i]) { writeLEDLevel(0); return; }
      // Smooth 25 ms fade at both edges, while preserving sharp signal timing.
      const uint16_t fadeMs = 25;
      float envelope = 1.0f;
      if (phase < fadeMs) envelope = (float)phase / fadeMs;
      const uint32_t remaining = segment - phase;
      if (remaining < fadeMs) {
        const float fadeOut = (float)remaining / fadeMs;
        if (fadeOut < envelope) envelope = fadeOut;
      }
      if (envelope < 0.0f) envelope = 0.0f;
      writeLEDLevel((uint8_t)lroundf(getSmoothedBrightness() * envelope));
      return;
    }
    phase -= segment;
  }
  writeLEDLevel(0);
}

void updateEffects() {
  updateBrightnessSmoothing();
  if (!lightOn) { writeLEDLevel(0); return; }
  const uint32_t now = millis();

  switch (currentEffect) {
    case FX_SOLID:
      writeLEDLevel(getSmoothedBrightness());
      break;
    case FX_PULSE: {
      // Gentle breathing pulse, never fully extinguishes.
      const uint32_t pulseCycle = max(200UL, 2600UL * 100UL / max((uint16_t)25, effectSpeedPercent));
      const float phase = (float)((now - effectStartedAt) % pulseCycle) / pulseCycle;
      const float wave = (1.0f - cosf(phase * 2.0f * PI)) * 0.5f;
      const float level = 7.0f + wave * 93.0f;
      writeLEDLevel((uint8_t)lroundf(getSmoothedBrightness() * level / 100.0f));
      break;
    }
    case FX_SLOW_BLINK: { const uint16_t d[] = {900, 1100}; const bool s[] = {true, false}; runPattern(d, s, 2); break; }
    case FX_FAST_BLINK: { const uint16_t d[] = {200, 200}; const bool s[] = {true, false}; runPattern(d, s, 2); break; }
    case FX_BEACON: { const uint16_t d[] = {120, 1880}; const bool s[] = {true, false}; runPattern(d, s, 2); break; }
    case FX_DOUBLE_FLASH: { const uint16_t d[] = {100,100,100,700}; const bool s[] = {true,false,true,false}; runPattern(d,s,4); break; }
    case FX_TRIPLE_FLASH: { const uint16_t d[] = {90,90,90,90,90,650}; const bool s[] = {true,false,true,false,true,false}; runPattern(d,s,6); break; }
    case FX_STROBE_5HZ: { const uint16_t d[] = {100,100}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_STROBE_8HZ: { const uint16_t d[] = {62,63}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_STROBE_12HZ: { const uint16_t d[] = {41,42}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_HEARTBEAT: { const uint16_t d[] = {100,100,100,700}; const bool s[] = {true,false,true,false}; runPattern(d,s,4); break; }
    case FX_SOS: {
      const uint16_t d[] = {100,100,100,100,100,300,300,100,300,100,300,300,100,100,100,100,100,700};
      const bool s[] = {true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,false};
      runPattern(d,s,18); break;
    }
    case FX_ALARM: { const uint16_t d[] = {250,250,250,1250}; const bool s[] = {true,false,true,false}; runPattern(d,s,4); break; }
    case FX_EXTRA_15: { const uint16_t d[] = {180,1820}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_EXTRA_16: { const uint16_t d[] = {100,100,100,100,100,1100}; const bool s[] = {true,false,true,false,true,false}; runPattern(d,s,6); break; }
    case FX_EXTRA_17: { const uint16_t d[] = {90,910}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_EXTRA_18: { const uint16_t d[] = {70,70,70,70,70,650}; const bool s[] = {true,false,true,false,true,false}; runPattern(d,s,6); break; }
    case FX_EXTRA_19: { const uint16_t d[] = {180,1320}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_EXTRA_20: { const uint16_t d[] = {80,100,80,740}; const bool s[] = {true,false,true,false}; runPattern(d,s,4); break; }
    case FX_EXTRA_21: { const uint16_t d[] = {100,100,100,700}; const bool s[] = {true,false,true,false}; runPattern(d,s,4); break; }
    case FX_EXTRA_22: { const uint16_t d[] = {70,100,70,100,70,790}; const bool s[] = {true,false,true,false,true,false}; runPattern(d,s,6); break; }
    case FX_EXTRA_23: { const uint16_t d[] = {450,150,100,150,100,550}; const bool s[] = {true,false,true,false,true,false}; runPattern(d,s,6); break; }
    case FX_EXTRA_24: { const uint16_t d[] = {120,880}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_EXTRA_25: { const uint16_t d[] = {500,1500}; const bool s[] = {true,false}; runPattern(d,s,2); break; }
    case FX_EXTRA_26: { const uint16_t d[] = {100,100,100,100,300,700}; const bool s[] = {true,false,true,false,true,false}; runPattern(d,s,6); break; }
    case FX_EXTRA_27: { const uint16_t d[] = {80,80,80,80,80,80,80,80,80,500}; const bool s[] = {true,false,true,false,true,false,true,false,true,false}; runPattern(d,s,10); break; }
    case FX_EXTRA_28: { const uint16_t d[] = {70,100,70,100,70,100,250,700}; const bool s[] = {true,false,true,false,true,false,true,false}; runPattern(d,s,8); break; }
    case FX_EXTRA_29: { const uint16_t d[] = {250,100,70,100,70,100,70,500}; const bool s[] = {true,false,true,false,true,false,true,false}; runPattern(d,s,8); break; }
    case FX_EXTRA_30: { const uint16_t d[] = {350,100,70,100,70,100,70,500}; const bool s[] = {true,false,true,false,true,false,true,false}; runPattern(d,s,8); break; }
    case FX_EXTRA_31: { const uint16_t d[] = {90,110,350,110,90,860}; const bool s[] = {true,false,true,false,true,false}; runPattern(d,s,6); break; }
    case FX_EXTRA_32: // Campfire: lively but bounded flicker.
    case FX_CANDLE:
      if (now - randomTimer >= randomInterval) {
        randomTimer = now;
        randomInterval = max(10UL, (uint32_t)random(35, 125) * 100UL / max((uint16_t)25, effectSpeedPercent));
        const int baseLevel = getSmoothedBrightness();
        int variation = currentEffect == FX_CANDLE ? random(-18, 12) : random(-28, 8);
        int level = constrain(baseLevel + variation, 5, 100);
        if (random(0, 100) < 7) level = random(4, max(5, baseLevel / 2));
        writeLEDLevel((uint8_t)level);
      }
      break;
    case FX_EXTRA_33: // Firefly
    case FX_EXTRA_34: // Starfall
    case FX_EXTRA_35: // Quiet mode
    case FX_RANDOM:
      if (now - randomTimer >= randomInterval) {
        randomTimer = now;
        if (currentEffect == FX_EXTRA_33) {
          randomInterval = max(30UL, (uint32_t)random(500, 2600) * 100UL / max((uint16_t)25, effectSpeedPercent));
          writeLEDLevel(random(0, 100) > 82 ? (uint8_t)random(max(1, (int)getSmoothedBrightness() / 3), getSmoothedBrightness() + 1) : 0);
        } else if (currentEffect == FX_EXTRA_34) {
          randomInterval = max(20UL, (uint32_t)random(90, 900) * 100UL / max((uint16_t)25, effectSpeedPercent));
          writeLEDLevel(random(0, 100) > 65 ? (uint8_t)random(max(1, (int)getSmoothedBrightness() / 2), getSmoothedBrightness() + 1) : 0);
        } else if (currentEffect == FX_EXTRA_35) {
          randomInterval = max(100UL, (uint32_t)random(2200, 5200) * 100UL / max((uint16_t)25, effectSpeedPercent));
          writeLEDLevel(random(0, 100) > 70 ? (uint8_t)max(1, (int)getSmoothedBrightness() / 4) : 0);
        } else {
          randomInterval = max(10UL, (uint32_t)random(60, 350) * 100UL / max((uint16_t)25, effectSpeedPercent));
          writeLEDLevel(random(0, 100) > 50 ? (uint8_t)random(1, getSmoothedBrightness() + 1) : 0);
        }
      }
      break;
    default: writeLEDLevel(0); break;
  }
}

// ============================================================
// Web interface
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#080b14">
<link rel="icon" type="image/svg+xml" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E%3Cdefs%3E%3ClinearGradient id='g' x1='0' y1='0' x2='1' y2='1'%3E%3Cstop stop-color='%237ca3ff'/%3E%3Cstop offset='1' stop-color='%233156d9'/%3E%3C/linearGradient%3E%3C/defs%3E%3Crect width='64' height='64' rx='18' fill='url(%23g)'/%3E%3Cpath d='M37 7 16 35h14l-3 22 22-31H34z' fill='white'/%3E%3C/svg%3E">
<link rel="apple-touch-icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E%3Crect width='64' height='64' rx='18' fill='%233f68df'/%3E%3Cpath d='M37 7 16 35h14l-3 22 22-31H34z' fill='white'/%3E%3C/svg%3E">
<title>GoodWin Smart Hub</title>
<style>
:root{color-scheme:dark;--bg:#080b14;--text:#f5f7ff;--muted:#8290a9;--blue:#648bff;--border:#29364b;--green:#36d9a0;--radius:22px}
*{box-sizing:border-box}body{margin:0;min-height:100vh;color:var(--text);font-family:Inter,system-ui,-apple-system,"Segoe UI",sans-serif;background:radial-gradient(ellipse at 15% -10%,#314b8b44,transparent 45%),radial-gradient(ellipse at 95% 30%,#4c347722,transparent 40%),var(--bg)}
button,input{font:inherit}button{cursor:pointer;color:inherit}.app{max-width:920px;margin:auto;padding:24px 18px 55px}.header{display:flex;justify-content:space-between;align-items:center;margin-bottom:25px;gap:12px}.brand{display:flex;align-items:center;gap:13px}.logo{width:48px;height:48px;border-radius:16px;display:grid;place-items:center;font-size:24px;background:linear-gradient(145deg,#7ca3ff,#3156d9);box-shadow:0 8px 35px #527dff55}.brand-name{font-size:19px;font-weight:750;letter-spacing:-.6px}.brand-sub{color:var(--muted);font-size:11px;margin-top:4px;letter-spacing:1px}.connection{display:flex;align-items:center;gap:8px;font-size:11px;color:var(--muted);padding:9px 12px;border:1px solid var(--border);border-radius:30px;background:#111827aa}.dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 12px var(--green)}
.grid-main{display:grid;grid-template-columns:1.15fr .85fr;gap:16px}.card{background:linear-gradient(145deg,#182338ee,#101827ee);border:1px solid #29364b;border-radius:var(--radius);padding:22px;box-shadow:0 18px 55px #0002;backdrop-filter:blur(20px)}.hero{position:relative;overflow:hidden;min-height:280px}.eyebrow{color:#94a8ca;font-size:11px;letter-spacing:2px;text-transform:uppercase}.hero-icon{width:74px;height:74px;margin-top:25px;display:grid;place-items:center;font-size:37px;border-radius:25px;background:#648bff15;border:1px solid #648bff30;transition:.3s}.hero-icon.active{color:#ffdc80;background:#ffdc8015;border-color:#ffdc8055;box-shadow:0 0 45px #ffdc8020;animation:glow 2s infinite alternate}@keyframes glow{to{box-shadow:0 0 40px #ffdc8030}}.hero h1{font-size:29px;margin:18px 0 5px;letter-spacing:-1px}.hero-description{font-size:13px;color:var(--muted)}.power-actions{display:flex;gap:10px;margin-top:22px}.btn{padding:13px 18px;border:1px solid var(--border);border-radius:13px;background:#202c42;font-size:13px;font-weight:700;transition:.18s}.btn:hover{border-color:var(--blue);transform:translateY(-1px)}.btn:active{transform:scale(.97)}.btn-primary{background:linear-gradient(135deg,#7199ff,#4268e8);border:0;box-shadow:0 8px 22px #4268e833}.btn-danger{color:#ff8293}.side-status{display:flex;flex-direction:column;gap:11px}.status-item{background:#0b1321;border:1px solid #28364d;border-radius:15px;padding:15px}.status-label{color:var(--muted);font-size:11px;margin-bottom:8px}.status-value{font-size:15px;font-weight:700;overflow-wrap:anywhere}.section{margin-top:17px}.section-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px;gap:10px}.section-head h2{font-size:17px;margin:0;letter-spacing:-.4px}.section-head span{color:var(--muted);font-size:11px}.effects-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.effect{min-height:108px;padding:14px 10px;text-align:left;border:1px solid #29364c;border-radius:16px;background:linear-gradient(145deg,#1a263b,#121b2b);transition:all .2s;position:relative;overflow:hidden}.effect:hover{transform:translateY(-3px);border-color:#587bd5;background:#202e48}.effect.active{border:1px solid #7397ff;background:linear-gradient(145deg,#293e6c,#182947);box-shadow:0 0 0 1px #7397ff25,0 8px 28px #416bda22}.effect.active:after{content:"";position:absolute;right:10px;top:10px;width:7px;height:7px;background:#75a0ff;border-radius:50%;box-shadow:0 0 12px #75a0ff}.effect-icon{display:block;font-size:23px;margin-bottom:12px}.effect-name{display:block;font-size:12px;font-weight:700;line-height:1.3}.effect-description{display:block;margin-top:5px;color:var(--muted);font-size:10px;line-height:1.35}.pair-code{margin-top:15px;padding:17px;display:flex;align-items:center;justify-content:space-between;gap:12px;background:#0b1423;border:1px solid #354765;border-radius:16px;cursor:pointer;transition:.2s}.pair-code:hover{border-color:#7397ff;background:#14223a}.pair-code strong{color:#a9c4ff;font-size:22px;letter-spacing:3px;overflow-wrap:anywhere}.copy-label{color:#9aaccb;font-size:10px;white-space:nowrap}.pair-note{color:var(--muted);font-size:11px;margin-top:10px}.qr-wrap{display:flex;justify-content:center;margin-top:20px}#qrcode{padding:12px;border-radius:14px;background:white;min-width:1px}#qrcode img{display:block}.wifi-form{display:grid;grid-template-columns:1fr 1fr;gap:10px}.field{width:100%;min-width:0;padding:14px;color:white;background:#0a1220;border:1px solid #2b3b55;border-radius:12px;outline:none}.field:focus{border-color:#7397ff;box-shadow:0 0 0 3px #7397ff18}.full{grid-column:1/-1}.toast{position:fixed;left:50%;bottom:25px;transform:translate(-50%,20px);opacity:0;pointer-events:none;background:#24334c;border:1px solid #52698f;color:white;border-radius:13px;padding:14px 20px;font-size:13px;box-shadow:0 12px 35px #0007;transition:.25s;z-index:100}.toast.show{opacity:1;transform:translate(-50%,0)}.footer{text-align:center;color:#64738c;font-size:10px;letter-spacing:2px;margin-top:28px}.warning{color:#e8bd78;font-size:11px;line-height:1.5;margin-top:12px}
.help details{border:1px solid #2b3b55;border-radius:15px;background:#0b1423;margin:10px 0;overflow:hidden}.help summary{cursor:pointer;padding:15px 16px;font-weight:700;font-size:13px;list-style:none}.help summary::-webkit-details-marker{display:none}.help summary:after{content:"＋";float:right;color:#91adf5}.help details[open] summary:after{content:"−"}.help-body{padding:0 16px 16px;color:#aebbd0;font-size:12px;line-height:1.65}.help-body strong{color:#f2f5ff}.help-body ul{padding-left:19px}.help-body li{margin:5px 0}.help-pill{display:inline-block;padding:3px 8px;border:1px solid #40577c;border-radius:20px;color:#b8ccff;font-size:10px;margin:2px}.help-danger{color:#f1bd83}.help-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.help-box{background:#111d30;border:1px solid #293b57;border-radius:12px;padding:12px}.help-box b{display:block;color:#e8eeff;margin-bottom:5px}.help-box p{margin:0;color:#aebbd0;font-size:11px;line-height:1.5}

/* FUTURISTIC CONTROL UI */
body:before{content:"";position:fixed;inset:0;pointer-events:none;z-index:-1;background:linear-gradient(#6c8cff08 1px,transparent 1px),linear-gradient(90deg,#6c8cff08 1px,transparent 1px);background-size:42px 42px;mask-image:linear-gradient(to bottom,#000,transparent 82%)}
body:after{content:"";position:fixed;width:420px;height:420px;right:-190px;top:20%;border-radius:50%;background:#5a65ff12;filter:blur(70px);pointer-events:none;z-index:-1}
.card{position:relative;border-color:#344766aa;background:linear-gradient(145deg,#1a2941ed,#0d1626ed);box-shadow:0 18px 60px #0005,inset 0 1px #ffffff0b;transition:border-color .25s,transform .25s}
.card:hover{border-color:#648bff77}.hero:before{content:"";position:absolute;width:260px;height:260px;right:-90px;top:-100px;border-radius:50%;border:1px solid #7199ff22;box-shadow:0 0 0 24px #7199ff08,0 0 0 52px #7199ff06;pointer-events:none}
.logo{position:relative;isolation:isolate}.logo:after{content:"";position:absolute;inset:-5px;border-radius:19px;border:1px solid #8baaff55;animation:logoPulse 3s infinite}@keyframes logoPulse{50%{transform:scale(1.12);opacity:.2}}
.btn{border-color:#405575;background:linear-gradient(145deg,#263957,#18263d);box-shadow:inset 0 1px #ffffff0c,0 6px 18px #0002}.btn-primary{background:linear-gradient(110deg,#6c9cff,#7659ef 55%,#4b6ee9);box-shadow:0 8px 25px #527dff44,inset 0 1px #ffffff55}
.effects-grid{grid-template-columns:repeat(6,minmax(0,1fr));gap:9px}.effect{min-height:112px;background:linear-gradient(145deg,#1b2b45,#111b2d);border-color:#304562;box-shadow:inset 0 1px #ffffff08}.effect-icon{filter:drop-shadow(0 0 8px #85aaff66)}.effect.active{background:linear-gradient(145deg,#294c7d,#302b68);border-color:#8bb2ff;box-shadow:0 0 22px #5b8fff2b,inset 0 0 20px #7199ff12}.effect.active .effect-icon{animation:effectFloat 2s ease-in-out infinite alternate}@keyframes effectFloat{to{transform:translateY(-3px);filter:drop-shadow(0 0 12px #9cc1ff)}}
.wifi-current{display:flex;align-items:center;gap:13px;padding:15px;border:1px solid #385273;border-radius:17px;background:linear-gradient(110deg,#172944,#111c30);margin-bottom:12px}.wifi-orb{width:44px;height:44px;display:grid;place-items:center;border-radius:14px;font-size:24px;background:#648bff20;color:#a8c5ff;box-shadow:0 0 25px #648bff20}.wifi-current-text{display:flex;flex:1;min-width:0;flex-direction:column;gap:4px}.wifi-current-text small{font-size:9px;letter-spacing:1.5px;color:#829ac1}.wifi-current-text strong{font-size:14px;overflow-wrap:anywhere}.wifi-current-text span{font-size:10px;color:#8fa2c0}.scan-state{font-size:11px;color:#8fa2c0;padding:9px 2px}.network-list{display:grid;gap:7px;margin:5px 0 14px}.network-item{display:flex;align-items:center;gap:10px;width:100%;text-align:left;padding:12px 13px;border:1px solid #2e425f;border-radius:13px;background:#101d30;transition:.18s}.network-item:hover,.network-item.selected{border-color:#80a9ff;background:#1c3150}.network-item .net-name{flex:1;min-width:0;font-size:12px;font-weight:700;overflow-wrap:anywhere}.network-item .net-meta{font-size:10px;color:#8fa2c0;white-space:nowrap}.network-item .net-lock{color:#91b3ff}.wifi-form{margin-top:8px}.field{background:#0a1424;border-color:#354c6b}.help details{border-color:#344b6c;background:linear-gradient(110deg,#101d30,#0c1626);transition:border-color .2s}.help details[open]{border-color:#719bff88;box-shadow:0 0 22px #547dff14}.help summary{transition:color .2s}.help details[open] summary{color:#b8d0ff}.section-head h2{letter-spacing:-.5px}.connection{border-color:#3b5273;background:#111d30}
@media(max-width:900px){.effects-grid{grid-template-columns:repeat(4,minmax(0,1fr))}}
@media(max-width:700px){.effects-grid{grid-template-columns:repeat(3,minmax(0,1fr))}.wifi-current{flex-wrap:wrap}.wifi-current .btn{margin-left:auto}}
@media(max-width:430px){.effects-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.wifi-current .btn{width:100%}}
@media(max-width:700px){.grid-main{grid-template-columns:1fr}.effects-grid{grid-template-columns:repeat(3,1fr)}}@media(max-width:430px){.app{padding:17px 12px 40px}.effects-grid{grid-template-columns:repeat(2,1fr)}.wifi-form{grid-template-columns:1fr}.full{grid-column:auto}.brand-name{font-size:16px}.card{padding:17px}.hero h1{font-size:25px}}
@media(prefers-reduced-motion:reduce){*,*::before,*::after{animation-duration:.01ms!important;transition-duration:.01ms!important}}

/* GOODWIN / FUTURE INTERFACE LAYER */
:root{--cyan:#62e7ff;--violet:#9b82ff;--line:#39577b}
body{background:radial-gradient(ellipse at 10% 0%,#3559a044,transparent 38%),radial-gradient(ellipse at 90% 24%,#5a39a333,transparent 35%),radial-gradient(ellipse at 50% 100%,#126b7840,transparent 45%),#070b14;background-attachment:fixed}
body:before{content:"";position:fixed;inset:0;pointer-events:none;z-index:-1;background:linear-gradient(#6c8cff0a 1px,transparent 1px),linear-gradient(90deg,#6c8cff0a 1px,transparent 1px);background-size:38px 38px;mask-image:linear-gradient(to bottom,#000,transparent 86%)}
.card{border:1px solid #344d70aa;background:linear-gradient(145deg,#1a2a43ed,#0c1525ef);box-shadow:0 22px 70px #0006,inset 0 1px #ffffff0c;backdrop-filter:blur(22px)}
.logo{background:linear-gradient(135deg,#65e6ff,#6688ff 48%,#9a68ff);box-shadow:0 0 32px #648bff66,0 8px 30px #0005}
.connection{border-color:#426287;background:#10213a;box-shadow:inset 0 0 18px #4e91ff0b}
.hero{isolation:isolate}.hero:after{content:"";position:absolute;z-index:-1;right:-85px;bottom:-145px;width:300px;height:300px;border-radius:50%;border:1px solid #6ee7ff30;box-shadow:0 0 0 22px #6ee7ff08,0 0 0 48px #8b72ff08,0 0 75px #557dff18;animation:radar 12s linear infinite}
@keyframes radar{to{transform:rotate(360deg)}}
.hero-icon.active{box-shadow:0 0 28px #65e6ff35,0 0 70px #7b75ff25;border-color:#75dcff88}
.btn-primary{background:linear-gradient(110deg,#53d7f4,#658cff 50%,#9468f3);box-shadow:0 8px 30px #547fff55,inset 0 1px #ffffff66}
.effects-panel{overflow:hidden}.effects-panel:before{content:"";position:absolute;inset:0;pointer-events:none;background:linear-gradient(115deg,transparent 10%,#8adfff05 45%,transparent 70%)}
.effect-tools{display:grid;gap:10px;margin:0 0 14px}.effect-search{max-width:100%;background:#081321cc;border-color:#385779}.effect-filters{display:flex;flex-wrap:wrap;gap:7px}.filter-chip{padding:8px 13px;border-radius:30px;border:1px solid #365171;background:#101d30;color:#a8bbd6;font-size:11px;transition:.2s}.filter-chip:hover{border-color:#7adfff;color:#eafaff}.filter-chip.active{color:#07111e;background:linear-gradient(100deg,#62e7ff,#8f9cff);border-color:transparent;box-shadow:0 0 20px #62dfff30;font-weight:800}
.effects-grid{grid-template-columns:repeat(6,minmax(0,1fr));gap:9px}.effect{min-height:116px;border-color:#355171;background:linear-gradient(145deg,#1a2c46,#101a2b);box-shadow:inset 0 1px #ffffff0a;transition:transform .22s,border-color .22s,box-shadow .22s}.effect:hover{transform:translateY(-4px);border-color:#70dfff;box-shadow:0 10px 28px #0005,0 0 18px #5edbff14}.effect.active{background:linear-gradient(145deg,#214c70,#34306b);border-color:#75e4ff;box-shadow:0 0 0 1px #75e4ff30,0 0 28px #5bdcff20,inset 0 0 20px #8adfff13}.effect.active:after{background:var(--cyan);box-shadow:0 0 13px var(--cyan)}.effect-icon{font-size:25px;filter:drop-shadow(0 0 9px #7edfff77)}
.wifi-current{border-color:#47739a;background:linear-gradient(110deg,#14334a,#14213b);box-shadow:inset 0 1px #ffffff0a,0 0 25px #3abfe90c}.wifi-orb{background:linear-gradient(145deg,#164b66,#292d64);color:#75eaff;box-shadow:0 0 24px #4bdfff2a}.network-item{border-color:#334d6b;background:linear-gradient(100deg,#101e31,#10192a)}.network-item.current{border-color:#3b9d86}.network-item:hover,.network-item.selected{border-color:#65e2ff;background:linear-gradient(100deg,#163d54,#252d58);box-shadow:0 0 18px #50cfff1c}
.help details{border-color:#334c6b;background:linear-gradient(110deg,#101f33,#0b1423);transition:border-color .2s,box-shadow .2s}.help details[open]{border-color:#68dfff99;box-shadow:0 0 25px #51d7ff15}.help summary:after{content:"＋";color:#69ddff}.help details[open] summary:after{content:"−"}.help-body{animation:helpReveal .22s ease-out}@keyframes helpReveal{from{opacity:0;transform:translateY(-5px)}to{opacity:1;transform:translateY(0)}}
.scan-button{white-space:nowrap}.section-head h2{margin-top:5px}
@media(max-width:900px){.effects-grid{grid-template-columns:repeat(4,minmax(0,1fr))}}
@media(max-width:700px){.effects-grid{grid-template-columns:repeat(3,minmax(0,1fr))}.wifi-current{flex-wrap:wrap}.scan-button{width:100%}}
@media(max-width:430px){.effects-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.effect{min-height:105px}.filter-chip{padding:8px 10px}}
@media(prefers-reduced-motion:reduce){.hero:after,.logo:after{animation:none!important}}
.mqtt-enable{display:flex;align-items:center;gap:12px;padding:13px 15px;border:1px solid var(--stroke);border-radius:14px;background:var(--surface2);cursor:pointer}.mqtt-enable input{width:18px;height:18px;accent-color:#67dfff;flex:0 0 auto}.mqtt-enable span{display:flex;flex-direction:column;gap:4px}.mqtt-enable strong{font-size:13px}.mqtt-enable small{font-size:10px;color:var(--muted-main);line-height:1.4}.mqtt-status-line{margin-bottom:12px}.mqtt-actions{display:flex;flex-wrap:wrap}.mqtt-card #mqttStatusBadge{font-size:10px;letter-spacing:1px;color:#9aaccb}.mqtt-card #mqttStatusBadge.online{color:#36d9a0}.mqtt-card #mqttStatusBadge.error{color:#ff8293}body[data-theme="light"] .mqtt-enable{background:#f6f8fd;border-color:#dce4f0}@media(max-width:700px){.mqtt-actions .btn{flex:1}}


/* iOS-inspired adaptive appearance + dimmer */
:root{--surface:rgba(20,30,48,.78);--surface2:rgba(10,19,33,.78);--text-main:#f5f7ff;--muted-main:#91a0b8;--stroke:rgba(137,166,210,.22);--accent:#69dfff}
body{color-scheme:dark;color:var(--text-main);transition:background .35s ease,color .25s ease}
.card{background:var(--surface);border-color:var(--stroke);backdrop-filter:blur(28px) saturate(155%);-webkit-backdrop-filter:blur(28px) saturate(155%);border-radius:26px}
.status-item,.pair-code,.help details,.help-box{background:var(--surface2);border-color:var(--stroke)}
.field{color:var(--text-main);background:var(--surface2);border-color:var(--stroke)}
.theme-switch{display:flex;gap:4px;padding:4px;border:1px solid var(--stroke);border-radius:30px;background:var(--surface2);backdrop-filter:blur(18px)}
.theme-switch button{width:32px;height:32px;border:0;border-radius:50%;background:transparent;color:var(--muted-main);font-size:15px;transition:.2s}
.theme-switch button.active{background:linear-gradient(135deg,#66dfff,#8c80ff);color:#07111e;box-shadow:0 3px 14px #62cfff55}
.header-tools{display:flex;align-items:center;gap:9px}
.dimmer{margin-top:19px;padding:15px 16px;border:1px solid var(--stroke);border-radius:19px;background:linear-gradient(120deg,#6a9fff12,#8a72ff10);position:relative;z-index:1}
.dimmer-head{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:12px}
.dimmer-head span:first-child{font-size:11px;letter-spacing:1.4px;color:var(--muted-main);font-weight:700}
.dimmer-value{font-size:23px;font-weight:800;letter-spacing:-1px;background:linear-gradient(100deg,#68e5ff,#a28cff);-webkit-background-clip:text;background-clip:text;color:transparent;font-variant-numeric:tabular-nums}
.dimmer input{width:100%;height:7px;appearance:none;-webkit-appearance:none;border-radius:20px;outline:none;background:linear-gradient(90deg,#5ce0ff,#8b80ff);cursor:pointer}
.dimmer input::-webkit-slider-thumb{appearance:none;width:23px;height:23px;border-radius:50%;background:#fff;border:5px solid #6d9bff;box-shadow:0 2px 12px #0005,0 0 0 5px #79cfff20}
.dimmer input::-moz-range-thumb{width:14px;height:14px;border-radius:50%;background:#fff;border:5px solid #6d9bff;box-shadow:0 2px 12px #0005}
.dimmer-foot{display:flex;justify-content:space-between;margin-top:8px;color:var(--muted-main);font-size:10px}
.hero-icon{transition:transform .35s,box-shadow .35s}.hero-icon.active{animation:heroBreathe 3.2s ease-in-out infinite alternate}
@keyframes heroBreathe{to{transform:translateY(-3px) scale(1.035);box-shadow:0 0 36px #65e6ff40,0 0 90px #8c76ff20}}
body[data-theme="light"]{color-scheme:light;--bg:#edf2fa;--text:#172238;--muted:#65738c;--surface:rgba(255,255,255,.78);--surface2:rgba(246,249,255,.86);--text-main:#172238;--muted-main:#65738c;--stroke:rgba(83,105,145,.18);--border:#d7e0ef;--blue:#5579e8;--green:#149b75;background:radial-gradient(ellipse at 12% 0%,#a8d9ff88,transparent 42%),radial-gradient(ellipse at 92% 20%,#cbbaff88,transparent 38%),linear-gradient(160deg,#f7faff,#e9eff8 65%,#f4f6fb);color:#172238}
body[data-theme="light"]:before{background:linear-gradient(#5676a00a 1px,transparent 1px),linear-gradient(90deg,#5676a00a 1px,transparent 1px)}
body[data-theme="light"]:after{background:#8aabff20}
body[data-theme="light"] .card{background:var(--surface);border-color:var(--stroke);box-shadow:0 18px 55px #4a63801a,inset 0 1px #ffffffd9}
body[data-theme="light"] .connection{background:#ffffffb8;border-color:#cbd8eb;color:#53627c}
body[data-theme="light"] .eyebrow{color:#667fa8}
body[data-theme="light"] .hero-description,body[data-theme="light"] .pair-note,body[data-theme="light"] .section-head span{color:#65738c}
body[data-theme="light"] .status-item,body[data-theme="light"] .pair-code,body[data-theme="light"] .help details,body[data-theme="light"] .help-box{background:#f6f8fd;border-color:#dce4f0}
body[data-theme="light"] .status-label,body[data-theme="light"] .help-body,body[data-theme="light"] .help-box p{color:#65738c}
body[data-theme="light"] .status-value,body[data-theme="light"] .help summary,body[data-theme="light"] .help-body strong,body[data-theme="light"] .help-box b{color:#172238}
body[data-theme="light"] .effect{background:linear-gradient(145deg,#fff,#f1f5fc);border-color:#dbe4f1;box-shadow:0 5px 16px #3e5b8410}
body[data-theme="light"] .effect-name{color:#1b2940}body[data-theme="light"] .effect-description{color:#687891}
body[data-theme="light"] .effect.active{background:linear-gradient(145deg,#e0f6ff,#e8e7ff);border-color:#7298ef;box-shadow:0 0 0 1px #7298ef22,0 10px 25px #6486cf22}
body[data-theme="light"] .btn{background:linear-gradient(145deg,#fff,#edf2fa);border-color:#d2ddec;color:#263650;box-shadow:0 5px 14px #46628a12}
body[data-theme="light"] .btn-primary{background:linear-gradient(110deg,#53cde9,#6689f4 55%,#9270e9);border:0;color:white}
body[data-theme="light"] .btn-danger{color:#c43c59}
body[data-theme="light"] .field{background:#fff;color:#172238;border-color:#d2ddec}
body[data-theme="light"] .filter-chip{background:#fff;border-color:#d7e1ef;color:#566781}
body[data-theme="light"] .filter-chip.active{background:linear-gradient(100deg,#5ed8ed,#8d8df3);color:#10213a}
body[data-theme="light"] .wifi-current{background:linear-gradient(110deg,#e7f7ff,#f0edff);border-color:#d0deef}
body[data-theme="light"] .wifi-current-text small,body[data-theme="light"] .wifi-current-text span,body[data-theme="light"] .scan-state{color:#687b96}
body[data-theme="light"] .network-item{background:#fff;border-color:#dbe4f0;color:#24344e}
body[data-theme="light"] .network-item .net-meta{color:#6c7d96}
body[data-theme="light"] .network-item:hover,body[data-theme="light"] .network-item.selected{background:#eaf5ff;border-color:#77a4e9}
body[data-theme="light"] .pair-code strong{color:#4f70ce}
body[data-theme="light"] .footer{color:#8794a9}
body[data-theme="light"] .toast{background:#263955;color:#fff}
@media(max-width:700px){.header{align-items:flex-start}.header-tools{flex-direction:column;align-items:flex-end}.theme-switch button{width:29px;height:29px}}


.log-toolbar{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap;margin-bottom:12px}.log-toolbar .log-actions{display:flex;gap:8px}.log-terminal{height:300px;overflow:auto;padding:14px;background:#050a12;border:1px solid #263750;border-radius:14px;color:#b8d6c7;font:11px/1.55 ui-monospace,SFMono-Regular,Consolas,monospace;white-space:pre-wrap;overflow-wrap:anywhere;tab-size:2}.log-terminal .log-empty{color:#71819b}.log-hint{font-size:11px;color:var(--muted);margin-top:10px;line-height:1.5}.log-toggle{display:flex;align-items:center;gap:7px;color:#aebbd0;font-size:12px}

/* GoodWin Smart Hub tabs and display controls */
.gw-tabs{display:flex;gap:8px;overflow-x:auto;padding:4px 2px 14px;margin:0 0 12px;position:sticky;top:0;z-index:30;background:linear-gradient(180deg,#080b14f5 80%,transparent)}
.gw-tab{flex:0 0 auto;border:1px solid #344766;border-radius:13px;padding:11px 15px;background:#121d30;color:#aebbd0;font-size:12px;font-weight:750;letter-spacing:.2px}
.gw-tab.active{background:linear-gradient(120deg,#527cf0,#765fe0);border-color:#86a5ff;color:white;box-shadow:0 7px 22px #527cf044}
.gw-panel{display:none}.gw-panel.active{display:block}.hub-controls{display:grid;grid-template-columns:1fr 1fr;gap:12px}.hub-control{display:flex;flex-direction:column;gap:7px}.hub-control label{font-size:11px;color:#aebbd0}.hub-control select,.hub-control input[type=range]{width:100%}.hub-control select{padding:12px;border:1px solid #344766;border-radius:12px;background:#0a1424;color:#f5f7ff}.hub-switch{display:flex;align-items:center;gap:10px;padding:12px;border:1px solid #344766;border-radius:12px;background:#0b1423}.hub-live{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;margin-top:12px}.hub-live .status-item{padding:11px}.hub-live .status-value{font-size:13px}#hubWeatherCard .hub-live{grid-template-columns:repeat(2,minmax(0,1fr))}.hub-note{font-size:11px;line-height:1.5;color:#9baac2;margin-top:12px}
@media(max-width:600px){.hub-controls{grid-template-columns:1fr}.gw-tabs{margin-left:-8px;margin-right:-8px;padding-left:8px}.gw-tab{padding:10px 12px}.hub-live{grid-template-columns:1fr 1fr}}
/* One coherent climate panel: indoor and outdoor zones share one parent card. */
.hub-climate-card{border:1px solid #527e9c88;background:linear-gradient(145deg,#142333,#101827 76%)}
.hub-climate-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:14px}
.climate-zone{min-width:0;border-radius:15px;padding:13px;border:1px solid #ffffff18}
.hub-sensors-card{background:linear-gradient(145deg,#102b24,#101d22)}
.hub-weather-card{background:linear-gradient(145deg,#102b3a,#111d2b)}
.climate-zone-head{display:flex;align-items:flex-start;justify-content:space-between;gap:8px;min-height:43px}
.climate-zone-head h3{font-size:15px;margin:3px 0 0;color:#f1f6ff}
.climate-zone-head span{font-size:10px;font-weight:750;white-space:nowrap;color:#82dcb0}
.hub-weather-card .climate-zone-head span{color:#83dff5}
.hub-climate-card .hub-live{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;margin-top:10px}
.hub-climate-card .status-item{min-width:0;min-height:67px;padding:10px;background:#071b1aaa;border-color:#397c5b88}
.hub-weather-card .status-item{background:#071b2aaa;border-color:#397b9b88}
.hub-climate-card .status-value{font-size:clamp(16px,2.4vw,21px);white-space:nowrap;color:#a5f0c5}
.hub-weather-card .status-value{color:#a5eaff}.hub-weather-tools{display:grid;grid-template-columns:1fr auto;gap:8px;align-items:end;margin:12px 0}.hub-weather-tools label{grid-column:1/-1;font-size:11px;color:#aebbd0}.hub-weather-tools select{min-width:0;width:100%;padding:10px;border:1px solid #344766;border-radius:12px;background:#0a1424;color:#f5f7ff}.hub-weather-tools button{white-space:nowrap}@media(max-width:430px){.hub-weather-tools{grid-template-columns:1fr}.hub-weather-tools button{width:100%}}
.hub-climate-card .status-label{font-size:10px;letter-spacing:.06em;text-transform:uppercase}
.hub-climate-card .hub-note{margin-top:13px}
.climate-live-dot{color:#78dfaa!important;font-size:11px;font-weight:800;white-space:nowrap}
@media(max-width:640px){.hub-climate-grid{grid-template-columns:1fr}.climate-zone{padding:12px}.hub-climate-card .hub-live{grid-template-columns:repeat(2,minmax(0,1fr))}}


/* GoodWin Smart Hub visual refinement */
body[data-theme="light"] .gw-tabs{background:linear-gradient(180deg,#edf2faf7 82%,transparent)}
body[data-theme="light"] .gw-tab{background:linear-gradient(145deg,#ffffff,#edf2fa);border-color:#d2ddec;color:#566781;box-shadow:0 4px 12px #46628a12}
body[data-theme="light"] .hub-control select{background:#fff;color:#263650;border-color:#d2ddec}
body[data-theme="light"] .hub-switch{background:#fff;color:#263650;border-color:#d2ddec}

.gw-tabs{gap:9px;padding-top:8px;padding-bottom:16px;scrollbar-width:none}
.gw-tabs::-webkit-scrollbar{display:none}
.gw-tab{min-height:42px;padding:11px 17px;border-radius:14px;background:linear-gradient(145deg,#17243a,#101a2b);border-color:#30415e;box-shadow:inset 0 1px 0 #ffffff08;transition:transform .18s,border-color .18s,background .18s}
.gw-tab:hover{transform:translateY(-1px);border-color:#7296ed}
.gw-tab.active{background:linear-gradient(115deg,#4e7bf0 0%,#735fe0 100%);box-shadow:0 8px 24px #527cf044,inset 0 1px 0 #ffffff38}
.gw-panel.active{animation:gwPanelIn .24s ease-out both}
@keyframes gwPanelIn{from{opacity:.65;transform:translateY(5px)}to{opacity:1;transform:translateY(0)}}
.card{box-shadow:0 18px 55px #0002,inset 0 1px 0 #ffffff08}
.hero:after{content:"";position:absolute;width:220px;height:220px;right:-105px;top:-105px;border-radius:50%;background:radial-gradient(circle,#7199ff22,transparent 68%);pointer-events:none}
.section-head{align-items:flex-start}
.status-item{transition:border-color .18s,transform .18s}
.status-item:hover{border-color:#5875a8;transform:translateY(-1px)}
.hub-live .status-item{min-width:0;overflow:hidden}
.hub-live .status-value{overflow-wrap:anywhere}
.system-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:9px}.system-grid .status-item{padding:11px;min-width:0}.system-grid .status-value{font-size:14px;white-space:nowrap}@media(max-width:700px){.system-grid{grid-template-columns:repeat(2,minmax(0,1fr))}}
.hub-weather-metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:9px}.hub-weather-metrics .status-item{padding:10px 11px}@media(max-width:700px){.hub-weather-metrics{grid-template-columns:repeat(2,minmax(0,1fr))}.app{padding:16px 12px 36px}.gw-tabs{top:0;backdrop-filter:blur(16px);background:linear-gradient(180deg,#080b14f7 82%,transparent)}.gw-tab{font-size:11px;padding:10px 13px}.card{padding:18px}.effects-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.grid-main{grid-template-columns:1fr}}

</style>
</head>
<body><div class="app">
<header class="header"><div class="brand"><div class="logo">✦</div><div><div class="brand-name">GoodWin Smart Hub</div><div class="brand-sub">SMART HOME CONTROL · FIRMWARE 3.11.0</div></div></div><div class="header-tools"><div class="theme-switch" aria-label="Тема интерфейса"><button type="button" id="themeAuto" title="Авто">A</button><button type="button" id="themeLight" title="Светлая">☼</button><button type="button" id="themeDark" title="Тёмная">☾</button></div><div class="connection"><span class="dot" id="connectionDot"></span><span id="connectionText">ONLINE</span></div></div></header>
<div class="grid-main">
<section class="card hero"><div class="eyebrow">Освещение · ESP32</div><div class="hero-icon" id="heroIcon">💡</div><h1 id="lightState">Загрузка...</h1><div class="hero-description" id="lightDescription">Подключение к устройству</div><div class="dimmer"><div class="dimmer-head"><span>ЯРКОСТЬ</span><strong class="dimmer-value" id="brightnessValue">80%</strong></div><input id="brightnessSlider" type="range" min="1" max="100" value="80" step="1" aria-label="Яркость светодиода"><div class="dimmer-foot"><span>МИНИМУМ</span><span>ПЛАВНОЕ PWM-УПРАВЛЕНИЕ</span><span>100%</span></div></div><div class="dimmer"><div class="dimmer-head"><span>СКОРОСТЬ ЭФФЕКТОВ</span><strong class="dimmer-value" id="speedValue">100%</strong></div><input id="speedSlider" type="range" min="25" max="300" value="100" step="5" aria-label="Скорость эффектов"><div class="dimmer-foot"><span>МЕДЛЕННО</span><span>ТЕМП АНИМАЦИИ</span><span>БЫСТРО</span></div></div><div class="power-actions"><button class="btn btn-primary" onclick="setPower(true)">Включить</button><button class="btn" onclick="setPower(false)">Выключить</button></div></section>
<section class="card side-status"><div class="section-head"><h2>Устройство</h2><span>LIVE STATUS</span></div><div class="status-item"><div class="status-label">Wi-Fi</div><div class="status-value" id="wifiStatus">—</div></div><div class="status-item"><div class="status-label">IP-адрес</div><div class="status-value" id="ipStatus">—</div></div><div class="status-item"><div class="status-label">Matter</div><div class="status-value" id="matterStatus">—</div></div><div class="status-item"><div class="status-label">Сопряжение</div><div class="status-value" id="commissionStatus">—</div></div></section>
</div>
<section class="card section effects-panel"><div class="section-head"><div><div class="eyebrow">LIGHT ENGINE · 36 PRESETS</div><h2>Световые эффекты</h2></div><span id="effectCounter">36 РЕЖИМОВ</span></div>
<div class="effect-tools"><input class="field effect-search" id="effectSearch" type="search" placeholder="Найти эффект…" autocomplete="off"><div class="effect-filters" id="effectFilters"><button class="filter-chip active" data-filter="all">Все</button><button class="filter-chip" data-filter="steady">Свет</button><button class="filter-chip" data-filter="rhythm">Ритм</button><button class="filter-chip" data-filter="signal">Сигналы</button><button class="filter-chip" data-filter="ambient">Атмосфера</button></div></div>
<div class="effects-grid" id="effectsGrid"></div><div class="warning">Стробоскопические режимы могут быть небезопасны для людей с фоточувствительностью.</div></section>
<section class="card section matter-pairing"><div class="section-head"><h2>Matter · Сопряжение</h2><span>LOCAL NETWORK</span></div><div class="pair-code" onclick="copyCode()"><strong id="pairCode">—</strong><span class="copy-label">▣ КОПИРОВАТЬ</span></div><div class="pair-note">Нажми на код, чтобы скопировать его. QR-код можно отсканировать в приложении умного дома.</div><div class="qr-wrap"><div id="qrcode"></div></div></section>
<section class="card section hub-config-card" id="hubConfigCard"><div class="section-head"><div><div class="eyebrow">DISPLAY ENGINE</div><h2>Экран и оформление</h2></div><span>ST7735 · 128×160 / 160×128</span></div>
<div class="hub-controls">
 <div class="hub-control"><label for="hubLayout">Макет экрана</label><select id="hubLayout"><option value="0">Dashboard — сводка</option><option value="1">Часы крупно</option><option value="2">Погода крупно</option><option value="3">Домашний климат</option><option value="4">Дом + улица</option><option value="5">Состояние света</option><option value="6">Aurora — погода PRO</option><option value="7">Тест TFT · FPS</option><option value="8">Orbit — neon telemetry</option><option value="9">Terminal — monochrome</option><option value="10">Dark Tech · prototype</option><option value="11">Amber · prototype</option><option value="12">Glass UI · prototype</option><option value="13">Cyber · prototype</option></select></div>
 <div class="hub-control"><label for="hubOrientation">Ориентация дисплея</label><select id="hubOrientation"><option value="0">Портрет · 128×160</option><option value="1">Альбом · 160×128</option><option value="2">Портрет перевёрнутый · 128×160</option><option value="3">Альбом перевёрнутый · 160×128</option></select></div>
 <div class="hub-control"><label for="hubSaver">Заставка</label><select id="hubSaver"><option value="0">Выключена</option><option value="1">Часы</option><option value="2">Плавающая точка</option><option value="3">Звёзды</option><option value="4">Дождь</option><option value="5">Снег</option><option value="6">Облака</option><option value="7">Температура и влажность</option><option value="8">Волна</option><option value="9">Пульс</option><option value="10">Северное сияние</option><option value="11">DNA · Double Helix</option></select><small class="hub-note">Автозапуск заставки по таймеру бездействия.</small></div><div class="hub-control"><label for="hubSaverIdle">Запускать через</label><select id="hubSaverIdle"><option value="0">Отключить автозапуск</option><option value="30">30 секунд</option><option value="60">1 минута</option><option value="120">2 минуты</option><option value="300">5 минут</option><option value="600">10 минут</option><option value="1800">30 минут</option></select><button id="hubSaverStart" type="button" class="btn" onclick="startHubSaverNow()">Запустить заставку сейчас</button></div>
 <div class="hub-control"><label for="hubBrightness">Яркость подсветки: <strong id="hubBrightnessValue">85%</strong></label><input id="hubBrightness" type="range" min="1" max="100" value="85"></div>
 <div class="hub-control"><label for="hubTestFps">FPS тестового экрана: <strong id="hubTestFpsValue">20 FPS</strong></label><input id="hubTestFps" type="range" min="1" max="60" value="20"><small class="hub-note">Выбери макет «Тест TFT · FPS». Целевой и фактический FPS будут показаны на дисплее и в Serial.</small></div>
 <label class="hub-switch"><input id="hubDisplayEnabled" type="checkbox" checked> <strong>Экран включён</strong></label>
 <div class="hub-note">Заданная яркость по Matter и вебу сохранится. Физическое PWM-регулирование заработает после подключения вывода LED подсветки через транзистор/MOSFET к GPIO25. Не подключай вывод LED подсветки напрямую к GPIO25.</div>
</div></section>
<section class="card section hub-climate-card" id="hubClimateCard"><div class="section-head"><div><div class="eyebrow">CLIMATE · LIVE DATA</div><h2>Климат дома и на улице</h2></div><span class="climate-live-dot">● LIVE</span></div>
<div class="hub-climate-grid">
 <div class="climate-zone hub-sensors-card" id="hubSensorsCard"><div class="climate-zone-head"><div><div class="eyebrow">INDOOR · DHT22</div><h3>В помещении</h3></div><span id="hubSensorState">Ожидание</span></div>
  <div class="hub-live"><div class="status-item"><div class="status-label">Температура</div><div class="status-value" id="hubIndoorTemp">—</div></div><div class="status-item"><div class="status-label">Влажность</div><div class="status-value" id="hubIndoorHumidity">—</div></div></div>
 </div>
 <div class="climate-zone hub-weather-card" id="hubWeatherCard"><div class="climate-zone-head"><div><div class="eyebrow">OUTDOOR · WEATHER API</div><h3>На улице</h3></div><span id="hubWeatherState">Ожидание</span></div><div class="hub-weather-tools"><label for="hubWeatherSource">Источник погоды</label><select id="hubWeatherSource" class="field" onchange="setWeatherSource(this.value)"><option value="0">Авто · резервирование</option><option value="1">Open-Meteo</option><option value="2">wttr.in</option><option value="3">MET Norway</option></select><button class="btn btn-primary" id="hubWeatherRefresh" type="button" onclick="forceWeatherRefresh()">↻ Обновить сейчас</button></div>
  <div class="hub-live hub-weather-metrics"><div class="status-item"><div class="status-label">Температура</div><div class="status-value" id="hubOutdoor">—</div></div><div class="status-item"><div class="status-label">Ощущается</div><div class="status-value" id="hubFeelsLike">—</div></div><div class="status-item"><div class="status-label">Влажность</div><div class="status-value" id="hubOutdoorHumidity">—</div></div><div class="status-item"><div class="status-label">Давление</div><div class="status-value" id="hubPressure">—</div></div><div class="status-item"><div class="status-label">Ветер</div><div class="status-value" id="hubWind">—</div></div><div class="status-item"><div class="status-label">Облачность</div><div class="status-value" id="hubCloud">—</div></div><div class="status-item"><div class="status-label">UV-индекс</div><div class="status-value" id="hubUv">—</div></div><div class="status-item"><div class="status-label">Осадки</div><div class="status-value" id="hubRain">—</div></div></div>
 </div>
</div><div class="hub-note">В помещении — показания DHT22. На улице — расширенные текущие условия Open-Meteo. Плановое обновление каждые 5 минут. Авто переключается на резервные источники при ошибке.</div></section>
<section class="card section wifi-card"><div class="section-head"><h2>Wi-Fi менеджер</h2><span>NETWORK CONTROL</span></div>
<div class="wifi-current"><div class="wifi-orb">⌁</div><div class="wifi-current-text"><small>ТЕКУЩЕЕ ПОДКЛЮЧЕНИЕ</small><strong id="currentSSID">Не подключено</strong><span id="wifiSignal">Ожидание подключения</span></div><button class="btn scan-button" onclick="scanNetworks()">⌁ Сканировать сети</button></div>
<div id="scanState" class="scan-state">Нажми «Сканировать», чтобы найти доступные сети поблизости.</div><div id="networkList" class="network-list"></div>
<form class="wifi-form" method="POST" action="/wifi"><input class="field" id="ssidInput" name="ssid" placeholder="Выбери сеть или введи SSID" required><input class="field" id="wifiPassword" name="password" type="password" placeholder="Пароль выбранной сети"><button class="btn btn-primary full" type="submit">Сохранить сеть и подключиться</button></form>
<div class="pair-note">ESP32 DevKit V1 подключается к Wi-Fi 2,4 ГГц — сети 5 ГГц не поддерживаются. Для скрытой сети введи SSID вручную. При смене сети Matter-сопряжение сохраняется; если подключение не удастся, появится резервная сеть ESP-SETUP.</div></section>
<section class="card section mqtt-card"><div class="section-head"><div><div class="eyebrow">HOME ASSISTANT · MQTT</div><h2>MQTT настройки</h2></div><span id="mqttStatusBadge">НЕ НАСТРОЕН</span></div>
<div class="status-item mqtt-status-line"><div class="status-label">Состояние брокера</div><div class="status-value" id="mqttStatusText">Загрузка...</div></div>
<form id="mqttForm" class="wifi-form mqtt-form">
<label class="mqtt-enable full"><input id="mqttEnabled" name="enabled" type="checkbox" value="1"><span><strong>Включить MQTT</strong><small>Подключение к Mosquitto и автоматическое обнаружение в Home Assistant</small></span></label>
<input class="field" id="mqttHost" name="host" placeholder="IP-адрес брокера, например 192.168.1.50" autocomplete="off">
<input class="field" id="mqttPort" name="port" type="number" min="1" max="65535" value="1883" placeholder="Порт">
<input class="field" id="mqttUser" name="user" placeholder="Логин MQTT" autocomplete="username">
<input class="field" id="mqttPassword" name="password" type="password" placeholder="Пароль MQTT" autocomplete="new-password">
<input class="field full" id="mqttRoot" name="root" placeholder="Корневой топик: goodwin/GW-ESP32-001" autocomplete="off">
<label class="mqtt-enable full"><input id="mqttClearPassword" name="clear_password" type="checkbox" value="1"><span><strong>Удалить сохранённый пароль</strong><small>Используй, если брокер работает без авторизации</small></span></label>
<div class="power-actions full mqtt-actions"><button class="btn btn-primary" type="submit">Сохранить настройки</button><button class="btn" type="button" id="mqttTestButton" onclick="testMqtt()">Проверить соединение</button></div>
</form><div class="pair-note">Укажи локальный IP Home Assistant, а не внешний URL. Стандартный порт Mosquitto — 1883. Пароль хранится во flash ESP32 и никогда не выводится обратно в веб-интерфейс. Matter работает независимо от MQTT.</div></section>
<section class="card section system-monitor"><div class="section-head"><div><div class="eyebrow">LIVE TELEMETRY</div><h2>Системный монитор</h2></div><span id="sysMonitorState">LIVE · 2 СЕК</span></div><div class="system-grid">
<div class="status-item"><div class="status-label">Частота CPU</div><div class="status-value" id="sysCpu">—</div></div><div class="status-item"><div class="status-label">Температура чипа</div><div class="status-value" id="sysTemp">—</div></div><div class="status-item"><div class="status-label">Свободная Heap</div><div class="status-value" id="sysHeap">—</div></div><div class="status-item"><div class="status-label">Минимальная Heap</div><div class="status-value" id="sysMinHeap">—</div></div><div class="status-item"><div class="status-label">Крупнейший блок Heap</div><div class="status-value" id="sysLargest">—</div></div><div class="status-item"><div class="status-label">Свободная PSRAM</div><div class="status-value" id="sysPsram">—</div></div><div class="status-item"><div class="status-label">Flash</div><div class="status-value" id="sysFlash">—</div></div><div class="status-item"><div class="status-label">Wi-Fi RSSI</div><div class="status-value" id="sysRssi">—</div></div><div class="status-item"><div class="status-label">Время работы</div><div class="status-value" id="sysUptime">—</div></div><div class="status-item"><div class="status-label">TFT FPS</div><div class="status-value" id="sysFps">—</div></div><div class="status-item"><div class="status-label">Время отрисовки TFT</div><div class="status-value" id="sysTftDraw">—</div></div><div class="status-item"><div class="status-label">Режим теста</div><div class="status-value" id="sysTftPhase">—</div></div></div>
<div class="system-grid hardware-grid">
<div class="status-item"><div class="status-label">Профиль платы</div><div class="status-value" id="sysBoard">—</div></div>
<div class="status-item"><div class="status-label">Модель чипа</div><div class="status-value" id="sysChip">—</div></div>
<div class="status-item"><div class="status-label">Ревизия / ядра</div><div class="status-value" id="sysChipInfo">—</div></div>
<div class="status-item"><div class="status-label">Flash · частота / режим</div><div class="status-value" id="sysFlashSpeed">—</div></div>
<div class="status-item"><div class="status-label">Arduino-ESP32</div><div class="status-value" id="sysCore">—</div></div>
<div class="status-item"><div class="status-label">ESP-IDF</div><div class="status-value" id="sysSdk">—</div></div>
<div class="status-item"><div class="status-label">Размер прошивки</div><div class="status-value" id="sysSketch">—</div></div>
</div><div class="log-hint">Модель и ревизия определяются по ESP-чипу. Профиль платы берётся из конфигурации Arduino: точную маркировку PCB программно определить нельзя. Температура — внутренний датчик ESP32, если он поддерживается ядром.</div></section>
<section class="card section" id="serialMonitor"><div class="section-head"><div><div class="eyebrow">ON-DEMAND DIAGNOSTICS</div><h2>Журнал диагностики</h2></div><span id="logState">ВЫКЛЮЧЕН</span></div>
<div class="log-toolbar"><label class="log-toggle"><input id="logAutoScroll" type="checkbox" checked> Автопрокрутка</label><div class="log-actions"><button class="btn btn-primary" type="button" id="logStart">Запустить</button><button class="btn" type="button" id="logStop" disabled>Остановить</button><button class="btn" type="button" id="logClear">Очистить</button></div></div>
<pre class="log-terminal" id="serialLog"><span class="log-empty">Диагностика выключена. Нажми «Запустить», чтобы временно собирать логи.</span></pre>
<div class="log-hint">Сбор веб-журнала выключен по умолчанию и не занимает буфер RAM. USB Serial (115200 бод) продолжает работать независимо. Включай сбор только на время диагностики.</div></section>
<section class="card section help-card"><div class="section-head"><h2>Инструкция и функции</h2><span>HELP CENTER</span></div>
<div class="help">
<details open><summary>Быстрый старт и подключение к Wi-Fi</summary><div class="help-body">
<ol><li>Подай питание на ESP32 и дождись запуска.</li><li>Если сохранённая сеть недоступна, подключись телефоном к Wi-Fi <strong>ESP-SETUP</strong>, пароль <strong>esp32setup</strong>.</li><li>Открой в браузере <strong>http://192.168.4.1</strong>.</li><li>Введи имя и пароль своей сети Wi-Fi и нажми «Сохранить и перезапустить».</li><li>После подключения открой веб-панель по IP устройства или адресу <strong>http://goodwin-light.local</strong>.</li></ol>
<p>В блоке «Wi-Fi менеджер» нажми «Сканировать», выбери найденную сеть, введи пароль и сохрани настройки. Текущая сеть и уровень сигнала отображаются над списком. ESP32 DevKit V1 поддерживает только Wi-Fi 2,4 ГГц; сети 5 ГГц не появятся. Скрытую сеть можно указать вручную.</p>
<p>Если связь с роутером пропадёт уже после настройки, устройство будет повторять подключение и поднимет резервную точку ESP-SETUP. Matter-сопряжение при обычной потере Wi-Fi не сбрасывается.</p>
</div></details>
<details><summary>Яркость, управление светом и 36 эффектов</summary><div class="help-body"><p><strong>Включить / Выключить</strong> — управляет Matter Dimmable Light и светодиодом GPIO2. Ползунок яркости задаёт общий уровень для всех эффектов.</p><div class="help-grid">
<div class="help-box"><b>Постоянный свет</b><p>Непрерывное свечение</p></div>
<div class="help-box"><b>Медленный импульс</b><p>Длинный цикл включения и паузы</p></div>
<div class="help-box"><b>Быстрое мигание</b><p>Равномерные короткие вспышки</p></div>
<div class="help-box"><b>Пульсация</b><p>Короткая вспышка с длинной паузой</p></div>
<div class="help-box"><b>Маяк</b><p>Редкий сигнальный импульс</p></div>
<div class="help-box"><b>Двойная вспышка</b><p>Два импульса подряд</p></div>
<div class="help-box"><b>Тройная вспышка</b><p>Три импульса подряд</p></div>
<div class="help-box"><b>Стробоскоп 5 Гц</b><p>Пять циклов в секунду</p></div>
<div class="help-box"><b>Стробоскоп 8 Гц</b><p>Восемь циклов в секунду</p></div>
<div class="help-box"><b>Стробоскоп 12 Гц</b><p>Двенадцать циклов в секунду</p></div>
<div class="help-box"><b>Сердцебиение</b><p>Двойной ритмичный импульс</p></div>
<div class="help-box"><b>SOS</b><p>Световой сигнал азбукой Морзе</p></div>
<div class="help-box"><b>Тревога</b><p>Повторяющийся тревожный сигнал</p></div>
<div class="help-box"><b>Свеча</b><p>Неровное мерцание</p></div>
<div class="help-box"><b>Случайный</b><p>Случайные интервалы и состояния</p></div>
<div class="help-box"><b>Маяк дальний</b><p>Один яркий сигнал и длинная пауза</p></div>
<div class="help-box"><b>Маяк двойной</b><p>Две группы сигнальных импульсов</p></div>
<div class="help-box"><b>Метроном</b><p>Равномерный ритм с акцентом</p></div>
<div class="help-box"><b>Сканер</b><p>Серия коротких импульсов с возвратом</p></div>
<div class="help-box"><b>Пульс медленный</b><p>Спокойный редкий пульс</p></div>
<div class="help-box"><b>Пульс быстрый</b><p>Ускоренный двойной пульс</p></div>
<div class="help-box"><b>Уведомление</b><p>Короткий двойной сигнал</p></div>
<div class="help-box"><b>Входящий сигнал</b><p>Три коротких импульса и пауза</p></div>
<div class="help-box"><b>Выходящий сигнал</b><p>Длинный импульс и два коротких</p></div>
<div class="help-box"><b>Код Морзе: E</b><p>Один короткий импульс</p></div>
<div class="help-box"><b>Код Морзе: T</b><p>Один длинный импульс</p></div>
<div class="help-box"><b>Код Морзе: OK</b><p>Сигнал OK азбукой Морзе</p></div>
<div class="help-box"><b>Четыре вспышки</b><p>Четыре коротких импульса</p></div>
<div class="help-box"><b>Пять вспышек</b><p>Пять коротких импульсов</p></div>
<div class="help-box"><b>Серия 3+1</b><p>Три быстрых и один длинный</p></div>
<div class="help-box"><b>Серия 1+3</b><p>Один длинный и три быстрых</p></div>
<div class="help-box"><b>Ритм сирены</b><p>Чередование коротких и длинных сигналов</p></div>
<div class="help-box"><b>Костёр</b><p>Живое нерегулярное мерцание</p></div>
<div class="help-box"><b>Светлячок</b><p>Редкие одиночные вспышки</p></div>
<div class="help-box"><b>Звёздный дождь</b><p>Нерегулярные короткие вспышки</p></div>
<div class="help-box"><b>Тихий режим</b><p>Очень редкие мягкие сигналы</p></div>
</div>
<p class="help-danger"><strong>Внимание:</strong> стробоскопические режимы могут провоцировать приступы у людей с фоточувствительной эпилепсией. Не используйте их рядом с такими людьми.</p></div></details>
<details><summary>Matter и голосовое управление</summary><div class="help-body"><ul><li>Устройство объявляется как стандартная категория <strong>Matter Dimmable Light</strong> с кластерами On/Off и Level Control.</li><li>Сопряжение выполняется QR-кодом или ручным кодом в совместимом приложении умного дома.</li><li>Matter передаёт состояние On/Off и яркость. Команда включения/выключения переключает постоянный свет; команда яркости меняет общий уровень, не прерывая выбранный локальный эффект.</li><li>При команде включения/выключения из Matter локальный эффект сбрасывается на постоянный свет.</li><li>Выбор эффекта в этой веб-панели включает устройство, но не добавляет новые типы функций в Matter: Matter предоставляет включение, выключение и яркость, а эффекты остаются локальными.</li><li>Сопряжение хранится отдельно от Wi-Fi пароля. Смена Wi-Fi не должна требовать повторного сопряжения.</li></ul><p>Если QR не отображается, проверь ручной код. Для генерации QR в браузере используется внешняя библиотека CDN, поэтому для неё нужен интернет.</p></div></details>
<details><summary>Кнопка BOOT: управление и сброс</summary><div class="help-body"><ul><li><strong>Короткое нажатие:</strong> переключить на следующий световой эффект.</li><li><strong>Удержание 5–10 секунд и отпускание:</strong> удалить только сохранённые Wi-Fi настройки. Matter Fabric сохраняется. После перезапуска появится ESP-SETUP.</li><li><strong>Удержание 10 секунд или дольше и отпускание:</strong> полный заводской сброс — Wi-Fi настройки и Matter-сопряжение будут удалены.</li></ul><p>Отсчёт длительности производится до отпускания кнопки. Для сброса Wi-Fi отпусти кнопку до достижения 10 секунд.</p></div></details>
<details><summary>Сеть, адрес устройства и OTA</summary><div class="help-body"><ul><li><strong>IP-адрес:</strong> текущий адрес ESP32 в локальной сети.</li><li><strong>goodwin-light.local:</strong> локальное имя через mDNS; поддержка зависит от устройства, с которого открывается страница.</li><li><strong>ESP-SETUP:</strong> резервная точка доступа для первичной настройки и восстановления.</li><li><strong>OTA:</strong> обновление прошивки по Wi-Fi с помощью Arduino IDE; доступно только после установки собственного пароля OTA в скетче.</li><li><strong>Локальное управление:</strong> панель работает в локальной сети; Matter не требует облачного сервера.</li></ul></div></details>
<details><summary>Индикаторы и диагностика</summary><div class="help-body"><ul><li>Красный светодиод питания на большинстве DevKit V1 аппаратный и не управляется прошивкой.</li><li>GPIO2 — управляемый светодиод, который используется как световой выход и для эффектов.</li><li>Статус Wi-Fi, Matter, сопряжения и текущего эффекта обновляется автоматически.</li><li>Serial Monitor: скорость <strong>115200</strong>. Логи имеют метки BOOT, WIFI, MATTER, BUTTON, EFFECT, WEB, RESET, STATUS и ERROR.</li><li>Если устройство не открывается по имени, используй IP-адрес из роутера.</li></ul></div></details>
</div></section>
<section class="card section maintenance"><div class="section-head"><h2>Обслуживание</h2><span>DEVICE</span></div><p class="pair-note">Сброс Wi-Fi оставляет Matter-сопряжение. Полный сброс удаляет и Wi-Fi, и Matter.</p><div class="power-actions"><button class="btn" onclick="resetWiFi()">Сбросить только Wi-Fi</button><button class="btn btn-danger" onclick="resetDevice()">Полный заводской сброс</button></div></section>
<div class="footer">GOODWIN SMART HOME · LOCAL CONTROL</div></div><div class="toast" id="toast">Готово</div>
<script>
const effects=[
  ["☀","Постоянный свет","Непрерывное свечение"],
  ["◉","Медленный импульс","Длинный цикл включения и паузы"],
  ["ϟ","Быстрое мигание","Равномерные короткие вспышки"],
  ["♥","Пульсация","Короткая вспышка с длинной паузой"],
  ["◈","Маяк","Редкий сигнальный импульс"],
  ["✦","Двойная вспышка","Два импульса подряд"],
  ["✧","Тройная вспышка","Три импульса подряд"],
  ["⚡","Стробоскоп 5 Гц","Пять циклов в секунду"],
  ["⚡","Стробоскоп 8 Гц","Восемь циклов в секунду"],
  ["⚡","Стробоскоп 12 Гц","Двенадцать циклов в секунду"],
  ["♡","Сердцебиение","Двойной ритмичный импульс"],
  ["⌁","SOS","Световой сигнал азбукой Морзе"],
  ["◉","Тревога","Повторяющийся тревожный сигнал"],
  ["♨","Свеча","Неровное мерцание"],
  ["✳","Случайный","Случайные интервалы и состояния"],
  ["◉","Маяк дальний","Один яркий сигнал и длинная пауза"],
  ["◎","Маяк двойной","Две группы сигнальных импульсов"],
  ["⌁","Метроном","Равномерный ритм с акцентом"],
  ["▥","Сканер","Серия коротких импульсов с возвратом"],
  ["♡","Пульс медленный","Спокойный редкий пульс"],
  ["♥","Пульс быстрый","Ускоренный двойной пульс"],
  ["✉","Уведомление","Короткий двойной сигнал"],
  ["↗","Входящий сигнал","Три коротких импульса и пауза"],
  ["↘","Выходящий сигнал","Длинный импульс и два коротких"],
  ["·","Код Морзе: E","Один короткий импульс"],
  ["—","Код Морзе: T","Один длинный импульс"],
  ["✓","Код Морзе: OK","Сигнал OK азбукой Морзе"],
  ["✣","Четыре вспышки","Четыре коротких импульса"],
  ["✥","Пять вспышек","Пять коротких импульсов"],
  ["⚑","Серия 3+1","Три быстрых и один длинный"],
  ["⚐","Серия 1+3","Один длинный и три быстрых"],
  ["🚨","Ритм сирены","Чередование коротких и длинных сигналов"],
  ["🔥","Костёр","Живое нерегулярное мерцание"],
  ["✦","Светлячок","Редкие одиночные вспышки"],
  ["✧","Звёздный дождь","Нерегулярные короткие вспышки"],
  ["☾","Тихий режим","Очень редкие мягкие сигналы"]
];
let currentEffect=0,currentCode="",updating=false;
const effectCategories = [
  "steady",
  ...Array(13).fill("rhythm"),
  "ambient",
  "signal","signal","rhythm","signal","rhythm","rhythm","signal","signal","signal","signal","signal","signal","signal","signal","signal","signal","signal",
  "ambient","ambient","ambient","ambient"
];
let activeEffectFilter="all";
function effectMatches(item,id){
  const q=(document.getElementById("effectSearch")?.value||"").trim().toLocaleLowerCase("ru");
  const category=effectCategories[id]||"signal";
  const categoryOK=activeEffectFilter==="all"||category===activeEffectFilter;
  const textOK=!q||(item.join(" ").toLocaleLowerCase("ru").includes(q));
  return categoryOK&&textOK;
}
function renderEffects(activeId){
  const grid=document.getElementById("effectsGrid");
  const visible=effects.map((item,id)=>({item,id})).filter(x=>effectMatches(x.item,x.id));
  grid.innerHTML=visible.map(({item,id})=>`<button class="effect ${id===activeId?"active":""}" id="effect-${id}" onclick="selectEffect(${id})" aria-label="${item[1]}: ${item[2]}"><span class="effect-icon">${item[0]}</span><span class="effect-name">${item[1]}</span><span class="effect-description">${item[2]}</span></button>`).join("");
  document.getElementById("effectCounter").textContent=`${visible.length} ИЗ ${effects.length}`;
  if(!visible.length)grid.innerHTML='<div class="pair-note">Ничего не найдено. Измени запрос или категорию.</div>';
}
function markActiveEffect(id){currentEffect=id;renderEffects(id)}
function toast(message){const el=document.getElementById("toast");el.textContent=message;el.classList.add("show");setTimeout(()=>el.classList.remove("show"),1900)}
async function request(url,options={}){const response=await fetch(url,{cache:"no-store",...options});if(!response.ok)throw new Error("HTTP "+response.status);return response}
async function selectEffect(id){if(updating||id<0||id>=effects.length)return;updating=true;try{await request("/effect?id="+id);markActiveEffect(id);await updateStatus()}catch(e){toast("Ошибка управления устройством")}finally{updating=false}}
async function setPower(state){try{await request(state?"/on":"/off");await updateStatus();toast(state?"Свет включён":"Свет выключен")}catch(e){toast("Нет связи с устройством")}}

let scanTimer=null;
async function scanNetworks(){const state=document.getElementById("scanState"),list=document.getElementById("networkList");state.textContent="Сканирование эфира…";list.innerHTML="";try{await request("/scan");if(scanTimer)clearInterval(scanTimer);scanTimer=setInterval(fetchNetworks,1200);fetchNetworks()}catch(e){state.textContent="Не удалось запустить сканирование";toast("Ошибка сканирования Wi-Fi")}}
async function fetchNetworks(){try{const r=await fetch("/scan-results",{cache:"no-store"});const d=await r.json();if(d.scanning){document.getElementById("scanState").textContent="Ищем сети поблизости…";return}clearInterval(scanTimer);scanTimer=null;const list=document.getElementById("networkList");list.innerHTML="";if(!d.networks||!d.networks.length){document.getElementById("scanState").textContent="Сети не найдены. Можно ввести SSID вручную.";return}document.getElementById("scanState").textContent="Найдено сетей: "+d.networks.length+". Выбери сеть для подключения.";d.networks.forEach(n=>{const b=document.createElement("button");b.type="button";b.className="network-item";b.innerHTML=`<span class="net-lock">${n.secure?"▣":"◉"}</span><span class="net-name"></span><span class="net-meta">${n.rssi} dBm · CH ${n.channel}</span>`;b.querySelector(".net-name").textContent=n.ssid||"Скрытая сеть";if(n.ssid&&n.ssid===document.getElementById("currentSSID").textContent){b.classList.add("current");b.querySelector(".net-meta").textContent+=" · ТЕКУЩАЯ";}b.onclick=()=>{document.getElementById("ssidInput").value=n.ssid;document.querySelectorAll(".network-item").forEach(x=>x.classList.remove("selected"));b.classList.add("selected");document.getElementById("wifiPassword").focus()};list.appendChild(b)})}catch(e){document.getElementById("scanState").textContent="Ошибка получения списка сетей"}}
async function copyCode(){if(!currentCode||currentCode==="—"){toast("Код пока недоступен");return}try{if(navigator.clipboard&&window.isSecureContext){await navigator.clipboard.writeText(currentCode)}else{const el=document.createElement("textarea");el.value=currentCode;el.style.position="fixed";el.style.opacity="0";document.body.appendChild(el);el.select();const ok=document.execCommand("copy");document.body.removeChild(el);if(!ok)throw new Error("Copy failed")}toast("Код сопряжения скопирован")}catch(e){toast("Не удалось скопировать код")}}
async function resetWiFi(){if(!confirm("Удалить только Wi-Fi настройки? Matter-сопряжение сохранится."))return;try{await request("/wifi-reset",{method:"POST"});toast("Wi-Fi настройки сбрасываются")}catch(e){toast("Команда отправлена")}}
async function resetDevice(){if(!confirm("Полностью сбросить устройство? Будут удалены Wi-Fi настройки и Matter-сопряжение."))return;try{await request("/reset");toast("Устройство сбрасывается")}catch(e){toast("Команда отправлена")}}
let statusFetchInProgress=false;
let brightnessDragging=false,brightnessPending=false,brightnessUiEpoch=0;
async function updateStatus(){if(statusFetchInProgress)return;const requestBrightnessEpoch=brightnessUiEpoch;statusFetchInProgress=true;try{const response=await fetch("/api",{cache:"no-store"});if(!response.ok)throw new Error("API");const d=await response.json();document.getElementById("lightState").textContent=d.on?"Свет включён":"Свет выключен";document.getElementById("lightDescription").textContent=d.effectName;document.getElementById("heroIcon").classList.toggle("active",d.on);if(!brightnessDragging&&!brightnessPending&&requestBrightnessEpoch===brightnessUiEpoch){brightnessSlider.value=d.brightness||80;document.getElementById("brightnessValue").textContent=(d.brightness||80)+"%";}if(!speedDragging){speedSlider.value=d.speed||100;document.getElementById("speedValue").textContent=(d.speed||100)+"%";}document.getElementById("wifiStatus").textContent=d.currentSSID||d.wifi;document.getElementById("currentSSID").textContent=d.currentSSID||"Не подключено";document.getElementById("wifiSignal").textContent=d.rssi>-127?`Сигнал ${d.rssi} dBm · ${d.wifi}`:d.wifi;document.getElementById("ipStatus").textContent=d.ip;document.getElementById("matterStatus").textContent=d.matter;document.getElementById("commissionStatus").textContent=d.commissioned?"Сопряжено":"Ожидает сопряжения";const mt=document.getElementById("mqttStatusText"),mb=document.getElementById("mqttStatusBadge");if(mt&&mb){mt.textContent=d.mqttStatus||"Не настроен";mb.textContent=d.mqttConnected?"ПОДКЛЮЧЕНО":(d.mqttEnabled?"НЕТ СВЯЗИ":"ВЫКЛЮЧЕНО");mb.className=d.mqttConnected?"online":(d.mqttEnabled?"error":"");}document.getElementById("connectionText").textContent="ONLINE";document.getElementById("connectionDot").style.background="#36d9a0";currentCode=d.code||"";document.getElementById("pairCode").textContent=currentCode||"Недоступен";if(d.effectId!==currentEffect)markActiveEffect(d.effectId);const qr=document.getElementById("qrcode");if(d.qr&&window.QRCode&&qr.dataset.value!==d.qr){qr.innerHTML="";new QRCode(qr,{text:d.qr,width:190,height:190,correctLevel:QRCode.CorrectLevel.M});qr.dataset.value=d.qr}}catch(e){document.getElementById("connectionText").textContent="OFFLINE";document.getElementById("connectionDot").style.background="#ff647e"}finally{statusFetchInProgress=false}}
// Theme preference: auto follows the operating-system appearance.
const themeButtons={auto:document.getElementById("themeAuto"),light:document.getElementById("themeLight"),dark:document.getElementById("themeDark")};
let themeMode=localStorage.getItem("gw-theme")||"auto";
function applyTheme(){const effective=themeMode==="auto"?(window.matchMedia&&window.matchMedia("(prefers-color-scheme: light)").matches?"light":"dark"):themeMode;document.body.dataset.theme=effective;Object.entries(themeButtons).forEach(([k,b])=>b.classList.toggle("active",k===themeMode));const meta=document.querySelector('meta[name="theme-color"]');if(meta)meta.content=effective==="light"?"#edf2fa":"#080b14";}
Object.entries(themeButtons).forEach(([mode,button])=>button.addEventListener("click",()=>{themeMode=mode;localStorage.setItem("gw-theme",mode);applyTheme()}));
if(window.matchMedia){const media=window.matchMedia("(prefers-color-scheme: light)");if(media.addEventListener)media.addEventListener("change",()=>{if(themeMode==="auto")applyTheme()});}
applyTheme();
let speedDragging=false;
const speedSlider=document.getElementById("speedSlider");
speedSlider.addEventListener("input",()=>{speedDragging=true;document.getElementById("speedValue").textContent=speedSlider.value+"%";});
speedSlider.addEventListener("change",async()=>{speedDragging=false;try{await request("/speed?value="+speedSlider.value);toast("Скорость: "+speedSlider.value+"%");await updateStatus()}catch(e){toast("Не удалось изменить скорость")}});
const brightnessSlider=document.getElementById("brightnessSlider");
brightnessSlider.addEventListener("input",()=>{brightnessDragging=true;brightnessUiEpoch++;document.getElementById("brightnessValue").textContent=brightnessSlider.value+"%";});
brightnessSlider.addEventListener("change",async()=>{const commandEpoch=++brightnessUiEpoch;const requested=Number(brightnessSlider.value);brightnessDragging=false;brightnessPending=true;try{await request("/brightness?value="+requested);if(commandEpoch===brightnessUiEpoch){brightnessPending=false;brightnessSlider.value=String(requested);document.getElementById("brightnessValue").textContent=requested+"%";}toast("Яркость установлена: "+requested+"%");await updateStatus()}catch(e){if(commandEpoch===brightnessUiEpoch)brightnessPending=false;toast("Не удалось изменить яркость");await updateStatus()}});

// Web diagnostics are opt-in: no polling and no RAM ring-buffer until started.
let logEnabled=false,logPaused=false,logLastText="";
async function refreshSerialLog(){if(!logEnabled||logPaused||!document.querySelector('[data-panel="system"].active'))return;try{const r=await fetch("/logs",{cache:"no-store"});if(!r.ok)throw new Error("HTTP "+r.status);const t=await r.text();if(t!==logLastText){logLastText=t;const box=document.getElementById("serialLog");box.textContent=t||"Ожидание сообщений…";if(document.getElementById("logAutoScroll").checked)box.scrollTop=box.scrollHeight;}document.getElementById("logState").textContent="СБОР · LIVE";}catch(e){document.getElementById("logState").textContent="НЕТ СВЯЗИ"}}
document.getElementById("logStart").addEventListener("click",async()=>{try{await request("/logs/start",{method:"POST"});logEnabled=true;logPaused=false;logLastText="";document.getElementById("logStart").disabled=true;document.getElementById("logStop").disabled=false;document.getElementById("serialLog").textContent="Сбор запущен…";document.getElementById("logState").textContent="СБОР · LIVE";refreshSerialLog();}catch(e){toast("Не удалось запустить диагностику")}});
document.getElementById("logStop").addEventListener("click",async()=>{try{await request("/logs/stop",{method:"POST"});logEnabled=false;document.getElementById("logStart").disabled=false;document.getElementById("logStop").disabled=true;document.getElementById("logState").textContent="ОСТАНОВЛЕНА";document.getElementById("serialLog").textContent=logLastText||"Сбор остановлен.";}catch(e){toast("Не удалось остановить диагностику")}});
document.getElementById("logClear").addEventListener("click",async()=>{try{await request("/logs/clear",{method:"POST"});logLastText="";document.getElementById("serialLog").textContent=logEnabled?"Буфер очищен. Сбор продолжается…":"Буфер очищен. Диагностика выключена.";}catch(e){toast("Не удалось очистить журнал")}});
setInterval(refreshSerialLog,4000);
async function loadMqttSettings(){try{const r=await request("/mqtt/config");const d=await r.json();document.getElementById("mqttEnabled").checked=!!d.enabled;document.getElementById("mqttHost").value=d.host||"";document.getElementById("mqttPort").value=d.port||1883;document.getElementById("mqttUser").value=d.user||"";document.getElementById("mqttRoot").value=d.root||"goodwin/GW-ESP32-001";document.getElementById("mqttPassword").placeholder=d.passwordSet?"Пароль сохранён · введите новый для замены":"Пароль MQTT";}catch(e){toast("Не удалось загрузить MQTT настройки")}}
async function saveMqttForm(showMessage=true){const f=document.getElementById("mqttForm"),body=new URLSearchParams();for(const [k,v] of new FormData(f).entries())body.append(k,v);await request("/mqtt/save",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body});document.getElementById("mqttPassword").value="";document.getElementById("mqttClearPassword").checked=false;await loadMqttSettings();if(showMessage)toast("MQTT настройки сохранены");await updateStatus()}
document.getElementById("mqttForm").addEventListener("submit",async e=>{e.preventDefault();try{await saveMqttForm(true)}catch(err){toast("Не удалось сохранить MQTT настройки: "+err.message)}});
async function testMqtt(){const b=document.getElementById("mqttTestButton");b.disabled=true;b.textContent="Проверяем…";try{await saveMqttForm(false);const r=await fetch("/mqtt/test",{method:"POST",cache:"no-store"});const d=await r.json();toast(d.ok?"MQTT: соединение установлено":"MQTT: "+(d.message||"ошибка подключения"));await updateStatus()}catch(e){toast("MQTT: "+(e.message||"ошибка проверки"))}finally{b.disabled=false;b.textContent="Проверить соединение"}}
loadMqttSettings();
renderEffects(0);
document.getElementById("effectSearch").addEventListener("input",()=>renderEffects(currentEffect));
document.querySelectorAll(".filter-chip").forEach(button=>button.addEventListener("click",()=>{activeEffectFilter=button.dataset.filter;document.querySelectorAll(".filter-chip").forEach(x=>x.classList.toggle("active",x===button));renderEffects(currentEffect)}));
document.querySelectorAll(".help details").forEach(item=>item.addEventListener("toggle",()=>{if(item.open){document.querySelectorAll(".help details").forEach(other=>{if(other!==item&&other.open)other.open=false;});}}));
updateStatus();setInterval(updateStatus,800);
</script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/qrcodejs/1.0.0/qrcode.min.js"></script>
<script>
// ---- GoodWin Smart Hub navigation: five persistent sections ----
(function initGoodWinTabs(){
 const app=document.querySelector('.app'); if(!app) return;
 const header=app.querySelector('.header');
 const tabs=document.createElement('nav'); tabs.className='gw-tabs'; tabs.setAttribute('aria-label','Разделы Smart Hub');
 const defs=[['overview','⌂  Обзор'],['screen','▣  Экран'],['network','⌁  Сеть'],['integrations','⌘  Интеграции'],['system','⚙  Система']];
 const panels={};
 defs.forEach((d,i)=>{const b=document.createElement('button');b.type='button';b.className='gw-tab'+(i===0?' active':'');b.textContent=d[1];b.dataset.tab=d[0];tabs.appendChild(b);const p=document.createElement('div');p.className='gw-panel'+(i===0?' active':'');p.dataset.panel=d[0];panels[d[0]]=p;});
 header.insertAdjacentElement('afterend',tabs);
 const footer=app.querySelector('.footer');
 const nodes=Array.from(app.children).filter(n=>n!==header&&n!==tabs&&n!==footer);
 nodes.forEach(n=>{
  let key='overview';
  if(n.classList.contains('hub-config-card')) key='screen';
  else if(n.classList.contains('hub-climate-card')) key='overview';
  else if(n.classList.contains('wifi-card')) key='network';
  else if(n.classList.contains('mqtt-card')||n.classList.contains('matter-pairing')) key='integrations';
  else if(n.id==='serialMonitor'||n.classList.contains('system-monitor')||n.classList.contains('help-card')||n.classList.contains('maintenance')) key='system';
  if(n.classList.contains('hub-climate-card') && key==='overview') panels[key].insertBefore(n,panels[key].firstChild);
  else panels[key].appendChild(n);
 });
 defs.forEach(d=>app.insertBefore(panels[d[0]],footer||null));
 tabs.addEventListener('click',e=>{const b=e.target.closest('[data-tab]');if(!b)return;tabs.querySelectorAll('.gw-tab').forEach(x=>x.classList.toggle('active',x===b));Object.keys(panels).forEach(k=>panels[k].classList.toggle('active',k===b.dataset.tab));});
})();

// ---- Screen configuration and weather telemetry ----
let hubConfigBusy=false,hubConfigDirty=false,hubConfigSaving=false,hubConfigSavePending=false,hubConfigRevision=0;
let hubBrightnessInteracting=false;
async function loadHubConfig(silent=false){
 const slider=document.getElementById('hubBrightness');
 const testFps=document.getElementById('hubTestFps');
 // Never let the periodic poll move a slider while a finger/mouse/key is using it,
 // or while a local change is waiting for the ESP32 to acknowledge the save.
 if(hubConfigBusy||hubConfigDirty||hubConfigSaving||hubBrightnessInteracting||document.activeElement===slider||document.activeElement===testFps)return;
 hubConfigBusy=true;
 const requestRevision=hubConfigRevision;
 try{const r=await fetch('/hub/config',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);const d=await r.json();
  if(hubConfigDirty||hubConfigSaving||hubBrightnessInteracting||document.activeElement===slider||document.activeElement===testFps||requestRevision!==hubConfigRevision)return;
  document.getElementById('hubLayout').value=d.layout??0;document.getElementById('hubSaver').value=d.saver??0;document.getElementById('hubSaverIdle').value=d.saverIdle??60;
  document.getElementById('hubOrientation').value=d.orientation??0;document.getElementById('hubDisplayEnabled').checked=!!d.display;
  // Server is authoritative only when the user is not editing the control.
  slider.value=d.brightness??85;document.getElementById('hubBrightnessValue').textContent=(d.brightness??85)+'%';
  testFps.value=d.testFps??20;document.getElementById('hubTestFpsValue').textContent=(d.testFps??20)+' FPS';
 }catch(e){if(!silent)toast('Не удалось загрузить настройки экрана');}
 finally{hubConfigBusy=false;}
}
async function startHubSaverNow(){
 const b=document.getElementById('hubSaverStart');b.disabled=true;
 try{const r=await fetch('/hub/saver/start',{method:'POST',cache:'no-store'});const d=await r.json();if(!r.ok)throw new Error(d.message||('HTTP '+r.status));toast('Заставка запущена');}
 catch(e){toast(e.message||'Не удалось запустить заставку');}
 finally{b.disabled=false;}
}
async function saveHubConfig(){
 hubConfigDirty=true;
 if(hubConfigSaving){hubConfigSavePending=true;return;}
 hubConfigSaving=true;
 const saveRevision=++hubConfigRevision;
 const submittedBrightness=document.getElementById('hubBrightness').value;
 const body=new URLSearchParams();body.set('layout',document.getElementById('hubLayout').value);body.set('saver',document.getElementById('hubSaver').value);body.set('saverIdle',document.getElementById('hubSaverIdle').value);
 body.set('orientation',document.getElementById('hubOrientation').value);body.set('display',document.getElementById('hubDisplayEnabled').checked?'1':'0');body.set('brightness',submittedBrightness);body.set('testFps',document.getElementById('hubTestFps').value);
 try{const r=await fetch('/hub/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body,cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);await r.json();
  // Do not copy the response value back into the thumb: a Matter update or a
  // newer local movement must not visually jump when an older POST completes.
  if(saveRevision===hubConfigRevision){document.getElementById('hubBrightnessValue').textContent=document.getElementById('hubBrightness').value+'%';}
 }catch(e){toast('Не удалось сохранить настройки экрана');}
 finally{
  hubConfigSaving=false;
  if(hubConfigSavePending){hubConfigSavePending=false;saveHubConfig();}
  else if(saveRevision===hubConfigRevision)hubConfigDirty=false;
 }
}
['hubLayout','hubSaver','hubOrientation','hubDisplayEnabled','hubBrightness','hubTestFps'].forEach(id=>{const el=document.getElementById(id);if(el){if(id==='hubBrightness'){
  el.addEventListener('pointerdown',()=>{hubBrightnessInteracting=true;});
  el.addEventListener('pointerup',()=>{hubBrightnessInteracting=false;});
  el.addEventListener('pointercancel',()=>{hubBrightnessInteracting=false;});
  el.addEventListener('blur',()=>{hubBrightnessInteracting=false;});
  el.addEventListener('input',()=>{hubConfigDirty=true;hubConfigRevision++;document.getElementById('hubBrightnessValue').textContent=el.value+'%';});
 }if(id==='hubTestFps'){el.addEventListener('input',()=>{document.getElementById('hubTestFpsValue').textContent=el.value+' FPS';});}el.addEventListener('change',saveHubConfig);}});
async function setWeatherSource(value){try{const r=await fetch("/weather/source?source="+encodeURIComponent(value),{method:"POST",cache:"no-store"});if(!r.ok)throw new Error("HTTP "+r.status);const d=await r.json();document.getElementById("hubWeatherState").textContent=d.message||"Источник сохранён · обновление…";}catch(e){document.getElementById("hubWeatherState").textContent="Ошибка выбора: "+e.message;}}
async function forceWeatherRefresh(){const b=document.getElementById("hubWeatherRefresh");b.disabled=true;b.textContent="Запрос…";try{const r=await fetch("/weather/refresh",{method:"POST",cache:"no-store"});if(!r.ok)throw new Error("HTTP "+r.status);document.getElementById("hubWeatherState").textContent="Принудительное обновление…";}catch(e){document.getElementById("hubWeatherState").textContent="Ошибка запроса: "+e.message;}finally{setTimeout(()=>{b.disabled=false;b.textContent="↻ Обновить сейчас";},1800);}}
async function updateHubTelemetry(){
 try{const r=await fetch('/hub/data',{cache:'no-store'});const d=await r.json();
  document.getElementById('hubIndoorTemp').textContent=d.indoorTemp==null?'—':Number(d.indoorTemp).toFixed(1)+' °C';
  document.getElementById('hubIndoorHumidity').textContent=d.indoorHumidity==null?'—':Number(d.indoorHumidity).toFixed(1)+' %';
  document.getElementById('hubSensorState').textContent=(d.indoorTemp==null||d.indoorHumidity==null)?'Нет данных':'DHT22 · ONLINE';
  document.getElementById('hubOutdoor').textContent=d.outdoorTemp==null?'—':Number(d.outdoorTemp).toFixed(1)+' °C';
  document.getElementById('hubFeelsLike').textContent=d.feelsLike==null?'—':Number(d.feelsLike).toFixed(1)+' °C';
  document.getElementById('hubOutdoorHumidity').textContent=d.outdoorHumidity==null?'—':Number(d.outdoorHumidity).toFixed(1)+' %';
  document.getElementById('hubPressure').textContent=d.pressure==null?'—':Number(d.pressure).toFixed(0)+' hPa';
  document.getElementById('hubWind').textContent=d.windSpeed==null?'—':Number(d.windSpeed).toFixed(1)+' km/h · '+(d.windDirection==null?'—':Number(d.windDirection).toFixed(0)+'°');
  document.getElementById('hubCloud').textContent=d.cloudCover==null?'—':Number(d.cloudCover).toFixed(0)+' %';
  document.getElementById('hubUv').textContent=d.uvIndex==null?'—':Number(d.uvIndex).toFixed(1);
  document.getElementById('hubRain').textContent=d.precipProbability==null?'—':d.precipProbability+'% · '+(d.precipitationMm==null?'—':Number(d.precipitationMm).toFixed(1))+' мм';
  document.getElementById('hubWeatherState').textContent=(d.weatherStatus||'Нет статуса')+' · '+(d.weatherSource||'—');const ws=document.getElementById('hubWeatherSource');if(ws&&ws.value!==String(d.weatherMode??0))ws.value=String(d.weatherMode??0);
 }catch(e){document.getElementById('hubWeatherState').textContent='Нет связи с ESP32';}
}
async function updateSystemMonitor(){if(!document.querySelector('[data-panel="system"].active'))return;try{const r=await fetch("/system",{cache:"no-store"});if(!r.ok)throw new Error("HTTP "+r.status);const d=await r.json();const mb=v=>(Number(v)/1024).toFixed(1)+" KB";document.getElementById("sysCpu").textContent=d.cpuMHz+" MHz";document.getElementById("sysTemp").textContent=(d.chipTempC===null||d.chipTempC===undefined||!Number.isFinite(Number(d.chipTempC)))?"недоступно":Number(d.chipTempC).toFixed(1)+" °C";document.getElementById("sysHeap").textContent=mb(d.freeHeap);document.getElementById("sysMinHeap").textContent=mb(d.minFreeHeap);document.getElementById("sysLargest").textContent=mb(d.largestHeap);document.getElementById("sysPsram").textContent=d.psramTotal?mb(d.psramFree)+" / "+mb(d.psramTotal):"нет";document.getElementById("sysFlash").textContent=(Number(d.flashSize)/1048576).toFixed(1)+" MB";document.getElementById("sysRssi").textContent=d.wifiRssi===-127?"нет связи":d.wifiRssi+" dBm";const u=Number(d.uptime);document.getElementById("sysUptime").textContent=Math.floor(u/86400)+"д "+Math.floor(u%86400/3600)+"ч "+Math.floor(u%3600/60)+"м";document.getElementById("sysFps").textContent=d.tftFps+" FPS";document.getElementById("sysTftDraw").textContent=d.tftDrawUs+" µs (avg "+d.tftDrawAvgUs+")";document.getElementById("sysTftPhase").textContent=["PATCH 5×10","AREA 40×40","FULL 128×160"][d.tftPhase]||"—";document.getElementById("sysBoard").textContent=d.boardProfile||"—";document.getElementById("sysChip").textContent=d.chipModel||"—";document.getElementById("sysChipInfo").textContent=(d.chipRevision??"—")+" · "+(d.chipCores??"—")+" ядра";document.getElementById("sysFlashSpeed").textContent=(d.flashSpeedMHz?d.flashSpeedMHz+" MHz":"—")+" · "+(d.flashMode||"—");document.getElementById("sysCore").textContent=d.coreVersion||"—";document.getElementById("sysSdk").textContent=d.sdkVersion||"—";document.getElementById("sysSketch").textContent=mb(d.sketchSize)+" / свободно "+mb(d.freeSketchSpace);document.getElementById("sysMonitorState").textContent="LIVE · "+new Date().toLocaleTimeString();}catch(e){document.getElementById("sysMonitorState").textContent="НЕТ СВЯЗИ · "+(e&&e.message?e.message:"ошибка запроса");}}
loadHubConfig();updateHubTelemetry();updateSystemMonitor();setInterval(updateSystemMonitor,2000);setInterval(updateHubTelemetry,5000);setInterval(()=>{if(document.querySelector('[data-panel=\"screen\"].active'))loadHubConfig(true);},2000);

</script>
</body></html>
)rawliteral";

// ============================================================
// HTTP handlers
// ============================================================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

String jsonEscape(const String &value) {
  String out; out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if ((uint8_t)c < 0x20) out += ' ';
    else out += c;
  }
  return out;
}


// ============================================================
// MQTT + Home Assistant MQTT Discovery
// ============================================================
String mqttTopic(const char *suffix) { return mqttRoot + "/" + suffix; }

String mqttJsonString(const String &payload, const char *key) {
  String needle = String("\"") + key + "\"";
  int p = payload.indexOf(needle);
  if (p < 0) return String();
  p = payload.indexOf(':', p + needle.length());
  if (p < 0) return String();
  p++;
  while (p < (int)payload.length() && isspace((unsigned char)payload[p])) p++;
  if (p >= (int)payload.length() || payload[p] != '"') return String();
  int e = payload.indexOf('"', p + 1);
  if (e < 0) return String();
  return payload.substring(p + 1, e);
}

long mqttJsonNumber(const String &payload, const char *key, bool *found = nullptr) {
  String needle = String("\"") + key + "\"";
  int p = payload.indexOf(needle);
  if (p < 0) { if (found) *found = false; return 0; }
  p = payload.indexOf(':', p + needle.length());
  if (p < 0) { if (found) *found = false; return 0; }
  p++;
  while (p < (int)payload.length() && isspace((unsigned char)payload[p])) p++;
  int e = p;
  while (e < (int)payload.length() && (isDigit(payload[e]) || payload[e] == '-')) e++;
  if (e == p) { if (found) *found = false; return 0; }
  if (found) *found = true;
  return payload.substring(p, e).toInt();
}

void publishMqttState() {
  // Matter callbacks may execute outside loop(); defer MQTT I/O to mqttLoop().
  mqttStateDirty = true;
}

void publishMqttStateNow() {
  if (!mqttClient.connected()) return;
  String payload = "{\"state\":\"";
  payload += lightOn ? "ON" : "OFF";
  payload += "\",\"brightness\":";
  payload += String((uint16_t)brightnessPercent * 255U / 100U);
  payload += ",\"effect\":\"" + jsonEscape(effects[currentEffect].name) + "\"}";
  mqttClient.publish(mqttTopic("light/state").c_str(), payload.c_str(), true);
  mqttClient.publish(mqttTopic("speed/state").c_str(), String(effectSpeedPercent).c_str(), true);
  mqttClient.publish(mqttTopic("effect/state").c_str(), effects[currentEffect].name, true);
  mqttClient.publish(mqttTopic("display/state").c_str(), (hubDisplayEnabled && displayMatterOn) ? "ON" : "OFF", true);
  mqttClient.publish(mqttTopic("display_brightness/state").c_str(), String(displayBrightnessPercent).c_str(), true);
  mqttClient.publish(mqttTopic("screen_layout/state").c_str(), HUB_LAYOUT_NAMES[hubLayoutId], true);
  mqttClient.publish(mqttTopic("screensaver/state").c_str(), HUB_SAVER_NAMES[hubSaverId], true);
  static const char *orientationNames[] = {"Портрет", "Альбомная", "Портрет 180°", "Альбомная 180°"};
  mqttClient.publish(mqttTopic("display_orientation/state").c_str(), orientationNames[hubOrientation], true);
  publishMqttTelemetry();
  mqttStateDirty = false;
}

void publishMqttSensorDiscovery(const char *objectId, const char *name, const char *unit,
                                const char *deviceClass, const char *icon) {
  String config = "{\"name\":\"" + String(name) + "\",\"unique_id\":\"gw_esp_001_" + String(objectId) +
    "\",\"state_topic\":\"" + mqttTopic((String(objectId) + "/state").c_str()) +
    "\",\"availability_topic\":\"" + mqttTopic("availability") +
    "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"";
  if (unit && unit[0]) config += ",\"unit_of_measurement\":\"" + String(unit) + "\"";
  if (deviceClass && deviceClass[0]) config += ",\"device_class\":\"" + String(deviceClass) + "\"";
  if (icon && icon[0]) config += ",\"icon\":\"" + String(icon) + "\"";
  config += ",\"state_class\":\"measurement\",\"device\":{\"identifiers\":[\"GW-ESP32-001\"],\"name\":\"GoodWin Smart Hub\",\"manufacturer\":\"GoodWin\",\"model\":\"ESP32 Smart Hub\",\"sw_version\":\"3.11.0\"}}";
  const String topic = "homeassistant/sensor/goodwin_" + String(objectId) + "/config";
  mqttClient.publish(topic.c_str(), config.c_str(), true);
}

void publishMqttDiscovery() {
  const String device = "\"device\":{\"identifiers\":[\"GW-ESP32-001\"],\"name\":\"GoodWin Smart Hub\",\"manufacturer\":\"GoodWin\",\"model\":\"ESP32 Smart Hub\",\"sw_version\":\"3.11.0\"}";
  String light = "{\"name\":\"GoodWin Smart Light\",\"default_entity_id\":\"light.goodwin_smart_light\",\"unique_id\":\"gw_esp_001_mqtt_light\",\"schema\":\"json\",\"state_topic\":\"";
  light += mqttTopic("light/state") + "\",\"command_topic\":\"" + mqttTopic("light/set");
  light += "\",\"availability_topic\":\"" + mqttTopic("availability");
  light += "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\",\"brightness\":true,\"brightness_scale\":255,\"effect\":true,\"effect_list\":[";
  for (uint8_t i = 0; i < FX_COUNT; ++i) { if (i) light += ','; light += "\"" + jsonEscape(effects[i].name) + "\""; }
  light += "],\"supported_color_modes\":[\"brightness\"]," + device + "}";
  mqttClient.publish("homeassistant/light/goodwin_smart_light/config", light.c_str(), true);

  // Dedicated effect selector: available in HA even when the light card hides
  // the MQTT light effect dropdown in its current frontend presentation.
  String effectSelect = "{\"name\":\"GoodWin Light Effect\",\"unique_id\":\"gw_esp_001_light_effect\",\"state_topic\":\"" +
    mqttTopic("effect/state") + "\",\"command_topic\":\"" + mqttTopic("effect/set") + "\",\"options\":[";
  for (uint8_t i = 0; i < FX_COUNT; ++i) {
    if (i) effectSelect += ',';
    effectSelect += "\"" + jsonEscape(effects[i].name) + "\"";
  }
  effectSelect += "],\"availability_topic\":\"" + mqttTopic("availability") +
    "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"," + device + "}";
  mqttClient.publish("homeassistant/select/goodwin_light_effect/config", effectSelect.c_str(), true);

  String speed = "{\"name\":\"GoodWin Effect Speed\",\"unique_id\":\"gw_esp_001_effect_speed\",\"state_topic\":\"" +
    mqttTopic("speed/state") + "\",\"command_topic\":\"" + mqttTopic("speed/set");
  speed += "\",\"min\":25,\"max\":300,\"step\":5,\"mode\":\"slider\",\"unit_of_measurement\":\"%\",\"icon\":\"mdi:speedometer\"," + device + "}";
  mqttClient.publish("homeassistant/number/goodwin_effect_speed/config", speed.c_str(), true);

  String display = "{\"name\":\"GoodWin Display\",\"unique_id\":\"gw_esp_001_display\",\"state_topic\":\"" + mqttTopic("display/state") +
    "\",\"command_topic\":\"" + mqttTopic("display/set") + "\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"availability_topic\":\"" +
    mqttTopic("availability") + "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"," + device + "}";
  mqttClient.publish("homeassistant/switch/goodwin_display/config", display.c_str(), true);
  String displayBrightness = "{\"name\":\"GoodWin Display Brightness\",\"unique_id\":\"gw_esp_001_display_brightness\",\"state_topic\":\"" +
    mqttTopic("display_brightness/state") + "\",\"command_topic\":\"" + mqttTopic("display_brightness/set") +
    "\",\"min\":1,\"max\":100,\"step\":1,\"mode\":\"slider\",\"unit_of_measurement\":\"%\",\"availability_topic\":\"" +
    mqttTopic("availability") + "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"," + device + "}";
  mqttClient.publish("homeassistant/number/goodwin_display_brightness/config", displayBrightness.c_str(), true);

  String layout = "{\"name\":\"GoodWin Screen Layout\",\"unique_id\":\"gw_esp_001_screen_layout\",\"state_topic\":\"" +
    mqttTopic("screen_layout/state") + "\",\"command_topic\":\"" + mqttTopic("screen_layout/set") + "\",\"options\":[";
  for (uint8_t i=0;i<HUB_LAYOUT_COUNT;++i) { if(i) layout+=','; layout += "\"" + jsonEscape(HUB_LAYOUT_NAMES[i]) + "\""; }
  layout += "],\"availability_topic\":\"" + mqttTopic("availability") + "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"," + device + "}";
  mqttClient.publish("homeassistant/select/goodwin_screen_layout/config", layout.c_str(), true);

  String saver = "{\"name\":\"GoodWin Screensaver\",\"unique_id\":\"gw_esp_001_screensaver\",\"state_topic\":\"" +
    mqttTopic("screensaver/state") + "\",\"command_topic\":\"" + mqttTopic("screensaver/set") + "\",\"options\":[";
  for (uint8_t i=0;i<HUB_SAVER_COUNT;++i) { if(i) saver+=','; saver += "\"" + jsonEscape(HUB_SAVER_NAMES[i]) + "\""; }
  saver += "],\"availability_topic\":\"" + mqttTopic("availability") + "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"," + device + "}";
  mqttClient.publish("homeassistant/select/goodwin_screensaver/config", saver.c_str(), true);

  static const char *orientationNames[] = {"Портрет", "Альбомная", "Портрет 180°", "Альбомная 180°"};
  String orientation = "{\"name\":\"GoodWin Display Orientation\",\"unique_id\":\"gw_esp_001_display_orientation\",\"state_topic\":\"" +
    mqttTopic("display_orientation/state") + "\",\"command_topic\":\"" + mqttTopic("display_orientation/set") + "\",\"options\":[";
  for (uint8_t i=0;i<4;++i) { if(i) orientation+=','; orientation += "\"" + String(orientationNames[i]) + "\""; }
  orientation += "],\"availability_topic\":\"" + mqttTopic("availability") + "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"," + device + "}";
  mqttClient.publish("homeassistant/select/goodwin_display_orientation/config", orientation.c_str(), true);

  publishMqttSensorDiscovery("indoor_temperature","GoodWin Indoor Temperature","°C","temperature","mdi:thermometer");
  publishMqttSensorDiscovery("indoor_humidity","GoodWin Indoor Humidity","%","humidity","mdi:water-percent");
  publishMqttSensorDiscovery("outdoor_temperature","GoodWin Outdoor Temperature","°C","temperature","mdi:thermometer");
  publishMqttSensorDiscovery("outdoor_humidity","GoodWin Outdoor Humidity","%","humidity","mdi:water-percent");
  publishMqttSensorDiscovery("feels_like_temperature","GoodWin Feels Like Temperature","°C","temperature","mdi:thermometer-lines");
  publishMqttSensorDiscovery("outdoor_pressure","GoodWin Outdoor Pressure","hPa","pressure","mdi:gauge");
  publishMqttSensorDiscovery("wind_speed","GoodWin Wind Speed","km/h","wind_speed","mdi:weather-windy");
  publishMqttSensorDiscovery("wind_direction","GoodWin Wind Direction","°","","mdi:compass");
  publishMqttSensorDiscovery("cloud_cover","GoodWin Cloud Cover","%","","mdi:weather-cloudy");
  publishMqttSensorDiscovery("uv_index","GoodWin UV Index","","","mdi:sunglasses");
  publishMqttSensorDiscovery("precipitation_probability","GoodWin Precipitation Probability","%","","mdi:weather-rainy");
  publishMqttSensorDiscovery("precipitation_mm","GoodWin Precipitation","mm","precipitation","mdi:weather-pouring");
  publishMqttSensorDiscovery("wifi_rssi","GoodWin WiFi Signal","dBm","signal_strength","mdi:wifi");
  publishMqttSensorDiscovery("uptime","GoodWin Uptime","s","duration","mdi:timer-outline");
  publishMqttSensorDiscovery("free_heap","GoodWin Free Heap","B","","mdi:memory");
  publishMqttSensorDiscovery("min_free_heap","GoodWin Minimum Free Heap","B","","mdi:memory");
  publishMqttSensorDiscovery("largest_heap_block","GoodWin Largest Heap Block","B","","mdi:memory");
  publishMqttSensorDiscovery("cpu_freq_mhz","GoodWin CPU Frequency","MHz","","mdi:chip");
  publishMqttSensorDiscovery("chip_temp_c","GoodWin Chip Temperature","°C","temperature","mdi:thermometer");
  publishMqttSensorDiscovery("psram_free","GoodWin Free PSRAM","B","","mdi:memory");
  publishMqttSensorDiscovery("tft_fps","GoodWin TFT FPS","fps","","mdi:monitor");
  publishMqttSensorDiscovery("tft_draw_us","GoodWin TFT Draw Time","µs","","mdi:timer-outline");
}

void publishMqttTelemetry() {
  if (!mqttClient.connected()) return;
  auto pub = [&](const char *suffix, const String &value) {
    const String topic = mqttTopic((String(suffix) + "/state").c_str());
    mqttClient.publish(topic.c_str(), value.c_str(), true);
  };
  pub("indoor_temperature", sensorValid ? String(indoorTempC,1) : String("unknown"));
  pub("indoor_humidity", sensorValid ? String(indoorHumidity,1) : String("unknown"));
  pub("outdoor_temperature", weatherValid ? String(outdoorTempC,1) : String("unknown"));
  pub("outdoor_humidity", weatherValid && !isnan(outdoorHumidity) ? String(outdoorHumidity,1) : String("unknown"));
  pub("feels_like_temperature", weatherValid && !isnan(outdoorFeelsLikeC) ? String(outdoorFeelsLikeC,1) : String("unknown"));
  pub("outdoor_pressure", weatherValid && !isnan(outdoorPressureHpa) ? String(outdoorPressureHpa,1) : String("unknown"));
  pub("wind_speed", weatherValid && !isnan(outdoorWindSpeedKmh) ? String(outdoorWindSpeedKmh,1) : String("unknown"));
  pub("wind_direction", weatherValid && !isnan(outdoorWindDirection) ? String(outdoorWindDirection,0) : String("unknown"));
  pub("cloud_cover", weatherValid && !isnan(outdoorCloudCover) ? String(outdoorCloudCover,0) : String("unknown"));
  pub("uv_index", weatherValid && !isnan(outdoorUvIndex) ? String(outdoorUvIndex,1) : String("unknown"));
  pub("precipitation_probability", weatherValid && outdoorPrecipProbability>=0 ? String(outdoorPrecipProbability) : String("unknown"));
  pub("precipitation_mm", weatherValid && !isnan(outdoorPrecipMm) ? String(outdoorPrecipMm,1) : String("unknown"));
  pub("wifi_rssi", WiFi.status()==WL_CONNECTED ? String(WiFi.RSSI()) : String("unknown"));
  pub("uptime", String(millis()/1000UL));
  pub("free_heap", String(ESP.getFreeHeap()));
  pub("min_free_heap", String(ESP.getMinFreeHeap()));
  pub("largest_heap_block", String(ESP.getMaxAllocHeap()));
  pub("cpu_freq_mhz", String(ESP.getCpuFreqMHz()));
  pub("chip_temp_c", (isnan(goodwinReadChipTemperatureC()) ? String("unknown") : String(goodwinReadChipTemperatureC(), 1)));
  pub("psram_free", String(ESP.getFreePsram()));
  pub("tft_fps", String(hubTestMeasuredFps));
  pub("tft_draw_us", String(hubTestDrawUsLast));
}
void persistHubConfigFromMqtt() {
  if (statePrefs.begin("hubcfg", false)) {
    statePrefs.putUChar("layout", hubLayoutId);
    statePrefs.putUChar("saver", hubSaverId);
    statePrefs.putUChar("orient", hubOrientation);
    statePrefs.putBool("display", hubDisplayEnabled);
    statePrefs.putBool("dispOn", displayMatterOn);
    statePrefs.putUChar("dispBri", displayBrightnessPercent);
    statePrefs.putUChar("testFps", hubTestFpsTarget);
    statePrefs.end();
  }
}

void applyMqttDisplayState(bool enabled) {
  hubMarkActivity();
  hubDisplayEnabled = enabled;
  displayMatterOn = enabled;
  if (!enabled) hubSaverActive = false;
  hubDisplayDirty = true;
  if (matterStarted) DisplayDimmer.setOnOff(enabled);
  writeDisplayBacklight();
  persistHubConfigFromMqtt();
}

void applyMqttDisplayBrightness(int value) {
  hubMarkActivity();
  displayBrightnessPercent = (uint8_t)constrain(value, 1, 100);
  if (matterStarted) DisplayDimmer.setBrightness(percentToMatter(displayBrightnessPercent));
  writeDisplayBacklight();
  persistHubConfigFromMqtt();
}

void applyMqttScreenLayout(const String &value) {
  for (uint8_t i = 0; i < HUB_LAYOUT_COUNT; ++i) {
    if (value == HUB_LAYOUT_NAMES[i]) {
      hubMarkActivity();
      hubLayoutId = i;
      hubDisplayDirty = true;
      persistHubConfigFromMqtt();
      return;
    }
  }
}

void applyMqttScreensaver(const String &value) {
  for (uint8_t i = 0; i < HUB_SAVER_COUNT; ++i) {
    if (value == HUB_SAVER_NAMES[i]) {
      hubMarkActivity();
      hubSaverId = i;
      hubSaverActive = false;
      hubDisplayDirty = true;
      persistHubConfigFromMqtt();
      return;
    }
  }
}

void applyMqttDisplayOrientation(const String &value) {
  static const char *orientationNames[] = {"Портрет", "Альбомная", "Портрет 180°", "Альбомная 180°"};
  for (uint8_t i = 0; i < 4; ++i) {
    if (value == orientationNames[i]) {
      hubMarkActivity();
      hubOrientation = i;
      hubTFT.setRotation(hubOrientation);
      hubTFT.fillScreen(0x0000);
      hubDisplayDirty = true;
      LOG_B("TFT rotation changed via MQTT: %u, dimensions=%dx%d", hubOrientation, hubTFT.width(), hubTFT.height());
      persistHubConfigFromMqtt();
      return;
    }
  }
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {
  String body;
  body.reserve(length + 1);
  for (unsigned int i = 0; i < length; ++i) body += (char)payload[i];
  body.trim();
  const String topicName(topic);

  if (topicName == mqttTopic("light/set")) {
    const String state = mqttJsonString(body, "state");
    if (state == "OFF") setPower(false);
    else if (state == "ON" && !lightOn) setPower(true);

    bool hasBrightness = false;
    const long b = mqttJsonNumber(body, "brightness", &hasBrightness);
    if (hasBrightness) {
      if (b <= 0) setPower(false);
      else setBrightness((uint8_t)constrain((int)lroundf((float)b * 100.0f / 255.0f), 1, 100));
    }

    const String effectName = mqttJsonString(body, "effect");
    if (!effectName.isEmpty()) {
      for (uint8_t i = 0; i < FX_COUNT; ++i) {
        if (effectName == effects[i].name) { setEffect(i); break; }
      }
    }
    publishMqttState();
  } else if (topicName == mqttTopic("speed/set")) {
    setEffectSpeed((uint16_t)constrain(body.toInt(), 25, 300));
    publishMqttState();
  } else if (topicName == mqttTopic("effect/set")) {
    for (uint8_t i = 0; i < FX_COUNT; ++i) {
      if (body == effects[i].name) {
        setEffect(i);
        break;
      }
    }
    publishMqttState();
  } else if (topicName == mqttTopic("display/set")) {
    if (body == "ON" || body == "1" || body == "true") applyMqttDisplayState(true);
    else if (body == "OFF" || body == "0" || body == "false") applyMqttDisplayState(false);
    publishMqttState();
  } else if (topicName == mqttTopic("display_brightness/set")) {
    applyMqttDisplayBrightness(body.toInt());
    publishMqttState();
  } else if (topicName == mqttTopic("screen_layout/set")) {
    applyMqttScreenLayout(body);
    publishMqttState();
  } else if (topicName == mqttTopic("screensaver/set")) {
    applyMqttScreensaver(body);
    publishMqttState();
  } else if (topicName == mqttTopic("display_orientation/set")) {
    applyMqttDisplayOrientation(body);
    publishMqttState();
  }
}

void loadMqttConfig() {
  Preferences prefs;
  prefs.begin("mqttcfg", true);
  mqttEnabled = prefs.getBool("enabled", false);
  mqttHost = prefs.getString("host", "");
  mqttPort = prefs.getUShort("port", MQTT_DEFAULT_PORT);
  mqttUser = prefs.getString("user", "");
  mqttPassword = prefs.getString("pass", "");
  mqttRoot = prefs.getString("root", MQTT_DEFAULT_ROOT);
  prefs.end();
  if (mqttPort == 0) mqttPort = MQTT_DEFAULT_PORT;
  mqttRoot.trim();
  while (mqttRoot.startsWith("/")) mqttRoot.remove(0, 1);
  while (mqttRoot.endsWith("/")) mqttRoot.remove(mqttRoot.length() - 1);
  if (mqttRoot.isEmpty()) mqttRoot = MQTT_DEFAULT_ROOT;
}

bool mqttConfigured() {
  return mqttEnabled && !mqttHost.isEmpty() && mqttPort > 0 && !mqttRoot.isEmpty();
}

void setupMqtt() {
  loadMqttConfig();
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(4096);
  mqttClient.setKeepAlive(30);
  // PubSubClient is synchronous: bound socket waits so a missing broker cannot
  // freeze the main UI loop for the library's long default timeout.
  mqttNetwork.setTimeout(1000);
  mqttClient.setSocketTimeout(1);
  if (mqttConfigured()) {
    mqttClient.setServer(mqttHost.c_str(), mqttPort);
    mqttLastError = "Ожидание подключения";
    LOG_M("MQTT configured from saved settings: %s:%u", mqttHost.c_str(), mqttPort);
  } else {
    mqttLastError = mqttEnabled ? "Укажи адрес брокера" : "MQTT выключен";
    LOG_M("MQTT disabled or not configured; configure it in the web panel");
  }
}

bool mqttConnectNow() {
  if (WiFi.status() != WL_CONNECTED) { mqttLastError = "Нет подключения к Wi-Fi"; return false; }
  if (!mqttConfigured()) { mqttLastError = mqttEnabled ? "Не указан адрес брокера" : "MQTT выключен"; return false; }
  if (mqttClient.connected()) return true;
  mqttClient.setServer(mqttHost.c_str(), mqttPort);
  String clientId = "GoodWin-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  const bool auth = !mqttUser.isEmpty();
  const bool ok = mqttClient.connect(clientId.c_str(), auth ? mqttUser.c_str() : nullptr,
    auth ? mqttPassword.c_str() : nullptr, mqttTopic("availability").c_str(), 0, true, "offline");
  if (ok) {
    mqttLastError = "Подключено";
    LOG_M("MQTT connected to %s:%u", mqttHost.c_str(), mqttPort);
    mqttClient.publish(mqttTopic("availability").c_str(), "online", true);
    mqttClient.subscribe(mqttTopic("light/set").c_str());
    mqttClient.subscribe(mqttTopic("speed/set").c_str());
    mqttClient.subscribe(mqttTopic("effect/set").c_str());
    mqttClient.subscribe(mqttTopic("display/set").c_str());
    mqttClient.subscribe(mqttTopic("display_brightness/set").c_str());
    mqttClient.subscribe(mqttTopic("screen_layout/set").c_str());
    mqttClient.subscribe(mqttTopic("screensaver/set").c_str());
    mqttClient.subscribe(mqttTopic("display_orientation/set").c_str());
    publishMqttDiscovery();
    publishMqttStateNow();
  } else {
    mqttLastError = "Ошибка MQTT: " + String(mqttClient.state());
    LOG_W("MQTT connect failed, state=%d", mqttClient.state());
  }
  return ok;
}

void mqttLoop() {
  if (!mqttConfigured() || WiFi.status() != WL_CONNECTED) return;
  static uint32_t lastAttempt = 0;
  if (!mqttClient.connected()) {
    // TCP connect is synchronous on this core. Defer retries during animations.
    if (hubSaverActive) return;
    if (millis() - lastAttempt < 30000) return;
    lastAttempt = millis();
    mqttConnectNow();
  }
  if (mqttClient.connected()) {
    mqttClient.loop();
    if (mqttStateDirty) publishMqttStateNow();
    static uint32_t lastTelemetryAt = 0;
    if ((uint32_t)(millis() - lastTelemetryAt) >= 30000UL) {
      lastTelemetryAt = millis();
      publishMqttTelemetry();
    }
  }
}

void handleMqttConfigGet() {
  String json = "{\"enabled\":" + String(mqttEnabled ? "true" : "false");
  json += ",\"host\":\"" + jsonEscape(mqttHost) + "\",\"port\":" + String(mqttPort);
  json += ",\"user\":\"" + jsonEscape(mqttUser) + "\",\"root\":\"" + jsonEscape(mqttRoot);
  json += "\",\"passwordSet\":" + String(mqttPassword.isEmpty() ? "false" : "true") + "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleMqttSave() {
  const bool enabled = server.hasArg("enabled") && server.arg("enabled") == "1";
  String host = server.arg("host"); host.trim();
  String user = server.arg("user"); user.trim();
  String root = server.arg("root"); root.trim();
  int port = server.arg("port").toInt();
  if (port < 1 || port > 65535) { server.send(400, "text/plain", "Invalid MQTT port"); return; }
  if (enabled && host.isEmpty()) { server.send(400, "text/plain", "MQTT host required"); return; }
  if (root.isEmpty()) root = MQTT_DEFAULT_ROOT;
  while (root.startsWith("/")) root.remove(0, 1);
  while (root.endsWith("/")) root.remove(root.length() - 1);
  if (root.indexOf(' ') >= 0 || root.indexOf('#') >= 0 || root.indexOf('+') >= 0) {
    server.send(400, "text/plain", "Invalid MQTT root topic"); return;
  }
  Preferences prefs;
  prefs.begin("mqttcfg", false);
  String password = prefs.getString("pass", "");
  if (server.hasArg("clear_password") && server.arg("clear_password") == "1") password = "";
  else if (server.hasArg("password") && !server.arg("password").isEmpty()) password = server.arg("password");
  prefs.putBool("enabled", enabled); prefs.putString("host", host); prefs.putUShort("port", (uint16_t)port);
  prefs.putString("user", user); prefs.putString("pass", password); prefs.putString("root", root);
  prefs.end();
  if (mqttClient.connected()) { mqttClient.publish(mqttTopic("availability").c_str(), "offline", true); mqttClient.disconnect(); }
  loadMqttConfig();
  mqttLastError = mqttEnabled ? "Настройки сохранены · подключение…" : "MQTT выключен";
  server.send(200, "application/json", "{\"ok\":true}");
  LOG_M("MQTT settings saved; enabled=%s host=%s port=%u", mqttEnabled ? "yes" : "no", mqttHost.c_str(), mqttPort);
}

void handleMqttTest() {
  const bool ok = mqttConnectNow();
  String json = "{\"ok\":" + String(ok ? "true" : "false") + ",\"message\":\"" + jsonEscape(mqttLastError) + "\"}";
  server.send(ok ? 200 : 503, "application/json; charset=utf-8", json);
}

void handleScanStart() {
  if (portalMode && WiFi.getMode() == WIFI_AP) WiFi.mode(WIFI_AP_STA);
  if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"started\":true,\"scanning\":true}");
    return;
  }
  WiFi.scanDelete();
  const int16_t result = WiFi.scanNetworks(true, true);
  if (result == WIFI_SCAN_FAILED) {
    server.send(500, "application/json", "{\"started\":false}");
    return;
  }
  server.send(200, "application/json", "{\"started\":true,\"scanning\":true}");
}

void handleScanResults() {
  const int16_t count = WiFi.scanComplete();
  if (count == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"scanning\":true}");
    return;
  }
  if (count == WIFI_SCAN_FAILED) {
    server.send(200, "application/json", "{\"scanning\":false,\"networks\":[]}");
    return;
  }
  String json = "{\"scanning\":false,\"networks\":[";
  for (int i = 0; i < count; ++i) {
    if (i) json += ',';
    json += "{\"ssid\":\"" + jsonEscape(WiFi.SSID(i)) + "\",\"rssi\":" + String(WiFi.RSSI(i));
    json += ",\"channel\":" + String(WiFi.channel(i));
    json += ",\"secure\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true") + "}";
  }
  json += "]}";
  server.send(200, "application/json; charset=utf-8", json);
  WiFi.scanDelete();
}

void handleApi() {
  String json;
  json.reserve(1000);
  json = "{";
  json += "\"on\":"; json += (lightOn ? "true" : "false"); json += ",";
  json += "\"effectId\":"; json += String(currentEffect); json += ",";
  json += "\"brightness\":"; json += String(brightnessPercent); json += ",";
  json += "\"speed\":"; json += String(effectSpeedPercent); json += ",";
  json += "\"effectName\":\""; json += effects[currentEffect].name; json += "\",";
  json += "\"wifi\":\"";
  if (portalMode) json += "SETUP AP";
  else if (recoveryAPStarted) json += "RECOVERY AP";
  else json += (WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
  json += "\",";
  json += "\"ip\":\"";
  if (portalMode || recoveryAPStarted) {
    json += WiFi.softAPIP().toString();
    if (WiFi.status() == WL_CONNECTED) { json += " / STA "; json += WiFi.localIP().toString(); }
  } else json += WiFi.localIP().toString();
  json += "\",";
  json += "\"currentSSID\":\""; json += jsonEscape(WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String("")); json += "\",";
  json += "\"rssi\":"; json += String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -127); json += ",";
  json += "\"matter\":\""; json += (matterStarted ? "RUNNING" : "NOT STARTED"); json += "\",";
  json += "\"commissioned\":";
  json += (matterStarted && Matter.isDeviceCommissioned() ? "true" : "false");
  json += ",\"code\":\""; json += pairingCode; json += "\",";
  json += "\"qr\":\""; json += pairingQR; json += "\",";
  json += "\"mqttEnabled\":" + String(mqttEnabled ? "true" : "false") + ",";
  json += "\"mqttConnected\":" + String(mqttClient.connected() ? "true" : "false") + ",";
  json += "\"mqttStatus\":\"" + jsonEscape(mqttLastError) + "\",\"hubLayout\":" + String(hubLayoutId) + ",\"weatherValid\":" + String(weatherValid ? "true" : "false") + "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleSystemMetrics() {
  String json = "{";
  json += "\"cpuMHz\":" + String(ESP.getCpuFreqMHz()) + ",";
  const float chipTempC = goodwinReadChipTemperatureC();
  json += "\"chipTempC\":";
  if (isfinite(chipTempC)) json += String(chipTempC, 1); else json += "null";
  json += ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"minFreeHeap\":" + String(ESP.getMinFreeHeap()) + ",";
  json += "\"largestHeap\":" + String(ESP.getMaxAllocHeap()) + ",";
  json += "\"psramTotal\":" + String(ESP.getPsramSize()) + ",";
  json += "\"psramFree\":" + String(ESP.getFreePsram()) + ",";
  json += "\"flashSize\":" + String(ESP.getFlashChipSize()) + ",";
  json += "\"boardProfile\":\"" + jsonEscape(String(GOODWIN_BOARD_PROFILE)) + "\",";
  json += "\"chipModel\":\"" + jsonEscape(String(ESP.getChipModel())) + "\",";
  json += "\"chipRevision\":" + String(ESP.getChipRevision()) + ",";
  json += "\"chipCores\":" + String(ESP.getChipCores()) + ",";
  json += "\"flashSpeedMHz\":" + String(ESP.getFlashChipSpeed() / 1000000UL) + ",";
  json += "\"flashMode\":\"" + String(goodwinFlashModeName(ESP.getFlashChipMode())) + "\",";
  json += "\"coreVersion\":\"" + jsonEscape(String(ESP.getCoreVersion())) + "\",";
  json += "\"sdkVersion\":\"" + jsonEscape(String(ESP.getSdkVersion())) + "\",";
  json += "\"sketchSize\":" + String(ESP.getSketchSize()) + ",";
  json += "\"freeSketchSpace\":" + String(ESP.getFreeSketchSpace()) + ",";
  json += "\"uptime\":" + String(millis() / 1000UL) + ",";
  json += "\"wifiRssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -127) + ",";
  json += "\"tftFps\":" + String(hubTestMeasuredFps) + ",";
  json += "\"tftDrawUs\":" + String(hubTestDrawUsLast) + ",";
  json += "\"tftDrawAvgUs\":" + String(hubTestDrawUsAvg) + ",";
  json += "\"tftDrawMaxUs\":" + String(hubTestDrawUsMax) + ",";
  json += "\"tftPhase\":" + String(hubTestPhase) + "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleOn() { setPower(true); server.send(200, "text/plain", "OK"); }
void handleOff() { setPower(false); server.send(200, "text/plain", "OK"); }

void handleBrightness() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int value = server.arg("value").toInt();
  if (value < 1 || value > 100) { server.send(400, "text/plain", "Brightness must be 1..100"); return; }
  setBrightness((uint8_t)value);
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (!server.hasArg("value")) { server.send(400, "text/plain", "Missing value"); return; }
  int value = server.arg("value").toInt();
  if (value < 25 || value > 300) { server.send(400, "text/plain", "Speed must be 25..300"); return; }
  setEffectSpeed((uint16_t)value);
  server.send(200, "text/plain", "OK");
}

void handleEffect() {
  if (server.hasArg("id")) {
    const int id = server.arg("id").toInt();
    if (id >= 0 && id < FX_COUNT) setEffect((uint8_t)id);
    else { server.send(400, "text/plain", "Invalid effect"); return; }
  }
  server.send(200, "text/plain", "OK");
}

void handleWiFiSave() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  ssid.trim();
  if (ssid.isEmpty()) { server.send(400, "text/plain", "SSID required"); return; }

  netPrefs.begin("netcfg", false);
  netPrefs.putString("ssid", ssid);
  netPrefs.putString("password", password);
  netPrefs.end();

  server.send(200, "text/html; charset=utf-8",
    "<!doctype html><html lang='ru'><meta charset='utf-8'><body style='background:#101827;color:white;font-family:Arial;padding:40px'><h2>Настройки сохранены</h2><p>Устройство перезапускается...</p></body></html>");
  restartPending = true;
  pendingFactoryReset = false;
  restartAt = millis() + 900;
}

void handleReset() {
  if (resetting) { server.send(409, "text/plain", "Reset already in progress"); return; }
  resetting = true;
  server.send(200, "text/plain", "Factory reset started");
  restartPending = true;
  pendingFactoryReset = true;
  restartAt = millis() + 300;
}

// Clear only Wi-Fi credentials. Matter commissioning/fabric is preserved.
void handleWiFiReset() {
  if (resetting) { server.send(409, "text/plain", "Reset already in progress"); return; }
  resetting = true;
  netPrefs.begin("netcfg", false);
  netPrefs.clear();
  netPrefs.end();
  server.send(200, "text/plain", "Wi-Fi settings cleared; rebooting");
  restartPending = true;
  pendingFactoryReset = false;
  restartAt = millis() + 400;
  LOG_R("Wi-Fi credentials cleared; Matter fabric preserved");
}

void handleLogsStart() {
  char (*newBuffer)[WEB_LOG_LINE_SIZE] = (char (*)[WEB_LOG_LINE_SIZE])calloc(WEB_LOG_CAPACITY, WEB_LOG_LINE_SIZE);
  if (!newBuffer) { server.send(503, "text/plain", "Not enough heap for diagnostics"); return; }
  char (*oldBuffer)[WEB_LOG_LINE_SIZE] = nullptr;
  portENTER_CRITICAL(&webLogMux);
  oldBuffer = webLogLines;
  webLogLines = newBuffer;
  memset(webLogLines, 0, WEB_LOG_CAPACITY * WEB_LOG_LINE_SIZE);
  webLogWriteIndex = 0;
  webLogEnabled = true;
  portEXIT_CRITICAL(&webLogMux);
  if (oldBuffer) free(oldBuffer);
  server.send(200, "text/plain", "Diagnostics started");
  LOG_H("Web diagnostics collection started");
}

void handleLogsStop() {
  char (*oldBuffer)[WEB_LOG_LINE_SIZE] = nullptr;
  portENTER_CRITICAL(&webLogMux);
  webLogEnabled = false;
  oldBuffer = webLogLines;
  webLogLines = nullptr;
  webLogWriteIndex = 0;
  portEXIT_CRITICAL(&webLogMux);
  if (oldBuffer) free(oldBuffer);
  server.send(200, "text/plain", "Diagnostics stopped");
}

void handleLogs() {
  if (!webLogEnabled || !webLogLines) { server.send(200, "text/plain; charset=utf-8", "Диагностика выключена"); return; }
  String out;
  out.reserve(WEB_LOG_CAPACITY * 100);
  uint32_t endIndex;
  portENTER_CRITICAL(&webLogMux); endIndex = webLogWriteIndex; portEXIT_CRITICAL(&webLogMux);
  const uint32_t count = min((uint32_t)WEB_LOG_CAPACITY, endIndex);
  const uint32_t start = endIndex - count;
  char line[WEB_LOG_LINE_SIZE];
  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t slot = (start + i) % WEB_LOG_CAPACITY;
    portENTER_CRITICAL(&webLogMux);
    if (webLogLines) strlcpy(line, webLogLines[slot], sizeof(line)); else line[0] = 0;
    portEXIT_CRITICAL(&webLogMux);
    if (line[0]) { out += line; out += '\n'; }
  }
  server.send(200, "text/plain; charset=utf-8", out);
}

void handleLogsClear() {
  portENTER_CRITICAL(&webLogMux);
  if (webLogLines) memset(webLogLines, 0, WEB_LOG_CAPACITY * WEB_LOG_LINE_SIZE);
  webLogWriteIndex = 0;
  portEXIT_CRITICAL(&webLogMux);
  server.send(200, "text/plain; charset=utf-8", "OK");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api", HTTP_GET, handleApi);
  server.on("/system", HTTP_GET, handleSystemMetrics);
  server.on("/weather/refresh", HTTP_POST, handleWeatherRefresh);
  server.on("/weather/source", HTTP_POST, handleWeatherSource);
  server.on("/hub/config", HTTP_GET, handleHubConfig);
  server.on("/hub/config", HTTP_POST, handleHubConfig);
  server.on("/hub/saver/start", HTTP_POST, handleHubSaverStart);
  server.on("/hub/data", HTTP_GET, handleHubData);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/logs/start", HTTP_POST, handleLogsStart);
  server.on("/logs/stop", HTTP_POST, handleLogsStop);
  server.on("/logs/clear", HTTP_POST, handleLogsClear);
  server.on("/mqtt/config", HTTP_GET, handleMqttConfigGet);
  server.on("/mqtt/save", HTTP_POST, handleMqttSave);
  server.on("/mqtt/test", HTTP_POST, handleMqttTest);
  server.on("/scan", HTTP_GET, handleScanStart);
  server.on("/scan-results", HTTP_GET, handleScanResults);
  server.on("/on", HTTP_GET, handleOn);
  server.on("/off", HTTP_GET, handleOff);
  server.on("/brightness", HTTP_GET, handleBrightness);
  server.on("/speed", HTTP_GET, handleSpeed);
  server.on("/effect", HTTP_GET, handleEffect);
  server.on("/wifi", HTTP_POST, handleWiFiSave);
  server.on("/reset", HTTP_GET, handleReset);
  server.on("/wifi-reset", HTTP_POST, handleWiFiReset);
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  });
  server.begin();
  LOG_H("HTTP server started on port 80");
}

// ============================================================
// Wi-Fi setup portal + connection
// ============================================================
void startPortal() {
  portalMode = true;
  matterStarted = false;
  recoveryAPStarted = false;
  setupAPStarted = false;
  portalLastStaAttemptAt = 0;
  portalStaAttemptStartedAt = 0;
  portalStaAttemptInProgress = false;
  portalWifiConnectedSince = 0;
  portalMatterStarted = false;

  LOG_W("Starting Wi-Fi setup portal in AP+STA mode; saved credentials will be retried");

  // Keep both interfaces enabled: the setup AP remains available while STA
  // periodically retries the saved router credentials.
  WiFi.setAutoReconnect(false); // We schedule retries ourselves to avoid overlapping attempts.
  WiFi.disconnect(false, false);
  delay(150);
  WiFi.mode(WIFI_AP_STA);
  delay(300);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  if (!startAccessPoint(true)) {
    LOG_ERR("First AP+STA start failed; restarting Wi-Fi driver and retrying");
    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.mode(WIFI_AP_STA);
    delay(400);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    if (!startAccessPoint(true)) {
      LOG_ERR("Setup AP could not be started after retry; will retry in loop");
      lastPortalApRetryAt = millis();
    } else {
      setupAPStarted = true;
    }
  } else {
    setupAPStarted = true;
  }

  // Start the first background STA attempt shortly after the AP is ready.
  portalLastStaAttemptAt = millis() - 15000;
  portalLedLastToggle = 0;
  portalLedState = false;
  writeLEDLevel(0);
}

bool startAccessPoint(bool recoveryMode) {
  const IPAddress apIP(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.softAPConfig(apIP, gateway, subnet)) {
    LOG_ERR("softAPConfig failed");
  }

  // Fully reset the AP interface before creating it. This avoids stale STA/AP state
  // after a failed association to a previously saved router.
  WiFi.softAPdisconnect(true);
  delay(180);
  WiFi.mode((recoveryMode || portalMode) ? WIFI_AP_STA : WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  if (!WiFi.softAPConfig(apIP, gateway, subnet)) LOG_ERR("softAPConfig retry failed");
  const bool ok = WiFi.softAP(AP_NAME, AP_PASSWORD, 1, false, 4);
  if (ok) {
    delay(250);
    const IPAddress ip = WiFi.softAPIP();
    if (!captiveDnsStarted) {
      captiveDnsStarted = captiveDns.start(53, "*", apIP);
      LOG_W("Captive DNS %s", captiveDnsStarted ? "started" : "failed");
    }
    LOG_W("%s AP started: SSID=%s IP=%s channel=1 clients=%u",
      recoveryMode ? "Recovery" : "Setup", AP_NAME, ip.toString().c_str(), WiFi.softAPgetStationNum());
    return true;
  }

  LOG_ERR("softAP start failed; mode=%u status=%u", (unsigned)WiFi.getMode(), (unsigned)WiFi.status());
  return false;
}

void updatePortalIndicator() {
  const uint32_t now = millis();
  if (now - portalLedLastToggle >= 500) {
    portalLedLastToggle = now;
    portalLedState = !portalLedState;
    writeLEDLevel(portalLedState ? 35 : 0);
  }

  // Keep the setup AP alive even if STA reconnects. Retry saved credentials
  // every 15 seconds, never starting a second attempt while already connected.
  if (!setupAPStarted && now - lastPortalApRetryAt >= 5000) {
    lastPortalApRetryAt = now;
    LOG_W("Retrying setup AP start");
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    setupAPStarted = startAccessPoint(true);
  }

  if (WiFi.status() == WL_CONNECTED) {
    portalStaAttemptInProgress = false;
    if (portalWifiConnectedSince == 0) {
      portalWifiConnectedSince = now;
      LOG_W("Saved Wi-Fi reconnected; keeping ESP-SETUP online for 30 seconds");
    }

    if (!portalMatterStarted && now - portalWifiConnectedSince >= 30000) {
      LOG_W("Wi-Fi stable; closing setup AP and starting Matter/MQTT");
      if (captiveDnsStarted) {
        captiveDns.stop();
        captiveDnsStarted = false;
      }
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      portalMode = false;
      setupAPStarted = false;
#if !CONFIG_ENABLE_CHIPOBLE
      matterConnectWiFi(wifiSSID.c_str(), wifiPassword.c_str());
#endif
      startMatter();
      setupMqtt();
      if (MDNS.begin(MDNS_HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        LOG_W("mDNS: http://%s.local", MDNS_HOSTNAME);
      }
      portalMatterStarted = true;
    }
    return;
  }

  portalWifiConnectedSince = 0;
  if (wifiSSID.isEmpty()) return;

  const bool attemptTimedOut = portalStaAttemptInProgress &&
                               (now - portalStaAttemptStartedAt >= 25000);
  const bool canStartAttempt = !portalStaAttemptInProgress &&
                               (now - portalLastStaAttemptAt >= 15000);
  if (attemptTimedOut || canStartAttempt) {
    if (attemptTimedOut) {
      LOG_W("Saved Wi-Fi attempt timed out; scheduling another attempt");
      // Disconnect only the STA interface; keep ESP-SETUP running.
      WiFi.disconnect(false, false);
      delay(80);
    }
    portalLastStaAttemptAt = now;
    portalStaAttemptStartedAt = now;
    portalStaAttemptInProgress = true;
    LOG_W("Retrying saved Wi-Fi: %s", wifiSSID.c_str());
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  }
}

bool connectSavedWiFi() {
  netPrefs.begin("netcfg", true);
  wifiSSID = netPrefs.getString("ssid", "");
  wifiPassword = netPrefs.getString("password", "");
  netPrefs.end();

  if (wifiSSID.isEmpty()) {
    LOG_W("No saved Wi-Fi credentials");
    return false;
  }

  LOG_W("Connecting to saved Wi-Fi network; DHCP hostname=%s", WIFI_HOSTNAME);
// Connect with the Arduino Wi-Fi driver first. Do not call matterConnectWiFi()
  // here: that starts CHIP network handling even when the saved router is absent,
  // which leaves Matter repeatedly querying a nonexistent configured network while
  // the setup AP is running. Matter is started only after STA is actually connected.
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 20000) delay(250);
  if (WiFi.status() != WL_CONNECTED) {
    LOG_ERR("Wi-Fi connection failed; retaining credentials and entering AP+STA retry mode");
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false, false);
    delay(100);
    return false;
  }
  WiFi.setAutoReconnect(true);
  LOG_W("Connected; IP=%s", WiFi.localIP().toString().c_str());
#if !CONFIG_ENABLE_CHIPOBLE
  // On the original ESP32 Matter build, this helper enables STA IPv6 for CHIP.
  // Run it only after the saved network has proven reachable, so a failed Wi-Fi
  // attempt never starts Matter's network-status polling before the setup AP.
  LOG_W("Preparing Matter Wi-Fi/IPv6 network interface");
  matterConnectWiFi(wifiSSID.c_str(), wifiPassword.c_str());
#endif
  return true;
}

// ============================================================
// Wi-Fi recovery while Matter remains commissioned
// ============================================================
void processWiFiRecovery() {
  if (portalMode) return; // Initial setup portal is already active.

  const uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnectedSince = 0;

    if (!recoveryAPStarted && (lastWifiRetryAt == 0 || now - lastWifiRetryAt >= 5000)) {
      lastWifiRetryAt = now;
      LOG_W("Wi-Fi lost; isolating STA and starting stable recovery AP; Matter fabric is preserved");
      // Do not let STA keep scanning/reconnecting while the user is trying to join
      // ESP-SETUP. On a single 2.4 GHz radio, STA channel hopping can destabilize
      // an AP+STA recovery hotspot. Credentials remain in Preferences; Matter
      // fabric is not erased. A reboot after saving new Wi-Fi starts Matter again.
      WiFi.setAutoReconnect(false);
      WiFi.disconnect(false, false); // Disconnect STA, keep the driver configuration.
      delay(150);
      WiFi.mode(WIFI_AP_STA);
      delay(150);
      if (startAccessPoint(true)) {
        recoveryAPStarted = true;
        lastWifiRetryAt = now;
      } else {
        LOG_ERR("Could not start recovery AP; will retry on next recovery cycle");
      }
    }

    // Keep the recovery AP available while retrying the stored router credentials.
    // Each attempt has a timeout; never call begin() while another STA attempt is active.
    if (wifiSSID.isEmpty()) return;
    const bool attemptTimedOut = portalStaAttemptInProgress &&
                                 (now - portalStaAttemptStartedAt >= 25000);
    const bool canStartAttempt = !portalStaAttemptInProgress &&
                                 (now - portalLastStaAttemptAt >= 15000);
    if (attemptTimedOut || canStartAttempt) {
      if (attemptTimedOut) {
        LOG_W("Recovery STA attempt timed out; retrying while ESP-SETUP stays online");
        WiFi.disconnect(false, false);
        delay(80);
      }
      portalLastStaAttemptAt = now;
      portalStaAttemptStartedAt = now;
      portalStaAttemptInProgress = true;
      WiFi.setHostname(WIFI_HOSTNAME);
      LOG_W("Retrying saved Wi-Fi in recovery mode: %s", wifiSSID.c_str());
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    }
    return;
  }

  portalStaAttemptInProgress = false;
  if (wifiConnectedSince == 0) wifiConnectedSince = now;

  // Keep the recovery AP available until the station link is stable for 30 s.
  if (recoveryAPStarted && now - wifiConnectedSince >= 30000) {
    LOG_W("Wi-Fi stable; stopping recovery AP");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    recoveryAPStarted = false;
  }
}

// BOOT button: short click = next effect; 5-10 s = Wi-Fi reset;
// 10 s or longer = full Matter + Wi-Fi factory reset.
void processBootButton() {
  const bool raw = (digitalRead(BOOT_PIN) == HIGH);
  const uint32_t now = millis();

  if (raw != bootButtonLastRaw) {
    bootButtonLastRaw = raw;
    bootButtonDebounceAt = now;
  }

  if (now - bootButtonDebounceAt < 40) return;
  if (raw == bootButtonStable) return;

  bootButtonStable = raw;
  if (!bootButtonStable) {
    bootButtonPressed = true;
    bootButtonPressedAt = now;
    return;
  }

  if (!bootButtonPressed) return;
  bootButtonPressed = false;
  const uint32_t held = now - bootButtonPressedAt;

  if (held >= 10000) {
    LOG_BTN("BOOT held >=10s: full factory reset");
    factoryReset();
  } else if (held >= 5000) {
    LOG_BTN("BOOT held 5-10s: Wi-Fi-only reset");
    netPrefs.begin("netcfg", false);
    netPrefs.clear();
    netPrefs.end();
    LOG_R("Wi-Fi credentials cleared; Matter fabric preserved");
    ESP.restart();
  } else if (held >= 40) {
    if (portalMode) {
      LOG_BTN("Short press ignored in setup mode; hold 5s to clear Wi-Fi settings");
    } else {
      LOG_BTN("Short press: next effect");
      nextEffect();
    }
  }
}

// ============================================================
// Factory reset
// ============================================================
void factoryReset() {
  resetting = true;
  LOG_R("Factory reset requested");
  writeLED(false);

  netPrefs.begin("netcfg", false);
  netPrefs.clear();
  netPrefs.end();

  statePrefs.begin("ESPstate", false);
  statePrefs.clear();
  statePrefs.end();

  Preferences mqttPrefs;
  mqttPrefs.begin("mqttcfg", false);
  mqttPrefs.clear();
  mqttPrefs.end();

  if (matterStarted) {
    LOG_R("Decommissioning Matter fabric");
    Matter.decommission();
  }
  delay(1000);
  ESP.restart();
}

// Extract and URL-decode the Matter onboarding payload from Arduino's URL.
String decodeUrlComponent(const String &src) {
  String out; out.reserve(src.length());
  for (size_t i = 0; i < src.length(); ++i) {
    char c = src[i];
    if (c == '+') out += ' ';
    else if (c == '%' && i + 2 < src.length()) {
      char hex[3] = { src[i + 1], src[i + 2], 0 };
      char *end = nullptr;
      long v = strtol(hex, &end, 16);
      if (end && *end == 0) { out += (char)v; i += 2; }
      else out += c;
    } else out += c;
  }
  return out;
}

String matterPayloadFromUrl(const String &url) {
  int q = url.indexOf('?');
  if (q < 0) return url.startsWith("MT:") ? url : String();
  String query = url.substring(q + 1);
  int start = 0;
  while (start < query.length()) {
    int amp = query.indexOf('&', start);
    if (amp < 0) amp = query.length();
    String item = query.substring(start, amp);
    int eq = item.indexOf('=');
    if (eq >= 0 && item.substring(0, eq) == "data")
      return decodeUrlComponent(item.substring(eq + 1));
    start = amp + 1;
  }
  return String();
}

// ============================================================
// Matter climate endpoint: one Temperature Sensor endpoint + RH cluster.
// Uses the public esp_matter API shipped with Arduino-ESP32 3.3.12.
static bool createClimateMatterEndpoint(float initialTempC, float initialHumidity) {
  esp_matter::node_t *node = esp_matter::node::get();
  if (!node) {
    LOG_ERR("Matter node is not initialized; create climate endpoint after first accessory begin()");
    return false;
  }

  int16_t rawTemp = 0;
  if (!isnan(initialTempC)) {
    const float scaled = initialTempC * 100.0f;
    if (scaled < -32768.0f || scaled > 32767.0f) {
      LOG_ERR("Initial climate temperature is outside Matter int16 range: %.2f C", initialTempC);
      return false;
    }
    rawTemp = (int16_t)lroundf(scaled);
  }

  uint16_t rawHumidity = 0;
  if (!isnan(initialHumidity)) {
    const float scaled = initialHumidity * 100.0f;
    if (scaled < 0.0f || scaled > 10000.0f) {
      LOG_ERR("Initial climate humidity is outside Matter range: %.2f %%", initialHumidity);
      return false;
    }
    rawHumidity = (uint16_t)lroundf(scaled);
  }

  esp_matter::endpoint::temperature_sensor::config_t tempConfig;
  tempConfig.temperature_measurement.measured_value = rawTemp;
  tempConfig.temperature_measurement.min_measured_value = nullptr;
  tempConfig.temperature_measurement.max_measured_value = nullptr;

  esp_matter::endpoint_t *endpoint = esp_matter::endpoint::temperature_sensor::create(
    node, &tempConfig, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
  if (!endpoint) {
    LOG_ERR("Failed to create native Matter climate endpoint");
    return false;
  }

  esp_matter::cluster::relative_humidity_measurement::config_t humidityConfig;
  humidityConfig.measured_value = rawHumidity;
  humidityConfig.min_measured_value = static_cast<uint16_t>(0);
  humidityConfig.max_measured_value = static_cast<uint16_t>(10000);

  esp_matter::cluster_t *humidityCluster = esp_matter::cluster::relative_humidity_measurement::create(
    endpoint, &humidityConfig, esp_matter::CLUSTER_FLAG_SERVER);
  if (!humidityCluster) {
    LOG_ERR("Failed to add Relative Humidity Measurement cluster to climate endpoint");
    return false;
  }

  climateEndpointId = esp_matter::endpoint::get_id(endpoint);
  climateEndpointReady = (climateEndpointId != 0);
  LOG_M("Combined climate endpoint: %s (id=%u, clusters=0x0402+0x0405; deviceType=0x0302)",
        climateEndpointReady ? "OK" : "FAILED", climateEndpointId);
  return climateEndpointReady;
}

static bool updateClimateMatterValues(float temperatureC, float humidityPercent) {
  if (!climateEndpointReady || climateEndpointId == 0 || !matterStarted) return false;

  bool allOk = true;
  if (!isnan(temperatureC)) {
    const float scaled = temperatureC * 100.0f;
    if (scaled < -32768.0f || scaled > 32767.0f) return false;
    const int16_t rawTemp = (int16_t)lroundf(scaled);
    esp_matter_attr_val_t tempValue = esp_matter_nullable_int16(rawTemp);
    const esp_err_t tempErr = esp_matter::attribute::update(
      climateEndpointId, 0x0402, 0x0000, &tempValue);
    if (tempErr != ESP_OK) {
      LOG_ERR("Matter climate temperature update failed: err=%d", (int)tempErr);
      allOk = false;
    }
  }

  if (!isnan(humidityPercent)) {
    const float scaled = humidityPercent * 100.0f;
    if (scaled < 0.0f || scaled > 10000.0f) return false;
    const uint16_t rawHumidity = (uint16_t)lroundf(scaled);
    esp_matter_attr_val_t humidityValue = esp_matter_nullable_uint16(rawHumidity);
    const esp_err_t humidityErr = esp_matter::attribute::update(
      climateEndpointId, 0x0405, 0x0000, &humidityValue);
    if (humidityErr != ESP_OK) {
      LOG_ERR("Matter climate humidity update failed: err=%d", (int)humidityErr);
      allOk = false;
    }
  }
  return allOk;
}

// Matter initialization
// ============================================================
void startMatter() {
  LOG_M("Initializing Matter endpoint");

  Matter.setVendorName(DEVICE_VENDOR);
  Matter.setProductName(DEVICE_PRODUCT);
  Matter.setDeviceName(DEVICE_NAME);
  Matter.setSerialNumber(DEVICE_SERIAL);
  Matter.setHardwareVersion(1);
  Matter.setHardwareVersionString("ESP32 DevKit V1");

  statePrefs.begin("ESPstate", true);
  lightOn = statePrefs.getBool("on", false);
  currentEffect = statePrefs.getUChar("effect", FX_SOLID);
  brightnessPercent = statePrefs.getUChar("brightness", 80);
  effectSpeedPercent = statePrefs.getUShort("speed", 100);
  effectSpeedPercent = constrain(effectSpeedPercent, 25, 300);
  if (brightnessPercent < 1 || brightnessPercent > 100) brightnessPercent = 80;
  smoothedBrightnessPercent = brightnessPercent;
  lastBrightnessSmoothAt = millis();
  statePrefs.end();
  if (currentEffect >= FX_COUNT) currentEffect = FX_SOLID;

  const bool ledEndpointReady = DimmableLight.begin(lightOn, percentToMatter(brightnessPercent));
  DimmableLight.onChange(setLightState);
  LOG_M("LED dimmer endpoint: %s (id=%u)", ledEndpointReady ? "OK" : "FAILED", DimmableLight.getEndPointId());

  // Restore the display state. PWM on GPIO25 becomes physically effective
  // after the TFT backlight is wired through a transistor/MOSFET stage.
  statePrefs.begin("hubcfg", true);
  hubSaverIdleSeconds = statePrefs.getUShort("idleSec", 60);
  if (hubSaverIdleSeconds > 3600) hubSaverIdleSeconds = 60;
  // "display" is the canonical persisted state shared by Web and Matter.
  // Older builds could leave display/dispOn out of sync and boot with BL off.
  hubDisplayEnabled = statePrefs.getBool("display", true);
  displayMatterOn = hubDisplayEnabled;
  displayBrightnessPercent = statePrefs.getUChar("dispBri", 85);
  statePrefs.end();
  if (displayBrightnessPercent < 1 || displayBrightnessPercent > 100) displayBrightnessPercent = 85;
  const bool displayEndpointReady = DisplayDimmer.begin(displayMatterOn, percentToMatter(displayBrightnessPercent));
  DisplayDimmer.onChange(setDisplayState);
  LOG_M("Display dimmer endpoint: %s (id=%u)", displayEndpointReady ? "OK" : "FAILED", DisplayDimmer.getEndPointId());
  LOG_M("Display restored: %s, brightness=%u%%", displayMatterOn ? "ON" : "OFF", displayBrightnessPercent);
  writeDisplayBacklight();

  // One physical DHT22 supplies both clusters on a single Matter climate endpoint.
  if (!sensorValid) {
    const float h = hubDHT.readHumidity();
    const float t = hubDHT.readTemperature();
    if (!isnan(h) && !isnan(t)) { indoorHumidity = h; indoorTempC = t; sensorValid = true; }
  }
  climateEndpointReady = createClimateMatterEndpoint(
    sensorValid ? indoorTempC : NAN,
    sensorValid ? indoorHumidity : NAN);

  Matter.begin();
  matterWaitUntilReady();
  matterStarted = true;

  // Publish first real DHT22 sample after Matter stack startup.
  if (sensorValid && climateEndpointReady) {
    const bool climateSet = updateClimateMatterValues(indoorTempC, indoorHumidity);
    if (climateSet) {
      lastMatterTemperature = indoorTempC;
      lastMatterHumidity = indoorHumidity;
    }
    LOG_M("Initial combined climate values: T=%.2f C, RH=%.2f %% (%s)",
          indoorTempC, indoorHumidity, climateSet ? "OK" : "PARTIAL/FAILED");
  } else {
    lastMatterTemperature = NAN;
    lastMatterHumidity = NAN;
    LOG_ERR("Climate endpoint not ready or DHT22 has no valid startup sample");
  }

  pairingCode = Matter.getManualPairingCode();
  pairingQR = matterPayloadFromUrl(Matter.getOnboardingQRCodeUrl());
  if (pairingQR.isEmpty()) LOG_ERR("Could not extract Matter QR payload; use manual pairing code");

  LOG_M("Started; name=%s", DEVICE_NAME);
  LOG_M("Pairing code: %s", pairingCode.c_str());
  LOG_M("Commissioned: %s", Matter.isDeviceCommissioned() ? "YES" : "NO");

  effectStartedAt = millis();
  randomTimer = millis();

  // Do not call updateAccessory() here: it would invoke the callback and
  // could overwrite the restored local effect.
  updateEffects();
}

// ============================================================
// GoodWin Smart Hub: TFT + AM2302 + direct Open-Meteo access
// ============================================================
void setupSmartHubDisplay() {
  pinMode(TFT1_BL, OUTPUT);
  displayPwmReady = ledcAttach(TFT1_BL, PWM_FREQUENCY, 8);
  if (!displayPwmReady) LOG_ERR("TFT backlight PWM attach failed; using digital fallback");
  // Backlight must be active before panel initialization on this module.
  writeDisplayBacklight();
  hubTFT.init();
  hubValueCanvas.setColorDepth(16);
  void *valuePatchBuffer = hubValueCanvas.createSprite(HUB_VALUE_CANVAS_W, HUB_VALUE_CANVAS_H);
  LOG_B("LovyanGFX value sprite: %s (%u bytes)",
        valuePatchBuffer ? "READY" : "UNAVAILABLE",
        (unsigned)(HUB_VALUE_CANVAS_W * HUB_VALUE_CANVAS_H * sizeof(uint16_t)));
  hubTFT.setRotation(0);
  hubTFT.fillScreen(0x0000);
  LOG_B("TFT direct renderer ready; free heap=%u", ESP.getFreeHeap());
  hubTFT.setTextWrap(false);
  hubDHT.begin();
  writeDisplayBacklight();
  LOG_B("Smart Hub display initialized; rotation=%u", hubOrientation);
}

void updateHubClock() {
  struct tm ti;
  if (getLocalTime(&ti, 5)) {
    char t[8], d[16];
    strftime(t, sizeof(t), "%H:%M", &ti);
    strftime(d, sizeof(d), "%d.%m.%Y", &ti);
    timeText = t;
    dateText = d;
  }
}

// ---------- TFT visual primitives (LovyanGFX / ST7735S) ----------
static constexpr uint16_t HUB_BG       = 0x0862; // deep blue-black
static constexpr uint16_t HUB_PANEL    = 0x10A4;
static constexpr uint16_t HUB_PANEL2   = 0x1928;
static constexpr uint16_t HUB_LINE     = 0x31AE;
static constexpr uint16_t HUB_WHITE    = 0xFFFF;
static constexpr uint16_t HUB_MUTED    = 0x9CF3;
static constexpr uint16_t HUB_CYAN     = 0x5DDF;
static constexpr uint16_t HUB_GREEN    = 0x5F76;
static constexpr uint16_t HUB_YELLOW   = 0xFEA0;
static constexpr uint16_t HUB_ORANGE   = 0xFC20;
static constexpr uint16_t HUB_RED      = 0xF986;
static void drawValueBox(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t bg, uint16_t fg, uint8_t textSize, const char *value);

void hubCard(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t fill, uint16_t edge=HUB_LINE) {
  hubTFT.fillRoundRect(x, y, w, h, 7, fill);
  hubTFT.drawRoundRect(x, y, w, h, 7, edge);
}
void hubLabel(int16_t x, int16_t y, const char *s, uint16_t color=HUB_MUTED) {
  hubTFT.setTextSize(1); hubTFT.setTextColor(color, HUB_PANEL); hubTFT.setCursor(x,y); hubTFT.print(s);
}
void hubSunIcon(int16_t cx, int16_t cy, uint8_t r, uint16_t color) {
  hubTFT.fillCircle(cx,cy,r,color);
  for (uint8_t a=0;a<8;a++) {
    float ang=a*0.785398f; int16_t x1=cx+(int16_t)(cosf(ang)*(r+3)); int16_t y1=cy+(int16_t)(sinf(ang)*(r+3));
    int16_t x2=cx+(int16_t)(cosf(ang)*(r+7)); int16_t y2=cy+(int16_t)(sinf(ang)*(r+7));
    hubTFT.drawLine(x1,y1,x2,y2,color);
  }
}
void hubCloudIcon(int16_t cx, int16_t cy, uint8_t scale, uint16_t color) {
  hubTFT.fillCircle(cx-(8*scale),cy,7*scale,color);
  hubTFT.fillCircle(cx,cy-(5*scale),10*scale,color);
  hubTFT.fillCircle(cx+(10*scale),cy,7*scale,color);
  hubTFT.fillRoundRect(cx-(15*scale),cy-(1*scale),30*scale,10*scale,5*scale,color);
}
void hubDropIcon(int16_t cx, int16_t cy, uint8_t scale, uint16_t color) {
  hubTFT.fillCircle(cx,cy+(3*scale),4*scale,color);
  hubTFT.fillTriangle(cx,cy-(8*scale),cx-(4*scale),cy+(2*scale),cx+(4*scale),cy+(2*scale),color);
}
void hubWeatherIcon(int16_t cx,int16_t cy,uint8_t scale){
  if(!weatherValid){hubCloudIcon(cx,cy,scale,HUB_MUTED);return;}
  const int code=outdoorWeatherCode;
  if(code==0){hubSunIcon(cx,cy,8*scale,HUB_YELLOW);return;}
  if(code==1||code==2){hubSunIcon(cx-7*scale,cy-5*scale,6*scale,HUB_YELLOW);hubCloudIcon(cx+5*scale,cy+4*scale,scale,HUB_CYAN);return;}
  hubCloudIcon(cx,cy,scale,HUB_CYAN);
  if(scale==1 && ((code>=51&&code<=67)||(code>=80&&code<=82))){
    hubDropIcon(cx-7*scale,cy+19*scale,scale,HUB_CYAN);
    hubDropIcon(cx+7*scale,cy+19*scale,scale,HUB_CYAN);
  }else if(scale==1 && ((code>=71&&code<=77)||(code>=85&&code<=86))){
    hubTFT.fillCircle(cx-7*scale,cy+14*scale,2*scale,HUB_WHITE);
    hubTFT.fillCircle(cx+7*scale,cy+14*scale,2*scale,HUB_WHITE);
  }
}
void hubThermoIcon(int16_t x, int16_t y, uint16_t color) {
  hubTFT.drawRoundRect(x+4,y,9,25,4,color);
  hubTFT.fillCircle(x+8,y+26,7,color);
  hubTFT.fillRoundRect(x+7,y+11,3,14,1,color);
}
void hubHumidityIcon(int16_t x, int16_t y, uint16_t color) {
  hubTFT.fillCircle(x+8,y+15,6,color);
  hubTFT.fillTriangle(x+8,y,x+2,y+15,x+14,y+15,color);
}
void hubThermoMini(int16_t x,int16_t y,uint16_t color){
  hubTFT.drawRoundRect(x+2,y,5,13,2,color);hubTFT.fillCircle(x+4,y+14,3,color);hubTFT.drawFastVLine(x+4,y+5,7,color);
}
void hubBulbIcon(int16_t cx, int16_t cy, uint8_t scale, uint16_t color) {
  hubTFT.drawCircle(cx,cy,13*scale,color);
  hubTFT.drawCircle(cx,cy,12*scale,color);
  hubTFT.drawRoundRect(cx-(5*scale),cy+(8*scale),10*scale,8*scale,2*scale,color);
  hubTFT.drawFastHLine(cx-(4*scale),cy+(19*scale),8*scale,color);
  hubTFT.drawFastHLine(cx-(3*scale),cy+(22*scale),6*scale,color);
  for(uint8_t a=0;a<8;a++){float ang=a*0.785398f;hubTFT.drawLine(cx+(int16_t)(cosf(ang)*17*scale),cy+(int16_t)(sinf(ang)*17*scale),cx+(int16_t)(cosf(ang)*21*scale),cy+(int16_t)(sinf(ang)*21*scale),color);}
}
void hubProgress(int16_t x,int16_t y,int16_t w,int16_t h,uint8_t value,uint16_t color){
  static int16_t oldX=-1,oldY=-1,oldW=0,oldH=0,oldInner=0;
  const int16_t inner=(int16_t)((w-4)*constrain(value,0,100)/100);
  const bool geometryChanged=(x!=oldX||y!=oldY||w!=oldW||h!=oldH);
  if(hubProgressNeedsInit||geometryChanged){
    hubTFT.drawRoundRect(x,y,w,h,3,HUB_LINE);
    hubTFT.fillRect(x+2,y+2,w-4,h-4,HUB_PANEL);
    if(inner>0)hubTFT.fillRoundRect(x+2,y+2,inner,h-4,2,color);
    hubProgressNeedsInit=false;
  }else if(inner<oldInner){
    // Erase only the obsolete tail; preserve the pixels that remain illuminated.
    hubTFT.fillRect(x+2+inner,y+2,oldInner-inner,h-4,HUB_PANEL);
  }else if(inner>oldInner){
    // Paint only the newly added segment.
    hubTFT.fillRect(x+2+oldInner,y+2,inner-oldInner,h-4,color);
  }
  oldX=x;oldY=y;oldW=w;oldH=h;oldInner=inner;
}
void hubHeader(int16_t w,const char *section){
  // drawSmartHub() clears the background only when the layout/orientation changes.
  hubTFT.setTextWrap(false);
  hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_CYAN,HUB_BG);hubTFT.setCursor(5,5);hubTFT.print("GOODWIN");
  hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(55,5);hubTFT.print(section);
  // Compact connectivity glyph: three radio arcs and a status dot.
  int16_t x=w-13; hubTFT.drawPixel(x-4,8,HUB_GREEN);hubTFT.drawLine(x-6,6,x-2,6,HUB_GREEN);hubTFT.drawLine(x-8,4,x,4,HUB_GREEN);
  hubTFT.fillCircle(x,10,2,WiFi.status()==WL_CONNECTED?HUB_GREEN:HUB_RED);
  hubTFT.drawFastHLine(5,17,w-10,HUB_LINE);
}
void hubClockFace(int16_t cx,int16_t cy,int16_t r){
  hubTFT.drawCircle(cx,cy,r,HUB_LINE);hubTFT.drawCircle(cx,cy,r-1,HUB_LINE);
  for(uint8_t i=0;i<12;i++){float a=(i*0.523599f)-1.570796f;int16_t x1=cx+(int16_t)(cosf(a)*(r-5));int16_t y1=cy+(int16_t)(sinf(a)*(r-5));int16_t x2=cx+(int16_t)(cosf(a)*(r-2));int16_t y2=cy+(int16_t)(sinf(a)*(r-2));hubTFT.drawLine(x1,y1,x2,y2,HUB_MUTED);}
  int hh=0,mm=0;if(timeText.length()>=5){hh=timeText.substring(0,2).toInt()%12;mm=timeText.substring(3,5).toInt();}
  float ma=(mm*0.10472f)-1.570796f;float ha=((hh*60+mm)*0.008727f)-1.570796f;
  hubTFT.drawLine(cx,cy,cx+(int16_t)(cosf(ha)*(r*0.48f)),cy+(int16_t)(sinf(ha)*(r*0.48f)),HUB_WHITE);
  hubTFT.drawLine(cx,cy,cx+(int16_t)(cosf(ma)*(r*0.75f)),cy+(int16_t)(sinf(ma)*(r*0.75f)),HUB_CYAN);
  hubTFT.fillCircle(cx,cy,3,HUB_YELLOW);
}


static void catalogHeader(int16_t w, const char *section, uint16_t bg, uint16_t accent) {
  hubTFT.setTextWrap(false);
  hubTFT.setTextSize(1);
  hubTFT.setTextColor(accent, bg);
  hubTFT.setCursor(5,5); hubTFT.print("GOODWIN");
  hubTFT.setTextColor(bg == 0xFFFF ? 0x0841 : HUB_MUTED, bg);
  hubTFT.setCursor(55,5); hubTFT.print(section);
  const int16_t x = w - 13;
  hubTFT.drawPixel(x-4,8,HUB_GREEN);
  hubTFT.drawLine(x-6,6,x-2,6,HUB_GREEN);
  hubTFT.drawLine(x-8,4,x,4,HUB_GREEN);
  hubTFT.fillCircle(x,10,2,WiFi.status()==WL_CONNECTED?HUB_GREEN:HUB_RED);
  hubTFT.drawFastHLine(5,17,w-10,bg == 0xFFFF ? 0xBDF7 : HUB_LINE);
}

static void catalogCard(int16_t x, int16_t y, int16_t w, int16_t h,
                        const char *label, uint16_t fill, uint16_t accent) {
  hubCard(x, y, w, h, fill, accent);
  hubTFT.setTextSize(1);
  hubTFT.setTextColor(accent, fill);
  hubTFT.setCursor(x + 6, y + 5);
  hubTFT.print(label);
}

static const char *catalogLayoutTitle(uint8_t id) {
  static const char *titles[42] = {
    "HOME", "CLOCK", "WEATHER", "INDOOR", "CLIMATE", "LIGHT", "AURORA WX", "TFT TEST",
    "ORBIT", "TERMINAL", "DARK TECH", "AMBER", "GLASS UI", "CYBER",
    "COMPACT GRID", "STATUS STRIPS", "BIG CLOCK", "BIG TEMP", "WEATHER CARDS", "CLIMATE PRO",
    "LIGHT + CLIMATE", "LIGHT + WEATHER", "SYSTEM STATUS", "WI-FI STATUS", "MATTER STATUS",
    "MQTT STATUS", "VERTICAL CLEAN", "VERTICAL WEATHER", "VERTICAL CLIMATE", "NIGHT MODE",
    "HIGH CONTRAST", "MONO TERMINAL", "WEATHER DARK", "WEATHER LIGHT", "ANALOG CLOCK",
    "DIGITAL CLOCK", "SENSOR INFO", "NETWORK INFO", "MATTER / MQTT", "WEATHER DIAG",
    "AMBIENT", "DIAGNOSTICS"
  };
  return id < 42 ? titles[id] : "GOODWIN";
}

static void drawExtendedLandscapeValues() {
  char v[28];
  const uint16_t black = hubLayoutId == 30 ? 0x0000 : (hubLayoutId == 33 ? 0xFFFF : HUB_BG);
  const uint16_t fg = hubLayoutId == 31 ? HUB_GREEN : (hubLayoutId == 33 ? 0x0841 : HUB_WHITE);
  switch (hubLayoutId) {
    case 14: case 15: case 16: case 29: case 35:
      drawValueBox(10,52,62,22,black,fg,2,timeText.c_str());
      drawValueBox(87,52,62,22,black,HUB_CYAN,1,dateText.c_str());
      snprintf(v,sizeof(v),sensorValid?"ROOM %.1f C  RH %.0f%%":"ROOM --.- C  RH --%%",sensorValid?indoorTempC:0.0f,sensorValid?indoorHumidity:0.0f);
      drawValueBox(10,101,140,14,black,HUB_MUTED,1,v);
      break;
    case 17: case 19: case 26: case 28: case 36:
      snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(10,52,62,22,black,fg,2,v);
      snprintf(v,sizeof(v),sensorValid?"%.0f%% RH":"--%% RH",sensorValid?indoorHumidity:0.0f); drawValueBox(87,52,62,22,black,HUB_CYAN,2,v);
      snprintf(v,sizeof(v),"%u KB FREE",(unsigned)(ESP.getFreeHeap()/1024)); drawValueBox(10,101,140,14,black,HUB_MUTED,1,v);
      break;
    case 18: case 27: case 32: case 33: case 39:
      snprintf(v,sizeof(v),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(10,52,62,22,black,fg,2,v);
      snprintf(v,sizeof(v),weatherValid?"WIND %.0f":"WIND --",weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(87,52,62,22,black,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),weatherValid?"RAIN %d%%  HUM %.0f%%":"RAIN --  HUM --",weatherValid?outdoorPrecipProbability:0,weatherValid?outdoorHumidity:0.0f);
      drawValueBox(10,101,140,14,black,HUB_MUTED,1,v);
      break;
    case 20: case 21:
      drawValueBox(10,52,62,22,black,lightOn?HUB_YELLOW:HUB_MUTED,2,lightOn?"ON":"OFF");
      snprintf(v,sizeof(v),hubLayoutId==20?(sensorValid?"%.1f C":"--.- C"):(weatherValid?"%.1f C":"--.- C"),hubLayoutId==20?(sensorValid?indoorTempC:0.0f):(weatherValid?outdoorTempC:0.0f));
      drawValueBox(87,52,62,22,black,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),"BRIGHTNESS %u%%",brightnessPercent); drawValueBox(10,101,140,14,black,HUB_MUTED,1,v);
      break;
    case 22:
      drawValueBox(10,52,62,22,black,WiFi.status()==WL_CONNECTED?HUB_GREEN:HUB_RED,1,WiFi.status()==WL_CONNECTED?"WI-FI OK":"WI-FI OFF");
      drawValueBox(87,52,62,22,black,matterStarted?HUB_GREEN:HUB_RED,1,matterStarted?"MATTER OK":"MATTER OFF");
      snprintf(v,sizeof(v),"MQTT %s  HEAP %uK",mqttClient.connected()?"OK":"OFF",(unsigned)(ESP.getFreeHeap()/1024)); drawValueBox(10,101,140,14,black,HUB_CYAN,1,v);
      break;
    case 23: case 37:
      snprintf(v,sizeof(v),"%d dBm",WiFi.status()==WL_CONNECTED?WiFi.RSSI():-127); drawValueBox(10,52,62,22,black,HUB_CYAN,1,v);
      drawValueBox(87,52,62,22,black,fg,1,WiFi.status()==WL_CONNECTED?"CONNECTED":"OFFLINE");
      drawValueBox(10,101,140,14,black,HUB_MUTED,1,WiFi.status()==WL_CONNECTED?WiFi.localIP().toString().c_str():"NO IP");
      break;
    case 24: case 38:
      drawValueBox(10,52,62,22,black,matterStarted?HUB_GREEN:HUB_RED,1,matterStarted?"MATTER ON":"MATTER OFF");
      drawValueBox(87,52,62,22,black,Matter.isDeviceCommissioned()?HUB_GREEN:HUB_ORANGE,1,Matter.isDeviceCommissioned()?"PAIRED":"UNPAIRED");
      drawValueBox(10,101,140,14,black,mqttClient.connected()?HUB_GREEN:HUB_ORANGE,1,mqttClient.connected()?"MQTT CONNECTED":"MQTT DISCONNECTED");
      break;
    case 25:
      drawValueBox(10,52,62,22,black,mqttClient.connected()?HUB_GREEN:HUB_ORANGE,1,mqttClient.connected()?"ONLINE":"OFFLINE");
      drawValueBox(87,52,62,22,black,HUB_CYAN,1,mqttEnabled?"ENABLED":"DISABLED");
      drawValueBox(10,101,140,14,black,HUB_MUTED,1,mqttHost.length()?mqttHost.c_str():"BROKER NOT SET");
      break;
    case 30:
      drawValueBox(10,52,62,22,black,0xFFFF,2,timeText.c_str());
      snprintf(v,sizeof(v),sensorValid?"IN %.1f":"IN --.-",sensorValid?indoorTempC:0.0f); drawValueBox(87,52,62,22,black,0x07E0,1,v);
      snprintf(v,sizeof(v),weatherValid?"OUT %.1f":"OUT --.-",weatherValid?outdoorTempC:0.0f); drawValueBox(10,101,140,14,black,0xFFE0,1,v);
      break;
    case 34:
      hubTFT.fillCircle(hubTFT.width()/2,62,31,black);
      hubClockFace(hubTFT.width()/2,62,29);
      drawValueBox(10,110,140,14,black,HUB_WHITE,1,timeText.c_str());
      break;
    case 31:
      drawValueBox(10,52,62,22,black,HUB_GREEN,2,timeText.c_str());
      snprintf(v,sizeof(v),sensorValid?"TEMP %.1f":"TEMP --.-",sensorValid?indoorTempC:0.0f); drawValueBox(87,52,62,22,black,HUB_GREEN,1,v);
      snprintf(v,sizeof(v),"RSSI %d  HEAP %uK",WiFi.status()==WL_CONNECTED?WiFi.RSSI():-127,(unsigned)(ESP.getFreeHeap()/1024)); drawValueBox(10,101,140,14,black,HUB_GREEN,1,v);
      break;
    case 40:
      break;
    case 41:
      snprintf(v,sizeof(v),"UP %lu MIN",(unsigned long)(millis()/60000UL)); drawValueBox(10,52,62,22,black,fg,1,v);
      snprintf(v,sizeof(v),"HEAP %u KB",(unsigned)(ESP.getFreeHeap()/1024)); drawValueBox(87,52,62,22,black,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),"RSSI %d dBm  MQTT %s",WiFi.status()==WL_CONNECTED?WiFi.RSSI():-127,mqttClient.connected()?"OK":"OFF"); drawValueBox(10,101,140,14,black,HUB_MUTED,1,v);
      break;
    default:
      snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(10,52,62,22,black,fg,2,v);
      snprintf(v,sizeof(v),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(87,52,62,22,black,HUB_CYAN,2,v);
      drawValueBox(10,101,140,14,black,HUB_MUTED,1,timeText.c_str());
      break;
  }
}

static void drawExtendedLandscapeLayout(int16_t w, int16_t h) {
  if (h <= 0) return;
  const uint16_t bg = hubLayoutId == 30 ? 0x0000 : (hubLayoutId == 33 ? 0xFFFF : HUB_BG);
  const uint16_t accent = hubLayoutId == 31 ? HUB_GREEN : (hubLayoutId == 33 ? 0x0841 : HUB_CYAN);
  if (hubLayoutId == 40) {
    hubTFT.fillScreen(0x0000);
    hubTFT.setTextSize(1); hubTFT.setTextColor(0x39E7,0x0000);
    hubTFT.setCursor((w-42)/2,(h/2)-5); hubTFT.print("GOODWIN");
    hubTFT.drawFastHLine(w/2-18,h/2+9,36,HUB_CYAN);
    return;
  }
  hubTFT.fillScreen(bg);
  catalogHeader(w,catalogLayoutTitle(hubLayoutId),bg,accent);
  if (hubLayoutId == 34) {
    hubClockFace(w/2,62,29);
    hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED,bg); hubTFT.setCursor(9,101); hubTFT.print("ANALOG / LOCAL TIME");
    drawExtendedLandscapeValues();
    return;
  }
  if (hubLayoutId == 35 || hubLayoutId == 16 || hubLayoutId == 29) {
    catalogCard(5,25,72,75,"LOCAL TIME",bg,accent);
    catalogCard(82,25,73,75,"DATE / CLIMATE",bg,accent);
  } else if (hubLayoutId >= 22 && hubLayoutId <= 25) {
    catalogCard(5,25,72,75,"SERVICE A",bg,accent);
    catalogCard(82,25,73,75,"SERVICE B",bg,accent);
  } else if (hubLayoutId == 30 || hubLayoutId == 31) {
    hubTFT.drawRoundRect(5,25,150,75,5,accent);
    hubTFT.drawFastHLine(10,91,140,accent);
  } else {
    const char *left = (hubLayoutId==18||hubLayoutId==27||hubLayoutId==32||hubLayoutId==33||hubLayoutId==39)?"OUTDOOR":
                       (hubLayoutId==20||hubLayoutId==21)?"LIGHT":"INDOOR";
    const char *right = (hubLayoutId==20||hubLayoutId==21)?"ENVIRONMENT":
                        (hubLayoutId==22||hubLayoutId==23)?"NETWORK":"LIVE DATA";
    catalogCard(5,25,72,75,left,bg,accent);
    catalogCard(82,25,73,75,right,bg,accent);
  }
  hubTFT.drawFastHLine(8,98,w-16,0x2149);
  drawExtendedLandscapeValues();
}

static void drawExtendedLayoutValues() {
  if (hubTFT.width() > hubTFT.height()) {
    drawExtendedLandscapeValues();
    return;
  }
  char v[28];
  switch (hubLayoutId) {
    case 14: // Compact grid
      drawValueBox(10, 43, 48, 18, HUB_PANEL, HUB_WHITE, 1, timeText.c_str());
      snprintf(v,sizeof(v),sensorValid?"%.1fC":"--.-C",sensorValid?indoorTempC:0.0f); drawValueBox(10,78,48,17,HUB_PANEL2,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),sensorValid?"%.0f%%":"--%%",sensorValid?indoorHumidity:0.0f); drawValueBox(69,78,48,17,HUB_PANEL2,HUB_WHITE,1,v);
      snprintf(v,sizeof(v),weatherValid?"%.1fC":"--.-C",weatherValid?outdoorTempC:0.0f); drawValueBox(10,127,48,17,HUB_PANEL,HUB_WHITE,1,v);
      drawValueBox(69,127,48,17,HUB_PANEL2,lightOn?HUB_YELLOW:HUB_MUTED,1,lightOn?"ON":"OFF");
      break;
    case 15: // Status strips
      drawValueBox(8,27,110,20,HUB_BG,HUB_WHITE,2,timeText.c_str());
      snprintf(v,sizeof(v),sensorValid?"%.1f C  /  %.0f%%":"--.- C  /  --%%",sensorValid?indoorTempC:0.0f,sensorValid?indoorHumidity:0.0f); drawValueBox(8,62,110,16,HUB_PANEL,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),weatherValid?"%.1f C  WIND %.0f":"--.- C  WIND --",weatherValid?outdoorTempC:0.0f,weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(8,96,110,16,HUB_PANEL2,HUB_WHITE,1,v);
      drawValueBox(8,130,110,14,HUB_BG,WiFi.status()==WL_CONNECTED?HUB_GREEN:HUB_RED,1,WiFi.status()==WL_CONNECTED?"NETWORK ONLINE":"NETWORK OFFLINE");
      break;
    case 16: // Big clock
      drawValueBox(8,48,112,30,HUB_BG,HUB_WHITE,3,timeText.c_str());
      drawValueBox(8,82,112,14,HUB_BG,HUB_MUTED,1,dateText.c_str());
      snprintf(v,sizeof(v),sensorValid?"ROOM  %.1f C":"ROOM  --.- C",sensorValid?indoorTempC:0.0f); drawValueBox(8,118,112,20,HUB_PANEL,HUB_CYAN,2,v);
      break;
    case 17: // Big indoor temperature
      snprintf(v,sizeof(v),sensorValid?"%.1f":"--.-",sensorValid?indoorTempC:0.0f); drawValueBox(8,55,112,32,HUB_PANEL,HUB_WHITE,3,v);
      snprintf(v,sizeof(v),sensorValid?"HUMIDITY  %.0f%%":"HUMIDITY  --%%",sensorValid?indoorHumidity:0.0f); drawValueBox(8,101,112,18,HUB_PANEL2,HUB_CYAN,1,v);
      drawValueBox(8,130,112,14,HUB_BG,HUB_MUTED,1,timeText.c_str());
      break;
    case 18: // Weather cards
      snprintf(v,sizeof(v),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(10,95,48,22,HUB_PANEL,HUB_WHITE,1,v);
      snprintf(v,sizeof(v),weatherValid?"FEELS %.1f":"FEELS --.-",weatherValid?outdoorFeelsLikeC:0.0f); drawValueBox(67,95,50,14,HUB_PANEL2,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),weatherValid?"WIND %.0f":"WIND --",weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(10,120,48,16,HUB_PANEL,HUB_WHITE,1,v);
      snprintf(v,sizeof(v),weatherValid?"RAIN %d%%":"RAIN --",weatherValid?outdoorPrecipProbability:0); drawValueBox(67,120,50,16,HUB_PANEL2,HUB_CYAN,1,v);
      break;
    case 19: // Climate cards
      snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(12,72,45,24,HUB_PANEL,HUB_WHITE,1,v);
      snprintf(v,sizeof(v),sensorValid?"%.0f%%":"--%%",sensorValid?indoorHumidity:0.0f); drawValueBox(71,72,45,24,HUB_PANEL2,HUB_CYAN,1,v);
      drawValueBox(12,127,104,14,HUB_BG,HUB_MUTED,1,sensorValid?"ROOM CLIMATE / LIVE":"SENSOR WAITING");
      break;
    case 20: // Light + climate
      hubTFT.fillRect(8,39,52,54,HUB_BG);
      hubBulbIcon(34,65,1,lightOn?HUB_YELLOW:HUB_MUTED);
      hubProgress(8,139,112,7,brightnessPercent,HUB_YELLOW);
      drawValueBox(10,91,48,22,HUB_PANEL,lightOn?HUB_YELLOW:HUB_MUTED,2,lightOn?"ON":"OFF");
      snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(69,70,48,18,HUB_PANEL2,HUB_WHITE,1,v);
      snprintf(v,sizeof(v),sensorValid?"%.0f%% RH":"--%% RH",sensorValid?indoorHumidity:0.0f); drawValueBox(69,108,48,18,HUB_PANEL2,HUB_CYAN,1,v);
      break;
    case 21: // Light + weather
      hubTFT.fillRect(8,39,52,54,HUB_BG);
      hubBulbIcon(34,65,1,lightOn?HUB_YELLOW:HUB_MUTED);
      hubProgress(8,139,112,7,brightnessPercent,HUB_YELLOW);
      drawValueBox(10,91,48,22,HUB_PANEL,lightOn?HUB_YELLOW:HUB_MUTED,2,lightOn?"ON":"OFF");
      snprintf(v,sizeof(v),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(69,70,48,18,HUB_PANEL2,HUB_WHITE,1,v);
      snprintf(v,sizeof(v),weatherValid?"WIND %.0f":"WIND --",weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(69,108,48,18,HUB_PANEL2,HUB_CYAN,1,v);
      break;
    case 22: // System status
      drawValueBox(10,56,47,18,HUB_PANEL,WiFi.status()==WL_CONNECTED?HUB_GREEN:HUB_RED,1,WiFi.status()==WL_CONNECTED?"ONLINE":"OFFLINE");
      drawValueBox(69,56,47,18,HUB_PANEL2,matterStarted?HUB_GREEN:HUB_RED,1,matterStarted?"READY":"STOPPED");
      drawValueBox(10,111,47,18,HUB_PANEL,mqttClient.connected()?HUB_GREEN:HUB_ORANGE,1,mqttClient.connected()?"LINK OK":"NO LINK");
      snprintf(v,sizeof(v),"%u KB",(unsigned)(ESP.getFreeHeap()/1024)); drawValueBox(69,111,47,18,HUB_PANEL2,HUB_CYAN,1,v);
      break;
    case 23: // Wi-Fi status
      snprintf(v,sizeof(v),"%d dBm",WiFi.status()==WL_CONNECTED?WiFi.RSSI():-127); drawValueBox(10,57,68,25,HUB_PANEL,HUB_CYAN,1,v);
      drawValueBox(10,96,106,14,HUB_PANEL2,HUB_WHITE,1,WiFi.status()==WL_CONNECTED?WiFi.localIP().toString().c_str():"NOT CONNECTED");
      drawValueBox(10,127,106,14,HUB_BG,HUB_MUTED,1,WiFi.status()==WL_CONNECTED?WiFi.SSID().c_str():"NO SSID");
      break;
    case 24: // Matter status
      drawValueBox(30,59,68,16,HUB_BG,matterStarted?HUB_GREEN:HUB_RED,1,matterStarted?"READY":"OFFLINE");
      drawValueBox(10,94,106,16,HUB_PANEL2,Matter.isDeviceCommissioned()?HUB_GREEN:HUB_ORANGE,1,Matter.isDeviceCommissioned()?"FABRIC COMMISSIONED":"WAITING TO PAIR");
      drawValueBox(10,127,106,14,HUB_BG,climateEndpointReady?HUB_CYAN:HUB_MUTED,1,climateEndpointReady?"CLIMATE ENDPOINT READY":"CLIMATE ENDPOINT --");
      break;
    case 25: // MQTT status
      drawValueBox(10,56,90,22,HUB_PANEL,mqttClient.connected()?HUB_GREEN:HUB_ORANGE,1,mqttClient.connected()?"CONNECTED":"DISCONNECTED");
      drawValueBox(10,94,106,14,HUB_PANEL2,HUB_WHITE,1,mqttEnabled?"MQTT ENABLED":"MQTT DISABLED");
      drawValueBox(10,126,106,16,HUB_BG,HUB_MUTED,1,mqttHost.length()?mqttHost.c_str():"BROKER NOT SET");
      break;
    case 26: // Vertical clean
      drawValueBox(10,36,106,25,HUB_BG,HUB_WHITE,2,timeText.c_str());
      snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(10,82,106,22,HUB_PANEL,HUB_CYAN,2,v);
      snprintf(v,sizeof(v),sensorValid?"HUMIDITY  %.0f%%":"HUMIDITY  --%%",sensorValid?indoorHumidity:0.0f); drawValueBox(10,119,106,16,HUB_PANEL2,HUB_WHITE,1,v);
      break;
    case 27: // Vertical weather
      snprintf(v,sizeof(v),weatherValid?"%.1f":"--.-",weatherValid?outdoorTempC:0.0f); drawValueBox(10,88,106,32,HUB_PANEL,HUB_WHITE,3,v);
      snprintf(v,sizeof(v),weatherValid?"WIND %.0f km/h":"WIND --",weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(10,122,106,16,HUB_PANEL2,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),weatherValid?"RAIN %d%%":"RAIN --",weatherValid?outdoorPrecipProbability:0); drawValueBox(10,140,106,10,HUB_BG,HUB_MUTED,1,v);
      break;
    case 28: // Vertical climate
      snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(10,62,106,24,HUB_PANEL,HUB_WHITE,2,v);
      snprintf(v,sizeof(v),sensorValid?"%.0f%% RH":"--%% RH",sensorValid?indoorHumidity:0.0f); drawValueBox(10,111,106,24,HUB_PANEL2,HUB_CYAN,2,v);
      break;
    case 29: // Night
      drawValueBox(10,54,106,30,HUB_BG,HUB_WHITE,3,timeText.c_str());
      drawValueBox(10,90,106,14,HUB_BG,HUB_MUTED,1,dateText.c_str());
      snprintf(v,sizeof(v),sensorValid?"%.1f C  ·  %.0f%%":"--.- C  ·  --%%",sensorValid?indoorTempC:0.0f,sensorValid?indoorHumidity:0.0f); drawValueBox(10,126,106,16,HUB_BG,0x5DDF,1,v);
      break;
    case 30: // High contrast
      drawValueBox(8,34,112,28,0x0000,0xFFFF,3,timeText.c_str());
      snprintf(v,sizeof(v),sensorValid?"ROOM %.1f C":"ROOM --.- C",sensorValid?indoorTempC:0.0f); drawValueBox(8,81,112,18,0x0000,0x07E0,2,v);
      snprintf(v,sizeof(v),weatherValid?"OUT  %.1f C":"OUT  --.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(8,116,112,18,0x0000,0xFFE0,2,v);
      break;
    case 31: // Monochrome terminal
      drawValueBox(10,54,106,18,HUB_BG,HUB_GREEN,2,timeText.c_str());
      snprintf(v,sizeof(v),"> ROOM T  %.1f",sensorValid?indoorTempC:0.0f); drawValueBox(10,82,106,14,HUB_BG,HUB_GREEN,1,v);
      snprintf(v,sizeof(v),"> ROOM RH %.0f",sensorValid?indoorHumidity:0.0f); drawValueBox(10,106,106,14,HUB_BG,HUB_GREEN,1,v);
      snprintf(v,sizeof(v),"> OUT T   %.1f",weatherValid?outdoorTempC:0.0f); drawValueBox(10,130,106,14,HUB_BG,HUB_GREEN,1,v);
      break;
    case 32: // Weather dark
      snprintf(v,sizeof(v),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(10,91,106,26,HUB_PANEL,HUB_WHITE,2,v);
      snprintf(v,sizeof(v),weatherValid?"FEELS %.1f C":"FEELS --.- C",weatherValid?outdoorFeelsLikeC:0.0f); drawValueBox(10,120,106,14,HUB_PANEL2,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),weatherValid?"WIND %.0f  RAIN %d%%":"WIND --  RAIN --",weatherValid?outdoorWindSpeedKmh:0.0f,weatherValid?outdoorPrecipProbability:0); drawValueBox(10,137,106,10,HUB_BG,HUB_MUTED,1,v);
      break;
    case 33: // Weather light
      snprintf(v,sizeof(v),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(10,84,106,26,0xFFFF,0x0841,2,v);
      snprintf(v,sizeof(v),weatherValid?"HUM %.0f%%":"HUM --%%",weatherValid?outdoorHumidity:0.0f); drawValueBox(10,116,106,14,0xFFFF,0x0841,1,v);
      snprintf(v,sizeof(v),weatherValid?"PRESS %.0f hPa":"PRESS ----",weatherValid?outdoorPressureHpa:0.0f); drawValueBox(10,136,106,14,0xFFFF,0x0841,1,v);
      break;
    case 34: // Analog clock
      hubTFT.fillCircle(hubTFT.width()/2,76,min(hubTFT.width(),hubTFT.height())/3+2,HUB_BG);
      hubClockFace(hubTFT.width()/2,76,min(hubTFT.width(),hubTFT.height())/3);
      drawValueBox(10,129,106,16,HUB_BG,HUB_WHITE,1,timeText.c_str());
      break;
    case 35: // Digital clock
      drawValueBox(8,59,112,32,HUB_BG,HUB_WHITE,3,timeText.c_str());
      drawValueBox(8,99,112,16,HUB_BG,HUB_MUTED,1,dateText.c_str());
      break;
    case 36: // Sensor info
      snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(10,55,106,20,HUB_PANEL,HUB_WHITE,2,v);
      snprintf(v,sizeof(v),sensorValid?"%.0f%% RH":"--%% RH",sensorValid?indoorHumidity:0.0f); drawValueBox(10,91,106,18,HUB_PANEL2,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),"%u KB FREE",(unsigned)(ESP.getFreeHeap()/1024)); drawValueBox(10,126,106,14,HUB_BG,HUB_MUTED,1,v);
      break;
    case 37: // Network info
      snprintf(v,sizeof(v),"%d dBm",WiFi.status()==WL_CONNECTED?WiFi.RSSI():-127); drawValueBox(10,54,106,22,HUB_PANEL,HUB_CYAN,2,v);
      drawValueBox(10,91,106,14,HUB_PANEL2,HUB_WHITE,1,WiFi.status()==WL_CONNECTED?WiFi.localIP().toString().c_str():"NO IP");
      drawValueBox(10,124,106,14,HUB_BG,HUB_MUTED,1,WiFi.status()==WL_CONNECTED?WiFi.SSID().c_str():"DISCONNECTED");
      break;
    case 38: // Matter + MQTT info
      drawValueBox(10,54,106,18,HUB_PANEL,matterStarted?HUB_GREEN:HUB_RED,1,matterStarted?"MATTER ONLINE":"MATTER OFFLINE");
      drawValueBox(10,88,106,18,HUB_PANEL2,Matter.isDeviceCommissioned()?HUB_GREEN:HUB_ORANGE,1,Matter.isDeviceCommissioned()?"FABRIC PAIRED":"NOT PAIRED");
      drawValueBox(10,122,106,18,HUB_BG,mqttClient.connected()?HUB_GREEN:HUB_ORANGE,1,mqttClient.connected()?"MQTT CONNECTED":"MQTT OFFLINE");
      break;
    case 39: { // Weather diagnostics
      char status[64], provider[24]; copyWeatherText(status,sizeof(status),provider,sizeof(provider));
      drawValueBox(10,55,106,20,HUB_PANEL,weatherValid?HUB_GREEN:HUB_ORANGE,1,weatherValid?"WEATHER LIVE":"NO WEATHER DATA");
      drawValueBox(10,91,106,14,HUB_PANEL2,HUB_WHITE,1,provider);
      drawValueBox(10,124,106,14,HUB_BG,HUB_MUTED,1,status);
      break;
    }
    case 40: // Ambient empty
      break;
    case 41: // Diagnostics
      snprintf(v,sizeof(v),"UP %lu MIN",(unsigned long)(millis()/60000UL)); drawValueBox(10,50,106,16,HUB_PANEL,HUB_WHITE,1,v);
      snprintf(v,sizeof(v),"HEAP %u KB",(unsigned)(ESP.getFreeHeap()/1024)); drawValueBox(10,81,106,16,HUB_PANEL2,HUB_CYAN,1,v);
      snprintf(v,sizeof(v),"RSSI %d dBm",WiFi.status()==WL_CONNECTED?WiFi.RSSI():-127); drawValueBox(10,112,106,16,HUB_BG,HUB_MUTED,1,v);
      break;
  }
}

static void drawExtendedLayout(int16_t w, int16_t h) {
  if (h <= 0) return;
  if (w > h) {
    drawExtendedLandscapeLayout(w, h);
    return;
  }
  const uint16_t accent = (hubLayoutId == 30 || hubLayoutId == 31) ? HUB_GREEN :
                          (hubLayoutId == 33 ? 0x0841 : HUB_CYAN);
  const char *names[] = {
    "", "", "", "", "", "", "", "", "", "", "", "", "", "",
    "COMPACT GRID", "STATUS STRIPS", "BIG CLOCK", "BIG TEMP", "WEATHER CARDS", "CLIMATE PRO",
    "LIGHT + CLIMATE", "LIGHT + WEATHER", "SYSTEM STATUS", "WI-FI STATUS", "MATTER STATUS",
    "MQTT STATUS", "VERTICAL CLEAN", "VERTICAL WEATHER", "VERTICAL CLIMATE", "NIGHT MODE",
    "HIGH CONTRAST", "MONO TERMINAL", "WEATHER DARK", "WEATHER LIGHT", "ANALOG CLOCK",
    "DIGITAL CLOCK", "SENSOR INFO", "NETWORK INFO", "MATTER / MQTT", "WEATHER DIAG",
    "AMBIENT", "DIAGNOSTICS"
  };
  if (hubLayoutId == 40) {
    hubTFT.fillScreen(0x0000);
    hubTFT.setTextSize(1);
    hubTFT.setTextColor(0x39E7, 0x0000);
    hubTFT.setCursor((w - 42) / 2, h / 2 - 5);
    hubTFT.print("GOODWIN");
    hubTFT.drawFastHLine(w / 2 - 18, h / 2 + 9, 36, HUB_CYAN);
    return;
  }
  if (hubLayoutId == 30) hubTFT.fillScreen(0x0000);
  else if (hubLayoutId == 33) hubTFT.fillScreen(0xFFFF);
  catalogHeader(w, names[hubLayoutId], hubLayoutId == 33 ? 0xFFFF : (hubLayoutId == 30 ? 0x0000 : HUB_BG), accent);
  hubTFT.setTextColor(hubLayoutId == 33 ? 0x0841 : accent, hubLayoutId == 33 ? 0xFFFF : HUB_BG);

  switch (hubLayoutId) {
    case 14:
      catalogCard(5,27,57,66,"TIME",HUB_PANEL,HUB_CYAN); catalogCard(66,27,57,66,"CLIMATE",HUB_PANEL2,HUB_CYAN);
      catalogCard(5,99,57,53,"OUTSIDE",HUB_PANEL,HUB_CYAN); catalogCard(66,99,57,53,"LIGHT",HUB_PANEL2,HUB_YELLOW);
      break;
    case 15:
      hubTFT.drawFastHLine(8,54,w-16,HUB_LINE); hubTFT.drawFastHLine(8,88,w-16,HUB_LINE); hubTFT.drawFastHLine(8,122,w-16,HUB_LINE);
      hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED,HUB_BG); hubTFT.setCursor(8,48); hubTFT.print("INDOOR CLIMATE");
      hubTFT.setCursor(8,82); hubTFT.print("OUTDOOR CONDITIONS"); hubTFT.setCursor(8,116); hubTFT.print("CONNECTIVITY");
      break;
    case 16:
      hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED,HUB_BG); hubTFT.setCursor(8,31); hubTFT.print("LOCAL TIME");
      hubTFT.drawFastHLine(8,105,w-16,HUB_LINE); hubTFT.setCursor(8,108); hubTFT.print("INDOOR TEMPERATURE");
      break;
    case 17:
      catalogCard(5,38,118,91,"ROOM TEMPERATURE",HUB_PANEL,HUB_CYAN);
      hubTFT.setTextColor(HUB_MUTED,HUB_BG); hubTFT.setCursor(8,137); hubTFT.print("LIVE CLIMATE SENSOR");
      break;
    case 18:
      hubWeatherIcon(w/2,47,1); catalogCard(5,77,57,69,"OUTSIDE",HUB_PANEL,HUB_CYAN);
      catalogCard(66,77,57,69,"WIND / RAIN",HUB_PANEL2,HUB_CYAN);
      break;
    case 19:
      catalogCard(5,36,57,96,"TEMPERATURE",HUB_PANEL,HUB_ORANGE);
      catalogCard(66,36,57,96,"HUMIDITY",HUB_PANEL2,HUB_CYAN);
      break;
    case 20:
      hubBulbIcon(34,65,1,lightOn?HUB_YELLOW:HUB_MUTED);
      catalogCard(67,39,56,93,"CLIMATE",HUB_PANEL2,HUB_CYAN);
      hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED,HUB_BG); hubTFT.setCursor(8,123); hubTFT.print("BRIGHTNESS");
      hubProgress(8,139,112,7,brightnessPercent,HUB_YELLOW);
      break;
    case 21:
      hubBulbIcon(34,65,1,lightOn?HUB_YELLOW:HUB_MUTED);
      catalogCard(67,39,56,93,"OUTSIDE",HUB_PANEL2,HUB_CYAN);
      hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED,HUB_BG); hubTFT.setCursor(8,123); hubTFT.print("GOODWIN LIGHT");
      hubProgress(8,139,112,7,brightnessPercent,HUB_YELLOW);
      break;
    case 22:
      catalogCard(5,31,57,51,"WI-FI",HUB_PANEL,HUB_GREEN); catalogCard(66,31,57,51,"MATTER",HUB_PANEL2,HUB_CYAN);
      catalogCard(5,88,57,51,"MQTT",HUB_PANEL2,HUB_YELLOW); catalogCard(66,88,57,51,"MEMORY",HUB_PANEL,HUB_CYAN);
      break;
    case 23:
      hubTFT.drawRoundRect(8,36,w-16,50,8,HUB_LINE);
      for(uint8_t i=0;i<5;i++){int bh=7+i*6;hubTFT.fillRoundRect(86+i*7,80-bh,5,bh,2,i<3?HUB_GREEN:HUB_CYAN);}
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(10,101);hubTFT.print("LOCAL NETWORK");
      hubTFT.drawFastHLine(10,119,w-20,HUB_LINE);
      break;
    case 24:
      hubTFT.drawCircle(w/2,67,28,HUB_LINE); hubTFT.drawCircle(w/2,67,23,matterStarted?HUB_GREEN:HUB_RED);
      hubTFT.fillCircle(w/2,67,5,matterStarted?HUB_GREEN:HUB_RED);
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(10,111);hubTFT.print("DEVICE COMMISSIONING");
      break;
    case 25:
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(10,37);hubTFT.print("BROKER CONNECTION");
      hubTFT.drawRoundRect(8,53,w-16,35,7,HUB_LINE);
      hubTFT.fillCircle(w-20,70,5,mqttClient.connected()?HUB_GREEN:HUB_ORANGE);
      hubTFT.setCursor(10,101);hubTFT.print("MQTT TELEMETRY");
      hubTFT.drawFastHLine(10,116,w-20,HUB_LINE);
      break;
    case 26:
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(10,25);hubTFT.print("YOUR HOME AT A GLANCE");
      catalogCard(5,63,118,37,"ROOM",HUB_PANEL,HUB_CYAN); catalogCard(5,106,118,43,"COMFORT",HUB_PANEL2,HUB_CYAN);
      break;
    case 27:
      hubWeatherIcon(w/2,42,1); catalogCard(5,70,118,76,"FORECAST NOW",HUB_PANEL,HUB_CYAN);
      break;
    case 28:
      catalogCard(5,31,118,54,"TEMPERATURE",HUB_PANEL,HUB_ORANGE);
      catalogCard(5,91,118,55,"RELATIVE HUMIDITY",HUB_PANEL2,HUB_CYAN);
      break;
    case 29:
      hubTFT.drawFastHLine(8,106,w-16,0x2149);
      hubTFT.setTextSize(1);hubTFT.setTextColor(0x5DDF,HUB_BG);hubTFT.setCursor(10,122);hubTFT.print("QUIET HOURS  /  AMBIENT");
      break;
    case 30:
      hubTFT.drawFastHLine(8,70,w-16,0xFFFF); hubTFT.drawFastHLine(8,105,w-16,0x07E0);
      hubTFT.setTextSize(1);hubTFT.setTextColor(0xFFFF,0x0000);hubTFT.setCursor(8,139);hubTFT.print("GOODWIN / HIGH CONTRAST");
      break;
    case 31:
      hubTFT.drawFastHLine(8,51,w-16,HUB_GREEN);hubTFT.drawFastHLine(8,139,w-16,HUB_GREEN);
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_GREEN,HUB_BG);hubTFT.setCursor(10,38);hubTFT.print("SYSTEM CONSOLE");
      break;
    case 32:
      hubWeatherIcon(w/2,43,1); catalogCard(5,75,118,72,"CURRENT CONDITIONS",HUB_PANEL,HUB_CYAN);
      break;
    case 33:
      hubTFT.fillRoundRect(5,28,118,122,10,0xFFFF);hubTFT.drawRoundRect(5,28,118,122,10,0xBDF7);
      hubTFT.setTextColor(0x0841,0xFFFF);hubTFT.setTextSize(1);hubTFT.setCursor(12,39);hubTFT.print("WEATHER REPORT");
      hubWeatherIcon(w/2,60,1);
      break;
    case 34:
      hubClockFace(w/2,76,min(w,h)/3);
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(10,148);hubTFT.print("ANALOG / LOCAL TIME");
      break;
    case 35:
      hubTFT.drawRoundRect(5,43,118,75,10,HUB_LINE);hubTFT.drawFastHLine(14,91,w-28,HUB_LINE);
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(10,29);hubTFT.print("DIGITAL CHRONOMETER");
      break;
    case 36:
      catalogCard(5,35,118,42,"ROOM SENSOR",HUB_PANEL,HUB_CYAN);catalogCard(5,84,118,57,"ESP32 HEALTH",HUB_PANEL2,HUB_GREEN);
      break;
    case 37:
      catalogCard(5,35,118,43,"WI-FI SIGNAL",HUB_PANEL,HUB_GREEN);
      catalogCard(5,86,118,57,"IP / NETWORK",HUB_PANEL2,HUB_CYAN);
      break;
    case 38:
      catalogCard(5,34,118,34,"MATTER FABRIC",HUB_PANEL,HUB_CYAN);
      catalogCard(5,74,118,34,"MQTT BROKER",HUB_PANEL2,HUB_GREEN);
      catalogCard(5,114,118,30,"CLIMATE ENDPOINT",HUB_PANEL,HUB_CYAN);
      break;
    case 39:
      catalogCard(5,36,118,39,"WEATHER SOURCE",HUB_PANEL,HUB_CYAN);
      catalogCard(5,83,118,59,"PROVIDER STATUS",HUB_PANEL2,HUB_CYAN);
      break;
    case 41:
      hubTFT.drawRoundRect(6,29,116,116,7,HUB_LINE);
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(12,37);hubTFT.print("DEVICE TELEMETRY");
      for(uint8_t i=0;i<4;i++)hubTFT.drawFastHLine(12,68+i*19,104,0x2149);
      break;
  }
  drawExtendedLayoutValues();
}

void drawSmartHub() {
  // A full layout render may repaint cards/labels underneath value widgets.
  // Invalidate their snapshots so every value is restored over its true background.
  hubInvalidateValueCache();
  hubProgressNeedsInit = true;
  if (!hubDisplayEnabled) {
    static bool disabledLogged = false;
    if (!disabledLogged) { LOG_W("TFT rendering skipped: display is disabled in saved state"); disabledLogged = true; }
    // Clear the panel even when its backlight is hard-wired to 3.3 V.
    hubTFT.fillScreen(0x0000);
    return;
  }
  static bool firstFrameLogged = false;
  if (!firstFrameLogged) { LOG_B("TFT first frame rendered via direct SPI; layout=%u", hubLayoutId); firstFrameLogged = true; }
  static uint8_t previousLayout=255, previousOrientation=255;
  const int16_t w=hubTFT.width(), h=hubTFT.height();
  const bool landscape=w>h;
  if(hubForceFullRedraw||previousLayout!=hubLayoutId||previousOrientation!=hubOrientation){hubInvalidateValueCache();hubTFT.fillScreen(HUB_BG);previousLayout=hubLayoutId;previousOrientation=hubOrientation;hubForceFullRedraw=false;}
  const uint16_t tempColor=(sensorValid&&indoorTempC>=27.0f)?HUB_ORANGE:HUB_GREEN;

  if (hubLayoutId >= 10 && hubLayoutId <= 13) {
    // Four compact visual prototypes, rendered directly to the ST7735.
    const bool amber = hubLayoutId == 11;
    const uint16_t accent = amber ? HUB_YELLOW : (hubLayoutId == 13 ? 0xF81F : HUB_CYAN);
    const char *heading = hubLayoutId == 10 ? "DARK TECH" : amber ? "AMBER" : hubLayoutId == 12 ? "GLASS UI" : "CYBER";
    hubTFT.drawRoundRect(4, 4, w - 8, h - 8, 8, accent);
    hubTFT.setTextSize(1); hubTFT.setTextColor(accent, HUB_BG); hubTFT.setCursor(10, 10); hubTFT.print(heading);
    hubTFT.setTextColor(HUB_MUTED, HUB_BG); hubTFT.setCursor(w - 44, 10); hubTFT.print("GW / 01");
    hubTFT.drawFastHLine(9, 23, w - 18, HUB_LINE);
    hubTFT.setTextSize(2); hubTFT.setTextColor(HUB_WHITE, HUB_BG); hubTFT.setCursor(10, 31); hubTFT.print(timeText);
    hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED, HUB_BG); hubTFT.setCursor(10, 53); hubTFT.print(dateText);
    const uint16_t panel = hubLayoutId == 12 ? 0x10A4 : HUB_PANEL;
    hubCard(8, 68, w - 16, 34, panel);
    hubTFT.setTextColor(HUB_MUTED, panel); hubTFT.setCursor(14, 73); hubTFT.print("INDOOR");
    char v[18]; snprintf(v, sizeof(v), sensorValid ? "%.1f C" : "--.- C", sensorValid ? indoorTempC : 0.0f);
    drawValueBox(14, 84, 55, 15, panel, HUB_WHITE, 1, v);
    snprintf(v, sizeof(v), sensorValid ? "%.0f%% RH" : "--%% RH", sensorValid ? indoorHumidity : 0.0f);
    drawValueBox(w - 66, 84, 52, 15, panel, accent, 1, v);
    hubCard(8, 108, (w - 22) / 2, 36, HUB_PANEL);
    hubCard(14 + (w - 22) / 2, 108, (w - 22) / 2, 36, HUB_PANEL2);
    hubTFT.setTextColor(HUB_MUTED, HUB_PANEL); hubTFT.setCursor(13, 113); hubTFT.print("OUTSIDE");
    hubTFT.setTextColor(HUB_MUTED, HUB_PANEL2); hubTFT.setCursor(19 + (w - 22) / 2, 113); hubTFT.print("WIND");
    snprintf(v, sizeof(v), weatherValid ? "%.1fC" : "--.-C", weatherValid ? outdoorTempC : 0.0f);
    drawValueBox(12, 126, (w - 30) / 2, 13, HUB_PANEL, HUB_WHITE, 1, v);
    snprintf(v, sizeof(v), weatherValid ? "%.0fkm/h" : "--km/h", weatherValid ? outdoorWindSpeedKmh : 0.0f);
    drawValueBox(18 + (w - 22) / 2, 126, (w - 30) / 2, 13, HUB_PANEL2, accent, 1, v);
    hubTFT.drawFastHLine(10, h - 18, w - 20, accent);
    hubTFT.setTextColor(accent, HUB_BG); hubTFT.setCursor(10, h - 13); hubTFT.print(WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE");
    pushChangedTftFrame(); return;
  }

  if (hubLayoutId >= 14 && hubLayoutId < HUB_LAYOUT_COUNT) {
    drawExtendedLayout(w, h);
    pushChangedTftFrame();
    return;
  }

  if(hubLayoutId==7){ // TFT performance test; static elements are painted once.
    hubHeader(w,"TFT TEST");
    hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(8,29);hubTFT.print("TARGET FPS");
    hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_CYAN,HUB_BG);hubTFT.setCursor(8,43);hubTFT.print(hubTestFpsTarget);hubTFT.print(" FPS");
    hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(8,70);hubTFT.print("MEASURED FPS");
    hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_BG);hubTFT.setCursor(8,84);hubTFT.print(hubTestMeasuredFps);hubTFT.print(" FPS");
    hubTFT.drawRoundRect(10,116,w-20,18,4,HUB_LINE);
    hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(8,143);hubTFT.print("SPI / TFT benchmark");
    hubTestLastFrameAt=0;hubTestFpsWindowAt=millis();hubTestFrameCount=0;hubTestFrameNumber=0;
    return;
  }

  if(hubLayoutId==8){ // Orbit: instrument-style climate dashboard.
    hubHeader(w,"ORBIT");
    hubTFT.setTextSize(2); hubTFT.setTextColor(HUB_WHITE,HUB_BG); hubTFT.setCursor(8,25); hubTFT.print(timeText);
    hubTFT.drawCircle(w/2,82,37,HUB_CYAN); hubTFT.drawCircle(w/2,82,33,HUB_LINE);
    hubTFT.drawCircle(w/2,82,29,HUB_LINE); hubTFT.drawFastHLine(25,82,w-50,HUB_LINE);
    hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED,HUB_BG); hubTFT.setCursor(51,58); hubTFT.print("ROOM");
    char v[18]; snprintf(v,sizeof(v),sensorValid?"%.1fC":"--.-C",sensorValid?indoorTempC:0.0f);
    drawValueBox(28,70,72,22,HUB_BG,HUB_WHITE,2,v);
    snprintf(v,sizeof(v),sensorValid?"%.0f%% RH":"--%% RH",sensorValid?indoorHumidity:0.0f);
    drawValueBox(35,94,58,12,HUB_BG,HUB_CYAN,1,v);
    hubCard(5,122,57,31,HUB_PANEL); hubCard(66,122,57,31,HUB_PANEL2);
    hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_MUTED,HUB_PANEL); hubTFT.setCursor(10,126); hubTFT.print("OUTSIDE");
    hubTFT.setTextColor(HUB_MUTED,HUB_PANEL2); hubTFT.setCursor(71,126); hubTFT.print("WIND");
    snprintf(v,sizeof(v),weatherValid?"%.1fC":"--.-C",weatherValid?outdoorTempC:0.0f);
    drawValueBox(9,137,49,12,HUB_PANEL,HUB_WHITE,1,v);
    snprintf(v,sizeof(v),weatherValid?"%.0f km/h":"-- km/h",weatherValid?outdoorWindSpeedKmh:0.0f);
    drawValueBox(70,137,49,12,HUB_PANEL2,HUB_CYAN,1,v);
    pushChangedTftFrame(); return;
  }

  if(hubLayoutId==9){ // Terminal: high-contrast monochrome telemetry.
    hubHeader(w,"TERMINAL");
    hubTFT.setTextSize(2); hubTFT.setTextColor(HUB_GREEN,HUB_BG); hubTFT.setCursor(8,25); hubTFT.print(timeText);
    hubTFT.drawFastHLine(7,44,w-14,HUB_GREEN);
    const char *labels[]={"ROOM TEMP","ROOM RH","OUT TEMP","WIND"};
    const int16_t ys[]={49,73,97,121};
    for(uint8_t i=0;i<4;i++){hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(9,ys[i]);hubTFT.print("> ");hubTFT.print(labels[i]);}
    char v[18];
    snprintf(v,sizeof(v),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(72,47,48,13,HUB_BG,HUB_GREEN,1,v);
    snprintf(v,sizeof(v),sensorValid?"%.0f %%":"-- %%",sensorValid?indoorHumidity:0.0f); drawValueBox(72,71,48,13,HUB_BG,HUB_GREEN,1,v);
    snprintf(v,sizeof(v),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(72,95,48,13,HUB_BG,HUB_GREEN,1,v);
    snprintf(v,sizeof(v),weatherValid?"%.0f km/h":"-- km/h",weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(72,119,48,13,HUB_BG,HUB_GREEN,1,v);
    hubTFT.drawFastHLine(7,141,w-14,HUB_GREEN);
    hubTFT.setTextSize(1); hubTFT.setTextColor(HUB_GREEN,HUB_BG); hubTFT.setCursor(9,147); hubTFT.print("GOODWIN // LIVE");
    pushChangedTftFrame(); return;
  }

  if(hubLayoutId==6){ // Aurora weather detail.
    hubHeader(w,"AURORA WX");
    if (landscape) {
      hubWeatherIcon(28,52,1);
      hubTFT.setTextSize(3); hubTFT.setTextColor(HUB_WHITE,HUB_BG); hubTFT.setCursor(56,31);
      if(weatherValid && !isnan(outdoorTempC)){hubTFT.print(outdoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-C");
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(58,60);
      hubTFT.print("FEELS "); if(weatherValid&&!isnan(outdoorFeelsLikeC)){hubTFT.print(outdoorFeelsLikeC,1);hubTFT.print("C");}else hubTFT.print("--");
      hubCard(5,78,73,43,HUB_PANEL); hubCard(82,78,73,43,HUB_PANEL2);
      hubLabel(12,85,"HUMIDITY"); hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_CYAN,HUB_PANEL);hubTFT.setCursor(12,99);if(weatherValid&&!isnan(outdoorHumidity)){hubTFT.print(outdoorHumidity,0);hubTFT.print("%");}else hubTFT.print("--%");
      hubLabel(89,85,"PRESSURE"); hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(89,99);if(weatherValid&&!isnan(outdoorPressureHpa)){hubTFT.print(outdoorPressureHpa,0);}else hubTFT.print("----");
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(6,126);hubTFT.print("WIND ");if(weatherValid&&!isnan(outdoorWindSpeedKmh)){hubTFT.print(outdoorWindSpeedKmh,0);hubTFT.print("km/h");}else hubTFT.print("--");
      hubTFT.setCursor(82,126);hubTFT.print("CLOUD ");if(weatherValid&&!isnan(outdoorCloudCover)){hubTFT.print(outdoorCloudCover,0);hubTFT.print("%");}else hubTFT.print("--");
    } else {
      hubWeatherIcon(100,43,1);
      hubTFT.setTextSize(3);hubTFT.setTextColor(HUB_WHITE,HUB_BG);hubTFT.setCursor(7,31);
      if(weatherValid&&!isnan(outdoorTempC)){hubTFT.print(outdoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-C");
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(8,62);hubTFT.print("FEELS LIKE ");if(weatherValid&&!isnan(outdoorFeelsLikeC)){hubTFT.print(outdoorFeelsLikeC,1);hubTFT.print("C");}else hubTFT.print("--");
      hubCard(5,79,57,31,HUB_PANEL);hubCard(66,79,57,31,HUB_PANEL2);
      hubLabel(10,85,"HUM");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_CYAN,HUB_PANEL);hubTFT.setCursor(10,96);if(weatherValid&&!isnan(outdoorHumidity)){hubTFT.print(outdoorHumidity,0);hubTFT.print("%");}else hubTFT.print("--");
      hubLabel(71,85,"hPa");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(71,96);if(weatherValid&&!isnan(outdoorPressureHpa)){hubTFT.print(outdoorPressureHpa,0);}else hubTFT.print("----");
      hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(7,119);hubTFT.print("WIND ");if(weatherValid&&!isnan(outdoorWindSpeedKmh)){hubTFT.print(outdoorWindSpeedKmh,0);hubTFT.print("km/h");}else hubTFT.print("--");
      hubTFT.setCursor(7,132);hubTFT.print("CLOUD ");if(weatherValid&&!isnan(outdoorCloudCover)){hubTFT.print(outdoorCloudCover,0);hubTFT.print("%");}else hubTFT.print("--");
      hubTFT.setCursor(7,145);hubTFT.print("RAIN ");if(weatherValid&&outdoorPrecipProbability>=0){hubTFT.print(outdoorPrecipProbability);hubTFT.print("%");}else hubTFT.print("--");
      hubTFT.setCursor(72,145);hubTFT.print("UV ");if(weatherValid&&!isnan(outdoorUvIndex)){hubTFT.print(outdoorUvIndex,1);}else hubTFT.print("--");
    }
    pushChangedTftFrame();
    return;
  }

  if(hubLayoutId==1){ // Analog + digital clock
    hubHeader(w,"CLOCK");
    const int16_t cx=w/2,cy=landscape?62:78,r=landscape?34:42;
    hubClockFace(cx,cy,r);
    hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_BG);
    int16_t tw=timeText.length()*12;hubTFT.setCursor((w-tw)/2,landscape?98:130);hubTFT.print(timeText);
    hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor((w-48)/2,landscape?117:148);hubTFT.print(dateText.substring(0,5));
    pushChangedTftFrame();
    return;
  }

  if(hubLayoutId==2){ // Weather hero
    hubHeader(w,"WEATHER");
    if(landscape){
      hubWeatherIcon(38,61,1);hubTFT.setTextColor(HUB_WHITE,HUB_BG);hubTFT.setTextSize(2);hubTFT.setCursor(78,43);
      if(weatherValid){hubTFT.print(outdoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-C");
      hubCard(7,91,146,28,HUB_PANEL);hubDropIcon(20,103,1,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(32,101);hubTFT.print("RAIN CHANCE");hubTFT.setCursor(111,101);if(weatherValid){hubTFT.print(outdoorPrecipProbability);hubTFT.print("%");}else hubTFT.print("--");
    }else{
      hubWeatherIcon(64,49,2);hubTFT.setTextSize((weatherValid && outdoorTempC < -9.9f)?2:3);hubTFT.setTextColor(HUB_WHITE,HUB_BG);hubTFT.setCursor((weatherValid && outdoorTempC < -9.9f)?10:20,76);if(weatherValid){hubTFT.print(outdoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-C");
      hubCard(7,119,114,31,HUB_PANEL);hubDropIcon(19,132,1,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(31,126);hubTFT.print("RAIN CHANCE");hubTFT.setCursor(31,138);if(weatherValid){hubTFT.print(outdoorPrecipProbability);hubTFT.print("%  ");hubTFT.print(outdoorPrecipMm,1);hubTFT.print("mm");}else hubTFT.print("Waiting for data");
    }
    pushChangedTftFrame();
    return;
  }

  if(hubLayoutId==3){ // Indoor climate, graphic cards
    hubHeader(w,"INDOOR");
    if(landscape){
      hubCard(6,28,72,91,HUB_PANEL);hubCard(82,28,72,91,HUB_PANEL2);
      hubThermoIcon(15,39,tempColor);hubLabel(33,43,"TEMP");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(13,76);if(sensorValid){hubTFT.print(indoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-");
      hubHumidityIcon(91,42,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_CYAN,HUB_PANEL2);hubTFT.setCursor(108,47);hubTFT.print("HUM");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(90,78);if(sensorValid){hubTFT.print(indoorHumidity,1);hubTFT.print("%");}else hubTFT.print("--.-%");
    }else{
      hubCard(6,27,116,57,HUB_PANEL);hubCard(6,91,116,59,HUB_PANEL2);
      hubThermoIcon(16,39,tempColor);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_PANEL);hubTFT.setCursor(39,38);hubTFT.print("TEMPERATURE");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(39,57);if(sensorValid){hubTFT.print(indoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-C");
      hubHumidityIcon(17,103,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_CYAN,HUB_PANEL2);hubTFT.setCursor(39,104);hubTFT.print("HUMIDITY");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(39,119);if(sensorValid){hubTFT.print(indoorHumidity,1);hubTFT.print("%");}else hubTFT.print("--.-%");
    }
    pushChangedTftFrame();
    return;
  }

  if(hubLayoutId==4){ // Indoor + outdoor comparison
    hubHeader(w,"CLIMATE");
    if(landscape){
      hubCard(6,27,72,91,HUB_PANEL);hubCard(82,27,72,91,HUB_PANEL2);
      hubThermoIcon(14,38,tempColor);hubLabel(32,43,"HOME");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setTextSize((sensorValid && (indoorTempC < -9.9f || indoorTempC >= 100.0f))?1:2);hubTFT.setCursor(12,77);if(sensorValid){hubTFT.print(indoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-");
      hubWeatherIcon(118,44,1);hubLabel(96,74,"OUTSIDE");hubTFT.setTextSize((weatherValid && outdoorTempC < -9.9f)?1:2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(87,92);if(weatherValid){hubTFT.print(outdoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-");
    }else{
      hubCard(6,27,116,55,HUB_PANEL);hubCard(6,88,116,62,HUB_PANEL2);
      hubThermoIcon(15,39,tempColor);hubLabel(38,39,"HOME");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(39,57);if(sensorValid){hubTFT.print(indoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-C");
      hubCloudIcon(27,112,1,HUB_CYAN);hubLabel(49,105,"OUTSIDE");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(49,123);if(weatherValid){hubTFT.print(outdoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-C");
    }
    pushChangedTftFrame();
    return;
  }

  if(hubLayoutId==5){ // Light graphic status
    hubHeader(w,"LIGHT");
    hubBulbIcon(w/2,landscape?47:58,1,lightOn?HUB_YELLOW:HUB_MUTED);
    hubTFT.setTextSize(2);hubTFT.setTextColor(lightOn?HUB_YELLOW:HUB_WHITE,HUB_BG);
    const char *state=lightOn?"ON":"OFF";int16_t sw=(int16_t)strlen(state)*12;hubTFT.setCursor((w-sw)/2,landscape?78:98);hubTFT.print(state);
    hubCard(landscape?18:8,landscape?99:121,landscape?124:112,landscape?23:29,HUB_PANEL);
    hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_PANEL);hubTFT.setCursor(landscape?25:15,landscape?107:128);hubTFT.print("BRIGHTNESS");
    hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(landscape?105:90,landscape?107:128);hubTFT.print(brightnessPercent);hubTFT.print("%");
    hubProgress(landscape?25:15,landscape?115:140,landscape?110:90,6,brightnessPercent,HUB_YELLOW);
    pushChangedTftFrame();
    return;
  }

  // Dashboard: clock + two climate cards + rain + lamp status.
  hubHeader(w,"HOME");
  if(landscape){
    hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_BG);hubTFT.setCursor(7,23);hubTFT.print(timeText);
    hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(91,29);hubTFT.print(dateText.substring(0,5));
    hubCard(5,45,73,47,HUB_PANEL);hubCard(82,45,73,47,HUB_PANEL2);
    hubThermoMini(10,52,tempColor);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_PANEL);hubTFT.setCursor(29,54);hubTFT.print("HOME");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setTextSize((sensorValid && (indoorTempC < -9.9f || indoorTempC >= 100.0f))?1:2);hubTFT.setCursor(12,72);if(sensorValid){hubTFT.print(indoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-");
    hubCloudIcon(98,57,1,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_PANEL2);hubTFT.setCursor(119,54);hubTFT.print("OUT");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setTextSize((weatherValid && outdoorTempC < -9.9f)?1:2);hubTFT.setCursor(88,72);if(weatherValid){hubTFT.print(outdoorTempC,1);hubTFT.print("C");}else hubTFT.print("--.-");
    hubCard(5,98,150,24,HUB_PANEL);hubDropIcon(16,108,1,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(28,106);hubTFT.print("RAIN");hubTFT.setCursor(67,106);if(weatherValid){hubTFT.print(outdoorPrecipProbability);hubTFT.print("%  ");hubTFT.print(outdoorPrecipMm,1);hubTFT.print("mm");}else hubTFT.print("--");
  }else{
    hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_BG);hubTFT.setCursor(7,23);hubTFT.print(timeText);
    hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_BG);hubTFT.setCursor(82,30);hubTFT.print(dateText.substring(0,5));
    hubCard(5,45,57,48,HUB_PANEL);hubCard(66,45,57,48,HUB_PANEL2);
    hubThermoMini(9,52,tempColor);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_PANEL);hubTFT.setCursor(27,54);hubTFT.print("HOME");hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(10,75);if(sensorValid){hubTFT.print(indoorTempC,1);}else hubTFT.print("--.-");
    hubCloudIcon(84,60,1,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_MUTED,HUB_PANEL2);hubTFT.setCursor(103,54);hubTFT.print("OUT");hubTFT.setTextSize((weatherValid && outdoorTempC < -9.9f)?1:2);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(71,75);if(weatherValid){hubTFT.print(outdoorTempC,1);}else hubTFT.print("--.-");
    hubCard(5,99,118,25,HUB_PANEL);hubDropIcon(15,109,1,HUB_CYAN);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL);hubTFT.setCursor(27,107);hubTFT.print("RAIN");hubTFT.setCursor(59,107);if(weatherValid){hubTFT.print(outdoorPrecipProbability);hubTFT.print("%");}else hubTFT.print("--");
    hubCard(5,129,118,24,HUB_PANEL2);hubTFT.fillCircle(16,141,4,lightOn?HUB_YELLOW:HUB_MUTED);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_WHITE,HUB_PANEL2);hubTFT.setCursor(27,137);hubTFT.print(lightOn?"LIGHT ON":"LIGHT OFF");hubTFT.setCursor(88,137);hubTFT.print(brightnessPercent);hubTFT.print("%");
  }
  pushChangedTftFrame();
}

// Drawing helpers now write directly to hubTFT. Kept as a no-op so the
// layout code stays readable without allocating or copying a framebuffer.
void pushChangedTftFrame() {
  // Intentionally empty: all pixels have already been sent to the ST7735.
}

// Legacy frame counter intentionally disabled. The display is event-driven,
// so an FPS value is misleading when no pixels need updating.
static void recordTftFrame() {}

// Partial redraw helpers. Each one clears and paints only its own widget area;
// the ST7735 never receives a full-screen clear for ordinary telemetry changes.
static void drawDashboardClockPartial() {
  const bool landscape = hubTFT.width() > hubTFT.height();
  drawValueBox(5, 21, 72, 19, HUB_BG, HUB_WHITE, 2, timeText.c_str());
  const String shortDate = dateText.substring(0, 5);
  drawValueBox(landscape ? 91 : 82, landscape ? 27 : 28,
               landscape ? 61 : 40, 12, HUB_BG, HUB_MUTED, 1, shortDate.c_str());
}

static void drawAnalogClockPartial() {
  const bool landscape = hubTFT.width() > hubTFT.height();
  const int16_t cx = hubTFT.width() / 2, cy = landscape ? 62 : 78, r = landscape ? 34 : 42;
  hubTFT.fillRect(cx - r - 3, cy - r - 3, 2 * r + 6, 2 * r + 6, HUB_BG);
  hubClockFace(cx, cy, r);
  const int16_t ty = landscape ? 98 : 130;
  drawValueBox((hubTFT.width() - 64) / 2, ty - 2, 64, 19, HUB_BG, HUB_WHITE, 2, timeText.c_str());
  const String shortDate = dateText.substring(0, 5);
  drawValueBox((hubTFT.width() - 48) / 2, (landscape ? 117 : 148) - 2,
               48, 12, HUB_BG, HUB_MUTED, 1, shortDate.c_str());
}

// Render each changing value completely in RAM, then transfer the finished patch.
// This removes the visible clear-then-print interval during telemetry refreshes. (5.8 KB RAM)
struct HubValueCacheEntry {
  bool valid;
  int16_t x, y, w, h;
  uint16_t bg, fg;
  uint8_t textSize;
  char value[32];
};
static HubValueCacheEntry hubValueCache[16] = {};
static uint8_t hubValueCacheNext = 0;

static void hubInvalidateValueCache() {
  for (uint8_t i = 0; i < 16; ++i) hubValueCache[i].valid = false;
}

static void drawValueBox(int16_t x, int16_t y, int16_t w, int16_t h,
                         uint16_t bg, uint16_t fg, uint8_t textSize,
                         const char *value) {
  if (w <= 0 || h <= 0 || !value) return;
  // Do not transmit unchanged values. A changed value is always composed over
  // a freshly restored background in the small RAM canvas before one SPI patch.
  if (w <= HUB_VALUE_CANVAS_W && h <= HUB_VALUE_CANVAS_H) {
    for (uint8_t i = 0; i < 16; ++i) {
      HubValueCacheEntry &entry = hubValueCache[i];
      if (entry.valid && entry.x == x && entry.y == y && entry.w == w && entry.h == h &&
          entry.bg == bg && entry.fg == fg && entry.textSize == textSize &&
          strcmp(entry.value, value) == 0) return;
    }
  }
  if (w > HUB_VALUE_CANVAS_W || h > HUB_VALUE_CANVAS_H) {
    // Defensive fallback for future widgets larger than the reusable patch.
    hubTFT.fillRect(x, y, w, h, bg);
    hubTFT.setTextSize(textSize);
    hubTFT.setTextColor(fg, bg);
    hubTFT.setCursor(x + 2, y + 2);
    hubTFT.print(value);
    return;
  }
  hubValueCanvas.fillScreen(bg);
  hubValueCanvas.setTextWrap(false);
  hubValueCanvas.setTextSize(textSize);
  hubValueCanvas.setTextColor(fg, bg);
  hubValueCanvas.setCursor(2, 2);
  hubValueCanvas.print(value);
  uint16_t *pixels = static_cast<uint16_t *>(hubValueCanvas.getBuffer());
  if (!pixels) {
    // Keep the display functional if the small canvas allocation ever fails.
    hubTFT.fillRect(x, y, w, h, bg);
    hubTFT.setTextSize(textSize);
    hubTFT.setTextColor(fg, bg);
    hubTFT.setCursor(x + 2, y + 2);
    hubTFT.print(value);
    return;
  }
  // Single address window; source rows use the canvas' fixed 114-pixel stride.
  hubTFT.startWrite();
  hubTFT.setAddrWindow(x, y, w, h);
  for (int16_t row = 0; row < h; ++row) {
    hubTFT.writePixels(pixels + (size_t)row * HUB_VALUE_CANVAS_W, w, true);
  }
  hubTFT.waitDMA();
  hubTFT.endWrite();
  HubValueCacheEntry &entry = hubValueCache[hubValueCacheNext++ % 16];
  entry.valid = true; entry.x = x; entry.y = y; entry.w = w; entry.h = h;
  entry.bg = bg; entry.fg = fg; entry.textSize = textSize;
  strlcpy(entry.value, value, sizeof(entry.value));
}

static void drawLayoutSensorPartial() {
  const bool landscape = hubTFT.width() > hubTFT.height();
  char value[20];
  if (hubLayoutId == 8) {
    snprintf(value,sizeof(value),sensorValid?"%.1fC":"--.-C",sensorValid?indoorTempC:0.0f); drawValueBox(28,70,72,22,HUB_BG,HUB_WHITE,2,value);
    snprintf(value,sizeof(value),sensorValid?"%.0f%% RH":"--%% RH",sensorValid?indoorHumidity:0.0f); drawValueBox(35,94,58,12,HUB_BG,HUB_CYAN,1,value); return;
  }
  if (hubLayoutId == 9) {
    snprintf(value,sizeof(value),sensorValid?"%.1f C":"--.- C",sensorValid?indoorTempC:0.0f); drawValueBox(72,47,48,13,HUB_BG,HUB_GREEN,1,value);
    snprintf(value,sizeof(value),sensorValid?"%.0f %%":"-- %%",sensorValid?indoorHumidity:0.0f); drawValueBox(72,71,48,13,HUB_BG,HUB_GREEN,1,value); return;
  }
  if (hubLayoutId == 0) {
    snprintf(value, sizeof(value), sensorValid ? "%.1f" : "--.-", sensorValid ? indoorTempC : 0.0f);
    drawValueBox(landscape ? 12 : 8, 73, landscape ? 60 : 52, 18, HUB_PANEL, HUB_WHITE,
      (sensorValid && indoorTempC < -9.9f) ? 1 : 2, value);
  } else if (hubLayoutId == 3) {
    if (landscape) {
      snprintf(value, sizeof(value), sensorValid ? "%.1fC" : "--.-C", sensorValid ? indoorTempC : 0.0f);
      drawValueBox(11, 74, 65, 23, HUB_PANEL, HUB_WHITE, 2, value);
      snprintf(value, sizeof(value), sensorValid ? "%.1f%%" : "--.-%%", sensorValid ? indoorHumidity : 0.0f);
      drawValueBox(89, 76, 63, 25, HUB_PANEL2, HUB_WHITE, 2, value);
    } else {
      snprintf(value, sizeof(value), sensorValid ? "%.1fC" : "--.-C", sensorValid ? indoorTempC : 0.0f);
      drawValueBox(37, 55, 80, 21, HUB_PANEL, HUB_WHITE, 2, value);
      snprintf(value, sizeof(value), sensorValid ? "%.1f%%" : "--.-%%", sensorValid ? indoorHumidity : 0.0f);
      drawValueBox(37, 117, 80, 25, HUB_PANEL2, HUB_WHITE, 2, value);
    }
  } else if (hubLayoutId == 4) {
    snprintf(value, sizeof(value), sensorValid ? "%.1fC" : "--.-C", sensorValid ? indoorTempC : 0.0f);
    drawValueBox(landscape ? 11 : 37, landscape ? 74 : 55, landscape ? 65 : 80, 23, HUB_PANEL, HUB_WHITE, 2, value);
  }
}
static void drawLayoutWeatherPartial() {
  const bool landscape = hubTFT.width() > hubTFT.height();
  char value[28];
  if (hubLayoutId == 8) {
    snprintf(value,sizeof(value),weatherValid?"%.1fC":"--.-C",weatherValid?outdoorTempC:0.0f); drawValueBox(9,137,49,12,HUB_PANEL,HUB_WHITE,1,value);
    snprintf(value,sizeof(value),weatherValid?"%.0f km/h":"-- km/h",weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(70,137,49,12,HUB_PANEL2,HUB_CYAN,1,value); return;
  }
  if (hubLayoutId == 9) {
    snprintf(value,sizeof(value),weatherValid?"%.1f C":"--.- C",weatherValid?outdoorTempC:0.0f); drawValueBox(72,95,48,13,HUB_BG,HUB_GREEN,1,value);
    snprintf(value,sizeof(value),weatherValid?"%.0f km/h":"-- km/h",weatherValid?outdoorWindSpeedKmh:0.0f); drawValueBox(72,119,48,13,HUB_BG,HUB_GREEN,1,value); return;
  }
  if (hubLayoutId == 0) {
    snprintf(value, sizeof(value), weatherValid ? "%.1f" : "--.-", weatherValid ? outdoorTempC : 0.0f);
    drawValueBox(landscape ? 88 : 69, 73, landscape ? 61 : 52, 18, HUB_PANEL2, HUB_WHITE,
      (weatherValid && outdoorTempC < -9.9f) ? 1 : 2, value);
    if (landscape) {
      snprintf(value, sizeof(value), weatherValid ? "%d%% %.1fmm" : "--", weatherValid ? outdoorPrecipProbability : 0, weatherValid ? outdoorPrecipMm : 0.0f);
      drawValueBox(67, 104, 80, 14, HUB_PANEL, HUB_WHITE, 1, value);
    } else {
      snprintf(value, sizeof(value), weatherValid ? "%d%%" : "--", weatherValid ? outdoorPrecipProbability : 0);
      drawValueBox(57, 105, 60, 14, HUB_PANEL, HUB_WHITE, 1, value);
    }
  } else if (hubLayoutId == 6) {
    if (landscape) {
      hubTFT.fillRect(5,25,45,48,HUB_BG); hubWeatherIcon(28,52,1);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorTempC)?"%.1fC":"--.-C",weatherValid?outdoorTempC:0.0f);
      drawValueBox(54,29,100,26,HUB_BG,HUB_WHITE,3,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorFeelsLikeC)?"FEELS %.1fC":"FEELS --",weatherValid?outdoorFeelsLikeC:0.0f);
      drawValueBox(56,58,96,12,HUB_BG,HUB_MUTED,1,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorHumidity)?"%.0f%%":"--%",weatherValid?outdoorHumidity:0.0f);
      drawValueBox(10,98,58,17,HUB_PANEL,HUB_CYAN,2,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorPressureHpa)?"%.0f":"----",weatherValid?outdoorPressureHpa:0.0f);
      drawValueBox(87,98,61,17,HUB_PANEL2,HUB_WHITE,2,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorWindSpeedKmh)?"WIND %.0fkm/h":"WIND --",weatherValid?outdoorWindSpeedKmh:0.0f);
      drawValueBox(5,124,72,10,HUB_BG,HUB_MUTED,1,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorCloudCover)?"CLOUD %.0f%%":"CLOUD --",weatherValid?outdoorCloudCover:0.0f);
      drawValueBox(81,124,74,10,HUB_BG,HUB_MUTED,1,value);
    } else {
      hubTFT.fillRect(88,22,38,45,HUB_BG); hubWeatherIcon(100,43,1);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorTempC)?"%.1fC":"--.-C",weatherValid?outdoorTempC:0.0f);
      drawValueBox(5,28,90,26,HUB_BG,HUB_WHITE,3,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorFeelsLikeC)?"FEELS %.1fC":"FEELS --",weatherValid?outdoorFeelsLikeC:0.0f);
      drawValueBox(7,60,112,12,HUB_BG,HUB_MUTED,1,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorHumidity)?"%.0f%%":"--%",weatherValid?outdoorHumidity:0.0f);
      drawValueBox(9,94,48,14,HUB_PANEL,HUB_CYAN,2,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorPressureHpa)?"%.0f":"----",weatherValid?outdoorPressureHpa:0.0f);
      drawValueBox(70,94,48,14,HUB_PANEL2,HUB_WHITE,2,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorWindSpeedKmh)?"WIND %.0fkm/h":"WIND --",weatherValid?outdoorWindSpeedKmh:0.0f);
      drawValueBox(7,119,112,10,HUB_BG,HUB_MUTED,1,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorCloudCover)?"CLOUD %.0f%%":"CLOUD --",weatherValid?outdoorCloudCover:0.0f);
      drawValueBox(7,132,112,10,HUB_BG,HUB_MUTED,1,value);
      snprintf(value,sizeof(value),weatherValid&&outdoorPrecipProbability>=0?"RAIN %d%%":"RAIN --",weatherValid?outdoorPrecipProbability:0);
      drawValueBox(7,145,60,10,HUB_BG,HUB_MUTED,1,value);
      snprintf(value,sizeof(value),weatherValid&&!isnan(outdoorUvIndex)?"UV %.1f":"UV --",weatherValid?outdoorUvIndex:0.0f);
      drawValueBox(72,145,48,10,HUB_BG,HUB_MUTED,1,value);
    }
  } else if (hubLayoutId == 2) {
    if (landscape) {
      snprintf(value, sizeof(value), weatherValid ? "%.1fC" : "--.-C", weatherValid ? outdoorTempC : 0.0f);
      drawValueBox(77, 41, 72, 26, HUB_BG, HUB_WHITE, 2, value);
      snprintf(value, sizeof(value), weatherValid ? "%d%%" : "--", weatherValid ? outdoorPrecipProbability : 0);
      drawValueBox(108, 99, 35, 14, HUB_PANEL, HUB_WHITE, 1, value);
    } else {
      snprintf(value, sizeof(value), weatherValid ? "%.1fC" : "--.-C", weatherValid ? outdoorTempC : 0.0f);
      drawValueBox(7, 72, 114, 26, HUB_BG, HUB_WHITE, (weatherValid && outdoorTempC < -9.9f) ? 2 : 3, value);
      snprintf(value, sizeof(value), weatherValid ? "%d%%  %.1fmm" : "--", weatherValid ? outdoorPrecipProbability : 0, weatherValid ? outdoorPrecipMm : 0.0f);
      drawValueBox(29, 136, 88, 12, HUB_PANEL, HUB_WHITE, 1, value);
    }
  } else if (hubLayoutId == 4) {
    snprintf(value, sizeof(value), weatherValid ? "%.1fC" : "--.-C", weatherValid ? outdoorTempC : 0.0f);
    drawValueBox(landscape ? 87 : 47, landscape ? 90 : 121, landscape ? 65 : 70, 23, HUB_PANEL2, HUB_WHITE, 2, value);
  }
}
static void drawLayoutLightPartial() {
  const bool landscape = hubTFT.width() > hubTFT.height();
  if (hubLayoutId == 0) {
    if (!landscape) {
      drawValueBox(25, 135, 58, 13, HUB_PANEL2, HUB_WHITE, 1, lightOn ? "LIGHT ON" : "LIGHT OFF");
      char value[8]; snprintf(value, sizeof(value), "%u%%", brightnessPercent);
      drawValueBox(86, 135, 34, 13, HUB_PANEL2, HUB_WHITE, 1, value);
    }
  } else if (hubLayoutId == 5) {
    char value[8]; snprintf(value, sizeof(value), "%u%%", brightnessPercent);
    if (landscape) {
      drawValueBox(105, 104, 34, 13, HUB_PANEL, HUB_WHITE, 1, value);
      hubProgress(25, 115, 110, 6, brightnessPercent, HUB_YELLOW);
    } else {
      drawValueBox(87, 126, 28, 13, HUB_PANEL, HUB_WHITE, 1, value);
      hubProgress(15, 140, 90, 6, brightnessPercent, HUB_YELLOW);
    }
  }
}

static void drawModernLayoutPartials() {
  const int16_t w = hubTFT.width(), h = hubTFT.height();
  const bool amber = hubLayoutId == 11;
  const uint16_t accent = amber ? HUB_YELLOW : (hubLayoutId == 13 ? 0xF81F : HUB_CYAN);
  const uint16_t panel = hubLayoutId == 12 ? 0x10A4 : HUB_PANEL;
  char value[24];

  // All live fields use the same checked patch renderer: unchanged text is
  // skipped; changed text is composed over the correct panel/background.
  drawValueBox(8, 29, 82, 18, HUB_BG, HUB_WHITE, 2, timeText.c_str());
  drawValueBox(8, 51, 112, 10, HUB_BG, HUB_MUTED, 1, dateText.c_str());

  snprintf(value, sizeof(value), sensorValid ? "%.1f C" : "--.- C", sensorValid ? indoorTempC : 0.0f);
  drawValueBox(14, 84, 55, 15, panel, HUB_WHITE, 1, value);
  snprintf(value, sizeof(value), sensorValid ? "%.0f%% RH" : "--%% RH", sensorValid ? indoorHumidity : 0.0f);
  drawValueBox(w - 66, 84, 52, 15, panel, accent, 1, value);

  snprintf(value, sizeof(value), weatherValid ? "%.1fC" : "--.-C", weatherValid ? outdoorTempC : 0.0f);
  drawValueBox(12, 126, (w - 30) / 2, 13, HUB_PANEL, HUB_WHITE, 1, value);
  snprintf(value, sizeof(value), weatherValid ? "%.0fkm/h" : "--km/h", weatherValid ? outdoorWindSpeedKmh : 0.0f);
  drawValueBox(18 + (w - 22) / 2, 126, (w - 30) / 2, 13, HUB_PANEL2, accent, 1, value);

  drawValueBox(10, h - 13, 62, 10, HUB_BG, accent, 1,
               WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE");
}

// ============================================================
// Unified LovyanGFX screensaver engine. All effects use one reusable RGB565
// scene buffer, one premium gradient background and one synchronized DMA transfer.
// ============================================================
void hubMarkActivity() {
  hubLastActivityAt = millis();
  if (hubSaverActive) {
    hubSaverActive = false;
    hubForceFullRedraw = true;
    hubDisplayDirty = true;
    hubClockDirty = hubSensorDirty = hubWeatherDirty = hubLightDirty = false;
  }
}

static void hubInvalidateValueCache();

static void initHubScreenSaver() {
  const int16_t w = hubTFT.width();
  const int16_t h = hubTFT.height();
  const int16_t bandTop = 25;
  const int16_t bandHeight = h - 43;

  // One shared RGB565 scene buffer is used by every animated saver. The
  // visible animation area is composed completely off-screen and transferred
  // in one DMA operation, so no saver exposes an erase-then-redraw sequence.
  if (!hubSaverCanvas || hubSaverCanvasW != w || hubSaverCanvasH != bandHeight) {
    delete hubSaverCanvas;
    hubSaverCanvas = nullptr;
    hubSaverCanvasW = 0;
    hubSaverCanvasH = 0;
    if (w > 0 && bandHeight > 0) {
      hubSaverCanvas = new lgfx::LGFX_Sprite(&hubTFT);
      if (hubSaverCanvas) {
        hubSaverCanvas->setColorDepth(16);
        void *spriteBuffer = hubSaverCanvas->createSprite(w, bandHeight);
        if (spriteBuffer) {
          hubSaverCanvasW = w;
          hubSaverCanvasH = bandHeight;
          LOG_B("LovyanGFX saver sprite ready: %dx%d, %u bytes; free heap=%u",
                w, bandHeight, (unsigned)(w * bandHeight * sizeof(uint16_t)), ESP.getFreeHeap());
        } else {
          delete hubSaverCanvas;
          hubSaverCanvas = nullptr;
          LOG_ERR("LovyanGFX saver sprite allocation failed; heap=%u", ESP.getFreeHeap());
        }
      } else {
        delete hubSaverCanvas;
        hubSaverCanvas = nullptr;
        LOG_ERR("Unified saver canvas allocation failed; heap=%u", ESP.getFreeHeap());
      }
    }
  }

  hubInvalidateValueCache();
  hubTFT.fillScreen(HUB_BG);

  // Shared frame chrome: the same title, divider and footer are used by every
  // saver. The animation canvas begins below the header and ends above footer.
  static const char *SAVER_TITLES[22] = {
    "", "CLOCK", "ORBIT DOT", "STARS", "RAIN", "SNOW", "CLOUDS", "INDOOR CLIMATE",
    "WAVE", "PULSE", "AURORA", "DNA / DOUBLE HELIX", "NEBULA DRIFT", "MATRIX RAIN",
    "COMET TRAILS", "FIREFLIES", "RADAR SWEEP", "PLANETARY ORBIT", "EQUALIZER",
    "PRISM TUNNEL", "FRACTAL BLOOM", "STAR WARP"
  };
  hubTFT.setTextSize(1);
  hubTFT.setTextColor(HUB_CYAN, HUB_BG);
  hubTFT.setCursor(7, 7);
  hubTFT.print(SAVER_TITLES[hubSaverId < HUB_SAVER_COUNT ? hubSaverId : 0]);
  hubTFT.drawFastHLine(7, 19, w - 14, HUB_LINE);
  hubTFT.drawFastHLine(7, h - 18, w - 14, HUB_LINE);
  hubTFT.setTextColor(HUB_MUTED, HUB_BG);
  hubTFT.setCursor(7, h - 11);
  hubTFT.print("GOODWIN  /  ESP32");

  saverDotX = w / 3;
  saverDotY = max(12, bandHeight / 3);
  saverDotVX = 2;
  saverDotVY = 1;
  saverDotRadius = 5;
  saverPulseRadius = 12;
  saverPulseDirection = 1;
  saverCloudX[0] = -36;
  saverCloudX[1] = w / 2;
  hubSaverLastValueAt = 0;
  saverLastTemperature = NAN;
  saverLastHumidity = NAN;

  for (uint8_t i = 0; i < 18; ++i) {
    saverStarX[i] = (uint8_t)random(3, max(4, (int)w - 3));
    saverStarY[i] = (uint8_t)random(3, max(4, (int)bandHeight - 3));
    saverStarPhase[i] = random(0, 2);
  }
  for (uint8_t i = 0; i < 10; ++i) {
    saverRainX[i] = (uint8_t)random(3, max(4, (int)w - 3));
    saverRainY[i] = (uint8_t)random(2, max(3, (int)bandHeight - 2));
    saverSnowX[i] = (uint8_t)random(3, max(4, (int)w - 3));
    saverSnowY[i] = (uint8_t)random(2, max(3, (int)bandHeight - 2));
  }
  for (uint8_t i = 0; i < 16; ++i) {
    saverMatrixY[i] = random(0, max(1, (int)bandHeight));
    saverMatrixSpeed[i] = random(1, 4);
  }
}

static void drawSaverCanvasBackdrop(lgfx::LGFX_Sprite &cv, int16_t w, int16_t h) {
  // LovyanGFX native radial gradient: a quiet midnight base with a soft
  // atmospheric blue lift. No graph-paper grid or high-contrast decoration.
  cv.fillGradientRect(0, 0, w, h, 0x0208, 0x10A5);
  cv.drawFastHLine(8, 3, w - 16, 0x19A7);
  cv.drawFastHLine(8, h - 4, w - 16, 0x10A4);
  cv.drawPixel(8, 8, 0x3A0D);
  cv.drawPixel(w - 9, 8, 0x3A0D);
}

static void drawHubScreenSaverFrame() {
  static uint32_t diagWindowAt = 0;
  static uint32_t diagFrames = 0;
  static uint32_t diagTotalUs = 0;
  static uint32_t diagMaxUs = 0;
  static bool allocationWarningShown = false;
  const uint32_t frameStartedUs = micros();

  if (!hubSaverCanvas || !hubSaverCanvas->getBuffer()) {
    if (!allocationWarningShown) {
      allocationWarningShown = true;
      LOG_ERR("Screen saver paused: shared RGB565 canvas is unavailable");
    }
    return;
  }

  lgfx::LGFX_Sprite &cv = *hubSaverCanvas;
  const int16_t w = hubSaverCanvasW;
  const int16_t h = hubSaverCanvasH;
  const uint32_t now = millis();
  const float phase = now * 0.0011f;
  bool presentFrame = true;

  drawSaverCanvasBackdrop(cv, w, h);

  switch (hubSaverId) {
    case 1: { // Minimal digital clock, refreshed only when the second changes.
      if ((uint32_t)(now - hubSaverLastValueAt) < 1000UL) {
        presentFrame = false;
        break;
      }
      hubSaverLastValueAt = now;
      cv.setTextSize(2);
      cv.setTextColor(HUB_WHITE);
      cv.setCursor((w - 60) / 2, h / 2 - 13);
      cv.print(timeText);
      cv.setTextSize(1);
      cv.setTextColor(HUB_MUTED);
      cv.setCursor((w - 48) / 2, h / 2 + 12);
      cv.print(dateText.substring(0, 5));
      break;
    }

    case 2: { // Orbit dot, with its frame composed atomically.
      cv.drawRoundRect(5, 5, w - 10, h - 10, 8, HUB_LINE);
      cv.drawCircle(w / 2, h / 2, 18, 0x2149);
      cv.fillCircle(saverDotX, saverDotY, saverDotRadius, HUB_CYAN);
      saverDotX += saverDotVX;
      saverDotY += saverDotVY;
      if (saverDotX < saverDotRadius + 7 || saverDotX > w - saverDotRadius - 7) saverDotVX = -saverDotVX;
      if (saverDotY < saverDotRadius + 7 || saverDotY > h - saverDotRadius - 7) saverDotVY = -saverDotVY;
      saverDotX = constrain(saverDotX, (int16_t)(saverDotRadius + 7), (int16_t)(w - saverDotRadius - 7));
      saverDotY = constrain(saverDotY, (int16_t)(saverDotRadius + 7), (int16_t)(h - saverDotRadius - 7));
      break;
    }

    case 3: { // Stars: stable backdrop, independently twinkling stars.
      const uint8_t changed = random(0, 18);
      saverStarPhase[changed] ^= 1;
      for (uint8_t i = 0; i < 18; ++i) {
        const uint16_t c = saverStarPhase[i] ? HUB_WHITE : (i % 3 ? HUB_MUTED : HUB_CYAN);
        cv.drawPixel(saverStarX[i], saverStarY[i], c);
        if (saverStarPhase[i] && (i % 4 == 0)) cv.drawPixel(saverStarX[i] + 1, saverStarY[i], HUB_CYAN);
      }
      break;
    }

    case 4: { // Rain: drops are redrawn into the scene buffer, never erased on TFT.
      for (uint8_t i = 0; i < 10; ++i) {
        cv.drawFastVLine(saverRainX[i], saverRainY[i], 5, (i % 3) ? HUB_CYAN : HUB_WHITE);
        saverRainY[i] = (uint8_t)(saverRainY[i] + 5);
        if (saverRainY[i] > h - 6) {
          saverRainY[i] = random(2, 18);
          saverRainX[i] = random(3, max(4, (int)w - 3));
        }
      }
      break;
    }

    case 5: { // Snow: subtle flakes with varied speed and brightness.
      for (uint8_t i = 0; i < 10; ++i) {
        cv.drawPixel(saverSnowX[i], saverSnowY[i], (i % 3) ? HUB_WHITE : HUB_CYAN);
        if ((i & 1) == 0) cv.drawPixel(saverSnowX[i] + 1, saverSnowY[i], HUB_MUTED);
        saverSnowY[i] = (uint8_t)(saverSnowY[i] + 1 + (i % 2));
        if (saverSnowY[i] > h - 3) {
          saverSnowY[i] = 2;
          saverSnowX[i] = random(3, max(4, (int)w - 3));
        }
      }
      break;
    }

    case 6: { // Clouds: two soft silhouettes drifting across the common backdrop.
      for (uint8_t i = 0; i < 2; ++i) {
        const int16_t y = h / 2 - 12 + i * 28;
        const int16_t x = saverCloudX[i];
        const uint16_t c = i ? 0x5DDF : 0xCE7F;
        cv.fillCircle(x + 10, y, 7, c);
        cv.fillCircle(x + 20, y - 5, 10, c);
        cv.fillCircle(x + 31, y, 7, c);
        cv.fillRoundRect(x + 7, y, 28, 8, 4, c);
        saverCloudX[i] += (i == 0 ? 1 : 2);
        if (saverCloudX[i] > w + 5) saverCloudX[i] = -42;
      }
      break;
    }

    case 7: { // Climate card: values update once per second, no flashing text box.
      if ((uint32_t)(now - hubSaverLastValueAt) < 1000UL &&
          !changedAtTenth(indoorTempC, saverLastTemperature) &&
          !changedAtTenth(indoorHumidity, saverLastHumidity)) {
        presentFrame = false;
        break;
      }
      hubSaverLastValueAt = now;
      saverLastTemperature = indoorTempC;
      saverLastHumidity = indoorHumidity;
      cv.drawRoundRect(8, h / 2 - 39, w - 16, 78, 8, HUB_LINE);
      cv.setTextSize(1);
      cv.setTextColor(HUB_MUTED);
      cv.setCursor(16, h / 2 - 29);
      cv.print("INDOOR TEMPERATURE");
      char value[20];
      snprintf(value, sizeof(value), sensorValid ? "%.1f C" : "--.- C", sensorValid ? indoorTempC : 0.0f);
      cv.setTextSize(2);
      cv.setTextColor(HUB_WHITE);
      cv.setCursor(16, h / 2 - 15);
      cv.print(value);
      cv.setTextSize(1);
      cv.setTextColor(HUB_MUTED);
      cv.setCursor(16, h / 2 + 8);
      cv.print("RELATIVE HUMIDITY");
      snprintf(value, sizeof(value), sensorValid ? "%.0f%%" : "--%%", sensorValid ? indoorHumidity : 0.0f);
      cv.setTextSize(2);
      cv.setTextColor(HUB_CYAN);
      cv.setCursor(16, h / 2 + 19);
      cv.print(value);
      break;
    }

    case 8: { // Wave: three layered ribbons, one complete scene transfer.
      for (int16_t x = 0; x < w - 1; ++x) {
        const int16_t y1 = h / 2 + (int16_t)(sinf(x * 0.075f + phase) * 17.0f);
        const int16_t y2 = h / 2 + (int16_t)(sinf((x + 1) * 0.075f + phase) * 17.0f);
        const int16_t g1 = h / 2 + 8 + (int16_t)(sinf(x * 0.055f - phase * 0.8f) * 12.0f);
        const int16_t g2 = h / 2 + 8 + (int16_t)(sinf((x + 1) * 0.055f - phase * 0.8f) * 12.0f);
        const int16_t b1 = h / 2 - 8 + (int16_t)(sinf(x * 0.095f + phase * 0.7f) * 9.0f);
        const int16_t b2 = h / 2 - 8 + (int16_t)(sinf((x + 1) * 0.095f + phase * 0.7f) * 9.0f);
        cv.drawLine(x, y1, x + 1, y2, HUB_CYAN);
        cv.drawLine(x, g1, x + 1, g2, HUB_GREEN);
        cv.drawLine(x, b1, x + 1, b2, HUB_YELLOW);
      }
      break;
    }

    case 9: { // Pulse: nested rings and a breathing luminous core.
      const int16_t cx = w / 2, cy = h / 2;
      cv.drawCircle(cx, cy, 43, 0x2149);
      cv.drawCircle(cx, cy, 35, HUB_LINE);
      cv.drawCircle(cx, cy, 27, 0x31AE);
      cv.fillCircle(cx, cy, saverPulseRadius, HUB_CYAN);
      cv.fillCircle(cx, cy, max(3, (int)saverPulseRadius / 2), HUB_WHITE);
      saverPulseRadius = (uint8_t)(saverPulseRadius + saverPulseDirection * 2);
      if (saverPulseRadius >= 32) { saverPulseRadius = 32; saverPulseDirection = -1; }
      if (saverPulseRadius <= 10) { saverPulseRadius = 10; saverPulseDirection = 1; }
      break;
    }

    case 10: { // Aurora: layered flowing curtains on the exact same base as DNA.
      for (int16_t x = 0; x < w - 1; ++x) {
        const int16_t y1 = h / 2 - 8 + (int16_t)(sinf(x * 0.052f + phase * 1.2f) * 21.0f);
        const int16_t y2 = h / 2 - 8 + (int16_t)(sinf((x + 1) * 0.052f + phase * 1.2f) * 21.0f);
        const int16_t g1 = h / 2 + 5 + (int16_t)(sinf(x * 0.041f - phase * 0.75f) * 16.0f);
        const int16_t g2 = h / 2 + 5 + (int16_t)(sinf((x + 1) * 0.041f - phase * 0.75f) * 16.0f);
        const int16_t b1 = h / 2 - 17 + (int16_t)(sinf(x * 0.071f + phase * 0.62f) * 12.0f);
        const int16_t b2 = h / 2 - 17 + (int16_t)(sinf((x + 1) * 0.071f + phase * 0.62f) * 12.0f);
        cv.drawLine(x, y1, x + 1, y2, HUB_CYAN);
        cv.drawLine(x, g1, x + 1, g2, HUB_GREEN);
        cv.drawLine(x, b1, x + 1, b2, HUB_YELLOW);
      }
      for (uint8_t i = 0; i < 8; ++i) {
        const int16_t sx = (int16_t)((i * 19U + now / 17U) % (uint32_t)w);
        const int16_t sy = 8 + (int16_t)((i * 13U + now / 31U) % (uint32_t)max(1, h - 16));
        cv.drawPixel(sx, sy, (i & 1) ? HUB_WHITE : HUB_CYAN);
      }
      break;
    }

    case 11: { // DNA: refined double helix on the shared premium background.
      int16_t prevAx = 0, prevAy = 0, prevBx = 0, prevBy = 0;
      for (int16_t y = 7; y < h - 7; y += 2) {
        const float t = (float)y * 0.105f + phase;
        const float wave = sinf(t);
        const float depth = cosf(t);
        const int16_t spread = 19 + (int16_t)(depth * 4.0f);
        const int16_t ax = w / 2 + (int16_t)(wave * spread);
        const int16_t bx = w / 2 - (int16_t)(wave * spread);
        const uint16_t ca = depth > 0.0f ? HUB_CYAN : 0x05BF;
        const uint16_t cb = depth > 0.0f ? 0xF81F : HUB_CYAN;
        if (y > 7) {
          cv.drawWideLine(prevAx, prevAy, ax, y, 1.25f, ca);
          cv.drawWideLine(prevBx, prevBy, bx, y, 1.25f, cb);
        }
        if ((y % 12) < 2) {
          cv.drawLine(ax, y, bx, y, 0x7BEF);
          const uint8_t r = depth > 0.55f ? 3 : 2;
          cv.fillSmoothCircle(ax, y, r, ca);
          cv.fillSmoothCircle(bx, y, r, cb);
          cv.drawPixel(ax - 1, y - 1, HUB_WHITE);
          cv.drawPixel(bx - 1, y - 1, HUB_WHITE);
        }
        prevAx = ax; prevAy = y; prevBx = bx; prevBy = y;
      }
      for (uint8_t i = 0; i < 5; ++i) {
        const float p = phase * (0.72f + i * 0.035f) + i * 0.91f;
        const int16_t x = w / 2 + (int16_t)(sinf(p) * (28.0f + (i % 3) * 4.0f));
        const int16_t y = 8 + (int16_t)((((now / (22U + i * 3U)) + i * 19U) % (uint32_t)max(1, h - 16)));
        cv.fillCircle(x, y, 2, (i & 1) ? HUB_CYAN : HUB_YELLOW);
        cv.drawPixel(x - 1, y - 1, HUB_WHITE);
      }
      break;
    }

    case 12: { // Nebula Drift: layered orbital clouds and luminous star clusters.
      const int16_t cx = w / 2, cy = h / 2;
      for (uint8_t ring = 0; ring < 4; ++ring) {
        const float a = phase * (0.22f + ring * 0.07f) + ring * 1.35f;
        const int16_t rx = 15 + ring * 11;
        const int16_t ry = 9 + ring * 7;
        cv.drawEllipse(cx, cy, rx, ry, ring & 1 ? 0x3A0D : 0x2149);
        const int16_t x = cx + (int16_t)(cosf(a) * rx);
        const int16_t y = cy + (int16_t)(sinf(a) * ry);
        cv.fillSmoothCircle(x, y, 2, ring & 1 ? 0xB01F : HUB_CYAN);
      }
      for (uint8_t i = 0; i < 7; ++i) {
        const float a = phase * (0.3f + i * 0.025f) + i * 0.897f;
        const int16_t x = cx + (int16_t)(cosf(a) * (12 + i * 5));
        const int16_t y = cy + (int16_t)(sinf(a * 1.23f) * (8 + i * 3));
        cv.fillCircle(x, y, (i % 3) + 1, i & 1 ? 0xB01F : 0x7DFF);
      }
      break;
    }

    case 13: { // Matrix Rain: falling glyph columns.
      cv.setTextSize(1);
      for (uint8_t i = 0; i < 16; ++i) {
        const int16_t x = 4 + i * (w - 8) / 16;
        const int16_t y = saverMatrixY[i];
        cv.setTextColor(0x07E0);
        cv.setCursor(x, y);
        cv.write((uint8_t)('0' + ((i * 7U + now / 83U) % 10U)));
        if (y > 7) {
          cv.setTextColor(0xAFE0);
          cv.setCursor(x, y - 8);
          cv.write((uint8_t)('A' + ((i * 3U + now / 137U) % 26U)));
        }
        saverMatrixY[i] = (uint8_t)(y + saverMatrixSpeed[i]);
        if (saverMatrixY[i] > h + 8) {
          saverMatrixY[i] = random(0, 18);
          saverMatrixSpeed[i] = random(1, 4);
        }
      }
      break;
    }

    case 14: { // Comet Trails: bright heads with tapered segmented tails.
      for (uint8_t i = 0; i < 3; ++i) {
        const float p = fmodf(phase * (0.48f + i * 0.13f) + i * 0.37f, 1.0f);
        const int16_t x = (int16_t)(p * (w + 50)) - 25;
        const int16_t y = h / 4 + i * (h / 4) + (int16_t)(sinf(phase + i) * 7.0f);
        cv.drawWideLine(x - 28, y - 7, x, y, 3.0f, 0x2149);
        cv.drawWideLine(x - 20, y - 5, x, y, 1.7f, HUB_CYAN);
        cv.fillSmoothCircle(x, y, 3, HUB_WHITE);
        cv.fillSmoothCircle(x - 2, y, 2, HUB_CYAN);
      }
      break;
    }

    case 15: { // Fireflies: slow organic motion with a restrained halo.
      for (uint8_t i = 0; i < 9; ++i) {
        const float p = phase * (0.22f + (i % 4) * 0.045f) + i * 1.17f;
        const int16_t x = w / 2 + (int16_t)(sinf(p * 1.13f) * (18 + (i % 3) * 12));
        const int16_t y = h / 2 + (int16_t)(cosf(p * 0.83f) * (12 + (i % 4) * 8));
        const uint16_t c = (i % 3 == 0) ? HUB_YELLOW : (i % 3 == 1 ? 0xBFE0 : HUB_CYAN);
        cv.drawSpot(x, y, 5.0f + (sinf(p * 2.0f) + 1.0f) * 2.0f, 0x2149);
        cv.fillSmoothCircle(x, y, 2, c);
        cv.drawPixel(x - 1, y - 1, HUB_WHITE);
      }
      break;
    }

    case 16: { // Radar Sweep: range rings, bearing marks and rotating sweep.
      const int16_t cx = w / 2, cy = h / 2;
      const int16_t r = min(w, h) / 2 - 8;
      cv.drawCircle(cx, cy, r, 0x2A89);
      cv.drawCircle(cx, cy, r * 2 / 3, 0x2149);
      cv.drawCircle(cx, cy, r / 3, 0x2149);
      cv.drawFastHLine(cx - r, cy, r * 2, 0x2149);
      cv.drawFastVLine(cx, cy - r, r * 2, 0x2149);
      for (uint8_t i = 0; i < 8; ++i) {
        const float a = i * 0.785398f;
        cv.drawLine(cx + (int16_t)(cosf(a) * (r - 3)), cy + (int16_t)(sinf(a) * (r - 3)),
                    cx + (int16_t)(cosf(a) * r), cy + (int16_t)(sinf(a) * r), 0x3A0D);
      }
      const float a = fmodf(phase * 0.72f, 6.283185f);
      const float aDeg = a * 57.2957795f;
      const int16_t sx = cx + (int16_t)(cosf(a) * r);
      const int16_t sy = cy + (int16_t)(sinf(a) * r);
      cv.drawArc(cx, cy, r, r - 3, aDeg - 28.0f, aDeg, 0x3A0D);
      cv.drawLine(cx, cy, sx, sy, HUB_CYAN);
      cv.drawWideLine(cx, cy, cx + (int16_t)(cosf(a) * r), cy + (int16_t)(sinf(a) * r), 1.1f, HUB_CYAN);
      for (uint8_t i = 0; i < 4; ++i) {
        const int16_t px = cx + (int16_t)(cosf(i * 1.71f + 0.5f) * (8 + i * 8));
        const int16_t py = cy + (int16_t)(sinf(i * 1.71f + 0.5f) * (8 + i * 8));
        cv.fillCircle(px, py, 2, HUB_GREEN);
      }
      break;
    }

    case 17: { // Planetary Orbit: elegant orbital paths and moving planets.
      const int16_t cx = w / 2, cy = h / 2;
      cv.drawCircle(cx, cy, 8, 0xFC60);
      cv.fillCircle(cx, cy, 5, HUB_YELLOW);
      for (uint8_t i = 0; i < 4; ++i) {
        const int16_t rx = 18 + i * 11;
        const int16_t ry = 9 + i * 6;
        cv.drawEllipse(cx, cy, rx, ry, 0x2149);
        const float a = phase * (0.36f + i * 0.11f) + i * 1.2f;
        const int16_t px = cx + (int16_t)(cosf(a) * rx);
        const int16_t py = cy + (int16_t)(sinf(a) * ry);
        cv.fillCircle(px, py, 2 + (i % 2), i & 1 ? HUB_CYAN : 0xF81F);
        cv.drawPixel(px - 1, py - 1, HUB_WHITE);
      }
      break;
    }

    case 18: { // Equalizer: mirrored audio bars with a smooth travelling envelope.
      const uint8_t bars = 12;
      const int16_t gap = 2;
      const int16_t bw = max(3, (w - 18 - (bars - 1) * gap) / bars);
      const int16_t total = bars * bw + (bars - 1) * gap;
      const int16_t left = (w - total) / 2;
      const int16_t base = h / 2 + 24;
      for (uint8_t i = 0; i < bars; ++i) {
        const float v = (sinf(phase * 2.1f + i * 0.62f) + cosf(phase * 1.37f - i * 0.39f) + 2.0f) * 0.25f;
        const int16_t bh = 7 + (int16_t)(v * (h / 2 - 13));
        const uint16_t c = i < 4 ? HUB_CYAN : (i < 8 ? 0x7DFF : 0xF81F);
        cv.fillRoundRect(left + i * (bw + gap), base - bh, bw, bh, 2, c);
        cv.fillRoundRect(left + i * (bw + gap), base + 3, bw, bh / 3, 2, 0x2149);
      }
      cv.drawFastHLine(8, base + 2, w - 16, 0x3A0D);
      break;
    }

    case 19: { // Prism Tunnel: nested perspective frames flowing toward a vanishing point.
      const int16_t cx = w / 2, cy = h / 2;
      for (uint8_t i = 0; i < 7; ++i) {
        const uint8_t phaseStep = (uint8_t)((now / 75U + i * 31U) % 100U);
        const int16_t ww = 12 + phaseStep * (w - 20) / 100;
        const int16_t hh = 10 + phaseStep * (h - 18) / 100;
        const uint16_t c = i % 3 == 0 ? HUB_CYAN : (i % 3 == 1 ? 0xF81F : 0x7DFF);
        cv.drawRoundRect(cx - ww / 2, cy - hh / 2, ww, hh, 4, c);
      }
      break;
    }

    case 20: { // Fractal Bloom: rotating petals built from concentric luminous geometry.
      const int16_t cx = w / 2, cy = h / 2;
      for (uint8_t i = 0; i < 12; ++i) {
        const float a = phase * 0.42f + i * 0.523599f;
        const int16_t x = cx + (int16_t)(cosf(a) * 25.0f);
        const int16_t y = cy + (int16_t)(sinf(a) * 25.0f);
        const int16_t cpx = cx + (int16_t)(cosf(a + 0.35f) * 40.0f);
        const int16_t cpy = cy + (int16_t)(sinf(a + 0.35f) * 40.0f);
        cv.drawBezier(cx, cy, cpx, cpy, x, y, i & 1 ? HUB_CYAN : 0xF81F);
        cv.drawBezier(x, y, cpx, cpy, cx, cy, i & 1 ? 0x7DFF : 0xB01F);
        cv.drawCircle(x, y, 5, i & 1 ? HUB_CYAN : 0xF81F);
        cv.fillSmoothCircle(x, y, 2, i & 1 ? 0x7DFF : HUB_WHITE);
      }
      cv.drawCircle(cx, cy, 13 + (int16_t)((sinf(phase) + 1.0f) * 5.0f), HUB_YELLOW);
      cv.fillCircle(cx, cy, 4, HUB_WHITE);
      break;
    }

    case 21: { // Star Warp: depth-scaled stars rushing from the centre.
      const int16_t cx = w / 2, cy = h / 2;
      for (uint8_t i = 0; i < 34; ++i) {
        const float seed = i * 2.399963f;
        const float z = fmodf((i * 17.0f + now * 0.075f), 100.0f) + 1.0f;
        const float scale = 1.0f + z / 22.0f;
        const int16_t x = cx + (int16_t)(cosf(seed) * scale * (w * 0.42f));
        const int16_t y = cy + (int16_t)(sinf(seed) * scale * (h * 0.42f));
        if (x >= 1 && x < w - 1 && y >= 1 && y < h - 1) {
          const int16_t tail = max(1, (int)(scale * 2.0f));
          cv.drawLine(cx + (x - cx) * 2 / 3, cy + (y - cy) * 2 / 3, x, y,
                      z > 65.0f ? HUB_WHITE : (z > 30.0f ? HUB_CYAN : 0x2149));
          if (tail > 2) cv.drawPixel(x, y, HUB_WHITE);
        }
      }
      break;
    }

    default:
      presentFrame = false;
      break;
  }

  if (presentFrame) {
    // pushSprite submits the native sprite to the display. Keep its RGB565
    // buffer unchanged until the destination DMA engine has finished reading it.
    hubTFT.startWrite();
    cv.pushSprite(0, 25);
    hubTFT.waitDMA();
    hubTFT.endWrite();
  }

  const uint32_t drawUs = micros() - frameStartedUs;
  if (presentFrame) {
    ++diagFrames;
    diagTotalUs += drawUs;
    if (drawUs > diagMaxUs) diagMaxUs = drawUs;
  }
  const uint32_t diagNow = millis();
  if (diagWindowAt == 0) diagWindowAt = diagNow;
  if ((uint32_t)(diagNow - diagWindowAt) >= 5000UL) {
    const uint32_t elapsedMs = diagNow - diagWindowAt;
    const uint32_t fps = elapsedMs ? (diagFrames * 1000UL) / elapsedMs : 0;
    LOG_S("SAVER id=%u fps=%lu draw_avg=%lu us draw_max=%lu us heap=%u",
          hubSaverId, (unsigned long)fps,
          (unsigned long)(diagFrames ? diagTotalUs / diagFrames : 0),
          (unsigned long)diagMaxUs, ESP.getFreeHeap());
    diagWindowAt = diagNow;
    diagFrames = 0;
    diagTotalUs = 0;
    diagMaxUs = 0;
  }
}

void updateSmartHub() {
  const uint32_t now = millis();
  if (now - lastSensorReadAt >= SENSOR_INTERVAL_MS) {
    lastSensorReadAt = now;
    const float h = hubDHT.readHumidity();
    const float t = hubDHT.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      const bool firstSample = !sensorValid;
      // DHT22 reports tenths. Compare rounded tenths to avoid float noise while
      // still refreshing the visible value on every real 0.1-unit change.
      const bool visibleChange = firstSample || isnan(indoorTempC) || isnan(indoorHumidity) ||
        changedAtTenth(t, indoorTempC) || changedAtTenth(h, indoorHumidity);
      indoorHumidity = h; indoorTempC = t; sensorValid = true;
      if (visibleChange) {
        hubSensorDirty = true;
        publishMqttState(); // Push updated climate readings to retained MQTT sensors.
      }
      if (matterStarted && climateEndpointReady) {
        const bool tempChanged = isnan(lastMatterTemperature) || changedAtTenth(indoorTempC, lastMatterTemperature);
        const bool humidityChanged = isnan(lastMatterHumidity) || changedAtTenth(indoorHumidity, lastMatterHumidity);
        if (tempChanged || humidityChanged) {
          const bool climateSet = updateClimateMatterValues(
            tempChanged ? indoorTempC : NAN,
            humidityChanged ? indoorHumidity : NAN);
          if (climateSet) {
            if (tempChanged) lastMatterTemperature = indoorTempC;
            if (humidityChanged) lastMatterHumidity = indoorHumidity;
          }
        }
      }
    } else LOG_ERR("AM2302 read failed; keeping last valid values");
  }

  updateHubClock();
  static String lastRenderedTime = "";
  static String lastRenderedDate = "";
  static bool lastWifiConnected = false;
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (timeText != lastRenderedTime || dateText != lastRenderedDate) {
    lastRenderedTime = timeText; lastRenderedDate = dateText; hubClockDirty = true;
  }
  if (wifiConnected != lastWifiConnected) {
    lastWifiConnected = wifiConnected;
    hubDisplayDirty = true; // Header connectivity indicator changed.
    hubForceFullRedraw = true;
  }

  // Extended status pages also follow Matter commissioning and MQTT link state.
  static bool lastUiMatter = false, lastUiFabric = false, lastUiMqtt = false;
  const bool uiMatter = matterStarted;
  const bool uiFabric = Matter.isDeviceCommissioned();
  const bool uiMqtt = mqttClient.connected();
  if (uiMatter != lastUiMatter || uiFabric != lastUiFabric || uiMqtt != lastUiMqtt) {
    lastUiMatter = uiMatter; lastUiFabric = uiFabric; lastUiMqtt = uiMqtt;
    if (hubLayoutId >= 14 && hubLayoutId < HUB_LAYOUT_COUNT) hubSensorDirty = true;
  }
  static int lastUiRssi = -128;
  const int uiRssi = wifiConnected ? WiFi.RSSI() : -127;
  if (abs(uiRssi - lastUiRssi) >= 5) {
    lastUiRssi = uiRssi;
    if (hubLayoutId == 23 || hubLayoutId == 37 || hubLayoutId == 41) hubSensorDirty = true;
  }
  static uint32_t lastDiagnosticsUiAt = 0;
  if (hubLayoutId == 41 && (uint32_t)(now - lastDiagnosticsUiAt) >= 1000UL) {
    lastDiagnosticsUiAt = now;
    hubSensorDirty = true;
  }

  if (!hubDisplayEnabled) {
    hubSaverActive = false;
  } else if (hubSaverId > 0 && hubSaverIdleSeconds > 0 && !hubSaverActive && (uint32_t)(now - hubLastActivityAt) >= (uint32_t)hubSaverIdleSeconds * 1000UL) {
    hubSaverActive = true;
    hubSaverLastFrameAt = now;
    hubClockDirty = hubSensorDirty = hubWeatherDirty = hubLightDirty = false;
    initHubScreenSaver();
    drawHubScreenSaverFrame();
    recordTftFrame();
    lastTftDrawAt = now;
    return;
  }

  if (hubSaverActive) {
    if (hubDisplayDirty) {
      hubSaverActive = false;
      hubForceFullRedraw = true;
      hubDisplayDirty = false;
      hubClockDirty = hubSensorDirty = hubWeatherDirty = hubLightDirty = false;
      lastTftDrawAt = now;
      drawSmartHub();
      return;
    }
    const uint32_t saverInterval = (hubSaverId == 11 || hubSaverId == 12 || hubSaverId == 20) ? HUB_SAVER_COMPLEX_FRAME_MS : HUB_SAVER_FRAME_MS;
    if (now - hubSaverLastFrameAt >= saverInterval) {
      hubSaverLastFrameAt = now;
      drawHubScreenSaverFrame();
      recordTftFrame();
    }
    return;
  }

  // Extended layouts 14..41 use the shared small RGB565 value canvas for live patches.
  // Static cards remain untouched; only changed numeric/text fields are transmitted.
  if (hubLayoutId >= 14 && hubLayoutId < HUB_LAYOUT_COUNT &&
      (hubClockDirty || hubSensorDirty || hubWeatherDirty || hubLightDirty)) {
    hubClockDirty = hubSensorDirty = hubWeatherDirty = hubLightDirty = false;
    const uint32_t patchStartedAt = micros();
    drawExtendedLayoutValues();
    lastTftDrawAt = now;
    LOG_S("TFT extended value patches completed in %lu ms", (unsigned long)((micros() - patchStartedAt) / 1000UL));
    recordTftFrame();
    return;
  }

  // Prototype layouts 10..13 are updated through checked value patches below;
  // telemetry changes must not trigger a complete screen repaint.
  if (hubDisplayDirty && now - lastTftDrawAt >= 33) {
    hubDisplayDirty = false; hubClockDirty = false; hubSensorDirty = false; hubWeatherDirty = false; hubLightDirty = false;
    lastTftDrawAt = now;
    const uint32_t renderStartedAt = micros();
    drawSmartHub();
    LOG_S("TFT full redraw completed in %lu ms", (unsigned long)((micros() - renderStartedAt) / 1000UL));
    recordTftFrame();
    return;
  }
  // Three-stage TFT benchmark: tiny patch, 40x40 area, full-screen transfer.
  if (hubDisplayEnabled && hubLayoutId == 7 && !hubDisplayDirty) {
    if (hubTestPhaseAt == 0) { hubTestPhaseAt=now; hubTestFpsWindowAt=now; hubTestLastFrameAt=0; hubTestPreviousX=-1; }
    if ((uint32_t)(now-hubTestPhaseAt)>=5000UL) {
      hubTestPhase=(hubTestPhase+1)%3; hubTestPhaseAt=now; hubTestFpsWindowAt=now;
      hubTestFrameCount=0; hubTestDrawUsAvg=hubTestDrawUsMax=hubTestDrawUsLast=0; hubTestPreviousX=-1; hubTFT.fillScreen(HUB_BG);
    }
    const uint32_t frameInterval=max(1UL,1000UL/max((uint8_t)1,hubTestFpsTarget));
    if ((uint32_t)(now-hubTestLastFrameAt)>=frameInterval) {
      hubTestLastFrameAt=now; const uint32_t started=micros();
      if (hubTestPhase==0) {
        if(hubTestPreviousX>=0)hubTFT.fillRect(hubTestPreviousX,119,5,10,HUB_BG);
        hubTestPreviousX=12+(int16_t)(hubTestFrameNumber%max(1,(int)hubTFT.width()-29));
        hubTFT.fillRect(hubTestPreviousX,119,5,10,HUB_CYAN);
      } else if(hubTestPhase==1) {
        if(hubTestPreviousX>=0)hubTFT.fillRect(hubTestPreviousX,95,40,40,HUB_BG);
        hubTestPreviousX=4+(int16_t)(hubTestFrameNumber%max(1,(int)hubTFT.width()-47));
        hubTFT.fillRect(hubTestPreviousX,95,40,40,HUB_CYAN);
      } else { hubTFT.fillScreen(hubTestFullToggle?HUB_BG:HUB_PANEL); hubTestFullToggle=!hubTestFullToggle; }
      hubTestDrawUsLast=micros()-started;
      hubTestDrawUsAvg=hubTestFrameCount?(hubTestDrawUsAvg*7UL+hubTestDrawUsLast)/8UL:hubTestDrawUsLast;
      if(hubTestDrawUsLast>hubTestDrawUsMax)hubTestDrawUsMax=hubTestDrawUsLast;
      hubTestFrameNumber++;hubTestFrameCount++;
    }
    if((uint32_t)(now-hubTestFpsWindowAt)>=1000UL) {
      const uint32_t elapsed=now-hubTestFpsWindowAt;
      hubTestMeasuredFps=(uint16_t)((hubTestFrameCount*1000UL)/max(1UL,elapsed));
      hubTestFpsWindowAt=now;hubTestFrameCount=0;
      const uint16_t bg=hubTestPhase==2?HUB_PANEL:HUB_BG;
      hubTFT.fillRect(5,5,hubTFT.width()-10,35,bg);hubTFT.setTextSize(1);hubTFT.setTextColor(HUB_CYAN,bg);hubTFT.setCursor(8,8);
      hubTFT.print(hubTestPhase==0?"PATCH 5x10":(hubTestPhase==1?"AREA 40x40":"FULL 128x160"));
      hubTFT.setTextSize(2);hubTFT.setTextColor(HUB_WHITE,bg);hubTFT.setCursor(8,20);hubTFT.print(hubTestMeasuredFps);hubTFT.print(" FPS");
      LOG_S("TFT TEST phase=%u target=%u actual=%u FPS draw=%lu us avg=%lu max=%lu heap=%u",hubTestPhase,hubTestFpsTarget,hubTestMeasuredFps,(unsigned long)hubTestDrawUsLast,(unsigned long)hubTestDrawUsAvg,(unsigned long)hubTestDrawUsMax,ESP.getFreeHeap());
    }
    return;
  }
  // Do not paint telemetry over the black screen when the display endpoint is off.
  if (!hubDisplayEnabled) { hubClockDirty = false; hubSensorDirty = false; hubWeatherDirty = false; hubLightDirty = false; return; }
  if (now - lastTftDrawAt < 33) return;
  if (hubLayoutId >= 10 && hubLayoutId <= 13 &&
      (hubClockDirty || hubSensorDirty || hubWeatherDirty || hubLightDirty)) {
    hubClockDirty = hubSensorDirty = hubWeatherDirty = hubLightDirty = false;
    const uint32_t patchStartedAt = micros();
    drawModernLayoutPartials();
    lastTftDrawAt = now;
    LOG_S("TFT checked value patches completed in %lu ms", (unsigned long)((micros() - patchStartedAt) / 1000UL));
    recordTftFrame();
    return;
  }
  bool drew = false;
  const uint32_t partialRenderStartedAt = micros();
  if (hubClockDirty) {
    hubClockDirty = false;
    if (hubLayoutId == 8 || hubLayoutId == 9) {
      hubTFT.fillRect(6,22,116,22,HUB_BG); hubTFT.setTextSize(2);
      hubTFT.setTextColor(hubLayoutId==9?HUB_GREEN:HUB_WHITE,HUB_BG);
      hubTFT.setCursor(8,25); hubTFT.print(timeText);
    }
    else if (hubLayoutId == 0) drawDashboardClockPartial();
    else if (hubLayoutId == 1) drawAnalogClockPartial();
    drew = true;
  }
  if (hubSensorDirty) { hubSensorDirty = false; drawLayoutSensorPartial(); drew = true; }
  if (hubWeatherDirty) { hubWeatherDirty = false; drawLayoutWeatherPartial(); drew = true; }
  if (hubLightDirty) { hubLightDirty = false; drawLayoutLightPartial(); drew = true; }
  if (drew) {
    lastTftDrawAt = now;
    LOG_S("TFT partial update completed in %lu ms", (unsigned long)((micros() - partialRenderStartedAt) / 1000UL));
    recordTftFrame();
  }
}

struct WeatherSnapshot {
  float temperature=NAN,humidity=NAN,feelsLike=NAN,pressure=NAN,windSpeed=NAN,windDirection=NAN;
  float cloudCover=NAN,uvIndex=NAN,precipitation=NAN;
  int precipitationProbability=-1,weatherCode=-1;
};

static bool applyWeatherSnapshot(const WeatherSnapshot &v,const char *provider){
  if(isnan(v.temperature))return false;
  const bool firstWeather=!weatherValid;
  const bool visibleChange=firstWeather||isnan(outdoorTempC)||isnan(outdoorHumidity)||isnan(outdoorPrecipMm)||
    changedAtTenth(v.temperature,outdoorTempC)||changedAtTenth(v.humidity,outdoorHumidity)||
    changedAtTenth(v.feelsLike,outdoorFeelsLikeC)||changedAtTenth(v.pressure,outdoorPressureHpa)||
    changedAtTenth(v.windSpeed,outdoorWindSpeedKmh)||changedAtTenth(v.windDirection,outdoorWindDirection)||
    changedAtTenth(v.cloudCover,outdoorCloudCover)||changedAtTenth(v.uvIndex,outdoorUvIndex)||
    changedAtTenth(v.precipitation,outdoorPrecipMm)||v.precipitationProbability!=outdoorPrecipProbability||v.weatherCode!=outdoorWeatherCode;
  portENTER_CRITICAL(&hubDataMux);
  outdoorTempC=v.temperature;outdoorHumidity=v.humidity;outdoorPrecipMm=v.precipitation;
  outdoorFeelsLikeC=v.feelsLike;outdoorPressureHpa=v.pressure;outdoorWindSpeedKmh=v.windSpeed;
  outdoorWindDirection=v.windDirection;outdoorCloudCover=v.cloudCover;outdoorUvIndex=v.uvIndex;
  outdoorPrecipProbability=v.precipitationProbability;outdoorWeatherCode=v.weatherCode;weatherValid=true;
  portEXIT_CRITICAL(&hubDataMux);
  setWeatherText("OK", provider);
  if(visibleChange){hubWeatherDirty=true;publishMqttState();}
  LOG_B("Weather OK via %s: %.1f C feels %.1f C, %.0f hPa, wind %.1f km/h, rain %.1f mm",
    provider,v.temperature,v.feelsLike,v.pressure,v.windSpeed,v.precipitation);
  return true;
}

static bool weatherResolveHost(const char *host, IPAddress &resolved) {
  if (WiFi.status() != WL_CONNECTED) {
    LOG_W("Weather DNS skipped: Wi-Fi disconnected");
    return false;
  }
  const uint32_t started = millis();
  const int ok = WiFi.hostByName(host, resolved);
  const uint32_t elapsed = millis() - started;
  if (ok == 1) {
    LOG_B("Weather DNS %s -> %s (%lu ms, RSSI=%d dBm)",
          host, resolved.toString().c_str(), (unsigned long)elapsed, WiFi.RSSI());
    return true;
  }
  LOG_ERR("Weather DNS failed for %s (%lu ms, RSSI=%d dBm)",
          host, (unsigned long)elapsed, WiFi.RSSI());
  return false;
}

static void configureWeatherHttp(WiFiClientSecure &client, HTTPClient &https, const char *agent) {
  client.setInsecure();                 // TLS encryption remains enabled; certificate verification is intentionally disabled.
  client.setHandshakeTimeout(6);
  client.setTimeout(7000);
  https.setTimeout(7000);
  https.setReuse(false);
  https.setUserAgent(agent);
}

static bool fetchOpenMeteo(WeatherSnapshot &v) {
  const uint32_t now = millis();
  if (weatherOpenMeteoLastAttemptAt && (uint32_t)(now - weatherOpenMeteoLastAttemptAt) < 60000UL) {
    LOG_W("Open-Meteo retry cooldown active");
    return false;
  }
  weatherOpenMeteoLastAttemptAt = now;
  const char *host = "api.open-meteo.com";
  IPAddress resolved;
  if (!weatherResolveHost(host, resolved)) return false;

  WiFiClientSecure client;
  HTTPClient https;
  configureWeatherHttp(client, https, "GoodWinSmartHub/3.11.1 (+https://github.com/Serg434/Arduino)");
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(WEATHER_LAT, 6) +
    "&longitude=" + String(WEATHER_LON, 6) +
    "&current=temperature_2m,relative_humidity_2m,apparent_temperature,surface_pressure,wind_speed_10m,wind_direction_10m,cloud_cover,uv_index,precipitation,weather_code" +
    "&hourly=precipitation_probability,precipitation&forecast_days=1&timezone=Asia%2FYekaterinburg";
  if (!https.begin(client, url)) {
    LOG_ERR("Open-Meteo HTTPS begin failed");
    return false;
  }
  const uint32_t requestStarted = millis();
  const int code = https.GET();
  if (code != HTTP_CODE_OK) {
    LOG_ERR("Open-Meteo HTTP/TLS failed: code=%d (%s), elapsed=%lu ms, DNS=%s, RSSI=%d dBm",
            code, https.errorToString(code).c_str(), (unsigned long)(millis() - requestStarted),
            resolved.toString().c_str(), WiFi.RSSI());
    https.end();
    return false;
  }

  StaticJsonDocument<1024> filter;
  filter["current"]["temperature_2m"] = true;
  filter["current"]["relative_humidity_2m"] = true;
  filter["current"]["apparent_temperature"] = true;
  filter["current"]["surface_pressure"] = true;
  filter["current"]["wind_speed_10m"] = true;
  filter["current"]["wind_direction_10m"] = true;
  filter["current"]["cloud_cover"] = true;
  filter["current"]["uv_index"] = true;
  filter["current"]["precipitation"] = true;
  filter["current"]["weather_code"] = true;
  filter["current"]["time"] = true;
  filter["hourly"]["time"][0] = true;
  filter["hourly"]["precipitation_probability"][0] = true;
  filter["hourly"]["precipitation"][0] = true;
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, https.getStream(), DeserializationOption::Filter(filter));
  https.end();
  if (err) {
    LOG_ERR("Open-Meteo JSON parse failed: %s, capacity=%u", err.c_str(), (unsigned)doc.capacity());
    return false;
  }
  if (doc.overflowed()) {
    LOG_ERR("Open-Meteo JSON document overflow");
    return false;
  }

  JsonObject cur = doc["current"];
  v.temperature = cur["temperature_2m"] | NAN;
  v.humidity = cur["relative_humidity_2m"] | NAN;
  v.feelsLike = cur["apparent_temperature"] | NAN;
  v.pressure = cur["surface_pressure"] | NAN;
  v.windSpeed = cur["wind_speed_10m"] | NAN;
  v.windDirection = cur["wind_direction_10m"] | NAN;
  v.cloudCover = cur["cloud_cover"] | NAN;
  v.uvIndex = cur["uv_index"] | NAN;
  v.precipitation = cur["precipitation"] | 0.0f;
  v.weatherCode = cur["weather_code"] | -1;
  const char *currentTime = cur["time"] | "";
  JsonArray times = doc["hourly"]["time"].as<JsonArray>();
  JsonArray probs = doc["hourly"]["precipitation_probability"].as<JsonArray>();
  JsonArray rain = doc["hourly"]["precipitation"].as<JsonArray>();
  for (size_t i = 0; i < times.size(); ++i) {
    const char *candidate = times[i] | "";
    if (strlen(currentTime) >= 13 && strncmp(candidate, currentTime, 13) == 0) {
      if (i < probs.size()) v.precipitationProbability = probs[i] | -1;
      if (i < rain.size()) v.precipitation = rain[i] | v.precipitation;
      break;
    }
  }
  LOG_B("Open-Meteo response parsed in %lu ms", (unsigned long)(millis() - requestStarted));
  return !isnan(v.temperature);
}

static bool fetchWttr(WeatherSnapshot &v) {
  const uint32_t now = millis();
  if (weatherWttrLastAttemptAt && (uint32_t)(now - weatherWttrLastAttemptAt) < WEATHER_WTTR_COOLDOWN_MS) {
    LOG_W("wttr.in cooldown active; skipping repeated request");
    return false;
  }
  weatherWttrLastAttemptAt = now;
  const char *host = "wttr.in";
  IPAddress resolved;
  if (!weatherResolveHost(host, resolved)) return false;

  WiFiClientSecure client;
  HTTPClient https;
  configureWeatherHttp(client, https, "GoodWinSmartHub/3.11.1 (+https://github.com/Serg434/Arduino)");
  String url = "https://wttr.in/" + String(WEATHER_LAT, 4) + "," + String(WEATHER_LON, 4) + "?format=j1";
  if (!https.begin(client, url)) {
    LOG_ERR("wttr.in HTTPS begin failed");
    return false;
  }
  const uint32_t requestStarted = millis();
  const int code = https.GET();
  if (code != HTTP_CODE_OK) {
    LOG_ERR("wttr.in HTTP/TLS failed: code=%d (%s), elapsed=%lu ms, DNS=%s, RSSI=%d dBm",
            code, https.errorToString(code).c_str(), (unsigned long)(millis() - requestStarted),
            resolved.toString().c_str(), WiFi.RSSI());
    https.end();
    return false;
  }

  StaticJsonDocument<768> filter;
  filter["current_condition"][0]["temp_C"] = true;
  filter["current_condition"][0]["FeelsLikeC"] = true;
  filter["current_condition"][0]["humidity"] = true;
  filter["current_condition"][0]["pressure"] = true;
  filter["current_condition"][0]["windspeedKmph"] = true;
  filter["current_condition"][0]["winddirDegree"] = true;
  filter["current_condition"][0]["cloudcover"] = true;
  filter["current_condition"][0]["uvIndex"] = true;
  filter["current_condition"][0]["precipMM"] = true;
  filter["current_condition"][0]["weatherCode"] = true;
  DynamicJsonDocument doc(1536);
  DeserializationError err = deserializeJson(doc, https.getStream(), DeserializationOption::Filter(filter));
  https.end();
  if (err) {
    LOG_ERR("wttr.in JSON parse failed: %s, capacity=%u", err.c_str(), (unsigned)doc.capacity());
    return false;
  }
  JsonObject cur = doc["current_condition"][0];
  v.temperature = cur["temp_C"] | NAN;
  v.feelsLike = cur["FeelsLikeC"] | NAN;
  v.humidity = cur["humidity"] | NAN;
  v.pressure = cur["pressure"] | NAN;
  v.windSpeed = cur["windspeedKmph"] | NAN;
  v.windDirection = cur["winddirDegree"] | NAN;
  v.cloudCover = cur["cloudcover"] | NAN;
  v.uvIndex = cur["uvIndex"] | NAN;
  v.precipitation = cur["precipMM"] | 0.0f;
  v.precipitationProbability = -1;
  v.weatherCode = -1;
  LOG_B("wttr.in response parsed in %lu ms", (unsigned long)(millis() - requestStarted));
  return !isnan(v.temperature);
}

static bool fetchMetNo(WeatherSnapshot &v) {
  const uint32_t now = millis();
  if (weatherMetNoLastAttemptAt && (uint32_t)(now - weatherMetNoLastAttemptAt) < WEATHER_METNO_COOLDOWN_MS) {
    LOG_W("MET Norway cooldown active (30 min policy/cache interval); skipping request");
    return false;
  }
  weatherMetNoLastAttemptAt = now;
  const char *host = "api.met.no";
  IPAddress resolved;
  if (!weatherResolveHost(host, resolved)) return false;

  WiFiClientSecure client;
  HTTPClient https;
  // MET Norway requires a unique identifying User-Agent. The project URL is the identifier.
  configureWeatherHttp(client, https, "GoodWinSmartHub/3.11.1 (https://github.com/Serg434/Arduino)");
  const char *responseHeaders[] = { "X-ErrorClass" };
  https.collectHeaders(responseHeaders, 1);
  String url = "https://api.met.no/weatherapi/locationforecast/2.0/compact?lat=" +
    String(WEATHER_LAT, 4) + "&lon=" + String(WEATHER_LON, 4);
  if (!https.begin(client, url)) {
    LOG_ERR("MET Norway HTTPS begin failed");
    return false;
  }
  const uint32_t requestStarted = millis();
  const int code = https.GET();
  if (code != HTTP_CODE_OK) {
    const String errorClass = https.header("X-ErrorClass");
    LOG_ERR("MET Norway HTTP/TLS failed: code=%d (%s), class=%s, elapsed=%lu ms, DNS=%s, RSSI=%d dBm",
            code, https.errorToString(code).c_str(), errorClass.c_str(),
            (unsigned long)(millis() - requestStarted), resolved.toString().c_str(), WiFi.RSSI());
    https.end();
    return false;
  }

  StaticJsonDocument<1024> filter;
  JsonObject details = filter["properties"]["timeseries"][0]["data"]["instant"]["details"].to<JsonObject>();
  details["air_temperature"] = true;
  details["relative_humidity"] = true;
  details["air_pressure_at_sea_level"] = true;
  details["wind_speed"] = true;
  details["wind_from_direction"] = true;
  details["cloud_area_fraction"] = true;
  details["ultraviolet_index_clear_sky"] = true;
  filter["properties"]["timeseries"][0]["data"]["next_1_hours"]["details"]["precipitation_amount"] = true;
  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, https.getStream(), DeserializationOption::Filter(filter));
  https.end();
  if (err) {
    LOG_ERR("MET Norway JSON parse failed: %s, capacity=%u", err.c_str(), (unsigned)doc.capacity());
    return false;
  }
  JsonObject point = doc["properties"]["timeseries"][0]["data"];
  JsonObject d = point["instant"]["details"];
  v.temperature = d["air_temperature"] | NAN;
  v.humidity = d["relative_humidity"] | NAN;
  v.pressure = d["air_pressure_at_sea_level"] | NAN;
  v.windSpeed = d["wind_speed"] | NAN;
  v.windDirection = d["wind_from_direction"] | NAN;
  v.cloudCover = d["cloud_area_fraction"] | NAN;
  v.uvIndex = d["ultraviolet_index_clear_sky"] | NAN;
  v.precipitation = point["next_1_hours"]["details"]["precipitation_amount"] | 0.0f;
  v.feelsLike = NAN;
  v.precipitationProbability = -1;
  v.weatherCode = -1;
  LOG_B("MET Norway response parsed in %lu ms", (unsigned long)(millis() - requestStarted));
  return !isnan(v.temperature);
}

void weatherTask(void *parameter){
  (void)parameter;
  for(;;){
    uint32_t nextDelay=WEATHER_INTERVAL_MS;
    if(WiFi.status()!=WL_CONNECTED){setWeatherText("Нет Wi-Fi", "—");nextDelay=60000UL;}
    else{
      const uint8_t mode=weatherSourceMode;
      bool ok=false;
      WeatherSnapshot snapshot;
      if(mode==0){
        setWeatherText("Запрос Open-Meteo");
        if(fetchOpenMeteo(snapshot))ok=applyWeatherSnapshot(snapshot,"Open-Meteo");
        if(!ok){setWeatherText("Резерв: wttr.in");LOG_W("Open-Meteo unavailable; trying wttr.in");if(fetchWttr(snapshot))ok=applyWeatherSnapshot(snapshot,"wttr.in");}
        if(!ok){setWeatherText("Резерв: MET Norway");LOG_W("wttr.in unavailable; trying MET Norway");if(fetchMetNo(snapshot))ok=applyWeatherSnapshot(snapshot,"MET Norway");}
      }else if(mode==1){
        setWeatherText("Запрос Open-Meteo");
        if(fetchOpenMeteo(snapshot))ok=applyWeatherSnapshot(snapshot,"Open-Meteo");
      }else if(mode==2){
        setWeatherText("Запрос wttr.in");
        if(fetchWttr(snapshot))ok=applyWeatherSnapshot(snapshot,"wttr.in");
      }else if(mode==3){
        setWeatherText("Запрос MET Norway");
        if(fetchMetNo(snapshot))ok=applyWeatherSnapshot(snapshot,"MET Norway");
      }
      if(ok)nextDelay=WEATHER_INTERVAL_MS;
      else{setWeatherText("Нет данных · источник недоступен");nextDelay=60000UL;LOG_ERR("Weather request failed; mode=%u",mode);}
    }
    // Web request interrupts the wait and starts the next refresh immediately.
    ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(nextDelay));
  }
}

void startWeatherTask() {
  if (weatherTaskHandle) return;
  BaseType_t created = xTaskCreatePinnedToCore(weatherTask, "hubWeather", 8192, nullptr, 1, &weatherTaskHandle, 0);
  if (created != pdPASS) { weatherTaskHandle = nullptr; setWeatherText("Ошибка задачи погоды"); LOG_ERR("Cannot create weather task"); }
}

void handleHubSaverStart() {
  if (!hubDisplayEnabled) { server.send(409, "application/json; charset=utf-8", "{\"ok\":false,\"message\":\"Дисплей выключен\"}"); return; }
  if (hubSaverId == 0) { server.send(409, "application/json; charset=utf-8", "{\"ok\":false,\"message\":\"Сначала выберите заставку\"}"); return; }
  hubLastActivityAt = millis(); hubSaverActive = true; hubDisplayDirty = false;
  hubClockDirty = hubSensorDirty = hubWeatherDirty = hubLightDirty = false;
  initHubScreenSaver(); drawHubScreenSaverFrame();
  hubSaverLastFrameAt = millis(); lastTftDrawAt = hubSaverLastFrameAt; recordTftFrame();
  server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleHubConfig() {
  if (server.method() == HTTP_POST) {
    hubMarkActivity();
    if (server.hasArg("layout")) {
      int v = server.arg("layout").toInt();
      if (v >= 0 && v < HUB_LAYOUT_COUNT && hubLayoutId != (uint8_t)v) { hubLayoutId = (uint8_t)v; hubForceFullRedraw = true; hubDisplayDirty = true; }
    }
    if (server.hasArg("saver")) {
      int v = server.arg("saver").toInt();
      if (v >= 0 && v < HUB_SAVER_COUNT) hubSaverId = (uint8_t)v;
      hubSaverActive = false;
      hubForceFullRedraw = true;
      hubDisplayDirty = true;
    }
    if (server.hasArg("saverIdle")) {
      int v = server.arg("saverIdle").toInt();
      if (v == 0) hubSaverIdleSeconds = 0;
      else hubSaverIdleSeconds = (uint16_t)constrain(v, 10, 3600);
    }
    if (server.hasArg("orientation")) {
      int v = server.arg("orientation").toInt();
      if (v >= 0 && v <= 3 && hubOrientation != (uint8_t)v) {
        hubOrientation = (uint8_t)v;
        hubTFT.setRotation(hubOrientation);
        hubTFT.fillScreen(0x0000);
        hubDisplayDirty = true;
        LOG_B("TFT rotation changed: %u, dimensions=%dx%d", hubOrientation, hubTFT.width(), hubTFT.height());
      }
    }
    if (server.hasArg("display")) {
      const bool requested = server.arg("display") != "0";
      if (requested != hubDisplayEnabled) hubDisplayDirty = true;
      hubDisplayEnabled = requested;
      displayMatterOn = hubDisplayEnabled;
      if (!hubDisplayEnabled) hubSaverActive = false;
      if (matterStarted) DisplayDimmer.setOnOff(displayMatterOn);
      writeDisplayBacklight();
    }
    if (server.hasArg("testFps")) {
      hubTestFpsTarget=(uint8_t)constrain(server.arg("testFps").toInt(),1,60);
      hubTestLastFrameAt=0;hubTestFpsWindowAt=millis();hubTestFrameCount=0;
    }
    if (server.hasArg("brightness")) {
      int v = constrain(server.arg("brightness").toInt(), 1, 100);
      displayBrightnessPercent = (uint8_t)v;
      if (matterStarted) DisplayDimmer.setBrightness(percentToMatter(displayBrightnessPercent));
      writeDisplayBacklight();
    }
    if (statePrefs.begin("hubcfg", false)) {
      statePrefs.putUChar("layout", hubLayoutId); statePrefs.putUChar("saver", hubSaverId); statePrefs.putUShort("idleSec", hubSaverIdleSeconds);
      statePrefs.putUChar("orient", hubOrientation); statePrefs.putBool("display", hubDisplayEnabled);
      statePrefs.putBool("dispOn", displayMatterOn); statePrefs.putUChar("dispBri", displayBrightnessPercent);
      statePrefs.end();
    }
  }
  String json = "{\"layout\":" + String(hubLayoutId) + ",\"layoutName\":\"" + HUB_LAYOUT_NAMES[hubLayoutId] +
    "\",\"saver\":" + String(hubSaverId) + ",\"orientation\":" + String(hubOrientation) +
    ",\"display\":" + String(hubDisplayEnabled ? "true" : "false") +
    ",\"brightness\":" + String(displayBrightnessPercent) + ",\"testFps\":" + String(hubTestFpsTarget) + ",\"saverIdle\":" + String(hubSaverIdleSeconds) + "}";
  server.send(200, "application/json; charset=utf-8", json);
}
void handleWeatherRefresh() {
  if (!weatherTaskHandle) { server.send(503, "application/json", "{\"ok\":false,\"message\":\"Задача погоды не запущена\"}"); return; }
  xTaskNotifyGive(weatherTaskHandle);
  server.send(202, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"Запрос поставлен в очередь\"}");
}

void handleWeatherSource() {
  if (!server.hasArg("source")) { server.send(400, "application/json", "{\"ok\":false,\"message\":\"Не указан источник\"}"); return; }
  const int mode=server.arg("source").toInt();
  if (mode<0 || mode>3) { server.send(400, "application/json", "{\"ok\":false,\"message\":\"Недопустимый источник\"}"); return; }
  weatherSourceMode=(uint8_t)mode;
  Preferences prefs;
  if (prefs.begin("hubcfg",false)) { prefs.putUChar("wxsrc",(uint8_t)mode); prefs.end(); }
  if (weatherTaskHandle) xTaskNotifyGive(weatherTaskHandle);
  static const char *names[]={"Авто","Open-Meteo","wttr.in","MET Norway"};
  String json="{\"ok\":true,\"message\":\"Источник: "+String(names[mode])+" · обновление…\"}";
  server.send(200,"application/json; charset=utf-8",json);
}

void handleHubData() {
  String json = "{";
  json += "\"indoorTemp\":"; if (sensorValid) json += String(indoorTempC, 1); else json += "null"; json += ",";
  json += "\"indoorHumidity\":"; if (sensorValid) json += String(indoorHumidity, 1); else json += "null"; json += ",";
  json += "\"outdoorTemp\":"; if (weatherValid) json += String(outdoorTempC, 1); else json += "null"; json += ",";
  json += "\"outdoorHumidity\":"; if (weatherValid && !isnan(outdoorHumidity)) json += String(outdoorHumidity, 1); else json += "null"; json += ",";
  json += "\"feelsLike\":"; if (weatherValid && !isnan(outdoorFeelsLikeC)) json += String(outdoorFeelsLikeC, 1); else json += "null"; json += ",";
  json += "\"pressure\":"; if (weatherValid && !isnan(outdoorPressureHpa)) json += String(outdoorPressureHpa, 1); else json += "null"; json += ",";
  json += "\"windSpeed\":"; if (weatherValid && !isnan(outdoorWindSpeedKmh)) json += String(outdoorWindSpeedKmh, 1); else json += "null"; json += ",";
  json += "\"windDirection\":"; if (weatherValid && !isnan(outdoorWindDirection)) json += String(outdoorWindDirection, 0); else json += "null"; json += ",";
  json += "\"cloudCover\":"; if (weatherValid && !isnan(outdoorCloudCover)) json += String(outdoorCloudCover, 0); else json += "null"; json += ",";
  json += "\"uvIndex\":"; if (weatherValid && !isnan(outdoorUvIndex)) json += String(outdoorUvIndex, 1); else json += "null"; json += ",";
  char weatherSourceCopy[24], weatherStatusCopy[64];
  copyWeatherText(weatherStatusCopy, sizeof(weatherStatusCopy), weatherSourceCopy, sizeof(weatherSourceCopy));
  json += "\"weatherSource\":\"" + jsonEscape(String(weatherSourceCopy)) + "\",";
  json += "\"precipProbability\":"; if (weatherValid && outdoorPrecipProbability >= 0) json += String(outdoorPrecipProbability); else json += "null"; json += ",";
  json += "\"precipitationMm\":"; if (weatherValid) json += String(outdoorPrecipMm, 1); else json += "null"; json += ",";
  json += "\"weatherMode\":" + String((uint8_t)weatherSourceMode) + ",";
  json += "\"time\":\"" + timeText + "\",\"date\":\"" + dateText + "\",\"weatherStatus\":\"" + jsonEscape(String(weatherStatusCopy)) + "\"}";
  server.send(200, "application/json; charset=utf-8", json);
}

// ============================================================
// Arduino setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("[BOOT] GoodWin Smart Hub v3.11.1-dev - Serial console ready @115200");
  Serial.flush();
  previousEspLogVprintf = esp_log_set_vprintf(webEspLogVprintf);
  LOG_B("GoodWin Smart Hub booting");
  LOG_B("Hardware: board-profile=%s chip=%s rev=%u cores=%u CPU=%uMHz Flash=%uMB @%uMHz PSRAM=%uKB ESP-IDF=%s",
    GOODWIN_BOARD_PROFILE, ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
    ESP.getCpuFreqMHz(), ESP.getFlashChipSize() / (1024UL * 1024UL),
    ESP.getFlashChipSpeed() / 1000000UL, ESP.getPsramSize() / 1024UL, ESP.getSdkVersion());
  setupSmartHubDisplay();
  configTzTime("YEKT-5", "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  statePrefs.begin("hubcfg", true);
  hubLayoutId = statePrefs.getUChar("layout", 0); if (hubLayoutId >= HUB_LAYOUT_COUNT) hubLayoutId = 0;
  hubSaverId = statePrefs.getUChar("saver", 0); if (hubSaverId >= HUB_SAVER_COUNT) hubSaverId = 0;
  hubOrientation = statePrefs.getUChar("orient", 0); if (hubOrientation > 3) hubOrientation = 0;
  hubTestFpsTarget = statePrefs.getUChar("testFps", 20); if (hubTestFpsTarget < 1 || hubTestFpsTarget > 60) hubTestFpsTarget = 20;
  weatherSourceMode = statePrefs.getUChar("wxsrc", 0); if (weatherSourceMode > 3) weatherSourceMode = 0;
  hubDisplayEnabled = statePrefs.getBool("display", true);
  hubTFT.setRotation(hubOrientation);
  displayMatterOn = hubDisplayEnabled;
  hubDisplayDirty = true;
  displayBrightnessPercent = statePrefs.getUChar("dispBri", 85);
  if (displayBrightnessPercent < 1 || displayBrightnessPercent > 100) displayBrightnessPercent = 85;
  statePrefs.end();
  writeDisplayBacklight();
  hubLastActivityAt = millis();
  LOG_B("TFT state: enabled=%s, brightness=%u%%, rotation=%u, dimensions=%dx%d, renderer=direct",
        hubDisplayEnabled ? "YES" : "NO", displayBrightnessPercent, hubOrientation, hubTFT.width(), hubTFT.height());
  hubDisplayDirty = true;
  loadMqttConfig(); // Also load settings in setup-AP mode, before Wi-Fi is configured.

  pinMode(LED_PIN, OUTPUT);
  pwmReady = ledcAttach(LED_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  if (!pwmReady) LOG_ERR("LEDC attach failed; falling back to digital output");
  writeLEDLevel(0);
  ledState = false;
  randomSeed(esp_random());

  pinMode(BOOT_PIN, INPUT_PULLUP);
  bootButtonStable = digitalRead(BOOT_PIN) == HIGH; // GPIO0: HIGH=released, LOW=pressed.
  bootButtonLastRaw = bootButtonStable;
  bootButtonDebounceAt = millis();

  if (!connectSavedWiFi()) {
    startPortal();
    setupWebServer();
    startWeatherTask();
    return;
  }

  startMatter();
  setupWebServer();
  setupMqtt();
  startWeatherTask();

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    LOG_W("mDNS: http://%s.local", MDNS_HOSTNAME);
  } else {
    LOG_ERR("mDNS startup failed");
  }

  if (String(OTA_PASSWORD) != "CHANGE_ME_GW_OTA_2026" && strlen(OTA_PASSWORD) >= 10) {
    ArduinoOTA.setHostname(MDNS_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    LOG_W("OTA enabled; hostname=%s", MDNS_HOSTNAME);
  } else {
    LOG_W("OTA disabled: set a unique OTA_PASSWORD (10+ chars) in sketch");
  }
  LOG_B("Initialization complete");
}

void loop() {
  server.handleClient();
  updateSmartHub();
  static uint32_t lastSerialHeartbeatAt=0;
  if((uint32_t)(millis()-lastSerialHeartbeatAt)>=30000UL){
    lastSerialHeartbeatAt=millis();
    char weatherStatusCopy[64], weatherProviderCopy[24];
    copyWeatherText(weatherStatusCopy, sizeof(weatherStatusCopy), weatherProviderCopy, sizeof(weatherProviderCopy));
    LOG_S("Heartbeat WiFi=%s RSSI=%d Matter=%s Fabric=%s heap=%u weather=%s/%s testFPS=%u",
      WiFi.status()==WL_CONNECTED?"Y":"N",WiFi.status()==WL_CONNECTED?WiFi.RSSI():0,
      Matter.isDeviceConnected()?"Y":"N",Matter.isDeviceCommissioned()?"Y":"N",ESP.getFreeHeap(),
      weatherStatusCopy,weatherProviderCopy,hubTestMeasuredFps);
  }

  // Persist the final display setting once the Matter slider settles.
  if (displayPrefsPending && (uint32_t)(millis() - displayPrefsChangedAt) >= 1500) {
    if (statePrefs.begin("hubcfg", false)) {
      statePrefs.putBool("display", hubDisplayEnabled);
      statePrefs.putBool("dispOn", displayMatterOn);
      statePrefs.putUChar("dispBri", displayBrightnessPercent);
    statePrefs.putUChar("testFps", hubTestFpsTarget);
      statePrefs.end();
    }
    displayPrefsPending = false;
  }
  if (!portalMode && String(OTA_PASSWORD) != "CHANGE_ME_GW_OTA_2026" && strlen(OTA_PASSWORD) >= 10) ArduinoOTA.handle();

  if (restartPending && (int32_t)(millis() - restartAt) >= 0) {
    restartPending = false;
    if (pendingFactoryReset) factoryReset();
    else ESP.restart();
    return;
  }
  // The BOOT button must remain active even when Wi-Fi setup AP mode is running.
  processBootButton();

  if (portalMode || recoveryAPStarted) {
    if (captiveDnsStarted) captiveDns.processNextRequest();
    if (portalMode) updatePortalIndicator();
    // Initial setup portal keeps running its scheduled saved-network retries.
    if (portalMode) return;
  }

  processWiFiRecovery();
  mqttLoop();
  updateEffects();
  saveStateIfNeeded();

  // Event-driven status logging: print once at startup, then only on changes.
  static bool statusSnapshotValid = false;
  static bool prevWifi = false, prevMatter = false, prevFabric = false, prevLed = false;
  static uint8_t prevBrightness = 255, prevEffect = 255;
  const bool statusWifi = WiFi.status() == WL_CONNECTED;
  const bool statusMatter = Matter.isDeviceConnected();
  const bool statusFabric = Matter.isDeviceCommissioned();
  const bool statusLed = lightOn;
  const uint8_t statusBrightness = brightnessPercent;
  const uint8_t statusEffect = currentEffect;
  const bool statusChanged = !statusSnapshotValid ||
    statusWifi != prevWifi || statusMatter != prevMatter ||
    statusFabric != prevFabric || statusLed != prevLed ||
    statusBrightness != prevBrightness || statusEffect != prevEffect;
  if (statusChanged) {
    prevWifi = statusWifi; prevMatter = statusMatter; prevFabric = statusFabric;
    prevLed = statusLed; prevBrightness = statusBrightness; prevEffect = statusEffect;
    statusSnapshotValid = true;
    LOG_S("WiFi=%s Matter=%s Fabric=%s LED=%s Brightness=%u%% Effect=%s",
      statusWifi ? "Y" : "N",
      statusMatter ? "Y" : "N",
      statusFabric ? "Y" : "N",
      statusLed ? "ON" : "OFF",
      statusBrightness,
      effects[statusEffect].name);
  }

  matterRestartIfNoFabric();
}
