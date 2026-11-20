/*
  GoodWin Smart Hub — standalone Wi-Fi initialization diagnostic
  Target: ESP32 Dev Module / ESP32-D0WDQ6
  Intended Arduino-ESP32 core: 3.3.12 (ESP-IDF 5.5.5)

  This sketch does NOT touch Preferences/NVS, Wi-Fi credentials, or Matter data.
  It does not connect to an access point and does not start an application server.

  Select one test path, upload, and capture the complete Serial output:
    WIFI_TEST_PATH 1 = Arduino WiFi API (the path that fails in the main sketch)
    WIFI_TEST_PATH 2 = direct ESP-IDF initialization (independent comparison)

  IMPORTANT: Change the path only between uploads. Do not mix Arduino WiFi and
  direct esp_wifi_init() in the same boot.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_idf_version.h>

#ifndef WIFI_TEST_PATH
  #define WIFI_TEST_PATH 1
#endif

static void printResult(const char *stage, esp_err_t err) {
  Serial.printf("[IDF] %-34s -> %s (0x%04X)\n",
                stage, esp_err_to_name(err), (unsigned)err);
}

static void printPlatformInfo() {
  Serial.println();
  Serial.println("========== GoodWin Wi-Fi INIT TEST ==========");
  Serial.printf("[INFO] Arduino-ESP32 version: %s\n", ESP_ARDUINO_VERSION_STR);
  Serial.printf("[INFO] ESP-IDF version:       %s\n", esp_get_idf_version());
  Serial.printf("[INFO] Chip model:            %s\n", ESP.getChipModel());
  Serial.printf("[INFO] Chip revision:         %u\n", ESP.getChipRevision());
  Serial.printf("[INFO] CPU frequency:         %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("[INFO] Flash size:            %u bytes\n", ESP.getFlashChipSize());
  Serial.printf("[INFO] Free heap:             %u bytes\n", ESP.getFreeHeap());
  Serial.printf("[INFO] Test path:             %u\n", (unsigned)WIFI_TEST_PATH);
  Serial.println("[INFO] NVS and Matter storage will NOT be erased or modified.");
  Serial.println("=============================================");
}

static bool testArduinoMode(const char *label, wifi_mode_t requested) {
  Serial.printf("\n[ARDUINO] Request WiFi.mode(%s=%u)\n",
                label, (unsigned)requested);

  const bool ok = WiFi.mode(requested);
  delay(250);

  const wifi_mode_t actual = WiFi.getMode();
  const wl_status_t status = WiFi.status();
  wifi_mode_t idfMode = WIFI_MODE_NULL;
  const esp_err_t getModeErr = esp_wifi_get_mode(&idfMode);

  Serial.printf("[ARDUINO] WiFi.mode return:  %s\n", ok ? "true" : "false");
  Serial.printf("[ARDUINO] WiFi.getMode:      %u\n", (unsigned)actual);
  Serial.printf("[ARDUINO] WiFi.status:       %u\n", (unsigned)status);
  printResult("esp_wifi_get_mode()", getModeErr);
  if (getModeErr == ESP_OK) {
    Serial.printf("[IDF] Driver mode:                  %u\n", (unsigned)idfMode);
  }

  if (!ok || actual != requested) {
    Serial.printf("[FAIL] Arduino path stopped at %s.\n", label);
    return false;
  }

  Serial.printf("[PASS] Arduino path entered %s.\n", label);
  return true;
}

static bool initNvsSafely() {
  Serial.println("\\n========== NVS INITIALIZATION ==========");
  esp_err_t err = nvs_flash_init();
  printResult("nvs_flash_init()", err);

  if (err == ESP_OK) {
    Serial.println("[PASS] NVS initialized; existing stored data was preserved.");
    return true;
  }

  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("[STOP] NVS reports no free pages or a version mismatch.");
    Serial.println("[SAFEGUARD] No erase/reformat will be attempted.");
    return false;
  }

  Serial.println("[STOP] NVS initialization failed. Wi-Fi test will not continue.");
  return false;
}

static void runArduinoPath() {
  Serial.println("\n========== PATH 1: ARDUINO WIFI ==========");
  if (!initNvsSafely()) return;
  Serial.printf("[BEFORE] WiFi.getMode() = %u\n", (unsigned)WiFi.getMode());
  wifi_mode_t mode = WIFI_MODE_NULL;
  printResult("esp_wifi_get_mode() before", esp_wifi_get_mode(&mode));

  if (!testArduinoMode("STA", WIFI_STA)) return;
  if (!testArduinoMode("AP", WIFI_AP)) return;
  if (!testArduinoMode("AP_STA", WIFI_AP_STA)) return;

  Serial.println("\n[PASS] All Arduino Wi-Fi mode transitions succeeded.");
}

static void runEspIdfPath() {
  Serial.println("\n========== PATH 2: DIRECT ESP-IDF ==========");
  if (!initNvsSafely()) return;

  esp_err_t err = esp_netif_init();
  printResult("esp_netif_init()", err);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return;

  err = esp_event_loop_create_default();
  printResult("esp_event_loop_create_default()", err);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return;

  esp_netif_t *staNetif = esp_netif_create_default_wifi_sta();
  Serial.printf("[IDF] create_default_wifi_sta(): %s\n", staNetif ? "OK" : "NULL");
  if (!staNetif) return;

  esp_netif_t *apNetif = esp_netif_create_default_wifi_ap();
  Serial.printf("[IDF] create_default_wifi_ap():  %s\n", apNetif ? "OK" : "NULL");
  if (!apNetif) return;

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&cfg);
  printResult("esp_wifi_init()", err);
  if (err != ESP_OK) return;

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  printResult("esp_wifi_set_storage(RAM)", err);
  if (err != ESP_OK) return;

  err = esp_wifi_set_mode(WIFI_MODE_STA);
  printResult("esp_wifi_set_mode(STA)", err);
  if (err != ESP_OK) return;

  err = esp_wifi_start();
  printResult("esp_wifi_start()", err);
  if (err != ESP_OK) return;

  wifi_mode_t actual = WIFI_MODE_NULL;
  err = esp_wifi_get_mode(&actual);
  printResult("esp_wifi_get_mode()", err);
  if (err == ESP_OK) {
    Serial.printf("[IDF] Actual driver mode: %u\n", (unsigned)actual);
  }

  err = esp_wifi_set_mode(WIFI_MODE_AP);
  printResult("esp_wifi_set_mode(AP)", err);
  if (err != ESP_OK) return;

  err = esp_wifi_set_mode(WIFI_MODE_APSTA);
  printResult("esp_wifi_set_mode(APSTA)", err);
  if (err != ESP_OK) return;

  Serial.println("\n[PASS] Direct ESP-IDF Wi-Fi initialization and mode transitions succeeded.");
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  printPlatformInfo();

#if WIFI_TEST_PATH == 1
  runArduinoPath();
#elif WIFI_TEST_PATH == 2
  runEspIdfPath();
#else
  Serial.println("[ERROR] Invalid WIFI_TEST_PATH. Use 1 or 2.");
#endif

  Serial.println("\n========== TEST FINISHED ==========");
  Serial.println("Device is held here. No Wi-Fi connection was attempted.");
}

void loop() {
  delay(1000);
}
