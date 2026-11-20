/*
  GoodWin Smart Hub — read-only NVS / partition diagnostic
  Target: ESP32 Dev Module / ESP32-D0WDQ6
  Intended Arduino-ESP32 core: 3.3.12 (ESP-IDF 5.5.5)

  SAFETY:
  - Does NOT call nvs_flash_init() or nvs_flash_erase().
  - Does NOT call Preferences.begin().
  - Does NOT initialize Wi-Fi or Matter.
  - Does NOT read NVS values, credentials, keys or certificates.
  - Only lists partition metadata and asks the NVS driver for aggregate statistics.
  - No writes, erases, formatting or recovery attempts.

  Upload with the SAME board and partition scheme as the main firmware.
*/

#include <Arduino.h>
#include <esp_partition.h>
#include <nvs.h>
#include <esp_err.h>
#include <esp_idf_version.h>

static void printErr(const char *label, esp_err_t err) {
  Serial.printf("[NVS] %-32s -> %s (0x%04X)\n",
                label, esp_err_to_name(err), (unsigned)err);
}

static const char *partitionTypeName(uint8_t type) {
  switch (type) {
    case ESP_PARTITION_TYPE_APP: return "APP";
    case ESP_PARTITION_TYPE_DATA: return "DATA";
    default: return "OTHER";
  }
}

static void printPartitionTable() {
  Serial.println("\n========== FLASH PARTITION TABLE (READ ONLY) ==========");
  esp_partition_iterator_t it =
      esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, nullptr);

  if (!it) {
    Serial.println("[PART] No partitions returned by partition iterator.");
    return;
  }

  while (it) {
    const esp_partition_t *p = esp_partition_get(it);
    if (p) {
      Serial.printf("[PART] label=%-16s type=%-5s subtype=0x%02X address=0x%06lX size=%lu bytes (%lu KB) encrypted=%s\n",
                    p->label,
                    partitionTypeName(p->type),
                    (unsigned)p->subtype,
                    (unsigned long)p->address,
                    (unsigned long)p->size,
                    (unsigned long)(p->size / 1024UL),
                    p->encrypted ? "yes" : "no");
    }
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);
  Serial.println("=======================================================");
}

static void inspectNvsStats(const char *partitionLabel) {
  nvs_stats_t stats = {};
  const esp_err_t err = nvs_get_stats(partitionLabel, &stats);

  char label[48];
  snprintf(label, sizeof(label), "nvs_get_stats(%s)",
           partitionLabel ? partitionLabel : "default");
  printErr(label, err);

  if (err == ESP_OK) {
    Serial.printf("[NVS] used_entries      = %lu\n", (unsigned long)stats.used_entries);
    Serial.printf("[NVS] free_entries      = %lu\n", (unsigned long)stats.free_entries);
    Serial.printf("[NVS] total_entries     = %lu\n", (unsigned long)stats.total_entries);
    Serial.printf("[NVS] namespace_count   = %lu\n", (unsigned long)stats.namespace_count);
    if (stats.total_entries > 0) {
      const float usedPct = 100.0f * (float)stats.used_entries / (float)stats.total_entries;
      Serial.printf("[NVS] approximate used  = %.1f%% of entry capacity\n", usedPct);
    }
  } else if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
    Serial.println("[NVS] Driver is not initialized in this boot; no initialization will be attempted.");
  } else {
    Serial.println("[NVS] Statistics unavailable. No repair or erase will be attempted.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========== GoodWin NVS READ-ONLY DIAGNOSTIC ==========");
  Serial.printf("[INFO] Arduino-ESP32: %s\n", ESP_ARDUINO_VERSION_STR);
  Serial.printf("[INFO] ESP-IDF:       %s\n", esp_get_idf_version());
  Serial.printf("[INFO] Chip:          %s rev %u\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("[INFO] Flash:         %lu bytes\n", (unsigned long)ESP.getFlashChipSize());
  Serial.printf("[INFO] Free heap:     %lu bytes\n", (unsigned long)ESP.getFreeHeap());
  Serial.println("[SAFE] Read-only diagnostic. No NVS init, writes, erase or formatting.");
  Serial.println("[SAFE] Wi-Fi credentials and Matter data are not read.");
  
  printPartitionTable();

  Serial.println("\n========== NVS STATISTICS (NO INIT) ==========");
  inspectNvsStats(nullptr);
  inspectNvsStats("nvs");

  Serial.println("\n========== DIAGNOSTIC FINISHED ==========");
  Serial.println("Please send the complete Serial Monitor output.");
}

void loop() {
  delay(1000);
}
