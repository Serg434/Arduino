/*
  GoodWin Smart Hub - display benchmark #02
  Library: TFT_eSPI
  Target: classic ESP32 + ST7735 128x160, SPI 20 MHz

  IMPORTANT:
  TFT_eSPI uses a library-level User_Setup.h. Configure it before compiling:
    #define ST7735_DRIVER
    #define TFT_WIDTH  128
    #define TFT_HEIGHT 160
    #define TFT_MOSI 23
    #define TFT_SCLK 18
    #define TFT_CS   13
    #define TFT_DC   14
    #define TFT_RST  26
    #define SPI_FREQUENCY 20000000
  Do not define TFT_MISO for this display. Leave TFT_BL undefined; GPIO25 is
  the external backlight control and is not needed for this graphics benchmark.
  Select the appropriate ST7735 color/inversion options for your panel if needed.

  Serial:
    p = partial-only benchmark
    f = mixed partial + full-screen benchmark
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_heap_caps.h>

static constexpr int16_t SCREEN_W = 128;
static constexpr int16_t SCREEN_H = 160;
static constexpr int16_t PATCH_H = 20;
static constexpr int16_t PATCH_Y = 40;
static constexpr uint32_t REPORT_MS = 5000;
static constexpr uint32_t FULL_EVERY_FRAMES = 250;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite patch = TFT_eSprite(&tft);

enum BenchMode : uint8_t { MODE_PARTIAL, MODE_MIXED_FULL };
BenchMode mode = MODE_MIXED_FULL;

uint32_t windowStartMs = 0;
uint32_t frames = 0;
uint32_t fullCount = 0;
uint64_t renderTotalUs = 0;
uint64_t patchTxTotalUs = 0;
uint64_t fullTotalUs = 0;
uint32_t renderMaxUs = 0;
uint32_t patchTxMaxUs = 0;
uint32_t fullMaxUs = 0;
uint32_t lastFullFrame = 0;

static inline uint32_t elapsedUs(uint32_t start) {
  return (uint32_t)(micros() - start);
}

void drawStaticScene() {
  tft.drawRect(2, 2, 124, 156, TFT_DARKGREY);
  tft.drawFastHLine(5, 24, 118, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("GoodWin TFT BENCH", 7, 8, 1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("TFT_eSPI", 7, 29, 2);
  tft.drawCircle(105, 105, 19, TFT_ORANGE);
  tft.drawCircle(105, 105, 12, TFT_YELLOW);
  tft.fillCircle(105, 105, 5, TFT_RED);
  tft.drawRoundRect(8, 112, 78, 25, 4, TFT_GREEN);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("128 x 160", 13, 120, 2);
}

void renderPatch(uint32_t frameNo) {
  const uint16_t bg = 0x0841;
  patch.fillSprite(bg);
  patch.drawFastHLine(0, 0, SCREEN_W, TFT_DARKGREY);
  patch.drawFastHLine(0, PATCH_H - 1, SCREEN_W, TFT_DARKGREY);
  patch.fillCircle(12 + (frameNo % 100), 10, 5, TFT_YELLOW);
  patch.drawRect(3, 3, 122, 14, TFT_CYAN);
  patch.setTextColor(TFT_WHITE, bg);
  patch.setTextDatum(MC_DATUM);
  char label[24];
  snprintf(label, sizeof(label), "FRAME %lu", (unsigned long)frameNo);
  patch.drawString(label, 64, 10, 2);
}

void resetStats() {
  windowStartMs = millis();
  frames = 0;
  fullCount = 0;
  renderTotalUs = 0;
  patchTxTotalUs = 0;
  fullTotalUs = 0;
  renderMaxUs = 0;
  patchTxMaxUs = 0;
  fullMaxUs = 0;
  lastFullFrame = 0;
}

void printStats() {
  const uint32_t elapsedMs = millis() - windowStartMs;
  if (elapsedMs == 0 || frames == 0) return;

  const float fps = (float)frames * 1000.0f / (float)elapsedMs;
  const uint32_t renderAvg = (uint32_t)(renderTotalUs / frames);
  const uint32_t patchAvg = (uint32_t)(patchTxTotalUs / frames);
  const uint32_t fullAvg = fullCount ? (uint32_t)(fullTotalUs / fullCount) : 0;
  const char *modeName = mode == MODE_PARTIAL ? "partial" : "mixed_full";

  Serial.printf(
    "BENCH,lib=TFT_eSPI,mode=%s,fps=%.2f,render_avg_us=%lu,render_max_us=%lu,"
    "patch_tx_avg_us=%lu,patch_tx_max_us=%lu,full_avg_us=%lu,full_max_us=%lu,"
    "heap=%lu,min_heap=%lu,largest_block=%lu,frames=%lu,full_count=%lu\n",
    modeName, fps,
    (unsigned long)renderAvg, (unsigned long)renderMaxUs,
    (unsigned long)patchAvg, (unsigned long)patchTxMaxUs,
    (unsigned long)fullAvg, (unsigned long)fullMaxUs,
    (unsigned long)ESP.getFreeHeap(),
    (unsigned long)ESP.getMinFreeHeap(),
    (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
    (unsigned long)frames, (unsigned long)fullCount
  );
  resetStats();
}

void handleSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == 'p' || c == 'P') {
      mode = MODE_PARTIAL;
      tft.fillScreen(TFT_BLACK);
      drawStaticScene();
      Serial.println("MODE=partial");
      resetStats();
    } else if (c == 'f' || c == 'F') {
      mode = MODE_MIXED_FULL;
      tft.fillScreen(TFT_BLACK);
      drawStaticScene();
      Serial.println("MODE=mixed_full");
      resetStats();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("GoodWin display benchmark #02 - TFT_eSPI");

  tft.init();
  tft.setRotation(0);
  tft.setSPISpeed(20000000);
  tft.fillScreen(TFT_BLACK);

  patch.setColorDepth(16);
  if (patch.createSprite(SCREEN_W, PATCH_H) == nullptr) {
    Serial.println("ERROR: sprite allocation failed");
    while (true) delay(1000);
  }

  drawStaticScene();
  Serial.printf("SPRITE_BYTES=%lu\n", (unsigned long)(SCREEN_W * PATCH_H * 2));
  Serial.printf("HEAP_AFTER_INIT=%lu,LARGEST_BLOCK=%lu\n",
    (unsigned long)ESP.getFreeHeap(),
    (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.println("Warmup 10 seconds...");
  delay(10000);
  Serial.println("MODE=mixed_full");
  resetStats();
}

void loop() {
  handleSerial();

  const uint32_t renderStart = micros();
  renderPatch(frames);
  const uint32_t renderUs = elapsedUs(renderStart);
  renderTotalUs += renderUs;
  if (renderUs > renderMaxUs) renderMaxUs = renderUs;

  const uint32_t txStart = micros();
  patch.pushSprite(0, PATCH_Y);
  const uint32_t txUs = elapsedUs(txStart);
  patchTxTotalUs += txUs;
  if (txUs > patchTxMaxUs) patchTxMaxUs = txUs;

  frames++;

  if (mode == MODE_MIXED_FULL && frames - lastFullFrame >= FULL_EVERY_FRAMES) {
    const uint32_t fullStart = micros();
    tft.fillScreen(TFT_BLACK);
    drawStaticScene();
    const uint32_t fullUs = elapsedUs(fullStart);
    fullTotalUs += fullUs;
    fullCount++;
    if (fullUs > fullMaxUs) fullMaxUs = fullUs;
    lastFullFrame = frames;
  }

  if (millis() - windowStartMs >= REPORT_MS) {
    printStats();
  }
}
