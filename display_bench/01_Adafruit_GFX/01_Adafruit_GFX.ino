/*
  GoodWin Smart Hub - Display Library Benchmark
  TEST 01: Adafruit_GFX + Adafruit_ST7735 (control)
  Target: classic ESP32 / DevKit V1, ST7735 128x160
  Wiring: SCK=18 MOSI=23 CS=13 DC=14 RST=26 BL=25
  Serial: 115200 baud

  Run each test for at least 60 seconds. Record the periodic BENCH lines.
  This is a standalone test sketch; it does not modify the Smart Hub firmware.
*/
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "esp_heap_caps.h"

static constexpr int PIN_TFT_SCK  = 18;
static constexpr int PIN_TFT_MOSI = 23;
static constexpr int PIN_TFT_CS   = 13;
static constexpr int PIN_TFT_DC   = 14;
static constexpr int PIN_TFT_RST  = 26;
static constexpr int PIN_TFT_BL   = 25;
static constexpr uint32_t SPI_HZ  = 20000000;
static constexpr int16_t TFT_W = 128;
static constexpr int16_t TFT_H = 160;
static constexpr int16_t PATCH_H = 20;

SPIClass tftSPI(VSPI);
Adafruit_ST7735 tft(&tftSPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GFXcanvas16 patchCanvas(TFT_W, PATCH_H);

uint32_t frames = 0;
uint32_t windowStart = 0;
uint32_t lastReport = 0;
uint32_t renderTotalUs = 0;
uint32_t renderMaxUs = 0;
uint32_t transferTotalUs = 0;
uint32_t transferMaxUs = 0;
uint32_t fullTotalUs = 0;
uint32_t fullMaxUs = 0;
uint32_t fullCount = 0;
uint32_t lastFullTest = 0;
bool runFullScreenTest = false;

static void drawStaticScene() {
  tft.drawRect(0, 0, TFT_W, TFT_H, ST77XX_CYAN);
  tft.drawFastHLine(0, 39, TFT_W, ST77XX_BLUE);
  tft.drawFastHLine(0, 119, TFT_W, ST77XX_BLUE);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(6, 8);
  tft.print("GOODWIN HUB");
  tft.setCursor(6, 22);
  tft.print("DISPLAY BENCH 01");
  tft.setCursor(6, 145);
  tft.print("ADAFRUIT / ST7735");
}

static void drawPatch(uint32_t frame) {
  const uint16_t bg = 0x0841; // Fixed background: avoid intentional frame-to-frame flicker
  patchCanvas.fillScreen(bg);
  patchCanvas.drawRect(0, 0, TFT_W, PATCH_H, ST77XX_CYAN);
  patchCanvas.setTextSize(1);
  patchCanvas.setTextColor(ST77XX_WHITE, bg);
  patchCanvas.setCursor(4, 6);
  patchCanvas.print("FRAME ");
  patchCanvas.print(frame);
  patchCanvas.fillCircle((frame * 3) % (TFT_W - 12) + 6, 10, 4, ST77XX_YELLOW);
}

static void reportStats() {
  const uint32_t now = millis();
  if (now - lastReport < 5000) return;
  const uint32_t elapsed = now - windowStart;
  const float fps = elapsed ? (frames * 1000.0f / elapsed) : 0.0f;
  const uint32_t freeHeap = ESP.getFreeHeap();
  const uint32_t minHeap = ESP.getMinFreeHeap();
  const uint32_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  Serial.printf(
    "BENCH,lib=Adafruit_GFX,mode=%s,fps=%.2f,render_avg_us=%lu,render_max_us=%lu,"
    "patch_tx_avg_us=%lu,patch_tx_max_us=%lu,full_avg_us=%lu,full_max_us=%lu,"
    "heap=%lu,min_heap=%lu,largest_block=%lu,frames=%lu\n",
    runFullScreenTest ? "mixed_full" : "partial",
    fps,
    frames ? renderTotalUs / frames : 0,
    renderMaxUs,
    frames ? transferTotalUs / frames : 0,
    transferMaxUs,
    fullCount ? fullTotalUs / fullCount : 0,
    fullMaxUs,
    freeHeap, minHeap, largest, frames, fullCount
  );
  frames = 0;
  fullCount = 0;
  renderTotalUs = renderMaxUs = 0;
  transferTotalUs = transferMaxUs = 0;
  fullTotalUs = fullMaxUs = 0;
  windowStart = now;
  lastReport = now;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  tftSPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(SPI_HZ);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticScene();

  Serial.println();
  Serial.println("GOODWIN_DISPLAY_BENCH_01_ADAFRUIT");
  Serial.printf("chip=%s,cores=%u,cpu_mhz=%u,flash=%lu\n",
                ESP.getChipModel(), ESP.getChipCores(), ESP.getCpuFreqMHz(),
                (unsigned long)ESP.getFlashChipSize());
  Serial.printf("display=%dx%d,spi_hz=%lu,patch=%dx%d,patch_bytes=%u\n",
                TFT_W, TFT_H, (unsigned long)SPI_HZ, TFT_W, PATCH_H,
                (unsigned)(TFT_W * PATCH_H * 2));
  Serial.printf("heap=%lu,min_heap=%lu,largest_block=%lu\n",
                (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
                (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.println("Commands: p=partial-only, f=mixed partial+full-screen test");
  windowStart = lastReport = millis();
}

void loop() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'p' || c == 'P') {
      runFullScreenTest = false;
      Serial.println("MODE=partial-only");
    } else if (c == 'f' || c == 'F') {
      runFullScreenTest = true;
      Serial.println("MODE=mixed_full");
    }
  }

  const uint32_t frame = frames;
  uint32_t t0 = micros();
  drawPatch(frame);
  uint32_t t1 = micros();
  tft.drawRGBBitmap(0, 40, patchCanvas.getBuffer(), TFT_W, PATCH_H);
  uint32_t t2 = micros();

  renderTotalUs += (t1 - t0);
  if (t1 - t0 > renderMaxUs) renderMaxUs = t1 - t0;
  transferTotalUs += (t2 - t1);
  if (t2 - t1 > transferMaxUs) transferMaxUs = t2 - t1;

  if (runFullScreenTest && millis() - lastFullTest >= 1000) {
    lastFullTest = millis();
    t0 = micros();
    tft.fillScreen(ST77XX_BLACK);
    drawStaticScene();
    tft.drawRGBBitmap(0, 40, patchCanvas.getBuffer(), TFT_W, PATCH_H);
    t1 = micros();
    const uint32_t dt = t1 - t0;
    fullTotalUs += dt;
    fullCount++;
    if (dt > fullMaxUs) fullMaxUs = dt;
  }

  frames++;
  reportStats();
}
