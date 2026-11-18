# 02 — TFT_eSPI benchmark

## Install

Install **TFT_eSPI** from Arduino Library Manager. This test is for the same classic ESP32 + ST7735 128x160 module used by the Adafruit control test.

## Configure TFT_eSPI

Unlike Adafruit_ST7735, TFT_eSPI takes its driver and pin configuration from the library's setup file, not from the sketch.

1. Locate the installed library folder, usually `Documents/Arduino/libraries/TFT_eSPI/`.
2. Back up `User_Setup.h`.
3. Edit `User_Setup.h` and ensure the active configuration contains:

```cpp
#define ST7735_DRIVER

#define TFT_WIDTH  128
#define TFT_HEIGHT 160

#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   13
#define TFT_DC   14
#define TFT_RST  26

#define SPI_FREQUENCY  20000000
```

Do not enable MISO for this write-only display. Do not configure TFT_BL in this benchmark; GPIO25 is the separate backlight control and is not used here.

Comment out conflicting driver, pin, and SPI-frequency definitions in the active setup. If the panel has incorrect colors, inversion, or offsets, adjust the ST7735 options only after confirming the benchmark compiles and draws.

## Arduino IDE

- Board: **ESP32 Dev Module**
- Serial: **115200**
- Sketch: `02_TFT_eSPI.ino`
- Install only the TFT_eSPI library for this test (plus its normal dependencies).

The sketch allocates one 128x20 RGB565 sprite (5,120 bytes), measures sprite rendering separately from `pushSprite()`, and periodically performs a full-screen repaint. It does not enable DMA, so this is the standard synchronous TFT_eSPI path.

## Run

1. Upload and wait for the 10-second warmup.
2. Leave the default `mixed_full` mode running and collect 5–6 `BENCH` lines.
3. Send `p` in Serial Monitor and check whether the partial-only animation flickers; collect 3–4 lines.
4. Send `f` to return to mixed mode if needed.

Compare the following fields with benchmark #01:

- `fps` — loop rate; not the display refresh rate.
- `render_avg_us` / `render_max_us` — sprite drawing time.
- `patch_tx_avg_us` / `patch_tx_max_us` — 128x20 sprite transfer time.
- `full_avg_us` / `full_max_us` — full-screen repaint time, averaged over full repaints only.
- `heap`, `min_heap`, `largest_block` — memory headroom in this standalone test.

Do not compare standalone heap values directly with the full GoodWin firmware. Record library version and any panel color/inversion adjustments alongside the results.
