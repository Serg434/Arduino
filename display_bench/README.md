# GoodWin Smart Hub — Display library benchmarks

This branch is isolated from both `stable/v3.11.0-live-telemetry` and the normal development branch.

## Test order

1. `01_Adafruit_GFX/01_Adafruit_GFX.ino` — control run using the library currently used by the firmware.
2. `02_TFT_eSPI/02_TFT_eSPI.ino`
3. `03_LovyanGFX/03_LovyanGFX.ino`
4. `04_Arduino_GFX/04_Arduino_GFX.ino`
5. `05_LVGL/05_LVGL.ino` — separate UI-framework evaluation.

Each test uses the same ESP32 + ST7735 pinout and should expose comparable serial metrics. Do not compare results unless the display orientation, SPI clock, scene and test duration match.

## Hardware profile

- ESP32 classic (DevKit V1 / ESP32-WROOM class)
- ST7735, 128×160, RGB565
- SCK 18, MOSI 23, CS 13, DC 14, RST 26, backlight 25
- SPI target: 20 MHz
- No PSRAM assumed

## Test procedure

1. Select the matching ESP32 Dev Module board in Arduino IDE.
2. Install only the library required by the current test (plus its dependencies).
3. Compile and upload the sketch.
4. Open Serial Monitor at 115200 baud.
5. Allow 10 seconds to warm up, then collect at least 60 seconds of `BENCH` lines.
6. Test the partial-update mode first. Where available, run the full-screen / stress mode for another 60 seconds.
7. Send the full serial log back with the test number and whether the display showed flicker, tearing, artifacts or freezes.

Record:
- compile success and library version
- FPS
- render average / maximum time
- transfer average / maximum time
- free heap, minimum heap and largest free block
- visual stability after 10 minutes

## Important

The benchmark sketches are diagnostic, not production firmware. Keep the main Smart Hub sketch unchanged until all candidate libraries have been compared. No benchmark has been compiled on the user's board or physically validated yet.
