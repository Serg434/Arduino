# GoodWin Smart Hub: AI and asynchronous services

## Current hardware budget

The reference target is ESP32-WROOM, 4 MB flash, no PSRAM, Arduino-ESP32 3.3.12. Matter, Wi-Fi, MQTT and TFT already consume substantial internal SRAM. A general-purpose multi-billion-parameter model is not a viable on-device target.

## Model decision

- Phi-3 Mini (3.8B) is excluded from the ESP32 target. Its quantized weights alone are measured in gigabytes.
- The first on-device generative-model experiment is **Micro-LM-Pico** (207K parameters, ternary 1.58-bit weights, approximately 120,960-byte exported model).
- The upstream Micro-LM project reports that Pico is intended for a standard ESP32 and uses flash memory mapping to avoid loading all weights into SRAM. Its training corpus is TinyStories/English text, however, and its published perplexity is still high. It is therefore a runtime proof-of-concept, not yet a reliable Russian command assistant.
- The model must be retrained or adapted on a compact Russian GoodWin command corpus before it is allowed to interpret user commands.
- Generated text must never directly invoke Matter, MQTT, GPIO or NVS actions. A deterministic allowlisted intent layer must validate any proposed action.

Upstream reference: https://github.com/adesgautam/micro-lm

## Asynchronous service boundaries

1. Weather already runs in a dedicated FreeRTOS worker and reports provider success/failure, latency and cooldown state.
2. Network diagnostics now use a bounded FreeRTOS queue and a worker task. The HTTP handler only validates and enqueues a job; DNS/TCP/HTTP work is performed outside the main loop.
3. The diagnostics interface intentionally supports only DNS lookup, TCP connect and bounded HTTP GET on port 80. It is not a shell. Arbitrary commands, filesystem access, ping and traceroute are not exposed in this first increment.
4. MQTT still uses PubSubClient and currently executes from the main loop. Moving it requires an ownership/command-queue refactor because MQTT callbacks can affect Matter state. Do not simply call the same PubSubClient instance from two tasks.
5. The HTTP server still uses WebServer and is polled from loop(). The dashboard is an additional /dashboard route. A full AsyncWebServer migration must port all existing routes and body handling, not just replace the server class.

## Dashboard and diagnostics

- /dashboard is a read-only observability page consuming /api, /system, and /hub/data.
- It displays climate gauges, Wi-Fi/Matter/MQTT status, heap/fragmentation, TFT timing, weather-provider history and current local adaptive-agent statistics.
- Its diagnostics panel submits jobs to /api/terminal/jobs and polls /api/terminal/jobs?id=....
- The job ring is deliberately bounded to four entries and the worker stack is limited to 4 KB. Increase either only after measuring heap and task-stack high-water marks on hardware.

## Rollout sequence

1. Keep dev/lovyangfx-tft-port untouched; work only on feature/async-core-3-12.
2. CI uses the available 3 MB Huge APP partition because Arduino-ESP32 3.3.12 does not expose the newer Matter partition menu key. This scheme has no OTA slot. The default 1.2 MB application slot is too small for the current 2.1 MB sketch. The exact physical partition table must still match the board's existing Matter/NVS layout before flashing.
3. Validate the dashboard and diagnostic worker on hardware, including weather endpoint tests.
4. Move MQTT to a single-owner worker and route state-changing callbacks through a bounded command queue.
5. Migrate WebServer routes incrementally to AsyncWebServer after dependency/API validation.
6. Integrate Micro-LM-Pico only after flash partition mapping, reduced static inference buffers, Russian retraining and inference-time budgets are verified.
7. Add ping and traceroute only after the ESP-IDF/lwIP APIs are validated on Arduino-ESP32 3.3.12.

## Safety and resource rules

- Never use setInsecure() as a default TLS workaround.
- Never expose an unrestricted shell on the device web server.
- Do not let an AI model directly perform hardware actions.
- Every worker must have bounded queue depth, bounded input/output buffers, explicit timeouts and observable task-stack/heap impact.
- Keep the TFT rendering path and Matter callbacks responsive; do not run inference or network handshakes in those contexts.