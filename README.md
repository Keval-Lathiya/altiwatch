AltiWatch - DIY skydiving altimeter built on Waveshare ESP32-S3-Touch-LCD-2.8 with BMP390 sensor.

## Hardware notes

### Power architecture

**POWER LATCH POLARITY (verified empirically):**
- GPIO 7 = power-hold pin for battery operation
- HIGH = latch on (board stays powered from battery)
- LOW = latch off (board powers down)
- USB power bypasses the latch, so USB-powered behavior never tested this
- Setting GPIO 7 HIGH in setup() is required for battery-only operation
- Long-press BAT button triggers graceful shutdown via GPIO 7 -> LOW

## Storage architecture
- `jumps/jump_*.csv` — jump data logs (LittleFS)
- `/settings.bin` — persisted BLE-configurable settings (LittleFS)
- NVS (Preferences) was attempted but failed silently on this board's
  partition setup, so all persistence uses LittleFS.