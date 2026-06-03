AltiWatch - DIY skydiving altimeter built on Waveshare ESP32-S3-Touch-LCD-2.8 with BMP390 sensor.

## Storage architecture
- `jumps/jump_*.csv` — jump data logs (LittleFS)
- `/settings.bin` — persisted BLE-configurable settings (LittleFS)
- NVS (Preferences) was attempted but failed silently on this board's
  partition setup, so all persistence uses LittleFS.