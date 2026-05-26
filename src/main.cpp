#include <Arduino.h>
#include <Arduino_GFX_Library.h>

#define BLUE  0x001F
#define WHITE 0xFFFF
#define GREEN 0x07E0

// Waveshare ESP32-S3-Touch-LCD-2.8 — ST7789T3 SPI 240x320
#define LCD_SCK   40
#define LCD_MOSI  45
#define LCD_CS    42
#define LCD_DC    41
#define LCD_RST   39
#define LCD_BL     5

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, 240, 320);

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 3000) {}
    delay(200);

    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    gfx->begin();
    gfx->fillScreen(BLUE);

    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(39, 110);
    gfx->println("AltiWatch");

    gfx->setTextColor(GREEN);
    gfx->setTextSize(2);
    gfx->setCursor(55, 165);
    gfx->println("Stage 1 OK");

    Serial.println("Stage 1 OK");
}

void loop() {
    delay(1000);
}
