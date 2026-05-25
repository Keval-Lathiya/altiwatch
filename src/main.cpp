#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Waveshare ESP32-S3-Touch-LCD-2.8 display pins
// Verify against the Waveshare schematic if the screen stays blank
#define TFT_CS   9
#define TFT_DC   8
#define TFT_RST  3
#define TFT_MOSI 11
#define TFT_SCLK 10
#define TFT_MISO 13
#define TFT_BL   14

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

void setup() {
    Serial.begin(115200);

    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    tft.begin();
    tft.setRotation(0);
    tft.fillScreen(ILI9341_BLACK);

    int16_t x, y;
    uint16_t w, h;

    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);
    tft.getTextBounds("AltiWatch", 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w) / 2, 130);
    tft.print("AltiWatch");

    tft.setTextColor(ILI9341_GREEN);
    tft.setTextSize(2);
    tft.getTextBounds("Stage 1 - Display OK", 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w) / 2, 170);
    tft.print("Stage 1 - Display OK");

    Serial.println("AltiWatch Stage 1 - Display OK");
}

void loop() {
}
