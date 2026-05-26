#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>

#define BLUE  0x001F
#define WHITE 0xFFFF
#define GREEN 0x07E0

// Display — ST7789 SPI 240x320
#define LCD_SCK  40
#define LCD_MOSI 45
#define LCD_CS   42
#define LCD_DC   41
#define LCD_RST  39
#define LCD_BL    5

// I2C — board's exposed port
#define I2C_SDA 11
#define I2C_SCL 10

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, 240, 320);

void drawHeader() {
    gfx->fillScreen(0x0000);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(20, 10);
    gfx->println("AltiWatch");
    gfx->setTextColor(GREEN);
    gfx->setTextSize(1);
    gfx->setCursor(20, 34);
    gfx->println("I2C Scan (SDA=11 SCL=10)");
    gfx->drawFastHLine(0, 46, 240, 0x4208);
}

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 3000) {}

    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    gfx->begin();
    drawHeader();

    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("AltiWatch Stage 2a — I2C Scanner");
    Serial.printf("SDA=GPIO%d  SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
}

void loop() {
    Serial.println("\n--- I2C scan ---");
    gfx->fillRect(0, 50, 240, 270, 0x0000);
    gfx->setTextSize(1);

    int found = 0;
    int y = 54;
    for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  Found: 0x%02X\n", addr);
            gfx->setTextColor(GREEN);
            gfx->setCursor(10, y);
            gfx->printf("0x%02X", addr);

            // annotate known devices
            const char *name = "";
            if (addr == 0x76 || addr == 0x77) name = " BMP390";
            else if (addr == 0x68 || addr == 0x69) name = " IMU";
            else if (addr == 0x6A || addr == 0x6B) name = " IMU";
            else if (addr == 0x51)                  name = " RTC";
            else if (addr == 0x38)                  name = " Touch";
            else if (addr == 0x15)                  name = " Touch";

            gfx->setTextColor(WHITE);
            gfx->printf("%s\n", name);
            y += 12;
            found++;
        }
    }

    if (found == 0) {
        Serial.println("  No devices found");
        gfx->setTextColor(0xF800);
        gfx->setCursor(10, 54);
        gfx->println("No devices found");
    } else {
        Serial.printf("  Total: %d device(s)\n", found);
    }

    delay(3000);
}
