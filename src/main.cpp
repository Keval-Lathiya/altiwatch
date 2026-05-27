#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define BLACK 0x0000
#define WHITE 0xFFFF
#define GREEN 0x07E0
#define CYAN  0x07FF
#define GRAY  0x4208

#define SEALEVEL_HPA 1013.25f

// Display — ST7789 SPI 240x320
#define LCD_SCK  40
#define LCD_MOSI 45
#define LCD_CS   42
#define LCD_DC   41
#define LCD_RST  39
#define LCD_BL    5

// I2C
#define I2C_SDA 11
#define I2C_SCL 10

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, 240, 320);

Adafruit_BMP3XX bmp;

// ── display helpers ───────────────────────────────────────────────────────────

void drawLabel(int16_t x, int16_t y, const char *text) {
    gfx->setTextSize(1);
    gfx->setTextColor(GRAY);
    gfx->setCursor(x, y);
    gfx->print(text);
}

void drawStaticUI() {
    gfx->fillScreen(BLACK);

    // Title
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(20, 10);
    gfx->print("AltiWatch");
    gfx->setTextColor(CYAN);
    gfx->setTextSize(1);
    gfx->setCursor(20, 30);
    gfx->print("Stage 2b - BMP390");
    gfx->drawFastHLine(0, 42, 240, GRAY);

    drawLabel(10, 52,  "ALTITUDE (m)");
    drawLabel(10, 140, "PRESSURE (hPa)");
    drawLabel(10, 196, "TEMPERATURE (C)");
    gfx->drawFastHLine(0, 180, 240, GRAY);
    gfx->drawFastHLine(0, 236, 240, GRAY);
}

// Erase and redraw a value field. w/h define the erase rect.
void updateValue(int16_t x, int16_t y, int16_t w, int16_t h,
                 uint8_t sz, uint16_t color, const char *text) {
    gfx->fillRect(x, y, w, h, BLACK);
    gfx->setTextColor(color);
    gfx->setTextSize(sz);
    gfx->setCursor(x, y);
    gfx->print(text);
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 3000) {}

    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    gfx->begin();
    drawStaticUI();

    Wire.begin(I2C_SDA, I2C_SCL);

    if (!bmp.begin_I2C(0x77, &Wire)) {
        Serial.println("ERROR: BMP390 not found at 0x77");
        gfx->setTextColor(0xF800);
        gfx->setCursor(10, 70);
        gfx->print("BMP390 not found!");
        while (true) delay(1000);
    }

    // Oversampling: 8x pressure, 1x temperature (good noise rejection, ~20ms)
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    // IIR filter coeff 3 — smooths pressure while keeping ~1s step response
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    // 50 Hz output data rate (we'll read every 500ms)
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Serial.println("AltiWatch Stage 2b — BMP390 live readings");
    Serial.println("Temp(C)  Pressure(hPa)  Altitude(m)");
}

void loop() {
    if (!bmp.performReading()) {
        Serial.println("ERROR: BMP390 read failed");
        return;
    }

    float tempC    = bmp.temperature;
    float pressHpa = bmp.pressure / 100.0f;
    float altM     = bmp.readAltitude(SEALEVEL_HPA);

    Serial.printf("%.2f C   %.2f hPa   %.1f m\n", tempC, pressHpa, altM);

    char buf[24];

    // Altitude — large font, centre of screen
    snprintf(buf, sizeof(buf), "%.1f", altM);
    updateValue(10, 62, 220, 66, 5, GREEN, buf);

    // Pressure
    snprintf(buf, sizeof(buf), "%.2f", pressHpa);
    updateValue(10, 152, 220, 36, 3, WHITE, buf);

    // Temperature
    snprintf(buf, sizeof(buf), "%.2f", tempC);
    updateValue(10, 206, 220, 28, 3, CYAN, buf);

    delay(500);
}
