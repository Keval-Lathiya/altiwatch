#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

// ── colour palette ────────────────────────────────────────────────────────────
#define BLACK  0x0000
#define WHITE  0xFFFF
#define GREEN  0x07E0
#define CYAN   0x07FF
#define YELLOW 0xFFE0
#define RED    0xF800
#define GRAY   0x4208
#define DKGRAY 0x2104

// ── display (ST7789 SPI 240×320) ──────────────────────────────────────────────
#define LCD_SCK  40
#define LCD_MOSI 45
#define LCD_CS   42
#define LCD_DC   41
#define LCD_RST  39
#define LCD_BL    5

// ── I2C ───────────────────────────────────────────────────────────────────────
#define I2C_SDA 11
#define I2C_SCL 10

// ── BOOT button (GPIO0, active LOW, internal pull-up) ────────────────────────
// Source: Waveshare ESP32-S3-Touch-LCD-2.8B docs — same BOOT button wiring
// as all ESP32-S3-Touch-LCD-2.x variants. PWR button is power-circuit only.
#define BTN_PIN        0
#define LONG_PRESS_MS  2000

// ── physics ───────────────────────────────────────────────────────────────────
#define SEALEVEL_HPA  1013.25f
#define M_TO_FT       3.28084f

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, 240, 320);
Adafruit_BMP3XX bmp;

// ── calibration state ─────────────────────────────────────────────────────────
float groundAltM = 0.0f;
bool  calibrated  = false;

// ── vertical-speed rolling window (8 slots × 500 ms ≈ 4 s window) ────────────
#define VS_SLOTS 8
float    vsAlt[VS_SLOTS];
uint32_t vsTime[VS_SLOTS];
uint8_t  vsIdx  = 0;
bool     vsFull = false;

// ── button state ──────────────────────────────────────────────────────────────
uint32_t btnPressStart    = 0;
bool     btnWasDown       = false;
bool     longFired        = false;

// ─────────────────────────────────────────────────────────────────────────────
// Display helpers
// ─────────────────────────────────────────────────────────────────────────────

void drawStaticUI() {
    gfx->fillScreen(BLACK);

    // Title bar
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(20, 8);
    gfx->print("AltiWatch");
    gfx->setTextSize(1);
    gfx->setTextColor(CYAN);
    gfx->setCursor(20, 28);
    gfx->print("Stage 3 - AGL + Vspeed");
    gfx->drawFastHLine(0, 40, 240, GRAY);

    // AGL section label
    gfx->setTextColor(GRAY);
    gfx->setCursor(8, 46);
    gfx->print("AGL ALTITUDE (ft)");

    // Vspeed divider + label
    gfx->drawFastHLine(0, 152, 240, GRAY);
    gfx->setTextColor(GRAY);
    gfx->setCursor(8, 158);
    gfx->print("VERT SPEED");

    // Bottom divider + aux row labels
    gfx->drawFastHLine(0, 224, 240, GRAY);
    gfx->setTextColor(GRAY);
    gfx->setCursor(8, 230);
    gfx->print("PRESS (hPa)");
    gfx->setCursor(128, 230);
    gfx->print("TEMP (C)");

    // Calibration hint
    gfx->drawFastHLine(0, 282, 240, DKGRAY);
    gfx->setTextColor(DKGRAY);
    gfx->setCursor(8, 288);
    gfx->print("Hold BOOT 2s = set ground zero");
}

// Erase and redraw a value in a fixed rect.
void updateField(int16_t x, int16_t y, int16_t w, int16_t h,
                 uint8_t sz, uint16_t color, const char *text) {
    gfx->fillRect(x, y, w, h, BLACK);
    gfx->setTextColor(color);
    gfx->setTextSize(sz);
    gfx->setCursor(x, y);
    gfx->print(text);
}

// ─────────────────────────────────────────────────────────────────────────────
// Ground calibration — average 10 BMP readings ~100 ms apart
// ─────────────────────────────────────────────────────────────────────────────
void doCalibration() {
    // Flash a brief "CAL..." indicator
    gfx->fillRect(0, 56, 240, 92, BLACK);
    gfx->setTextColor(YELLOW);
    gfx->setTextSize(3);
    gfx->setCursor(50, 80);
    gfx->print("CAL...");

    float sum = 0.0f;
    int n = 0;
    for (int i = 0; i < 10; i++) {
        if (bmp.performReading()) {
            sum += bmp.readAltitude(SEALEVEL_HPA);
            n++;
        }
        delay(100);
    }
    if (n > 0) {
        groundAltM = sum / n;
        calibrated = true;
    }

    // Reset vspeed buffer
    vsIdx  = 0;
    vsFull = false;

    Serial.printf("Calibrated: ground = %.2f m (%.1f ft)\n",
                  groundAltM, groundAltM * M_TO_FT);

    // Clear the CAL indicator
    gfx->fillRect(0, 56, 240, 92, BLACK);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 3000) {}

    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    gfx->begin();
    drawStaticUI();

    pinMode(BTN_PIN, INPUT_PULLUP);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!bmp.begin_I2C(0x77, &Wire)) {
        Serial.println("ERROR: BMP390 not found");
        gfx->setTextColor(RED);
        gfx->setCursor(10, 70);
        gfx->print("BMP390 not found!");
        while (true) delay(1000);
    }
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Serial.println("AltiWatch Stage 3 — AGL + Vertical Speed");
    Serial.println("Hold BOOT (GPIO0) 2 s to set ground zero");
    Serial.println("AGL(ft)   Vspeed(ft/min)   Press(hPa)   Temp(C)");
}

// ─────────────────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // ── Button long-press detection ───────────────────────────────────────────
    bool btnDown = (digitalRead(BTN_PIN) == LOW);
    if (btnDown && !btnWasDown) {
        btnPressStart = millis();
        btnWasDown    = true;
        longFired     = false;
    } else if (!btnDown) {
        btnWasDown = false;
        longFired  = false;
    }
    if (btnDown && !longFired && (millis() - btnPressStart >= LONG_PRESS_MS)) {
        longFired = true;
        doCalibration();
    }

    // ── Sensor read ───────────────────────────────────────────────────────────
    if (!bmp.performReading()) return;

    float absAltM  = bmp.readAltitude(SEALEVEL_HPA);
    float pressHpa = bmp.pressure / 100.0f;
    float tempC    = bmp.temperature;
    uint32_t now   = millis();

    // ── Vertical speed (rolling window) ──────────────────────────────────────
    vsAlt[vsIdx]  = absAltM;
    vsTime[vsIdx] = now;
    vsIdx = (vsIdx + 1) % VS_SLOTS;
    if (vsIdx == 0) vsFull = true;

    float vsMs   = 0.0f;  // m/s
    float vsFtm  = 0.0f;  // ft/min
    if (vsFull || vsIdx >= 2) {
        // oldest slot: if full the slot we're about to overwrite; else slot 0
        uint8_t oldest = vsFull ? vsIdx : 0;
        uint8_t newest = (vsIdx == 0) ? VS_SLOTS - 1 : vsIdx - 1;
        float dAlt  = vsAlt[newest]  - vsAlt[oldest];
        float dTime = (vsTime[newest] - vsTime[oldest]) / 1000.0f;  // seconds
        if (dTime > 0.1f) {
            vsMs  = dAlt / dTime;
            vsFtm = vsMs * M_TO_FT * 60.0f;
        }
    }

    // ── AGL ───────────────────────────────────────────────────────────────────
    float aglM  = calibrated ? (absAltM - groundAltM) : 0.0f;
    float aglFt = aglM * M_TO_FT;

    // ── Serial output ─────────────────────────────────────────────────────────
    if (calibrated) {
        Serial.printf("%.1f ft   %+.0f ft/min   %.2f hPa   %.2f C\n",
                      aglFt, vsFtm, pressHpa, tempC);
    } else {
        Serial.printf("(uncal) abs=%.1fm   %+.0f ft/min   %.2f hPa   %.2f C\n",
                      absAltM, vsFtm, pressHpa, tempC);
    }

    // ── Display update ────────────────────────────────────────────────────────
    char buf[24];

    // AGL altitude — big (size 4 = 24×32 px/char), up to ±9999 ft
    if (calibrated) {
        snprintf(buf, sizeof(buf), "%+.0f", aglFt);
    } else {
        snprintf(buf, sizeof(buf), "SET GND");
    }
    uint16_t aglColor = calibrated
        ? (aglFt > 1.0f ? GREEN : (aglFt < -1.0f ? RED : WHITE))
        : YELLOW;
    updateField(8, 56, 224, 88, 4, aglColor, buf);

    // CAL indicator (small, top-right when calibrated)
    gfx->fillRect(168, 8, 64, 14, BLACK);
    if (calibrated) {
        gfx->setTextColor(GREEN);
        gfx->setTextSize(1);
        gfx->setCursor(170, 10);
        gfx->print("GND SET");
    }

    // Vertical speed — size 3, with sign and units
    snprintf(buf, sizeof(buf), "%+.0f ft/m", vsFtm);
    uint16_t vsColor = (fabsf(vsFtm) < 30.0f) ? WHITE
                     : (vsFtm > 0) ? GREEN : RED;
    updateField(8, 168, 224, 52, 2, vsColor, buf);

    // m/s subscript
    snprintf(buf, sizeof(buf), "%+.1f m/s", vsMs);
    updateField(8, 206, 200, 16, 1, GRAY, buf);

    // Pressure + temperature (bottom row)
    snprintf(buf, sizeof(buf), "%.2f", pressHpa);
    updateField(8, 242, 116, 18, 1, WHITE, buf);
    snprintf(buf, sizeof(buf), "%.1f", tempC);
    updateField(130, 242, 100, 18, 1, WHITE, buf);

    delay(500);
}
