#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

// ═══════════════════════════════════════════════════════════════════════════
// Settings — single source of truth. BLE-ready: serialise this struct.
// INVARIANT: calibration state (groundAltM, calSource) is NEVER modified by
// jump logging. Only doAutoCalibration() and doManualCalibration() may write it.
// ═══════════════════════════════════════════════════════════════════════════
struct Settings {
    // Sensor
    float   seaLevelHpa         = 1013.25f;
    uint8_t bmpAddr             = 0x77;

    // Boot auto-calibration
    uint8_t autoCalSamples      = 10;     // readings collected (× 500 ms = 5 s)
    float   autoCalStableThresh = 3.05f;  // metres  ≈ 10 ft — stable if below

    // Manual re-calibration
    uint32_t calPressMs         = 2000;   // BOOT hold duration
    uint8_t  calAvgCount        = 10;     // readings averaged (×100 ms)

    // Vertical speed
    uint8_t  vsDeadband         = 30;     // ft/min noise floor

    // Battery ADC  (GPIO6, 2:1 divider assumed — adjust *Mv values if wrong)
    uint8_t  batAdcPin          = 6;
    int      batMvFull          = 4200;
    int      batMvEmpty         = 3300;

    // Future state-machine thresholds (ft/min)
    float threshClimb           =  200.0f;
    float threshFreefall        = -3000.0f;
    float threshCanopy          = -1500.0f;
    float threshLanded          =   50.0f;
} cfg;

// ═══════════════════════════════════════════════════════════════════════════
// Calibration state
// Source tracks HOW we got a ground zero so the UI can show it.
// Jump logging code must never touch these variables.
// ═══════════════════════════════════════════════════════════════════════════
enum CalSource { CAL_NONE, CAL_AUTO, CAL_MANUAL };

struct CalState {
    float     groundAltM = 0.0f;
    CalSource source      = CAL_NONE;
    bool isValid() const { return source != CAL_NONE; }
} cal;

// ═══════════════════════════════════════════════════════════════════════════
// Boot phase — auto-cal sampling runs once on startup
// ═══════════════════════════════════════════════════════════════════════════
enum BootPhase { BOOT_SAMPLING, BOOT_RUNNING };
BootPhase bootPhase = BOOT_SAMPLING;

static const uint8_t MAX_BOOT_SAMPLES = 10;
float   bootBuf[MAX_BOOT_SAMPLES];
uint8_t bootIdx = 0;

// ═══════════════════════════════════════════════════════════════════════════
// Hardware
// ═══════════════════════════════════════════════════════════════════════════
#define LCD_SCK   40
#define LCD_MOSI  45
#define LCD_CS    42
#define LCD_DC    41
#define LCD_RST   39
#define LCD_BL     5
#define I2C_SDA   11
#define I2C_SCL   10
// BOOT button GPIO0, active LOW — Waveshare ESP32-S3-Touch-LCD-2.x family
#define BTN_PIN    0

// ═══════════════════════════════════════════════════════════════════════════
// Colours (RGB565)
// ═══════════════════════════════════════════════════════════════════════════
#define C_BG      0x0000
#define C_WHITE   0xFFFF
#define C_GRAY    0x4208
#define C_DKGRAY  0x2104
#define C_GREEN   0x07E0
#define C_RED     0xF800
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF
#define C_ORANGE  0xFC60

// ═══════════════════════════════════════════════════════════════════════════
// Display layout (240×320 portrait)
// ═══════════════════════════════════════════════════════════════════════════
#define DISP_W  240
#define DISP_H  320

// Row positions
#define Y_HDR        3    // "AltiWatch" | cal badge | bat%
#define Y_DIV1      16
#define Y_ALT_LBL   21    // "FT AGL" / "FT ABS" small label
#define Y_ALT       31    // big altitude  (s6 = 36×48 px/char)
#define Y_DIV2     107
#define Y_VS_LBL   112    // "VERT SPEED"
#define Y_VS       122    // vspeed  (s3 = 18×24 px/char)
#define Y_VS_SUB   150    // m/s subscript
#define Y_DIV3     162
// y 163-284 reserved (barograph future)
#define Y_DIV4     285
#define Y_STATUS   294    // status / cal-warning line

// ═══════════════════════════════════════════════════════════════════════════
// Runtime state
// ═══════════════════════════════════════════════════════════════════════════
static const uint8_t VS_SLOTS = 8;
float    vsAlt[VS_SLOTS];
uint32_t vsTime[VS_SLOTS];
uint8_t  vsIdx  = 0;
bool     vsFull = false;

uint32_t btnPressStart = 0;
bool     btnWasDown    = false;
bool     longFired     = false;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX    *gfx  = new Arduino_ST7789(bus, LCD_RST, 0, true, DISP_W, DISP_H);
Adafruit_BMP3XX bmp;

// ═══════════════════════════════════════════════════════════════════════════
// Display helpers
// ═══════════════════════════════════════════════════════════════════════════
void centeredText(int16_t y, uint8_t sz, uint16_t col, const char *s) {
    int16_t x = (DISP_W - (int16_t)strlen(s) * 6 * sz) / 2;
    gfx->setTextColor(col); gfx->setTextSize(sz);
    gfx->setCursor(max(x, (int16_t)0), y);
    gfx->print(s);
}
void updateCentered(int16_t y, int16_t h, uint8_t sz, uint16_t col, const char *s) {
    gfx->fillRect(0, y, DISP_W, h, C_BG);
    centeredText(y, sz, col, s);
}

void drawStaticUI() {
    gfx->fillScreen(C_BG);
    gfx->setTextColor(C_GRAY); gfx->setTextSize(1);
    gfx->setCursor(4, Y_HDR);  gfx->print("AltiWatch");
    gfx->drawFastHLine(0, Y_DIV1, DISP_W, C_GRAY);
    gfx->drawFastHLine(0, Y_DIV2, DISP_W, C_GRAY);
    gfx->drawFastHLine(0, Y_DIV3, DISP_W, C_GRAY);
    gfx->drawFastHLine(0, Y_DIV4, DISP_W, C_GRAY);
    gfx->setTextColor(C_DKGRAY); gfx->setTextSize(1);
    gfx->setCursor(4, Y_VS_LBL); gfx->print("VERT SPEED");
}

// Cal badge — top-centre of header row
void updateCalBadge() {
    gfx->fillRect(80, Y_HDR, 80, 9, C_BG);
    switch (cal.source) {
        case CAL_AUTO:
            gfx->setTextColor(C_GREEN);  gfx->setTextSize(1);
            gfx->setCursor(92, Y_HDR);   gfx->print("[AUTO CAL]");
            break;
        case CAL_MANUAL:
            gfx->setTextColor(C_CYAN);   gfx->setTextSize(1);
            gfx->setCursor(89, Y_HDR);   gfx->print("[MANUAL CAL]");
            break;
        case CAL_NONE:
            gfx->setTextColor(C_YELLOW); gfx->setTextSize(1);
            gfx->setCursor(96, Y_HDR);   gfx->print("[UNCAL]");
            break;
    }
}

void updateBatDisplay() {
    int adcMv = analogReadMilliVolts(cfg.batAdcPin);
    int batMv = adcMv * 2;
    int pct   = constrain((int)(100.0f * (batMv - cfg.batMvEmpty) /
                                (float)(cfg.batMvFull  - cfg.batMvEmpty)), 0, 100);
    char buf[6]; snprintf(buf, sizeof(buf), "%3d%%", pct);
    gfx->fillRect(DISP_W - 44, Y_HDR, 42, 9, C_BG);
    uint16_t col = pct > 30 ? C_GREEN : (pct > 10 ? C_YELLOW : C_RED);
    gfx->setTextColor(col); gfx->setTextSize(1);
    gfx->setCursor(DISP_W - 42, Y_HDR); gfx->print(buf);
}

// Alt label + big number
void updateAltDisplay(float aglFt) {
    // Label: "FT AGL" when calibrated, "FT ABS" when raw
    gfx->fillRect(0, Y_ALT_LBL, DISP_W, 8, C_BG);
    centeredText(Y_ALT_LBL, 1, C_DKGRAY, cal.isValid() ? "FT  AGL" : "FT  ABS");

    char buf[12];
    uint16_t col;
    if (cal.isValid()) {
        snprintf(buf, sizeof(buf), "%+.0f", aglFt);
        col = aglFt >  1.0f ? C_GREEN
            : aglFt < -1.0f ? C_RED
            :                  C_WHITE;
    } else {
        // No calibration: show raw absolute in feet, white
        snprintf(buf, sizeof(buf), "%.0f", aglFt);
        col = C_WHITE;
    }
    updateCentered(Y_ALT, 52, 6, col, buf);
}

void updateVsDisplay(float vsFpm, float vsMs) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%+.0f fpm", vsFpm);
    uint16_t col = fabsf(vsFpm) < cfg.vsDeadband ? C_GRAY
                 : (vsFpm > 0)                    ? C_GREEN : C_RED;
    updateCentered(Y_VS, 26, 3, col, buf);

    snprintf(buf, sizeof(buf), "%+.1f m/s", vsMs);
    updateCentered(Y_VS_SUB, 10, 1, C_DKGRAY, buf);
}

void updateStatusLine() {
    if (cal.isValid()) {
        updateCentered(Y_STATUS, 18, 2, C_CYAN, "READY");
    } else {
        updateCentered(Y_STATUS, 18, 1, C_YELLOW, "HOLD BOOT 2s TO CALIBRATE");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Calibration routines
// Only these two functions may write cal.groundAltM / cal.source.
// Jump logging must never call either of them.
// ═══════════════════════════════════════════════════════════════════════════
void applyCalibration(float groundM, CalSource src) {
    cal.groundAltM = groundM;
    cal.source      = src;
    vsIdx = 0; vsFull = false;   // reset vspeed so first post-cal reading is clean
}

void doAutoCalibration() {
    // Called once after boot sampling completes.
    float mn = bootBuf[0], mx = bootBuf[0], sum = 0.0f;
    for (uint8_t i = 0; i < cfg.autoCalSamples; i++) {
        mn   = min(mn, bootBuf[i]);
        mx   = max(mx, bootBuf[i]);
        sum += bootBuf[i];
    }
    float variationM = mx - mn;

    float variationFt = variationM * 3.28084f;
    Serial.printf("Auto-cal: variation=%.2fm (%.1fft), threshold=%.2fm (%.0fft)\n",
                  variationM, variationFt,
                  cfg.autoCalStableThresh, cfg.autoCalStableThresh * 3.28084f);

    if (variationM <= cfg.autoCalStableThresh) {
        applyCalibration(sum / cfg.autoCalSamples, CAL_AUTO);
        Serial.printf("Auto-cal OK: ground=%.2fm\n", cal.groundAltM);
        updateCentered(Y_ALT, 52, 2, C_GREEN, "CALIBRATED");
        delay(2000);
    } else {
        Serial.printf("Auto-cal SKIPPED: not stable\n");
        // Leave cal.source = CAL_NONE — show uncalibrated warning
        updateCentered(Y_ALT, 26, 1, C_YELLOW, "NOT CALIBRATED");
        updateCentered(Y_ALT + 28, 24, 1, C_YELLOW, "Hold BOOT on ground to set zero");
        delay(3000);
    }
}

void doManualCalibration() {
    // Always succeeds — averages cfg.calAvgCount readings.
    // This is the user override; works regardless of motion state.
    updateCentered(Y_ALT, 52, 3, C_YELLOW, "CAL...");
    float sum = 0.0f; int n = 0;
    for (int i = 0; i < cfg.calAvgCount; i++) {
        if (bmp.performReading()) { sum += bmp.readAltitude(cfg.seaLevelHpa); n++; }
        delay(100);
    }
    if (n > 0) {
        applyCalibration(sum / n, CAL_MANUAL);
        Serial.printf("Manual cal: ground=%.2fm\n", cal.groundAltM);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Boot sampling — feeds bootBuf until we have autoCalSamples readings,
// then calls doAutoCalibration() and transitions to BOOT_RUNNING.
// ═══════════════════════════════════════════════════════════════════════════
void handleBootSample(float altM) {
    bootBuf[bootIdx++] = altM;

    char buf[20];
    snprintf(buf, sizeof(buf), "SAMPLING %d/%d", bootIdx, cfg.autoCalSamples);
    updateCentered(Y_ALT, 52, 2, C_YELLOW, buf);

    if (bootIdx >= cfg.autoCalSamples) {
        bootPhase = BOOT_RUNNING;
        doAutoCalibration();
        updateCalBadge();
        updateStatusLine();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// setup
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 3000) {}

    analogSetAttenuation(ADC_11db);
    pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, HIGH);
    gfx->begin();
    drawStaticUI();
    updateCalBadge();   // shows [UNCAL] until sampling completes
    updateBatDisplay();
    updateStatusLine();

    centeredText(Y_ALT_LBL, 1, C_DKGRAY, "AUTO-CAL IN 5s...");

    pinMode(BTN_PIN, INPUT_PULLUP);
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!bmp.begin_I2C(cfg.bmpAddr, &Wire)) {
        Serial.println("ERROR: BMP390 not found");
        updateCentered(Y_ALT, 52, 1, C_RED, "BMP390 NOT FOUND");
        while (true) delay(1000);
    }
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Warm up the IIR filter before boot sampling begins.
    // IIR coeff 3 needs ~8 readings to settle from a cold start; the first
    // few samples after power-on are significantly off, which would push the
    // stability variance above the 10 ft threshold and cause auto-cal to fail.
    Serial.println("Warming up BMP390 IIR filter...");
    for (int i = 0; i < 8; i++) { bmp.performReading(); delay(100); }

    Serial.println("AltiWatch — collecting boot samples for auto-cal...");
}

// ═══════════════════════════════════════════════════════════════════════════
// loop
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    // ── Button (active during RUNNING only; ignored during SAMPLING) ─────
    if (bootPhase == BOOT_RUNNING) {
        bool btnDown = (digitalRead(BTN_PIN) == LOW);
        if (btnDown && !btnWasDown) {
            btnPressStart = millis(); btnWasDown = true; longFired = false;
        } else if (!btnDown) {
            btnWasDown = false; longFired = false;
        }
        if (btnDown && !longFired && (millis() - btnPressStart >= cfg.calPressMs)) {
            longFired = true;
            doManualCalibration();
            updateCalBadge();
            updateStatusLine();
        }
    }

    // ── Sensor read ──────────────────────────────────────────────────────
    if (!bmp.performReading()) { delay(500); return; }
    float absAltM  = bmp.readAltitude(cfg.seaLevelHpa);
    float pressHpa = bmp.pressure / 100.0f;
    float tempC    = bmp.temperature;
    uint32_t now   = millis();

    // ── Boot sampling phase ──────────────────────────────────────────────
    if (bootPhase == BOOT_SAMPLING) {
        handleBootSample(absAltM);
        delay(500);
        return;
    }

    // ── Vertical speed ───────────────────────────────────────────────────
    vsAlt[vsIdx] = absAltM; vsTime[vsIdx] = now;
    vsIdx = (vsIdx + 1) % VS_SLOTS;
    if (vsIdx == 0) vsFull = true;

    float vsMs = 0.0f, vsFpm = 0.0f;
    if (vsFull || vsIdx >= 2) {
        uint8_t oldest = vsFull ? vsIdx : 0;
        uint8_t newest = vsIdx == 0 ? VS_SLOTS - 1 : vsIdx - 1;
        float dAlt  = vsAlt[newest]  - vsAlt[oldest];
        float dTime = (vsTime[newest] - vsTime[oldest]) / 1000.0f;
        if (dTime > 0.1f) {
            vsMs  = dAlt / dTime;
            vsFpm = vsMs * 3.28084f * 60.0f;
            if (fabsf(vsFpm) < cfg.vsDeadband) { vsMs = 0.0f; vsFpm = 0.0f; }
        }
    }

    // ── AGL / ABS altitude ───────────────────────────────────────────────
    float dispAltM  = cal.isValid() ? (absAltM - cal.groundAltM) : absAltM;
    float dispAltFt = dispAltM * 3.28084f;

    // ── Serial (pressure + temp kept for logging even though off display) ─
    Serial.printf("%s %+.0fft  %+.0ffpm  %.2fhPa  %.1fC\n",
                  cal.source == CAL_AUTO   ? "[A]" :
                  cal.source == CAL_MANUAL ? "[M]" : "[?]",
                  dispAltFt, vsFpm, pressHpa, tempC);

    // ── Display ──────────────────────────────────────────────────────────
    updateAltDisplay(dispAltFt);
    updateVsDisplay(vsFpm, vsMs);
    updateBatDisplay();

    delay(500);
}
