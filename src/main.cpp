#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <LittleFS.h>
#include <NimBLEDevice.h>

// ═══════════════════════════════════════════════════════════════════════════
// Settings — single source of truth. BLE-ready: serialise this struct.
// INVARIANT: cal.groundAltM / cal.source are written ONLY by applyCalibration().
// Jump logging code must never touch those fields.
// ═══════════════════════════════════════════════════════════════════════════
struct Settings {
    float   seaLevelHpa          = 1013.25f;
    uint8_t bmpAddr              = 0x77;
    uint8_t imuAddr              = 0x6B;
    uint8_t rtcAddr              = 0x51;

    // Boot auto-cal
    uint8_t  autoCalSamples      = 10;
    float    autoCalStableThresh = 3.05f;   // metres ≈ 10 ft

    // Manual cal (long press)
    uint32_t calPressMs          = 2000;
    uint8_t  calAvgCount         = 10;

    // Short press (log toggle)
    uint32_t shortPressMaxMs     = 1000;

    // Vertical speed deadband (ft/s)
    float    vsDeadbandFps       = 0.5f;

    // Battery ADC (GPIO6, 2:1 divider)
    uint8_t  batAdcPin           = 6;
    int      batMvFull           = 4200;
    int      batMvEmpty          = 3300;

    // Auto-start: freefall sustained threshold
    float    autoStartFps        = -30.0f;
    float    autoStartSustainSec = 2.0f;

    // Auto-stop: slow + stable altitude
    float    autoStopFps         = -15.0f;
    float    autoStopAltStabFt   = 10.0f;
    float    autoStopSustainSec  = 30.0f;

    // Future state-machine thresholds (ft/s)
    float threshClimb            =   3.3f;
    float threshFreefall         = -50.0f;
    float threshCanopy           = -25.0f;
    float threshLanded           =   0.8f;

    // Altitude alert zone — flash screen when descending through this band
    // REAL: alertStartFt = 3000.0f, alertStopFt = 500.0f  |  TEST: 10 / 2
    float alertStartFt           =   10.0f;  // ft AGL where flashing begins  (REAL: 3000)
    float alertStopFt            =    2.0f;  // ft AGL where flashing stops   (REAL:  500)
    bool  alertsEnabled          = true;

    // Vibration motor (GPIO15 via S8050 driver)
    bool    vibrationEnabled     = true;

    // Auto jump logging (BLE-toggleable)
    bool    autoLogEnabled       = true;

    // Display units: 0 = ft/fps,  1 = m/m·s⁻¹
    uint8_t units                = 0;
} cfg;

// ─── Calibration ────────────────────────────────────────────────────────────
enum CalSource { CAL_NONE, CAL_AUTO, CAL_MANUAL };
struct CalState {
    float     groundAltM = 0.0f;
    CalSource source      = CAL_NONE;
    bool isValid() const { return source != CAL_NONE; }
} cal;

// ─── Boot phase ─────────────────────────────────────────────────────────────
enum BootPhase { BOOT_SAMPLING, BOOT_RUNNING };
BootPhase bootPhase = BOOT_SAMPLING;
static const uint8_t MAX_BOOT_SAMPLES = 10;
float   bootBuf[MAX_BOOT_SAMPLES];
uint8_t bootIdx = 0;

// ─── Jump logging ────────────────────────────────────────────────────────────
enum LogTrigger { TRIG_MANUAL, TRIG_AUTO };

struct Sample {
    uint32_t time_ms;
    float    agl_ft, vspeed_fps, pressure_hpa, temp_c, ax, ay, az;
};

#define ROLL_SIZE 300
static Sample   rollBuf[ROLL_SIZE];
static uint16_t rollHead  = 0;
static uint16_t rollCount = 0;

static File       logFile;
static bool       logActive      = false;
static uint32_t   logStartMs     = 0;
static LogTrigger logTrigger     = TRIG_MANUAL;
static uint32_t   logSampleCount = 0;

// ─── Hardware pins ───────────────────────────────────────────────────────────
#define LCD_SCK      40
#define LCD_MOSI     45
#define LCD_CS       42
#define LCD_DC       41
#define LCD_RST      39
#define LCD_BL        5
#define I2C_SDA      11
#define I2C_SCL      10
#define BTN_PIN       0
#define MOTOR_PIN    15  // S8050 base driver — vibration motor

// ─── Colours (RGB565) ────────────────────────────────────────────────────────
#define C_BG      0x0000
#define C_WHITE   0xFFFF
#define C_GRAY    0x4208
#define C_DKGRAY  0x2104
#define C_GREEN   0x07E0
#define C_RED     0xF800
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF

// ─── Display layout (240×320 portrait) ──────────────────────────────────────
#define DISP_W   240
#define DISP_H   320
#define Y_HDR      3
#define Y_DIV1    16
#define Y_ALT_LBL 21
#define Y_ALT     31
#define Y_DIV2   107
#define Y_VS_LBL 112
#define Y_VS     122
#define Y_VS_SUB 150
#define Y_DIV3   162
#define Y_DIV4   285
#define Y_STATUS 294

// ─── Runtime state ──────────────────────────────────────────────────────────
static const uint8_t VS_SLOTS = 8;
static float    vsAlt[VS_SLOTS];
static uint32_t vsTime[VS_SLOTS];
static uint8_t  vsIdx  = 0;
static bool     vsFull = false;

static float    imuAx = 0.0f, imuAy = 0.0f, imuAz = 0.0f;

static uint32_t btnPressStart = 0;
static bool     btnWasDown    = false;
static bool     longFired     = false;

static uint32_t lastSampleMs  = 0;
static uint32_t lastDisplayMs = 0;
static uint32_t lastBootMs    = 0;

static uint32_t autoStartSustainMs = 0;
static uint32_t autoStopSustainMs  = 0;
static float    autoStopRefAlt     = 0.0f;

// ─── Motor buzz state ────────────────────────────────────────────────────────
static uint32_t motorOffAt = 0;   // millis() deadline to cut motor; 0 = off

// ─── Alert flash state ───────────────────────────────────────────────────────
enum AlertState { ALERT_OFF, ALERT_ON };
static AlertState alertState  = ALERT_OFF;
static uint32_t   lastFlashMs = 0;
static bool       flashRed    = false;
static float      cachedAltFt = 0.0f;  // updated each 10Hz tick
static float      cachedVsFps = 0.0f;

// ─── BLE state ───────────────────────────────────────────────────────────────
static volatile bool bleConnected    = false;
static volatile bool bleStateChanged = false;
// Command flags — set from BLE task, consumed in main loop
static volatile bool bleCmd_calibrate = false;
static volatile bool bleCmd_startLog  = false;
static volatile bool bleCmd_stopLog   = false;
static volatile bool bleCmd_setRTC    = false;
static uint8_t bleRtcYr = 0, bleRtcMo = 1, bleRtcDay = 1;
static uint8_t bleRtcHr = 0, bleRtcMn = 0, bleRtcSec = 0;

static NimBLECharacteristic *pAltChar      = nullptr;
static NimBLECharacteristic *pVsChar       = nullptr;
static NimBLECharacteristic *pBatChar      = nullptr;
static NimBLECharacteristic *pCalChar      = nullptr;
static NimBLECharacteristic *pLogStateChar = nullptr;
static NimBLECharacteristic *pLogDurChar   = nullptr;
static NimBLECharacteristic *pRtcChar      = nullptr;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX    *gfx  = new Arduino_ST7789(bus, LCD_RST, 0, true, DISP_W, DISP_H);
Adafruit_BMP3XX bmp;

// ═══════════════════════════════════════════════════════════════════════════
// I2C helpers
// ═══════════════════════════════════════════════════════════════════════════
static void iicWrite(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}
static uint8_t iicRead1(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}
static bool iicReadBuf(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(addr, (uint8_t)len);
    if ((uint8_t)Wire.available() < len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}
static uint8_t bcdDec(uint8_t bcd) { return (bcd >> 4) * 10 + (bcd & 0x0F); }

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
void updateCalBadge() {
    gfx->fillRect(80, Y_HDR, 80, 9, C_BG);
    switch (cal.source) {
        case CAL_AUTO:
            gfx->setTextColor(C_GREEN);  gfx->setTextSize(1);
            gfx->setCursor(92, Y_HDR);   gfx->print("[AUTO CAL]");  break;
        case CAL_MANUAL:
            gfx->setTextColor(C_CYAN);   gfx->setTextSize(1);
            gfx->setCursor(89, Y_HDR);   gfx->print("[MANUAL CAL]"); break;
        case CAL_NONE:
            gfx->setTextColor(C_YELLOW); gfx->setTextSize(1);
            gfx->setCursor(96, Y_HDR);   gfx->print("[UNCAL]");      break;
    }
}
void updateBLEBadge() {
    gfx->fillRect(58, Y_HDR, 22, 9, C_BG);
    if (bleConnected) {
        gfx->setTextColor(C_CYAN); gfx->setTextSize(1);
        gfx->setCursor(60, Y_HDR); gfx->print("BLE");
    }
}
void updateBatDisplay() {
    int adcMv = analogReadMilliVolts(cfg.batAdcPin);
    int batMv = adcMv * 2;
    int pct   = constrain((int)(100.0f * (batMv - cfg.batMvEmpty) /
                                (float)(cfg.batMvFull - cfg.batMvEmpty)), 0, 100);
    char buf[6]; snprintf(buf, sizeof(buf), "%3d%%", pct);
    gfx->fillRect(DISP_W - 44, Y_HDR, 42, 9, C_BG);
    uint16_t col = pct > 30 ? C_GREEN : (pct > 10 ? C_YELLOW : C_RED);
    gfx->setTextColor(col); gfx->setTextSize(1);
    gfx->setCursor(DISP_W - 42, Y_HDR); gfx->print(buf);
}
void updateAltDisplay(float aglFt) {
    gfx->fillRect(0, Y_ALT_LBL, DISP_W, 8, C_BG);
    char buf[12]; uint16_t col;
    if (cfg.units == 0) {
        centeredText(Y_ALT_LBL, 1, C_DKGRAY, cal.isValid() ? "FT  AGL" : "FT  ABS");
        if (cal.isValid()) {
            snprintf(buf, sizeof(buf), "%+.0f", aglFt);
            col = aglFt >  1.0f ? C_GREEN : aglFt < -1.0f ? C_RED : C_WHITE;
        } else {
            snprintf(buf, sizeof(buf), "%.0f", aglFt);
            col = C_WHITE;
        }
    } else {
        float aglM = aglFt / 3.28084f;
        centeredText(Y_ALT_LBL, 1, C_DKGRAY, cal.isValid() ? "M   AGL" : "M   ABS");
        if (cal.isValid()) {
            snprintf(buf, sizeof(buf), "%+.0f", aglM);
            col = aglM >  0.3f ? C_GREEN : aglM < -0.3f ? C_RED : C_WHITE;
        } else {
            snprintf(buf, sizeof(buf), "%.0f", aglM);
            col = C_WHITE;
        }
    }
    updateCentered(Y_ALT, 52, 6, col, buf);
}
void updateVsDisplay(float vsFps) {
    char buf[16];
    float absVal = fabsf(vsFps);
    uint16_t col = absVal < cfg.vsDeadbandFps ? C_GRAY
                 : (vsFps > 0)                ? C_GREEN : C_RED;
    if (cfg.units == 0) {
        if (absVal < 10.0f) snprintf(buf, sizeof(buf), "%+.1f fps", vsFps);
        else                snprintf(buf, sizeof(buf), "%+.0f fps", vsFps);
        updateCentered(Y_VS, 26, 3, col, buf);
        float vsMs = vsFps / 3.28084f;
        snprintf(buf, sizeof(buf), "%+.1f m/s", vsMs);
        updateCentered(Y_VS_SUB, 10, 1, C_DKGRAY, buf);
    } else {
        float vsMs = vsFps / 3.28084f;
        float absMs = fabsf(vsMs);
        if (absMs < 3.0f) snprintf(buf, sizeof(buf), "%+.1f m/s", vsMs);
        else              snprintf(buf, sizeof(buf), "%+.0f m/s", vsMs);
        updateCentered(Y_VS, 26, 3, col, buf);
        snprintf(buf, sizeof(buf), "%+.1f fps", vsFps);
        updateCentered(Y_VS_SUB, 10, 1, C_DKGRAY, buf);
    }
}
void updateStatusLine() {
    if (logActive) {
        uint32_t elapsed = (millis() - logStartMs) / 1000;
        char buf[24];
        snprintf(buf, sizeof(buf), "* REC %02u:%02u [%s]",
                 (unsigned)(elapsed / 60), (unsigned)(elapsed % 60),
                 logTrigger == TRIG_AUTO ? "AUTO" : "MAN");
        updateCentered(Y_STATUS, 18, 1, C_RED, buf);
    } else if (cal.isValid()) {
        updateCentered(Y_STATUS, 18, 2, C_CYAN, "READY");
    } else {
        updateCentered(Y_STATUS, 18, 1, C_YELLOW, "HOLD BOOT 2s TO CALIBRATE");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Calibration — ONLY these functions write cal.groundAltM / cal.source
// ═══════════════════════════════════════════════════════════════════════════
void applyCalibration(float groundM, CalSource src) {
    cal.groundAltM = groundM;
    cal.source      = src;
    vsIdx = 0; vsFull = false;
}
void doAutoCalibration() {
    float mn = bootBuf[0], mx = bootBuf[0], sum = 0.0f;
    for (uint8_t i = 0; i < cfg.autoCalSamples; i++) {
        mn   = min(mn, bootBuf[i]);
        mx   = max(mx, bootBuf[i]);
        sum += bootBuf[i];
    }
    float variationM  = mx - mn;
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
        updateCentered(Y_ALT, 26, 1, C_YELLOW, "NOT CALIBRATED");
        updateCentered(Y_ALT + 28, 24, 1, C_YELLOW, "Hold BOOT on ground to set zero");
        delay(3000);
    }
}
void doManualCalibration() {
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

// ─── Boot sampling ──────────────────────────────────────────────────────────
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
// RTC (PCF85063 at 0x51)
// ═══════════════════════════════════════════════════════════════════════════
static bool rtcIsValid() {
    return !(iicRead1(cfg.rtcAddr, 0x04) & 0x80);  // OS flag = oscillator-stop
}
static void rtcBuildFilename(char *buf, size_t n) {
    if (rtcIsValid()) {
        uint8_t regs[7];
        if (iicReadBuf(cfg.rtcAddr, 0x04, regs, 7)) {
            uint8_t sec = bcdDec(regs[0] & 0x7F);
            uint8_t mn  = bcdDec(regs[1] & 0x7F);
            uint8_t hr  = bcdDec(regs[2] & 0x3F);
            uint8_t day = bcdDec(regs[3] & 0x3F);
            // regs[4] = weekday (skip)
            uint8_t mo  = bcdDec(regs[5] & 0x1F);
            uint8_t yr  = bcdDec(regs[6]);
            snprintf(buf, n, "/jump_20%02d-%02d-%02d_%02d%02d%02d.csv",
                     yr, mo, day, hr, mn, sec);
            return;
        }
    }
    snprintf(buf, n, "/jump_t%lu.csv", (unsigned long)(millis() / 1000));
}
static void rtcSetTime(uint8_t yr, uint8_t mo, uint8_t day,
                        uint8_t hr, uint8_t mn, uint8_t sec) {
    Wire.beginTransmission(cfg.rtcAddr);
    Wire.write(0x04);
    Wire.write((((sec/10)<<4)|(sec%10)) & 0x7F);  // clear OS flag
    Wire.write((((mn /10)<<4)|(mn %10)) & 0x7F);
    Wire.write((((hr /10)<<4)|(hr %10)) & 0x3F);
    Wire.write((((day/10)<<4)|(day%10)) & 0x3F);
    Wire.write(0x00);                               // weekday (don't care)
    Wire.write((((mo /10)<<4)|(mo %10)) & 0x1F);
    Wire.write( ((yr /10)<<4)|(yr %10));
    Wire.endTransmission();
}

// ═══════════════════════════════════════════════════════════════════════════
// Rolling pre-jump buffer (300 samples × 32 B = 9.4 KB)
// ═══════════════════════════════════════════════════════════════════════════
static void rollPush(const Sample &s) {
    rollBuf[rollHead] = s;
    rollHead = (rollHead + 1) % ROLL_SIZE;
    if (rollCount < ROLL_SIZE) rollCount++;
}

// ═══════════════════════════════════════════════════════════════════════════
// Jump log — NEVER touches cal.groundAltM or cal.source
// ═══════════════════════════════════════════════════════════════════════════
static void rollFlushToFile() {
    uint16_t count = rollCount;
    uint16_t start = (count == ROLL_SIZE) ? rollHead : 0;
    for (uint16_t i = 0; i < count; i++) {
        const Sample &s = rollBuf[(start + i) % ROLL_SIZE];
        logFile.printf("%lu,%.1f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f\n",
                       (unsigned long)s.time_ms,
                       s.agl_ft, s.vspeed_fps, s.pressure_hpa, s.temp_c,
                       s.ax, s.ay, s.az);
        logSampleCount++;
    }
    logFile.flush();
    Serial.printf("Roll buffer flushed: %u pre-samples\n", (unsigned)logSampleCount);
}

static void writeSampleToLog(const Sample &s) {
    if (!logActive) return;
    logFile.printf("%lu,%.1f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f\n",
                   (unsigned long)s.time_ms,
                   s.agl_ft, s.vspeed_fps, s.pressure_hpa, s.temp_c,
                   s.ax, s.ay, s.az);
    logSampleCount++;
    if (logSampleCount % 50 == 0) {
        logFile.flush();
        Serial.printf("Log: %lu samples\n", (unsigned long)logSampleCount);
    }
}

void startLog(LogTrigger trigger) {
    if (logActive) return;
    char filename[40];
    rtcBuildFilename(filename, sizeof(filename));
    logFile = LittleFS.open(filename, "w");
    if (!logFile) {
        Serial.printf("LOG FAIL: cannot open %s\n", filename);
        return;
    }
    logSampleCount = 0;
    logFile.println("time_ms,agl_ft,vspeed_fps,pressure_hpa,temp_c,ax,ay,az");
    if (trigger == TRIG_AUTO) rollFlushToFile();
    logActive  = true;
    logTrigger = trigger;
    logStartMs = millis();
    Serial.printf("Log START [%s]: %s\n",
                  trigger == TRIG_AUTO ? "AUTO" : "MAN", filename);
    updateStatusLine();
}

void stopLog() {
    if (!logActive) return;
    logFile.flush();
    logFile.close();
    logActive = false;
    Serial.printf("Log STOP: %lu samples total\n", (unsigned long)logSampleCount);
    updateStatusLine();
}

// ═══════════════════════════════════════════════════════════════════════════
// Serial command interface
// ═══════════════════════════════════════════════════════════════════════════
static void cmdList() {
    Serial.println("LIST_START");
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    while (f) {
        Serial.printf("%s %u\n", f.name(), (unsigned)f.size());
        f.close();
        f = root.openNextFile();
    }
    root.close();
    Serial.println("LIST_END");
}

static void cmdDump() {
    if (logActive)
        Serial.println("WARN stop recording first — current open file not yet flushed");
    Serial.println("DUMP_START");
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    while (f) {
        Serial.printf("FILE_START %s\n", f.name());
        uint8_t buf[256];
        size_t n;
        while ((n = f.read(buf, sizeof(buf))) > 0)
            Serial.write(buf, n);
        if (f.size() > 0) {
            f.seek(f.size() - 1);
            if (f.read() != '\n') Serial.write('\n');
        }
        Serial.printf("FILE_END %s\n", f.name());
        f.close();
        f = root.openNextFile();
    }
    root.close();
    Serial.println("DUMP_END");
}

static void handleSerialCmd(const char *cmd) {
    if      (strcmp(cmd, "PING") == 0) Serial.println("PONG");
    else if (strcmp(cmd, "LIST") == 0) cmdList();
    else if (strcmp(cmd, "DUMP") == 0) cmdDump();
}

static void handleSerialInput() {
    static char buf[8];
    static uint8_t len = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (len > 0) { buf[len] = '\0'; handleSerialCmd(buf); len = 0; }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = (char)toupper((unsigned char)c);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// IMU (QMI8658 at 0x6B)
// ═══════════════════════════════════════════════════════════════════════════
static void initIMU() {
    uint8_t who = iicRead1(cfg.imuAddr, 0x00);
    Serial.printf("QMI8658 WHO_AM_I=0x%02X\n", who);
    iicWrite(cfg.imuAddr, 0x02, 0x40);  // CTRL1: addr auto-increment
    iicWrite(cfg.imuAddr, 0x03, 0x28);  // CTRL2: ±8g, ~31 Hz ODR
    iicWrite(cfg.imuAddr, 0x08, 0x01);  // CTRL7: enable accelerometer
}
static void readIMU() {
    uint8_t buf[6];
    if (!iicReadBuf(cfg.imuAddr, 0x35, buf, 6)) return;
    int16_t ax = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    int16_t ay = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    int16_t az = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    const float scale = 8.0f / 32768.0f;
    imuAx = ax * scale;
    imuAy = ay * scale;
    imuAz = az * scale;
}

// ═══════════════════════════════════════════════════════════════════════════
// Auto-start / auto-stop — never touches cal state
// ═══════════════════════════════════════════════════════════════════════════
static void checkAutoTriggers(float vsFps, float aglFt, uint32_t now) {
    if (!logActive) {
        if (vsFps <= cfg.autoStartFps) {
            if (autoStartSustainMs == 0) autoStartSustainMs = now;
            if ((now - autoStartSustainMs) >= (uint32_t)(cfg.autoStartSustainSec * 1000.0f)) {
                startLog(TRIG_AUTO);
                autoStartSustainMs = 0;
            }
        } else {
            autoStartSustainMs = 0;
        }
    } else {
        if (vsFps > cfg.autoStopFps) {
            if (autoStopSustainMs == 0) {
                autoStopSustainMs = now;
                autoStopRefAlt    = aglFt;
            } else if (fabsf(aglFt - autoStopRefAlt) > cfg.autoStopAltStabFt) {
                autoStopSustainMs = now;
                autoStopRefAlt    = aglFt;
            } else if ((now - autoStopSustainMs) >= (uint32_t)(cfg.autoStopSustainSec * 1000.0f)) {
                stopLog();
                autoStopSustainMs = 0;
            }
        } else {
            autoStopSustainMs = 0;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Motor — non-blocking pulse driver; never touches cal or log state
// ═══════════════════════════════════════════════════════════════════════════
static void stopBuzz() {
    digitalWrite(MOTOR_PIN, LOW);
    motorOffAt = 0;
}
static void startBuzz(uint32_t halfMs, uint32_t now) {
    if (!cfg.vibrationEnabled) return;
    uint32_t onMs = (halfMs < 150) ? halfMs : 150;
    digitalWrite(MOTOR_PIN, HIGH);
    motorOffAt = now + onMs;
}
static void tickBuzz(uint32_t now) {
    if (motorOffAt == 0) return;
    if (now - motorOffAt < 0x80000000UL) stopBuzz();
}

// ═══════════════════════════════════════════════════════════════════════════
// Alert zone flash — non-blocking; never touches cal or log state
// ═══════════════════════════════════════════════════════════════════════════
static uint32_t alertFlashHalfMs(float aglFt) {
    float range = cfg.alertStartFt - cfg.alertStopFt;
    if (range <= 0.0f) return 150;
    float t = constrain((aglFt - cfg.alertStopFt) / range, 0.0f, 1.0f);
    return (uint32_t)(100.0f + t * 400.0f);
}

static void checkAlert(float aglFt, float vsFps, uint32_t now) {
    if (!cfg.alertsEnabled || !cal.isValid()) return;
    bool inZone = (vsFps < 0.0f)
               && (aglFt < cfg.alertStartFt)
               && (aglFt > cfg.alertStopFt);
    if (inZone && alertState == ALERT_OFF) {
        alertState  = ALERT_ON;
        flashRed    = true;
        lastFlashMs = now;
        Serial.printf("ALERT ON:  %.0fft  flash-half=%ums\n",
                      aglFt, (unsigned)alertFlashHalfMs(aglFt));
    } else if (!inZone && alertState == ALERT_ON) {
        alertState = ALERT_OFF;
        flashRed   = false;
        stopBuzz();
        drawStaticUI();
        updateBLEBadge();
        updateCalBadge();
        updateBatDisplay();
        updateAltDisplay(aglFt);
        updateVsDisplay(vsFps);
        updateStatusLine();
        Serial.printf("ALERT OFF: %.0fft\n", aglFt);
    }
}

static void tickAlert(uint32_t now) {
    if (alertState != ALERT_ON) return;
    uint32_t halfMs = alertFlashHalfMs(cachedAltFt);
    if (now - lastFlashMs < halfMs) return;
    lastFlashMs = now;
    flashRed = !flashRed;
    gfx->fillScreen(flashRed ? C_RED : C_BG);
    char buf[12];
    if (cal.isValid()) snprintf(buf, sizeof(buf), "%+.0f", cachedAltFt);
    else               snprintf(buf, sizeof(buf), "%.0f",  cachedAltFt);
    centeredText(Y_ALT, 6, C_WHITE, buf);
    if (flashRed) startBuzz(halfMs, now);
}

// ═══════════════════════════════════════════════════════════════════════════
// Button — short press (<1 s on release) = toggle log
//           long  press (≥2 s on hold)   = manual calibration
// ═══════════════════════════════════════════════════════════════════════════
static void handleButton() {
    bool btnDown = (digitalRead(BTN_PIN) == LOW);
    if (btnDown && !btnWasDown) {
        btnPressStart = millis(); btnWasDown = true; longFired = false;
        return;
    }
    if (btnDown && btnWasDown && !longFired) {
        if (millis() - btnPressStart >= cfg.calPressMs) {
            longFired = true;
            doManualCalibration();
            updateCalBadge();
            updateStatusLine();
        }
        return;
    }
    if (!btnDown && btnWasDown) {
        uint32_t held = millis() - btnPressStart;
        btnWasDown = false;
        if (!longFired && held < cfg.shortPressMaxMs) {
            if (logActive) stopLog(); else startLog(TRIG_MANUAL);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BLE GATT server  (NimBLE-Arduino)
//
// Service:    A1710000-0000-0000-0000-000000000000
//
// Settings (Read + Write):
//   A1710001  alertStartFt      uint16 LE, feet          default 10  (REAL 3000)
//   A1710002  alertStopFt       uint16 LE, feet          default  2  (REAL  500)
//   A1710003  alertsEnabled     uint8  0=off 1=on
//   A1710004  vibrationEnabled  uint8  0=off 1=on
//   A1710005  autoLogEnabled    uint8  0=off 1=on
//   A1710006  units             uint8  0=ft/fps  1=m/m·s⁻¹
//
// Live data (Read + Notify, updated at 2 Hz):
//   A1710011  altitude          int16 LE, signed feet
//   A1710012  vspeed            int16 LE, fps×10  (divide by 10 for fps)
//   A1710013  battery           uint8  0–100 %
//   A1710014  calState          uint8  0=auto 1=manual 2=uncal
//   A1710015  logState          uint8  0=idle 1=recording
//   A1710016  logDuration       uint32 LE, seconds since log start
//   A1710017  rtcDatetime       UTF-8 "YYYY-MM-DD HH:MM:SS"
//
// Commands (Write-only — write any value to trigger):
//   A1710021  calibrate         uint8 — triggers manual ground calibration
//   A1710022  startLog          uint8 — starts jump log (manual trigger)
//   A1710023  stopLog           uint8 — stops jump log
//   A1710024  setRTC            UTF-8 "YYMMDDHHmmss"  e.g. "260529143000"
// ═══════════════════════════════════════════════════════════════════════════

#define BLE_SVC  "A1710000-0000-0000-0000-000000000000"
#define BLE_C001 "A1710001-0000-0000-0000-000000000000"
#define BLE_C002 "A1710002-0000-0000-0000-000000000000"
#define BLE_C003 "A1710003-0000-0000-0000-000000000000"
#define BLE_C004 "A1710004-0000-0000-0000-000000000000"
#define BLE_C005 "A1710005-0000-0000-0000-000000000000"
#define BLE_C006 "A1710006-0000-0000-0000-000000000000"
#define BLE_C011 "A1710011-0000-0000-0000-000000000000"
#define BLE_C012 "A1710012-0000-0000-0000-000000000000"
#define BLE_C013 "A1710013-0000-0000-0000-000000000000"
#define BLE_C014 "A1710014-0000-0000-0000-000000000000"
#define BLE_C015 "A1710015-0000-0000-0000-000000000000"
#define BLE_C016 "A1710016-0000-0000-0000-000000000000"
#define BLE_C017 "A1710017-0000-0000-0000-000000000000"
#define BLE_C021 "A1710021-0000-0000-0000-000000000000"
#define BLE_C022 "A1710022-0000-0000-0000-000000000000"
#define BLE_C023 "A1710023-0000-0000-0000-000000000000"
#define BLE_C024 "A1710024-0000-0000-0000-000000000000"

class AW_ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*)    override { bleConnected = true;  bleStateChanged = true; }
    void onDisconnect(NimBLEServer*) override {
        bleConnected = false; bleStateChanged = true;
        NimBLEDevice::startAdvertising();
    }
};
class CB_AlertStart : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *p) override {
        if (p->getValue().size() >= 2) {
            uint16_t v; memcpy(&v, p->getValue().data(), 2);
            cfg.alertStartFt = (float)v;
        }
    }
};
class CB_AlertStop : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *p) override {
        if (p->getValue().size() >= 2) {
            uint16_t v; memcpy(&v, p->getValue().data(), 2);
            cfg.alertStopFt = (float)v;
        }
    }
};
class CB_AlertsEn : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *p) override {
        if (p->getValue().size() >= 1) cfg.alertsEnabled = p->getValue()[0] != 0;
    }
};
class CB_VibEn : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *p) override {
        if (p->getValue().size() >= 1) cfg.vibrationEnabled = p->getValue()[0] != 0;
    }
};
class CB_AutoLogEn : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *p) override {
        if (p->getValue().size() >= 1) cfg.autoLogEnabled = p->getValue()[0] != 0;
    }
};
class CB_Units : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *p) override {
        if (p->getValue().size() >= 1) cfg.units = p->getValue()[0] & 1;
    }
};
class CB_Calibrate : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic*) override { bleCmd_calibrate = true; }
};
class CB_StartLog : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic*) override { bleCmd_startLog = true; }
};
class CB_StopLog : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic*) override { bleCmd_stopLog = true; }
};
class CB_SetRTC : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *p) override {
        auto v = p->getValue();
        if (v.size() >= 12) {
            bleRtcYr  = (v[0]-'0')*10 + (v[1]-'0');
            bleRtcMo  = (v[2]-'0')*10 + (v[3]-'0');
            bleRtcDay = (v[4]-'0')*10 + (v[5]-'0');
            bleRtcHr  = (v[6]-'0')*10 + (v[7]-'0');
            bleRtcMn  = (v[8]-'0')*10 + (v[9]-'0');
            bleRtcSec = (v[10]-'0')*10 + (v[11]-'0');
            bleCmd_setRTC = true;
        }
    }
};

static void initBLE() {
    NimBLEDevice::init("AltiWatch");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer  *srv = NimBLEDevice::createServer();
    srv->setCallbacks(new AW_ServerCB());
    NimBLEService *svc = srv->createService(BLE_SVC);

    // ── Settings (R/W) ────────────────────────────────────────────────────
    {
        auto *c = svc->createCharacteristic(BLE_C001, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        uint16_t v = (uint16_t)cfg.alertStartFt; c->setValue((uint8_t*)&v, 2);
        c->setCallbacks(new CB_AlertStart());
    }
    {
        auto *c = svc->createCharacteristic(BLE_C002, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        uint16_t v = (uint16_t)cfg.alertStopFt; c->setValue((uint8_t*)&v, 2);
        c->setCallbacks(new CB_AlertStop());
    }
    {
        auto *c = svc->createCharacteristic(BLE_C003, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        uint8_t v = cfg.alertsEnabled ? 1 : 0; c->setValue(&v, 1);
        c->setCallbacks(new CB_AlertsEn());
    }
    {
        auto *c = svc->createCharacteristic(BLE_C004, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        uint8_t v = cfg.vibrationEnabled ? 1 : 0; c->setValue(&v, 1);
        c->setCallbacks(new CB_VibEn());
    }
    {
        auto *c = svc->createCharacteristic(BLE_C005, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        uint8_t v = cfg.autoLogEnabled ? 1 : 0; c->setValue(&v, 1);
        c->setCallbacks(new CB_AutoLogEn());
    }
    {
        auto *c = svc->createCharacteristic(BLE_C006, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        uint8_t v = cfg.units; c->setValue(&v, 1);
        c->setCallbacks(new CB_Units());
    }

    // ── Live data (Read + Notify) ─────────────────────────────────────────
    pAltChar      = svc->createCharacteristic(BLE_C011, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pVsChar       = svc->createCharacteristic(BLE_C012, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pBatChar      = svc->createCharacteristic(BLE_C013, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pCalChar      = svc->createCharacteristic(BLE_C014, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pLogStateChar = svc->createCharacteristic(BLE_C015, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pLogDurChar   = svc->createCharacteristic(BLE_C016, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pRtcChar      = svc->createCharacteristic(BLE_C017, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // ── Commands (Write-only) ─────────────────────────────────────────────
    svc->createCharacteristic(BLE_C021, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CB_Calibrate());
    svc->createCharacteristic(BLE_C022, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CB_StartLog());
    svc->createCharacteristic(BLE_C023, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CB_StopLog());
    svc->createCharacteristic(BLE_C024, NIMBLE_PROPERTY::WRITE)->setCallbacks(new CB_SetRTC());

    svc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(svc->getUUID());
    adv->setScanResponse(true);
    adv->start();

    Serial.println("BLE advertising as 'AltiWatch'");
}

static void updateBLENotify(float aglFt, float vsFps) {
    if (!bleConnected) return;

    int16_t altVal = (int16_t)aglFt;
    pAltChar->setValue((uint8_t*)&altVal, 2);
    pAltChar->notify();

    int16_t vsVal = (int16_t)(vsFps * 10.0f);
    pVsChar->setValue((uint8_t*)&vsVal, 2);
    pVsChar->notify();

    int     batMv = analogReadMilliVolts(cfg.batAdcPin) * 2;
    uint8_t pct   = (uint8_t)constrain((int)(100.0f * (batMv - cfg.batMvEmpty) /
                                       (float)(cfg.batMvFull - cfg.batMvEmpty)), 0, 100);
    pBatChar->setValue(&pct, 1);
    pBatChar->notify();

    uint8_t calVal = (cal.source == CAL_AUTO) ? 0 : (cal.source == CAL_MANUAL) ? 1 : 2;
    pCalChar->setValue(&calVal, 1);
    pCalChar->notify();

    uint8_t logVal = logActive ? 1 : 0;
    pLogStateChar->setValue(&logVal, 1);
    pLogStateChar->notify();

    uint32_t durSec = logActive ? (millis() - logStartMs) / 1000 : 0;
    pLogDurChar->setValue((uint8_t*)&durSec, 4);
    pLogDurChar->notify();

    char rtcStr[20] = "RTC NOT SET";
    if (rtcIsValid()) {
        uint8_t regs[7];
        if (iicReadBuf(cfg.rtcAddr, 0x04, regs, 7))
            snprintf(rtcStr, sizeof(rtcStr), "20%02d-%02d-%02d %02d:%02d:%02d",
                     bcdDec(regs[6]), bcdDec(regs[5] & 0x1F), bcdDec(regs[3] & 0x3F),
                     bcdDec(regs[2] & 0x3F), bcdDec(regs[1] & 0x7F), bcdDec(regs[0] & 0x7F));
    }
    pRtcChar->setValue((uint8_t*)rtcStr, strlen(rtcStr));
    pRtcChar->notify();
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
    updateBLEBadge();
    updateCalBadge();
    updateBatDisplay();
    updateStatusLine();
    centeredText(Y_ALT_LBL, 1, C_DKGRAY, "AUTO-CAL IN 5s...");

    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);
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

    Serial.println("Warming up BMP390 IIR filter...");
    for (int i = 0; i < 8; i++) { bmp.performReading(); delay(100); }

    initIMU();

    if (!LittleFS.begin(true)) {
        Serial.println("WARNING: LittleFS mount failed — logging disabled");
    } else {
        Serial.printf("LittleFS: %u / %u bytes used\n",
                      LittleFS.usedBytes(), LittleFS.totalBytes());
    }

    initBLE();

    Serial.println("AltiWatch stage 9 — collecting boot samples...");
}

// ═══════════════════════════════════════════════════════════════════════════
// loop — 10 Hz samples, 2 Hz display
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    if (bootPhase == BOOT_RUNNING) handleButton();
    handleSerialInput();

    // ── Boot sampling (500 ms interval) ───────────────────────────────────
    if (bootPhase == BOOT_SAMPLING) {
        if (now - lastBootMs < 500) return;
        lastBootMs = now;
        if (!bmp.performReading()) return;
        handleBootSample(bmp.readAltitude(cfg.seaLevelHpa));
        return;
    }

    // Flash + buzz run at full loop() speed, independent of 10Hz sensor gate
    tickAlert(now);
    tickBuzz(now);

    // ── 10 Hz sample tick ─────────────────────────────────────────────────
    if (now - lastSampleMs < 100) return;
    lastSampleMs = now;

    if (!bmp.performReading()) return;
    float absAltM  = bmp.readAltitude(cfg.seaLevelHpa);
    float pressHpa = bmp.pressure / 100.0f;
    float tempC    = bmp.temperature;

    readIMU();

    // ── Vertical speed (8-slot ring, ft/s) ────────────────────────────────
    vsAlt[vsIdx]  = absAltM;
    vsTime[vsIdx] = now;
    vsIdx = (vsIdx + 1) % VS_SLOTS;
    if (vsIdx == 0) vsFull = true;

    float vsFps = 0.0f;
    if (vsFull || vsIdx >= 2) {
        uint8_t oldest = vsFull ? vsIdx : 0;
        uint8_t newest = (vsIdx == 0) ? VS_SLOTS - 1 : vsIdx - 1;
        float dAlt  = vsAlt[newest]  - vsAlt[oldest];
        float dTime = (vsTime[newest] - vsTime[oldest]) / 1000.0f;
        if (dTime > 0.05f) {
            vsFps = (dAlt / dTime) * 3.28084f;
            if (fabsf(vsFps) < cfg.vsDeadbandFps) vsFps = 0.0f;
        }
    }

    // ── AGL / ABS altitude ────────────────────────────────────────────────
    float dispAltM  = cal.isValid() ? (absAltM - cal.groundAltM) : absAltM;
    float dispAltFt = dispAltM * 3.28084f;

    cachedAltFt = dispAltFt;
    cachedVsFps = vsFps;

    // ── Build sample, push to rolling buffer ──────────────────────────────
    Sample s = { now, dispAltFt, vsFps, pressHpa, tempC, imuAx, imuAy, imuAz };
    rollPush(s);
    if (logActive) writeSampleToLog(s);

    if (cfg.autoLogEnabled) checkAutoTriggers(vsFps, dispAltFt, now);
    checkAlert(dispAltFt, vsFps, now);

    // ── BLE command dispatch (deferred from BLE-task callbacks) ───────────
    if (bleCmd_calibrate) {
        bleCmd_calibrate = false;
        doManualCalibration(); updateCalBadge(); updateStatusLine();
    }
    if (bleCmd_startLog)  { bleCmd_startLog  = false; startLog(TRIG_MANUAL); }
    if (bleCmd_stopLog)   { bleCmd_stopLog   = false; stopLog(); }
    if (bleCmd_setRTC) {
        bleCmd_setRTC = false;
        rtcSetTime(bleRtcYr, bleRtcMo, bleRtcDay, bleRtcHr, bleRtcMn, bleRtcSec);
        Serial.printf("RTC set: 20%02d-%02d-%02d %02d:%02d:%02d\n",
                      bleRtcYr, bleRtcMo, bleRtcDay, bleRtcHr, bleRtcMn, bleRtcSec);
    }

    // ── Serial ────────────────────────────────────────────────────────────
    Serial.printf("%s %+.0fft  %+.1ffps  %.2fhPa  %.1fC  [%.2f,%.2f,%.2f]g%s\n",
                  cal.source == CAL_AUTO   ? "[A]" :
                  cal.source == CAL_MANUAL ? "[M]" : "[?]",
                  dispAltFt, vsFps, pressHpa, tempC,
                  imuAx, imuAy, imuAz,
                  logActive ? "  REC" : "");

    // ── 2 Hz display + BLE notify (suppressed during alert) ───────────────
    if (alertState == ALERT_OFF && now - lastDisplayMs >= 500) {
        lastDisplayMs = now;
        updateAltDisplay(dispAltFt);
        updateVsDisplay(vsFps);
        updateBatDisplay();
        updateStatusLine();
        if (bleStateChanged) { bleStateChanged = false; updateBLEBadge(); }
        updateBLENotify(dispAltFt, vsFps);
    }
}
