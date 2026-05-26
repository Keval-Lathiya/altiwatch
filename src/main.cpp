#include <Arduino.h>

extern "C" {
    #include "esp_lcd_panel_rgb.h"
    #include "esp_lcd_panel_ops.h"
    #include "esp_heap_caps.h"
    #include "driver/i2c.h"
}

#include <lvgl.h>

// ── Backlight ─────────────────────────────────────────────────────────
#define LCD_BL 6

// ── ST7701 3-wire SPI init (GPIO1=MOSI, GPIO2=SCK) ───────────────────
#define LCD_SPI_MOSI 1
#define LCD_SPI_SCK  2

// ── PCA9554 I/O expander (I2C on GPIO15/GPIO7, addr 0x20) ────────────
// Uses bit-bang I2C to avoid hardware Wire library hang after GPIO2=LOW.
#define I2C_SDA      15
#define I2C_SCL       7
#define PCA9554_ADDR 0x20

#define PCA_LCD_RST (1 << 0)   // EXIO1 → LCD reset
#define PCA_TP_RST  (1 << 1)   // EXIO2 → touch reset
#define PCA_LCD_CS  (1 << 2)   // EXIO3 → ST7701 SPI chip-select

// ── RGB parallel interface pins ───────────────────────────────────────
#define LCD_PCLK  41
#define LCD_DE    40
#define LCD_VSYNC 39
#define LCD_HSYNC 38

// R[1-5] on board = R[0-4] in protocol
#define LCD_R0 46
#define LCD_R1  3
#define LCD_R2  8
#define LCD_R3 18
#define LCD_R4 17

// G[0-5]
#define LCD_G0 14
#define LCD_G1 13
#define LCD_G2 12
#define LCD_G3 11
#define LCD_G4 10
#define LCD_G5  9

// B[1-5] on board = B[0-4] in protocol
#define LCD_B0  5
#define LCD_B1 45
#define LCD_B2 48
#define LCD_B3 47
#define LCD_B4 21

// ── PCA9554 driver — ESP-IDF I2C master ──────────────────────────────
// Uses driver/i2c.h directly; avoids the Arduino Wire library which hangs
// after GPIO2 (SPI SCK) is driven LOW on this board.
static uint8_t _pca_out = 0xFF;

static void pca9554_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_master_write_to_device(I2C_NUM_0, PCA9554_ADDR, buf, 2,
                               pdMS_TO_TICKS(50));
}
static void pca9554_write(uint8_t val) {
    pca9554_write_reg(0x01, val);
    _pca_out = val;
}
static void pca9554_pin(uint8_t mask, bool hi) {
    if (hi) _pca_out |=  mask;
    else    _pca_out &= ~mask;
    pca9554_write(_pca_out);
}
static void pca9554_init() {
    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = I2C_SDA;
    cfg.scl_io_num       = I2C_SCL;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 400000;
    i2c_param_config(I2C_NUM_0, &cfg);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    pca9554_write_reg(0x03, 0x00);  // config register: all pins → output
    pca9554_write(0xFF);             // all pins high
}

// ── ST7701 3-wire SPI ─────────────────────────────────────────────────
// CS is asserted once (via I2C) BEFORE the SPI GPIO pins are configured,
// then held low for the entire init sequence.  The ST7701 datasheet
// explicitly states CSX may be held low across multiple 9-bit words; the
// chip uses the SCK count to delineate each word.  This avoids any I2C
// access while GPIO2 (SCK) is toggling, which confuses the IDF I2C driver.
static void spi_write9(uint16_t word9) {
    for (int i = 8; i >= 0; i--) {
        digitalWrite(LCD_SPI_MOSI, (word9 >> i) & 1 ? HIGH : LOW);
        delayMicroseconds(1);
        digitalWrite(LCD_SPI_SCK, HIGH);
        delayMicroseconds(1);
        digitalWrite(LCD_SPI_SCK, LOW);
        delayMicroseconds(1);
    }
}

#define ST_CMD(c)  spi_write9(c)
#define ST_DAT(d)  spi_write9(0x100 | (d))

static void st7701_init() {
    ST_CMD(0xFF); ST_DAT(0x77); ST_DAT(0x01); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x13);
    ST_CMD(0xEF); ST_DAT(0x08);

    ST_CMD(0xFF); ST_DAT(0x77); ST_DAT(0x01); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x10);
    ST_CMD(0xC0); ST_DAT(0x3B); ST_DAT(0x00);
    ST_CMD(0xC1); ST_DAT(0x10); ST_DAT(0x0C);
    ST_CMD(0xC2); ST_DAT(0x07); ST_DAT(0x0A);
    ST_CMD(0xC7); ST_DAT(0x00);
    ST_CMD(0xCC); ST_DAT(0x10);
    ST_CMD(0xCD); ST_DAT(0x08);

    ST_CMD(0xB0);
    ST_DAT(0x05); ST_DAT(0x12); ST_DAT(0x98); ST_DAT(0x0E);
    ST_DAT(0x0F); ST_DAT(0x07); ST_DAT(0x07); ST_DAT(0x09);
    ST_DAT(0x09); ST_DAT(0x23); ST_DAT(0x05); ST_DAT(0x52);
    ST_DAT(0x0F); ST_DAT(0x67); ST_DAT(0x2C); ST_DAT(0x11);

    ST_CMD(0xB1);
    ST_DAT(0x0B); ST_DAT(0x11); ST_DAT(0x97); ST_DAT(0x0C);
    ST_DAT(0x12); ST_DAT(0x06); ST_DAT(0x06); ST_DAT(0x08);
    ST_DAT(0x08); ST_DAT(0x22); ST_DAT(0x03); ST_DAT(0x51);
    ST_DAT(0x11); ST_DAT(0x66); ST_DAT(0x2B); ST_DAT(0x0F);

    ST_CMD(0xFF); ST_DAT(0x77); ST_DAT(0x01); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x11);
    ST_CMD(0xB0); ST_DAT(0x5D);
    ST_CMD(0xB1); ST_DAT(0x3E);
    ST_CMD(0xB2); ST_DAT(0x81);
    ST_CMD(0xB3); ST_DAT(0x80);
    ST_CMD(0xB5); ST_DAT(0x4E);
    ST_CMD(0xB7); ST_DAT(0x85);
    ST_CMD(0xB8); ST_DAT(0x20);
    ST_CMD(0xC1); ST_DAT(0x78);
    ST_CMD(0xC2); ST_DAT(0x78);
    ST_CMD(0xD0); ST_DAT(0x88);

    ST_CMD(0xE0); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x02);
    ST_CMD(0xE1); ST_DAT(0x06); ST_DAT(0x30); ST_DAT(0x08); ST_DAT(0x30);
                  ST_DAT(0x05); ST_DAT(0x30); ST_DAT(0x07); ST_DAT(0x30);
                  ST_DAT(0x00); ST_DAT(0x33); ST_DAT(0x33);
    ST_CMD(0xE2); ST_DAT(0x11); ST_DAT(0x11); ST_DAT(0x33); ST_DAT(0x33);
                  ST_DAT(0xF4); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x00);
                  ST_DAT(0xF4); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x00);
    ST_CMD(0xE3); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x11); ST_DAT(0x11);
    ST_CMD(0xE4); ST_DAT(0x44); ST_DAT(0x44);
    ST_CMD(0xE5); ST_DAT(0x0D); ST_DAT(0xF5); ST_DAT(0x30); ST_DAT(0xF0);
                  ST_DAT(0x0F); ST_DAT(0xF7); ST_DAT(0x30); ST_DAT(0xF0);
                  ST_DAT(0x09); ST_DAT(0xF1); ST_DAT(0x30); ST_DAT(0xF0);
                  ST_DAT(0x0B); ST_DAT(0xF3); ST_DAT(0x30); ST_DAT(0xF0);
    ST_CMD(0xE6); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x11); ST_DAT(0x11);
    ST_CMD(0xE7); ST_DAT(0x44); ST_DAT(0x44);
    ST_CMD(0xE8); ST_DAT(0x0C); ST_DAT(0xF4); ST_DAT(0x30); ST_DAT(0xF0);
                  ST_DAT(0x0E); ST_DAT(0xF6); ST_DAT(0x30); ST_DAT(0xF0);
                  ST_DAT(0x08); ST_DAT(0xF0); ST_DAT(0x30); ST_DAT(0xF0);
                  ST_DAT(0x0A); ST_DAT(0xF2); ST_DAT(0x30); ST_DAT(0xF0);
    ST_CMD(0xE9); ST_DAT(0x36); ST_DAT(0x01);
    ST_CMD(0xEB); ST_DAT(0x00); ST_DAT(0x01); ST_DAT(0xE4); ST_DAT(0xE4);
                  ST_DAT(0x44); ST_DAT(0x88); ST_DAT(0x40);
    ST_CMD(0xED); ST_DAT(0xFF); ST_DAT(0x10); ST_DAT(0xAF); ST_DAT(0x76);
                  ST_DAT(0x54); ST_DAT(0x2B); ST_DAT(0xCF); ST_DAT(0xFF);
                  ST_DAT(0xFF); ST_DAT(0xFC); ST_DAT(0xB2); ST_DAT(0x45);
                  ST_DAT(0x67); ST_DAT(0xFA); ST_DAT(0x01); ST_DAT(0xFF);
    ST_CMD(0xEF); ST_DAT(0x08); ST_DAT(0x08); ST_DAT(0x08); ST_DAT(0x45);
                  ST_DAT(0x3F); ST_DAT(0x54);

    ST_CMD(0xFF); ST_DAT(0x77); ST_DAT(0x01); ST_DAT(0x00); ST_DAT(0x00); ST_DAT(0x00);
    delay(120);
    ST_CMD(0x11);
    delay(120);
    ST_CMD(0x3A); ST_DAT(0x55);  // 16-bit colour (RGB565)
    ST_CMD(0x36); ST_DAT(0x00);
    ST_CMD(0x35); ST_DAT(0x00);
    ST_CMD(0x20);
    delay(120);
    ST_CMD(0x29);
}

// ── LVGL flush callback ───────────────────────────────────────────────
static esp_lcd_panel_handle_t s_panel = NULL;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_draw_bitmap(s_panel,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              px_map);
    lv_display_flush_ready(disp);
}

// ── setup / loop ──────────────────────────────────────────────────────
void setup() {
    // Backlight on immediately so we know the BL circuit works
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    Serial.begin(115200);
    delay(500);
    Serial.println("AltiWatch Stage 1 starting...");

    // PCA9554 I/O expander — verify I2C is alive with a register readback
    pca9554_init();
    {
        uint8_t reg = 0x01, rbk = 0xFF;
        esp_err_t e = i2c_master_write_read_device(I2C_NUM_0, PCA9554_ADDR,
                                                   &reg, 1, &rbk, 1,
                                                   pdMS_TO_TICKS(50));
        Serial.printf("PCA9554 init: err=%d  output reg readback=0x%02X (expect 0xFF)\n",
                      e, rbk);
    }

    // ── All I2C operations BEFORE any SPI GPIO is configured ─────────────
    // The ESP-IDF I2C driver times out if GPIO2 (SPI SCK) is toggling; keep
    // the two phases completely separate.

    // LCD hard reset
    pca9554_pin(PCA_LCD_RST, false);
    delay(100);
    pca9554_pin(PCA_LCD_RST, true);
    delay(120);
    Serial.println("LCD reset OK");

    // Assert CS now, while I2C is still clean.  CS stays low for the entire
    // SPI init; the ST7701 is fine with continuous CSX across writes.
    pca9554_pin(PCA_LCD_CS, false);
    Serial.println("CS asserted");

    // ── SPI GPIO setup + ST7701 init (no I2C from here) ─────────────────
    pinMode(LCD_SPI_MOSI, OUTPUT);
    pinMode(LCD_SPI_SCK,  OUTPUT);
    digitalWrite(LCD_SPI_SCK, LOW);

    st7701_init();       // all 9-bit words sent with CS already held low
    // CS remains low permanently — irrelevant once RGB parallel mode starts
    Serial.println("ST7701 init OK");

    // RGB panel config (ESP-IDF 4.4 API)
    esp_lcd_rgb_panel_config_t panel_cfg = {};
    panel_cfg.clk_src                    = LCD_CLK_SRC_PLL160M;
    panel_cfg.timings.pclk_hz            = 16 * 1000000;
    panel_cfg.timings.h_res              = 480;
    panel_cfg.timings.v_res              = 480;
    panel_cfg.timings.hsync_pulse_width  = 8;
    panel_cfg.timings.hsync_back_porch   = 10;
    panel_cfg.timings.hsync_front_porch  = 50;
    panel_cfg.timings.vsync_pulse_width  = 3;
    panel_cfg.timings.vsync_back_porch   = 8;
    panel_cfg.timings.vsync_front_porch  = 8;
    panel_cfg.data_width                 = 16;
    panel_cfg.hsync_gpio_num             = LCD_HSYNC;
    panel_cfg.vsync_gpio_num             = LCD_VSYNC;
    panel_cfg.de_gpio_num                = LCD_DE;
    panel_cfg.pclk_gpio_num              = LCD_PCLK;
    panel_cfg.disp_gpio_num              = -1;
    // D[4:0]=B[4:0], D[10:5]=G[5:0], D[15:11]=R[4:0]  →  RGB565
    panel_cfg.data_gpio_nums[0]  = LCD_B0;
    panel_cfg.data_gpio_nums[1]  = LCD_B1;
    panel_cfg.data_gpio_nums[2]  = LCD_B2;
    panel_cfg.data_gpio_nums[3]  = LCD_B3;
    panel_cfg.data_gpio_nums[4]  = LCD_B4;
    panel_cfg.data_gpio_nums[5]  = LCD_G0;
    panel_cfg.data_gpio_nums[6]  = LCD_G1;
    panel_cfg.data_gpio_nums[7]  = LCD_G2;
    panel_cfg.data_gpio_nums[8]  = LCD_G3;
    panel_cfg.data_gpio_nums[9]  = LCD_G4;
    panel_cfg.data_gpio_nums[10] = LCD_G5;
    panel_cfg.data_gpio_nums[11] = LCD_R0;
    panel_cfg.data_gpio_nums[12] = LCD_R1;
    panel_cfg.data_gpio_nums[13] = LCD_R2;
    panel_cfg.data_gpio_nums[14] = LCD_R3;
    panel_cfg.data_gpio_nums[15] = LCD_R4;
    panel_cfg.psram_trans_align       = 64;   // required for PSRAM DMA
    panel_cfg.flags.fb_in_psram      = 1;
    panel_cfg.flags.disp_active_low  = 1;

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    Serial.println("RGB panel OK");

    // Hardware test: alternate red/blue fills for 10 s — impossible to miss
    {
        static uint16_t test_line[480 * 10];
        const uint16_t colors[] = { 0xF800, 0x001F };  // red, blue (RGB565)
        for (int c = 0; c < 10; c++) {
            uint16_t col = colors[c & 1];
            for (int i = 0; i < 480 * 10; i++) test_line[i] = col;
            for (int y = 0; y < 480; y += 10)
                esp_lcd_panel_draw_bitmap(s_panel, 0, y, 480, y + 10, test_line);
            Serial.printf("Color test %d: 0x%04X\n", c, col);
            delay(1000);
        }
        Serial.println("Color test done");
    }

    // LVGL init
    lv_init();
    lv_tick_set_cb([]() -> uint32_t { return (uint32_t)millis(); });

    lv_display_t *disp = lv_display_create(480, 480);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

    // Draw buffer in SRAM (30 lines = 28 KB)
    static uint8_t lvgl_buf[480 * 30 * 2];
    lv_display_set_buffers(disp, lvgl_buf, NULL, sizeof(lvgl_buf),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    // Build Stage 1 UI
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_t *lbl1 = lv_label_create(scr);
    lv_label_set_text(lbl1, "AltiWatch");
    lv_obj_set_style_text_color(lbl1, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl1, &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_align(lbl1, LV_ALIGN_CENTER, 0, -30);

    lv_obj_t *lbl2 = lv_label_create(scr);
    lv_label_set_text(lbl2, "Stage 1 - Display OK");
    lv_obj_set_style_text_color(lbl2, lv_color_hex(0x00FF00), LV_PART_MAIN);
    lv_obj_align(lbl2, LV_ALIGN_CENTER, 0, 20);

    lv_refr_now(disp);
    Serial.println("Stage 1 display complete");
}

void loop() {
    lv_task_handler();
    delay(5);
}
