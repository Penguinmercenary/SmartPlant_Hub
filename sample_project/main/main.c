/*
 * 智能花盆 P0 —— 第 4 步：自动浇水闭环
 *
 * 功能：每 2 秒读一次温湿度(SHT30) + 土壤湿度(ADS1115)。
 *       土壤干了 → GPIO47 拉高开泵浇 5 秒 → 关泵 → 等 30 秒渗透 → 继续监测。
 *       全程 OLED 显示 + 串口打印。
 *
 * 引脚（GOOUUU ESP32-S3-CAM V1.5）：
 *   SDA = GPIO1, SCL = GPIO14
 *   泵  = GPIO47（MOS 模块高电平触发）
 *   OLED=0x3C, SHT30=0x44, ADS1115=0x48
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

static const char *TAG = "plant";

/* ============ 引脚 & 地址 ============ */
#define I2C_PORT     I2C_NUM_0
#define PIN_SDA      GPIO_NUM_1
#define PIN_SCL      GPIO_NUM_14
#define I2C_FREQ_HZ  100000

#define PIN_PUMP     GPIO_NUM_47   /* MOS 模块 IO 脚，高电平 = 开泵 */

#define ADDR_SHT30   0x44
#define ADDR_OLED    0x3C
#define ADDR_ADS1115 0x48

/* ============ 浇水参数（一定要标定！） ============ */
/* 土壤湿度【高于】这个值(mV)就认为"干"，需要浇水。
 * 注意：你的传感器是反的——越干电压越高、越湿越低（空气223mV、水33mV）。
 * 标定：干土和湿土各读一次，阈值取中间偏干。当前 150mV 是估算值，按需微调。 */
#define SOIL_DRY_THRESHOLD_MV  150
#define WATER_DURATION_MS      5000    /* 每次浇水 5 秒 */
#define SOAK_TIME_MS           30000   /* 浇完等 30 秒，让水渗透再重新判断 */
#define MIN_WATER_INTERVAL_MS  300000  /* 两次自动浇水至少间隔 5 分钟，防过度浇 */

/* ============ I2C 句柄 ============ */
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t sht30_dev = NULL;
static i2c_master_dev_handle_t oled_dev  = NULL;
static i2c_master_dev_handle_t ads_dev   = NULL;

/* ============ OLED（SSD1306/SSD1315）============ */
#define OLED_W         128
#define OLED_H         64
#define OLED_BUF_SIZE  (OLED_W * OLED_H / 8)

static uint8_t fb[OLED_BUF_SIZE];

static void oled_clear(void) { memset(fb, 0, sizeof(fb)); }

static void oled_flush(void)
{
    uint8_t setup[] = {0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07};
    i2c_master_transmit(oled_dev, setup, sizeof(setup), 100);
    for (int i = 0; i < OLED_BUF_SIZE; i += 128) {
        uint8_t buf[129];
        buf[0] = 0x40;
        memcpy(buf + 1, fb + i, 128);
        i2c_master_transmit(oled_dev, buf, sizeof(buf), 100);
    }
}

static void oled_init(void)
{
    const uint8_t init[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0xAF,
    };
    uint8_t buf[1 + sizeof(init)];
    buf[0] = 0x00;
    memcpy(buf + 1, init, sizeof(init));
    i2c_master_transmit(oled_dev, buf, sizeof(buf), 100);
    oled_clear();
    oled_flush();
}

static void oled_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    if (on) fb[x + (y / 8) * OLED_W] |=  (1 << (y & 7));
    else    fb[x + (y / 8) * OLED_W] &= ~(1 << (y & 7));
}

/* ============ 5x7 字体（列优先，低位在上） ============ */
static const uint8_t *glyph(char c)
{
    static const uint8_t g0[5]  = {0x3E,0x51,0x49,0x45,0x3E}; /* 0 */
    static const uint8_t g1[5]  = {0x00,0x42,0x7F,0x40,0x00}; /* 1 */
    static const uint8_t g2[5]  = {0x42,0x61,0x51,0x49,0x46}; /* 2 */
    static const uint8_t g3[5]  = {0x21,0x41,0x45,0x4B,0x31}; /* 3 */
    static const uint8_t g4[5]  = {0x18,0x14,0x12,0x7F,0x10}; /* 4 */
    static const uint8_t g5[5]  = {0x27,0x45,0x45,0x45,0x39}; /* 5 */
    static const uint8_t g6[5]  = {0x3C,0x4A,0x49,0x49,0x30}; /* 6 */
    static const uint8_t g7[5]  = {0x01,0x71,0x09,0x05,0x03}; /* 7 */
    static const uint8_t g8[5]  = {0x36,0x49,0x49,0x49,0x36}; /* 8 */
    static const uint8_t g9[5]  = {0x06,0x49,0x49,0x29,0x1E}; /* 9 */
    static const uint8_t gA[5]  = {0x7E,0x11,0x11,0x11,0x7E}; /* A */
    static const uint8_t gC[5]  = {0x3E,0x41,0x41,0x41,0x22}; /* C */
    static const uint8_t gD[5]  = {0x7F,0x41,0x41,0x22,0x1C}; /* D */
    static const uint8_t gE[5]  = {0x7F,0x49,0x49,0x49,0x41}; /* E */
    static const uint8_t gG[5]  = {0x3E,0x41,0x41,0x51,0x73}; /* G */
    static const uint8_t gH[5]  = {0x7F,0x08,0x08,0x08,0x7F}; /* H */
    static const uint8_t gI[5]  = {0x00,0x41,0x7F,0x41,0x00}; /* I */
    static const uint8_t gK[5]  = {0x7F,0x08,0x14,0x22,0x41}; /* K */
    static const uint8_t gL[5]  = {0x7F,0x40,0x40,0x40,0x40}; /* L */
    static const uint8_t gM[5]  = {0x7F,0x02,0x0C,0x02,0x7F}; /* M */
    static const uint8_t gN[5]  = {0x7F,0x04,0x08,0x10,0x7F}; /* N */
    static const uint8_t gO[5]  = {0x3E,0x41,0x41,0x41,0x3E}; /* O */
    static const uint8_t gP[5]  = {0x7F,0x09,0x09,0x09,0x06}; /* P */
    static const uint8_t gR[5]  = {0x7F,0x09,0x19,0x29,0x46}; /* R */
    static const uint8_t gS[5]  = {0x26,0x49,0x49,0x49,0x36}; /* S */
    static const uint8_t gT[5]  = {0x01,0x01,0x7F,0x01,0x01}; /* T */
    static const uint8_t gU[5]  = {0x3F,0x40,0x40,0x40,0x3F}; /* U */
    static const uint8_t gW[5]  = {0x3F,0x40,0x38,0x40,0x3F}; /* W */
    static const uint8_t gY[5]  = {0x07,0x08,0x70,0x08,0x07}; /* Y */
    static const uint8_t gm[5]  = {0x7C,0x04,0x78,0x04,0x78}; /* m */
    static const uint8_t gV[5]  = {0x07,0x18,0x60,0x18,0x07}; /* V */
    static const uint8_t gsp[5] = {0x00,0x00,0x00,0x00,0x00}; /* 空格 */
    static const uint8_t gdot[5]= {0x00,0x60,0x60,0x00,0x00}; /* . */
    static const uint8_t gpct[5]= {0x63,0x13,0x08,0x64,0x63}; /* % */

    switch (c) {
        case '0': return g0;  case '1': return g1;  case '2': return g2;
        case '3': return g3;  case '4': return g4;  case '5': return g5;
        case '6': return g6;  case '7': return g7;  case '8': return g8;
        case '9': return g9;  case 'A': return gA;  case 'C': return gC;
        case 'D': return gD;  case 'E': return gE;  case 'G': return gG;
        case 'H': return gH;  case 'I': return gI;  case 'K': return gK;
        case 'L': return gL;  case 'M': return gM;  case 'N': return gN;
        case 'O': return gO;  case 'P': return gP;  case 'R': return gR;
        case 'S': return gS;  case 'T': return gT;  case 'U': return gU;
        case 'W': return gW;  case 'Y': return gY;  case 'm': return gm;
        case 'V': return gV;  case '.': return gdot; case '%': return gpct;
        default:  return gsp;
    }
}

static void oled_char(int col, int row, char c)
{
    const uint8_t *g = glyph(c);
    int x0 = col * 6, y0 = row * 8;
    for (int cx = 0; cx < 5; cx++)
        for (int ry = 0; ry < 7; ry++)
            if (g[cx] & (1 << ry))
                oled_set_pixel(x0 + cx, y0 + ry, true);
}

static void oled_string(int col, int row, const char *s)
{
    while (*s) oled_char(col++, row, *s++);
}

/* ============ SHT30 ============ */
static esp_err_t sht30_read(float *temp, float *hum)
{
    uint8_t cmd[2] = {0x2C, 0x06};
    esp_err_t err = i2c_master_transmit(sht30_dev, cmd, 2, 100);
    if (err != ESP_OK) return err;
    uint8_t data[6] = {0};
    err = i2c_master_receive(sht30_dev, data, 6, 100);
    if (err != ESP_OK) return err;
    uint16_t t_raw = (data[0] << 8) | data[1];
    uint16_t h_raw = (data[3] << 8) | data[4];
    *temp = -45.0f + 175.0f * (float)t_raw / 65535.0f;
    *hum  = 100.0f * (float)h_raw / 65535.0f;
    return ESP_OK;
}

/* ============ ADS1115 ============ */
static esp_err_t ads1115_read_raw(int16_t *out)
{
    uint8_t config[3] = {0x01, 0x83, 0x80};   /* 单次，AIN0-GND，±4.096V，128SPS */
    esp_err_t err = i2c_master_transmit(ads_dev, config, 3, 100);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(15));
    uint8_t reg = 0x00;
    err = i2c_master_transmit(ads_dev, &reg, 1, 100);
    if (err != ESP_OK) return err;
    uint8_t data[2] = {0};
    err = i2c_master_receive(ads_dev, data, 2, 100);
    if (err != ESP_OK) return err;
    *out = (int16_t)((data[0] << 8) | data[1]);
    return ESP_OK;
}

/* ============ 浇水状态机 ============ */
typedef enum { WS_IDLE, WS_WATERING, WS_SOAKING } water_state_t;
static water_state_t ws = WS_IDLE;
static int64_t ws_time = 0;          /* 当前状态起始时间(ms) */
static int64_t last_water_ms = -MIN_WATER_INTERVAL_MS;  /* 负数=还没浇过水，开机可立即浇第一次 */
static bool pump_on = false;

/* ============ 主程序 ============ */
void app_main(void)
{
    /* I2C 总线 + 三设备 */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR_SHT30,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &sht30_dev));
    dev_cfg.device_address = ADDR_OLED;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &oled_dev));
    dev_cfg.device_address = ADDR_ADS1115;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &ads_dev));

    oled_init();

    /* 泵 GPIO：输出，默认关（低电平） */
    gpio_config_t pump_cfg = {
        .pin_bit_mask = (1ULL << PIN_PUMP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pump_cfg));
    gpio_set_level(PIN_PUMP, 0);

    /* 开机扫描一次 I2C 总线，打印所有设备（排查接线用） */
    ESP_LOGI(TAG, "开机扫描 I2C 设备...");
    for (int addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(bus_handle, (uint16_t)addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  发现设备 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "扫描完成，期望看到 0x3C(OLED) 0x44(SHT30) 0x48(ADS1115)");

    ESP_LOGI(TAG, "P0 auto-water loop start, threshold=%d mV", SOIL_DRY_THRESHOLD_MV);

    while (1) {
        /* 1. 读传感器 */
        float t = 0.0f, h = 0.0f;
        esp_err_t err_sht = sht30_read(&t, &h);
        int16_t soil_raw = 0;
        esp_err_t err_soil = ads1115_read_raw(&soil_raw);
        int soil_mv = soil_raw * 4096 / 32768;   /* ±4.096V -> mV */

        /* 1.5 I2C 故障恢复：连续多次都失败 → 总线卡住了，重置一次 */
        static int i2c_fail_count = 0;
        if (err_sht != ESP_OK && err_soil != ESP_OK) {
            if (++i2c_fail_count >= 5) {
                ESP_LOGW(TAG, "I2C 连续失败，重置总线恢复");
                i2c_master_bus_reset(bus_handle);
                i2c_fail_count = 0;
            }
        } else {
            i2c_fail_count = 0;
        }

        /* 2. 浇水状态机 */
        int64_t now_ms = esp_timer_get_time() / 1000;
        const char *status = "OK";

        switch (ws) {
            case WS_IDLE:
                /* 土壤确实干 + 距上次浇水够久 → 开泵 */
                if (err_soil == ESP_OK && soil_mv > SOIL_DRY_THRESHOLD_MV &&
                    (now_ms - last_water_ms) >= MIN_WATER_INTERVAL_MS) {
                    gpio_set_level(PIN_PUMP, 1);
                    pump_on = true;
                    ws = WS_WATERING;
                    ws_time = now_ms;
                    ESP_LOGI(TAG, "土壤干(%d mV) -> 浇水 %d 秒", soil_mv, WATER_DURATION_MS / 1000);
                }
                if (err_soil == ESP_OK && soil_mv > SOIL_DRY_THRESHOLD_MV)
                    status = "DRY";
                break;

            case WS_WATERING:
                if (now_ms - ws_time >= WATER_DURATION_MS) {
                    gpio_set_level(PIN_PUMP, 0);
                    pump_on = false;
                    last_water_ms = now_ms;
                    ws = WS_SOAKING;
                    ws_time = now_ms;
                    ESP_LOGI(TAG, "浇水完成，等待渗透 30 秒");
                }
                status = "WATERING";
                break;

            case WS_SOAKING:
                if (now_ms - ws_time >= SOAK_TIME_MS) {
                    ws = WS_IDLE;
                    ESP_LOGI(TAG, "回到待机");
                }
                status = "SOAKING";
                break;
        }

        /* 3. OLED 显示 */
        oled_clear();
        char line[22];
        if (err_sht == ESP_OK) {
            int t10 = (int)(t * 10 + 0.5f), h10 = (int)(h * 10 + 0.5f);
            snprintf(line, sizeof(line), "TEMP %d.%d C", t10 / 10, t10 % 10);
            oled_string(0, 0, line);
            snprintf(line, sizeof(line), "HUM  %d.%d %%", h10 / 10, h10 % 10);
            oled_string(0, 1, line);
        } else {
            oled_string(0, 1, "SHT30 ERR");
        }

        if (err_soil == ESP_OK) {
            snprintf(line, sizeof(line), "SOIL %d mV", soil_mv);
            oled_string(0, 2, line);
            ESP_LOGI(TAG, "soil=%d mV (阈值 %d) 状态=%s 泵=%s",
                     soil_mv, SOIL_DRY_THRESHOLD_MV, status, pump_on ? "ON" : "OFF");
        } else {
            oled_string(0, 2, "SOIL ERR");
        }
        oled_string(0, 3, status);

        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
