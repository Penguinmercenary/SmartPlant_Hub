/*
 * 智能花盆 P0 —— 第 3 步：OLED + SHT30 温湿度 + ADS1115 土壤湿度
 *
 * 功能：每 2 秒读一次 SHT30（温湿度）和 ADS1115（土壤湿度，经 AIN0），
 *       显示在 OLED 上，并打印到串口。
 *
 * 引脚（GOOUUU ESP32-S3-CAM V1.5）：
 *   SDA = GPIO1
 *   SCL = GPIO14
 *   OLED 地址   = 0x3C
 *   SHT30 地址  = 0x44
 *   ADS1115 地址 = 0x48（ADDR 脚接 GND）
 *
 * 土壤湿度：电容式传感器 -> ADS1115 的 AIN0（A0）。
 *   数值单位是毫伏 mV：越干数值越低、越湿越高（下一步标定阈值）。
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/i2c_master.h"

static const char *TAG = "plant";

/* ============ 引脚 & 地址 ============ */
#define I2C_PORT     I2C_NUM_0
#define PIN_SDA      GPIO_NUM_1
#define PIN_SCL      GPIO_NUM_14
#define I2C_FREQ_HZ  100000

#define ADDR_SHT30   0x44
#define ADDR_OLED    0x3C
#define ADDR_ADS1115 0x48

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t sht30_dev = NULL;
static i2c_master_dev_handle_t oled_dev  = NULL;
static i2c_master_dev_handle_t ads_dev   = NULL;

/* ============ OLED（SSD1306/SSD1315）============ */
#define OLED_W         128
#define OLED_H         64
#define OLED_BUF_SIZE  (OLED_W * OLED_H / 8)   /* 1024 字节显存 */

static uint8_t fb[OLED_BUF_SIZE];

/* 清空显存 */
static void oled_clear(void)
{
    memset(fb, 0, sizeof(fb));
}

/* 把显存整体刷到屏幕 */
static void oled_flush(void)
{
    /* 设列范围 0~127、页范围 0~7 */
    uint8_t setup[] = {0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07};
    i2c_master_transmit(oled_dev, setup, sizeof(setup), 100);
    /* 分 8 块发送显存 */
    for (int i = 0; i < OLED_BUF_SIZE; i += 128) {
        uint8_t buf[129];
        buf[0] = 0x40;              /* 0x40 = 后面是数据 */
        memcpy(buf + 1, fb + i, 128);
        i2c_master_transmit(oled_dev, buf, sizeof(buf), 100);
    }
}

static void oled_init(void)
{
    /* 标准 SSD1306 128x64 初始化（SSD1315 也基本兼容） */
    const uint8_t init[] = {
        0xAE,           /* 关显示 */
        0xD5, 0x80,     /* 时钟分频 */
        0xA8, 0x3F,     /* 多路复用 = 64 行 */
        0xD3, 0x00,     /* 显示偏移 = 0 */
        0x40,           /* 起始行 = 0 */
        0x8D, 0x14,     /* 开电荷泵 */
        0x20, 0x00,     /* 内存寻址模式 = 水平 */
        0xA1,           /* 段重映射 */
        0xC8,           /* COM 扫描方向 */
        0xDA, 0x12,     /* COM 引脚配置 */
        0x81, 0xCF,     /* 对比度 */
        0xD9, 0xF1,     /* 预充电 */
        0xDB, 0x40,     /* VCOM 检测 */
        0xA4,           /* 显示 RAM 内容 */
        0xA6,           /* 正常显示（不反色） */
        0xAF,           /* 开显示 */
    };
    uint8_t buf[1 + sizeof(init)];
    buf[0] = 0x00;              /* 0x00 = 后面是命令 */
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
    static const uint8_t gT[5]  = {0x01,0x01,0x7F,0x01,0x01}; /* T */
    static const uint8_t gE[5]  = {0x7F,0x49,0x49,0x49,0x41}; /* E */
    static const uint8_t gM[5]  = {0x7F,0x02,0x0C,0x02,0x7F}; /* M */
    static const uint8_t gP[5]  = {0x7F,0x09,0x09,0x09,0x06}; /* P */
    static const uint8_t gH[5]  = {0x7F,0x08,0x08,0x08,0x7F}; /* H */
    static const uint8_t gU[5]  = {0x3F,0x40,0x40,0x40,0x3F}; /* U */
    static const uint8_t gC[5]  = {0x3E,0x41,0x41,0x41,0x22}; /* C */
    static const uint8_t gS[5]  = {0x26,0x49,0x49,0x49,0x36}; /* S */
    static const uint8_t gO[5]  = {0x3E,0x41,0x41,0x41,0x3E}; /* O */
    static const uint8_t gI[5]  = {0x00,0x41,0x7F,0x41,0x00}; /* I */
    static const uint8_t gL[5]  = {0x7F,0x40,0x40,0x40,0x40}; /* L */
    static const uint8_t gm[5]  = {0x7C,0x04,0x78,0x04,0x78}; /* m */
    static const uint8_t gV[5]  = {0x07,0x18,0x60,0x18,0x07}; /* V */
    static const uint8_t gsp[5] = {0x00,0x00,0x00,0x00,0x00}; /* 空格 */
    static const uint8_t gdot[5]= {0x00,0x60,0x60,0x00,0x00}; /* . */
    static const uint8_t gcol[5]= {0x00,0x36,0x36,0x00,0x00}; /* : */
    static const uint8_t gpct[5]= {0x63,0x13,0x08,0x64,0x63}; /* % */

    switch (c) {
        case '0': return g0;
        case '1': return g1;
        case '2': return g2;
        case '3': return g3;
        case '4': return g4;
        case '5': return g5;
        case '6': return g6;
        case '7': return g7;
        case '8': return g8;
        case '9': return g9;
        case 'T': return gT;
        case 'E': return gE;
        case 'M': return gM;
        case 'P': return gP;
        case 'H': return gH;
        case 'U': return gU;
        case 'C': return gC;
        case 'S': return gS;
        case 'O': return gO;
        case 'I': return gI;
        case 'L': return gL;
        case 'm': return gm;
        case 'V': return gV;
        case '.': return gdot;
        case ':': return gcol;
        case '%': return gpct;
        default:  return gsp;   /* 不认识的字符当空格 */
    }
}

/* 在 (col, row) 画一个字符，col 每格 6px，row 每格 8px */
static void oled_char(int col, int row, char c)
{
    const uint8_t *g = glyph(c);
    int x0 = col * 6;
    int y0 = row * 8;
    for (int cx = 0; cx < 5; cx++) {
        for (int ry = 0; ry < 7; ry++) {
            if (g[cx] & (1 << ry)) {
                oled_set_pixel(x0 + cx, y0 + ry, true);
            }
        }
    }
}

static void oled_string(int col, int row, const char *s)
{
    while (*s) {
        oled_char(col++, row, *s++);
    }
}

/* ============ SHT30 温湿度 ============ */
static esp_err_t sht30_read(float *temp, float *hum)
{
    uint8_t cmd[2] = {0x2C, 0x06};   /* 单次测量，高精度，时钟拉伸 */
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

/* ============ ADS1115 土壤湿度 ============ */
static esp_err_t ads1115_read_raw(int16_t *out)
{
    /* 写配置寄存器：OS=启动单次转换, MUX=AIN0-GND, PGA=±4.096V, 单次模式, 128SPS */
    uint8_t config[3] = {0x01, 0x83, 0x80};
    esp_err_t err = i2c_master_transmit(ads_dev, config, 3, 100);
    if (err != ESP_OK) return err;

    /* 等待转换完成（128SPS 约 8ms，留 15ms 余量） */
    vTaskDelay(pdMS_TO_TICKS(15));

    /* 指向转换寄存器 0x00，读 2 字节 */
    uint8_t reg = 0x00;
    err = i2c_master_transmit(ads_dev, &reg, 1, 100);
    if (err != ESP_OK) return err;

    uint8_t data[2] = {0};
    err = i2c_master_receive(ads_dev, data, 2, 100);
    if (err != ESP_OK) return err;

    *out = (int16_t)((data[0] << 8) | data[1]);
    return ESP_OK;
}

/* ============ 主程序 ============ */
void app_main(void)
{
    /* 1. 初始化 I2C 总线 */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    /* 2. 挂载三个 I2C 设备 */
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

    /* 3. 初始化 OLED */
    oled_init();
    ESP_LOGI(TAG, "P0 sensor demo start");

    /* 4. 主循环：读三个传感器 -> 显示 */
    while (1) {
        float t = 0.0f, h = 0.0f;
        esp_err_t err_sht = sht30_read(&t, &h);
        int16_t soil_raw = 0;
        esp_err_t err_soil = ads1115_read_raw(&soil_raw);
        int soil_mv = soil_raw * 4096 / 32768;   /* ±4.096V 量程 -> mV */

        oled_clear();

        char line[22];
        if (err_sht == ESP_OK) {
            int t10 = (int)(t * 10 + 0.5f);
            int h10 = (int)(h * 10 + 0.5f);
            snprintf(line, sizeof(line), "TEMP %d.%d C", t10 / 10, t10 % 10);
            oled_string(0, 0, line);
            snprintf(line, sizeof(line), "HUM  %d.%d %%", h10 / 10, h10 % 10);
            oled_string(0, 1, line);
            ESP_LOGI(TAG, "T=%d.%d C  H=%d.%d %%",
                     t10 / 10, t10 % 10, h10 / 10, h10 % 10);
        } else {
            oled_string(0, 1, "SHT30 ERR");
        }

        if (err_soil == ESP_OK) {
            snprintf(line, sizeof(line), "SOIL %d mV", soil_mv);
            oled_string(0, 2, line);
            ESP_LOGI(TAG, "SOIL=%d mV (raw=%d)", soil_mv, soil_raw);
        } else {
            oled_string(0, 2, "SOIL ERR");
        }

        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
