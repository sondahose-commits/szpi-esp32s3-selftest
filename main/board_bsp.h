#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 常用 RGB565 颜色（逻辑值，写入 ST7789 前由绘制函数自动做大端字节序交换） */
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_GRAY      0x8410
#define COLOR_DARK_BG   0x10A2
#define COLOR_HEADER_BG 0x19F4

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyr_x_dps;
    float gyr_y_dps;
    float gyr_z_dps;
    float pitch_deg;
    float roll_deg;
} bsp_qmi8658_data_t;

/* ---------- I2C0 ---------- */
esp_err_t bsp_i2c_init(void);
bool bsp_i2c_probe(uint8_t addr_7bit);

/* ---------- PCA9557 IO 扩展芯片 ---------- */
esp_err_t bsp_pca9557_init(void);
esp_err_t bsp_pca9557_read_reg(uint8_t reg, uint8_t *val);
esp_err_t bsp_pca9557_write_reg(uint8_t reg, uint8_t val);
esp_err_t bsp_pca9557_set_output_bit(uint8_t bit_mask, uint8_t level);
esp_err_t bsp_pca9557_set_lcd_cs(uint8_t level);
esp_err_t bsp_pca9557_set_pa_en(uint8_t level);
esp_err_t bsp_pca9557_set_dvp_pwdn(uint8_t level);

/* ---------- QMI8658 6轴姿态传感器 ---------- */
esp_err_t bsp_qmi8658_init(uint8_t *out_who_am_i);
esp_err_t bsp_qmi8658_read(bsp_qmi8658_data_t *data);

/* ---------- ST7789 LCD 与背光 PWM ---------- */
esp_err_t bsp_lcd_init(void);
bool bsp_lcd_is_ready(void);
esp_lcd_panel_handle_t bsp_lcd_get_panel_handle(void);
esp_err_t bsp_lcd_set_backlight(int brightness_percent);
void bsp_lcd_fill_screen(uint16_t color);
void bsp_lcd_fill_rect(int x, int y, int w, int h, uint16_t color);
void bsp_lcd_draw_bitmap_raw(int x, int y, int w, int h, const void *rgb565_be_buf);
void bsp_lcd_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale);
void bsp_lcd_draw_string(int x, int y, const char *str, uint16_t fg, uint16_t bg, int scale);
void bsp_lcd_show_banner(const char *title, const char *line1, const char *line2);

/* ---------- BOOT 按键 ---------- */
void bsp_boot_key_init(void);
bool bsp_boot_key_is_pressed(void);

#ifdef __cplusplus
}
#endif
