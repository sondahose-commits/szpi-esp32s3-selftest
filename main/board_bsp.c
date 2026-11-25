#include "board_bsp.h"
#include "board_pins.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "board_bsp";

/* QMI8658 寄存器定义 */
#define QMI8658_REG_WHO_AM_I    0x00
#define QMI8658_REG_CTRL1       0x02
#define QMI8658_REG_CTRL2       0x03
#define QMI8658_REG_CTRL3       0x04
#define QMI8658_REG_CTRL7       0x08
#define QMI8658_REG_STATUS0     0x2E
#define QMI8658_REG_AX_L        0x35
#define QMI8658_REG_RESET       0x60
#define QMI8658_WHO_AM_I_VAL    0x05

static bool s_i2c_inited = false;
static bool s_lcd_inited = false;
static bool s_bl_inited = false;
static esp_lcd_panel_io_handle_t s_lcd_io = NULL;
static esp_lcd_panel_handle_t s_lcd_panel = NULL;
static uint16_t *s_line_buf = NULL; /* 320 * 24 像素行缓冲，用于快速刷屏 */

#define LCD_BUF_LINES   24

static inline uint16_t swap16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

/* ============================================================================
 * 1. I2C0 总线驱动 (传统 I2C API，与 esp32-camera/es8311/es7210/ft5x06 统一)
 * ============================================================================ */
esp_err_t bsp_i2c_init(void)
{
    if (s_i2c_inited) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA_IO,
        .scl_io_num = BSP_I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(BSP_I2C_NUM, &conf), TAG, "i2c_param_config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(BSP_I2C_NUM, conf.mode, 0, 0, 0), TAG, "i2c_driver_install failed");
    s_i2c_inited = true;
    return ESP_OK;
}

bool bsp_i2c_probe(uint8_t addr_7bit)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr_7bit << 1) | I2C_MASTER_WRITE), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BSP_I2C_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
}

/* ============================================================================
 * 2. PCA9557 IO 扩展芯片 (0x19)
 * ============================================================================ */
esp_err_t bsp_pca9557_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(
        BSP_I2C_NUM, BSP_I2C_ADDR_PCA9557,
        &reg, 1, val, 1, pdMS_TO_TICKS(200));
}

esp_err_t bsp_pca9557_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(
        BSP_I2C_NUM, BSP_I2C_ADDR_PCA9557,
        buf, sizeof(buf), pdMS_TO_TICKS(200));
}

esp_err_t bsp_pca9557_init(void)
{
    /* 初始状态: DVP_PWDN(IO2)=1(休眠), PA_EN(IO1)=0(关闭功放), LCD_CS(IO0)=1(未片选) => 0x05 */
    ESP_RETURN_ON_ERROR(bsp_pca9557_write_reg(PCA9557_REG_OUTPUT, 0x05), TAG, "PCA9557 write output failed");
    /* IO0/IO1/IO2 设为输出(0)，IO3..IO7 保持输入(1) => 0xF8 */
    ESP_RETURN_ON_ERROR(bsp_pca9557_write_reg(PCA9557_REG_CONFIG, 0xF8), TAG, "PCA9557 write config failed");
    return ESP_OK;
}

esp_err_t bsp_pca9557_set_output_bit(uint8_t bit_mask, uint8_t level)
{
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(bsp_pca9557_read_reg(PCA9557_REG_OUTPUT, &val), TAG, "PCA9557 read output failed");
    if (level) {
        val |= bit_mask;
    } else {
        val &= (uint8_t)(~bit_mask);
    }
    return bsp_pca9557_write_reg(PCA9557_REG_OUTPUT, val);
}

esp_err_t bsp_pca9557_set_lcd_cs(uint8_t level)
{
    return bsp_pca9557_set_output_bit(PCA9557_BIT_LCD_CS, level);
}

esp_err_t bsp_pca9557_set_pa_en(uint8_t level)
{
    return bsp_pca9557_set_output_bit(PCA9557_BIT_PA_EN, level);
}

esp_err_t bsp_pca9557_set_dvp_pwdn(uint8_t level)
{
    return bsp_pca9557_set_output_bit(PCA9557_BIT_DVP_PWDN, level);
}

/* ============================================================================
 * 3. QMI8658 6轴姿态传感器 (0x6A)
 * ============================================================================ */
static esp_err_t qmi8658_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(
        BSP_I2C_NUM, BSP_I2C_ADDR_QMI8658,
        buf, sizeof(buf), pdMS_TO_TICKS(200));
}

static esp_err_t qmi8658_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(
        BSP_I2C_NUM, BSP_I2C_ADDR_QMI8658,
        &reg, 1, buf, len, pdMS_TO_TICKS(200));
}

esp_err_t bsp_qmi8658_init(uint8_t *out_who_am_i)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(qmi8658_read_regs(QMI8658_REG_WHO_AM_I, &id, 1), TAG, "QMI8658 read WHO_AM_I failed");
    if (out_who_am_i) {
        *out_who_am_i = id;
    }
    if (id != QMI8658_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "QMI8658 unexpected WHO_AM_I=0x%02X (expect 0x%02X)", id, QMI8658_WHO_AM_I_VAL);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_RETURN_ON_ERROR(qmi8658_write_reg(QMI8658_REG_RESET, 0xB0), TAG, "QMI8658 reset failed");
    vTaskDelay(pdMS_TO_TICKS(15));
    ESP_RETURN_ON_ERROR(qmi8658_write_reg(QMI8658_REG_CTRL1, 0x40), TAG, "QMI8658 CTRL1 failed");
    ESP_RETURN_ON_ERROR(qmi8658_write_reg(QMI8658_REG_CTRL7, 0x03), TAG, "QMI8658 CTRL7 failed");
    ESP_RETURN_ON_ERROR(qmi8658_write_reg(QMI8658_REG_CTRL2, 0x95), TAG, "QMI8658 CTRL2 failed"); /* +/-4g, 250Hz */
    ESP_RETURN_ON_ERROR(qmi8658_write_reg(QMI8658_REG_CTRL3, 0xD5), TAG, "QMI8658 CTRL3 failed"); /* +/-512dps, 250Hz */
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

esp_err_t bsp_qmi8658_read(bsp_qmi8658_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 等待数据就绪（最多 50ms） */
    uint8_t status = 0;
    for (int i = 0; i < 10; i++) {
        ESP_RETURN_ON_ERROR(qmi8658_read_regs(QMI8658_REG_STATUS0, &status, 1), TAG, "QMI8658 STATUS0 failed");
        if (status & 0x03) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint8_t raw[12] = { 0 };
    ESP_RETURN_ON_ERROR(qmi8658_read_regs(QMI8658_REG_AX_L, raw, sizeof(raw)), TAG, "QMI8658 read data failed");

    data->acc_x = (int16_t)((raw[1] << 8) | raw[0]);
    data->acc_y = (int16_t)((raw[3] << 8) | raw[2]);
    data->acc_z = (int16_t)((raw[5] << 8) | raw[4]);
    data->gyr_x = (int16_t)((raw[7] << 8) | raw[6]);
    data->gyr_y = (int16_t)((raw[9] << 8) | raw[8]);
    data->gyr_z = (int16_t)((raw[11] << 8) | raw[10]);

    /* CTRL2=0x95 (+/-4g -> 8192 LSB/g), CTRL3=0xD5 (+/-512dps -> 64 LSB/dps) */
    data->acc_x_g = (float)data->acc_x / 8192.0f;
    data->acc_y_g = (float)data->acc_y / 8192.0f;
    data->acc_z_g = (float)data->acc_z / 8192.0f;
    data->gyr_x_dps = (float)data->gyr_x / 64.0f;
    data->gyr_y_dps = (float)data->gyr_y / 64.0f;
    data->gyr_z_dps = (float)data->gyr_z / 64.0f;

    float ayz = sqrtf(data->acc_y_g * data->acc_y_g + data->acc_z_g * data->acc_z_g);
    float axz = sqrtf(data->acc_x_g * data->acc_x_g + data->acc_z_g * data->acc_z_g);
    data->pitch_deg = atan2f(data->acc_x_g, ayz) * 57.2957795f;
    data->roll_deg = atan2f(data->acc_y_g, axz) * 57.2957795f;
    return ESP_OK;
}

/* ============================================================================
 * 4. ST7789 2.0寸 IPS LCD 与 LEDC 背光驱动
 * ============================================================================ */
static esp_err_t bsp_lcd_backlight_init(void)
{
    if (s_bl_inited) {
        return ESP_OK;
    }

    const ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = BSP_LCD_BL_LEDC_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "LEDC timer config failed");

    const ledc_channel_config_t ch_cfg = {
        .gpio_num = BSP_LCD_BACKLIGHT_IO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BSP_LCD_BL_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BSP_LCD_BL_LEDC_TIMER,
        .duty = 1023, /* 低电平有效背光：初始化时占空比 100%(高电平)=熄灭 */
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "LEDC channel config failed");
    s_bl_inited = true;
    return ESP_OK;
}

esp_err_t bsp_lcd_set_backlight(int brightness_percent)
{
    if (!s_bl_inited) {
        ESP_RETURN_ON_ERROR(bsp_lcd_backlight_init(), TAG, "Backlight init failed");
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    } else if (brightness_percent > 100) {
        brightness_percent = 100;
    }

    /* 实战派 ESP32-S3 背光电路为低电平点亮 (PMOS)，占空比反相 */
    uint32_t duty = (uint32_t)((1023 * (100 - brightness_percent)) / 100);
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_BL_LEDC_CH, duty), TAG, "ledc_set_duty failed");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_BL_LEDC_CH);
}

esp_err_t bsp_lcd_init(void)
{
    if (s_lcd_inited) {
        bsp_pca9557_set_lcd_cs(0);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bsp_lcd_backlight_init(), TAG, "Backlight init failed");

    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SPI_CLK_IO,
        .mosi_io_num = BSP_LCD_SPI_MOSI_IO,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = BSP_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI bus init failed");

    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC_IO,
        .cs_gpio_num = GPIO_NUM_NC, /* 片选由 PCA9557 IO0 独立控制 */
        .pclk_hz = BSP_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 2,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST, &io_config, &s_lcd_io),
        TAG, "esp_lcd_new_panel_io_spi failed");

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_st7789(s_lcd_io, &panel_config, &s_lcd_panel),
        TAG, "esp_lcd_new_panel_st7789 failed");

    /* 必须先通过 PCA9557 拉低 LCD_CS，再发送 ST7789 初始化命令 */
    ESP_RETURN_ON_ERROR(bsp_pca9557_set_lcd_cs(0), TAG, "LCD CS assert failed");
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_lcd_panel), TAG, "LCD reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_lcd_panel), TAG, "LCD init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_lcd_panel, true), TAG, "LCD invert_color failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_lcd_panel, true), TAG, "LCD swap_xy failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_lcd_panel, true, false), TAG, "LCD mirror failed");

    s_line_buf = heap_caps_malloc(BSP_LCD_H_RES * LCD_BUF_LINES * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_line_buf) {
        s_line_buf = malloc(BSP_LCD_H_RES * LCD_BUF_LINES * sizeof(uint16_t));
    }
    ESP_RETURN_ON_FALSE(s_line_buf != NULL, ESP_ERR_NO_MEM, TAG, "LCD line buffer alloc failed");

    s_lcd_inited = true;
    bsp_lcd_fill_screen(COLOR_BLACK);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_lcd_panel, true), TAG, "LCD display on failed");
    ESP_RETURN_ON_ERROR(bsp_lcd_set_backlight(85), TAG, "LCD backlight on failed");
    return ESP_OK;
}

bool bsp_lcd_is_ready(void)
{
    return s_lcd_inited;
}

esp_lcd_panel_handle_t bsp_lcd_get_panel_handle(void)
{
    return s_lcd_panel;
}

void bsp_lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (!s_lcd_inited || !s_line_buf || w <= 0 || h <= 0) {
        return;
    }
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > BSP_LCD_H_RES) { w = BSP_LCD_H_RES - x; }
    if (y + h > BSP_LCD_V_RES) { h = BSP_LCD_V_RES - y; }
    if (w <= 0 || h <= 0) {
        return;
    }

    uint16_t be_color = swap16(color);
    int max_rows = (BSP_LCD_H_RES * LCD_BUF_LINES) / w;
    if (max_rows < 1) {
        max_rows = 1;
    }

    int fill_count = w * (h < max_rows ? h : max_rows);
    for (int i = 0; i < fill_count; i++) {
        s_line_buf[i] = be_color;
    }

    int cur_y = y;
    int remain = h;
    while (remain > 0) {
        int chunk = remain < max_rows ? remain : max_rows;
        esp_lcd_panel_draw_bitmap(s_lcd_panel, x, cur_y, x + w, cur_y + chunk, s_line_buf);
        cur_y += chunk;
        remain -= chunk;
    }
}

void bsp_lcd_fill_screen(uint16_t color)
{
    bsp_lcd_fill_rect(0, 0, BSP_LCD_H_RES, BSP_LCD_V_RES, color);
}

void bsp_lcd_draw_bitmap_raw(int x, int y, int w, int h, const void *rgb565_be_buf)
{
    if (!s_lcd_inited || !rgb565_be_buf || w <= 0 || h <= 0) {
        return;
    }
    esp_lcd_panel_draw_bitmap(s_lcd_panel, x, y, x + w, y + h, rgb565_be_buf);
}

/* ============================================================================
 * 5. 内建极简 5x7 ASCII 像素字库 (0x20..0x7E)，无需外部字体/LVGL即可绘制UI
 * ============================================================================ */
static const uint8_t s_font5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* ' ' */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* '!' */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* '"' */
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, /* '#' */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* '$' */
    {0x23, 0x13, 0x08, 0x64, 0x62}, /* '%' */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* '&' */
    {0x00, 0x05, 0x03, 0x00, 0x00}, /* '\'' */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* '(' */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* ')' */
    {0x14, 0x08, 0x3E, 0x08, 0x14}, /* '*' */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* '+' */
    {0x00, 0x50, 0x30, 0x00, 0x00}, /* ',' */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* '-' */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* '.' */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* '/' */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* '0' */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* '1' */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* '2' */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* '3' */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* '4' */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* '5' */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* '6' */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* '7' */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* '8' */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* '9' */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* ':' */
    {0x00, 0x56, 0x36, 0x00, 0x00}, /* ';' */
    {0x08, 0x14, 0x22, 0x41, 0x00}, /* '<' */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* '=' */
    {0x00, 0x41, 0x22, 0x14, 0x08}, /* '>' */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* '?' */
    {0x32, 0x49, 0x79, 0x41, 0x3E}, /* '@' */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* 'A' */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* 'B' */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* 'C' */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* 'D' */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* 'E' */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* 'F' */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* 'G' */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 'H' */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* 'I' */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* 'J' */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* 'K' */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* 'L' */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 'M' */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* 'N' */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 'O' */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* 'P' */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 'Q' */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* 'R' */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* 'S' */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* 'T' */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 'U' */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 'V' */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* 'W' */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* 'X' */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* 'Y' */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* 'Z' */
    {0x00, 0x7F, 0x41, 0x41, 0x00}, /* '[' */
    {0x02, 0x04, 0x08, 0x10, 0x20}, /* '\\' */
    {0x00, 0x41, 0x41, 0x7F, 0x00}, /* ']' */
    {0x04, 0x02, 0x01, 0x02, 0x04}, /* '^' */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* '_' */
    {0x00, 0x01, 0x02, 0x04, 0x00}, /* '`' */
    {0x20, 0x54, 0x54, 0x54, 0x78}, /* 'a' */
    {0x7F, 0x48, 0x44, 0x44, 0x38}, /* 'b' */
    {0x38, 0x44, 0x44, 0x44, 0x20}, /* 'c' */
    {0x38, 0x44, 0x44, 0x48, 0x7F}, /* 'd' */
    {0x38, 0x54, 0x54, 0x54, 0x18}, /* 'e' */
    {0x08, 0x7E, 0x09, 0x01, 0x02}, /* 'f' */
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, /* 'g' */
    {0x7F, 0x08, 0x04, 0x04, 0x78}, /* 'h' */
    {0x00, 0x44, 0x7D, 0x40, 0x00}, /* 'i' */
    {0x20, 0x40, 0x44, 0x3D, 0x00}, /* 'j' */
    {0x7F, 0x10, 0x28, 0x44, 0x00}, /* 'k' */
    {0x00, 0x41, 0x7F, 0x40, 0x00}, /* 'l' */
    {0x7C, 0x04, 0x18, 0x04, 0x78}, /* 'm' */
    {0x7C, 0x08, 0x04, 0x04, 0x78}, /* 'n' */
    {0x38, 0x44, 0x44, 0x44, 0x38}, /* 'o' */
    {0x7C, 0x14, 0x14, 0x14, 0x08}, /* 'p' */
    {0x08, 0x14, 0x14, 0x18, 0x7C}, /* 'q' */
    {0x7C, 0x08, 0x04, 0x04, 0x08}, /* 'r' */
    {0x48, 0x54, 0x54, 0x54, 0x20}, /* 's' */
    {0x04, 0x3F, 0x44, 0x40, 0x20}, /* 't' */
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, /* 'u' */
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, /* 'v' */
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, /* 'w' */
    {0x44, 0x28, 0x10, 0x28, 0x44}, /* 'x' */
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, /* 'y' */
    {0x44, 0x64, 0x54, 0x4C, 0x44}, /* 'z' */
    {0x00, 0x08, 0x36, 0x41, 0x00}, /* '{' */
    {0x00, 0x00, 0x7F, 0x00, 0x00}, /* '|' */
    {0x00, 0x41, 0x36, 0x08, 0x00}, /* '}' */
    {0x08, 0x04, 0x08, 0x10, 0x08}, /* '~' */
};

void bsp_lcd_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    if (!s_lcd_inited || !s_line_buf || scale < 1) {
        return;
    }
    if (c < 0x20 || c > 0x7E) {
        c = '?';
    }
    int cw = 6 * scale;
    int ch = 8 * scale;
    if (x < 0 || y < 0 || x + cw > BSP_LCD_H_RES || y + ch > BSP_LCD_V_RES) {
        return;
    }
    if (cw * ch > BSP_LCD_H_RES * LCD_BUF_LINES) {
        return;
    }

    const uint8_t *glyph = s_font5x7[c - 0x20];
    uint16_t fg_be = swap16(fg);
    uint16_t bg_be = swap16(bg);

    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 6; col++) {
            bool on = (col < 5 && row < 7) ? ((glyph[col] >> row) & 0x01) : false;
            uint16_t px = on ? fg_be : bg_be;
            for (int sy = 0; sy < scale; sy++) {
                int py = row * scale + sy;
                for (int sx = 0; sx < scale; sx++) {
                    int px_pos = col * scale + sx;
                    s_line_buf[py * cw + px_pos] = px;
                }
            }
        }
    }
    esp_lcd_panel_draw_bitmap(s_lcd_panel, x, y, x + cw, y + ch, s_line_buf);
}

void bsp_lcd_draw_string(int x, int y, const char *str, uint16_t fg, uint16_t bg, int scale)
{
    if (!s_lcd_inited || !str) {
        return;
    }
    int cx = x;
    int cy = y;
    int step_x = 6 * scale;
    int step_y = 9 * scale;

    while (*str) {
        if (*str == '\n') {
            cx = x;
            cy += step_y;
        } else {
            if (cx + step_x <= BSP_LCD_H_RES && cy + 8 * scale <= BSP_LCD_V_RES) {
                bsp_lcd_draw_char(cx, cy, *str, fg, bg, scale);
            }
            cx += step_x;
        }
        str++;
    }
}

void bsp_lcd_show_banner(const char *title, const char *line1, const char *line2)
{
    if (!s_lcd_inited) {
        return;
    }
    bsp_lcd_fill_screen(COLOR_DARK_BG);
    bsp_lcd_fill_rect(0, 0, BSP_LCD_H_RES, 34, COLOR_HEADER_BG);
    if (title) {
        bsp_lcd_draw_string(10, 9, title, COLOR_WHITE, COLOR_HEADER_BG, 2);
    }
    if (line1) {
        bsp_lcd_draw_string(12, 60, line1, COLOR_CYAN, COLOR_DARK_BG, 2);
    }
    if (line2) {
        bsp_lcd_draw_string(12, 105, line2, COLOR_WHITE, COLOR_DARK_BG, 1);
    }
}

/* ============================================================================
 * 6. BOOT 按键 (GPIO0)
 * ============================================================================ */
void bsp_boot_key_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BSP_BOOT_KEY_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

bool bsp_boot_key_is_pressed(void)
{
    return (gpio_get_level(BSP_BOOT_KEY_IO) == 0);
}
