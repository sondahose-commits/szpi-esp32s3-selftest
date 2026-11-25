#include "selftest.h"
#include "board_bsp.h"
#include "board_pins.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

static const char *TAG = "test_periph";

/* ============================================================================
 * 1. 系统芯片、16MB Flash 与 8MB Octal PSRAM 读写自检
 * ============================================================================ */
void test_system_memory(test_result_t *res)
{
    res->name = "1. MCU/Flash/PSRAM";

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = 0;
    esp_err_t flash_ret = esp_flash_get_size(NULL, &flash_size);
    uint32_t flash_mb = (flash_ret == ESP_OK) ? (flash_size / (1024 * 1024)) : 0;

    size_t psram_size = 0;
    bool psram_rw_ok = false;

#if CONFIG_SPIRAM
    if (esp_psram_is_initialized()) {
        psram_size = esp_psram_get_size();
        /* 申请 512KB PSRAM 做实际读写校验 */
        const size_t test_bytes = 512 * 1024;
        uint32_t *buf = heap_caps_malloc(test_bytes, MALLOC_CAP_SPIRAM);
        if (buf) {
            size_t words = test_bytes / sizeof(uint32_t);
            psram_rw_ok = true;
            for (size_t i = 0; i < words; i++) {
                buf[i] = (uint32_t)(0xA55A0000U ^ (i * 2654435761U));
            }
            for (size_t i = 0; i < words; i++) {
                if (buf[i] != (uint32_t)(0xA55A0000U ^ (i * 2654435761U))) {
                    psram_rw_ok = false;
                    break;
                }
            }
            free(buf);
        }
    }
#endif

    size_t psram_mb = psram_size / (1024 * 1024);
    size_t free_internal_kb = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;

    ESP_LOGI(TAG, "Chip: ESP32-S3 rev%d, cores=%d, Flash=%" PRIu32 "MB, PSRAM=%uMB (RW=%s), IntHeap=%uKB",
             chip_info.revision, chip_info.cores, flash_mb, (unsigned)psram_mb,
             psram_rw_ok ? "OK" : "FAIL", (unsigned)free_internal_kb);

    if (flash_mb >= 16 && psram_mb >= 8 && psram_rw_ok) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "S3 %dC Flash:%" PRIu32 "MB PSRAM:%uMB OK",
                 chip_info.cores, flash_mb, (unsigned)psram_mb);
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Flash:%" PRIu32 "MB PSRAM:%uMB RW:%s",
                 flash_mb, (unsigned)psram_mb, psram_rw_ok ? "OK" : "ERR");
    }
}

/* ============================================================================
 * 2. 共享 I2C0 总线扫描 (核验板载 5 颗常供电 I2C 芯片)
 * ============================================================================ */
void test_i2c_bus_scan(test_result_t *res)
{
    res->name = "2. I2C0 Bus Scan";

    if (bsp_i2c_init() != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "I2C0 driver init failed");
        return;
    }

    int found = 0;
    ESP_LOGI(TAG, "Scanning I2C0 (SDA=IO%d, SCL=IO%d)...", BSP_I2C_SDA_IO, BSP_I2C_SCL_IO);
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (bsp_i2c_probe(addr)) {
            ESP_LOGI(TAG, "  -> Found I2C device at 0x%02X", addr);
            found++;
        }
    }

    typedef struct {
        uint8_t addr;
        const char *chip;
    } expected_dev_t;

    const expected_dev_t expected[] = {
        { BSP_I2C_ADDR_ES8311,  "ES8311"  },
        { BSP_I2C_ADDR_PCA9557, "PCA9557" },
        { BSP_I2C_ADDR_FT6336,  "FT6336"  },
        { BSP_I2C_ADDR_ES7210,  "ES7210"  },
        { BSP_I2C_ADDR_QMI8658, "QMI8658" },
    };

    int ok_cnt = 0;
    char missing[64] = { 0 };
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        if (bsp_i2c_probe(expected[i].addr)) {
            ok_cnt++;
        } else {
            ESP_LOGE(TAG, "Missing onboard I2C chip: %s (0x%02X)", expected[i].chip, expected[i].addr);
            if (missing[0] != '\0') {
                strncat(missing, ",", sizeof(missing) - strlen(missing) - 1);
            }
            strncat(missing, expected[i].chip, sizeof(missing) - strlen(missing) - 1);
        }
    }

    if (ok_cnt == 5) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "5/5 chips ACK (total %d devs)", found);
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "%d/5 ACK, miss:%s", ok_cnt, missing);
    }
}

/* ============================================================================
 * 3. PCA9557 8位 IO 扩展芯片读写校验 (0x19)
 * ============================================================================ */
void test_pca9557_expander(test_result_t *res)
{
    res->name = "3. PCA9557 IO Exp";

    if (bsp_pca9557_init() != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "PCA9557 init failed");
        return;
    }

    uint8_t reg_out = 0;
    uint8_t reg_cfg = 0;
    esp_err_t r1 = bsp_pca9557_read_reg(PCA9557_REG_OUTPUT, &reg_out);
    esp_err_t r2 = bsp_pca9557_read_reg(PCA9557_REG_CONFIG, &reg_cfg);

    /* 翻转 LCD_CS (IO0) 验证输出寄存器可读写 */
    esp_err_t r3 = bsp_pca9557_set_lcd_cs(0);
    uint8_t out_low = 0xFF;
    esp_err_t r4 = bsp_pca9557_read_reg(PCA9557_REG_OUTPUT, &out_low);
    esp_err_t r5 = bsp_pca9557_set_lcd_cs(1);
    uint8_t out_high = 0x00;
    esp_err_t r6 = bsp_pca9557_read_reg(PCA9557_REG_OUTPUT, &out_high);

    if (r1 == ESP_OK && r2 == ESP_OK && r3 == ESP_OK && r4 == ESP_OK && r5 == ESP_OK && r6 == ESP_OK &&
        reg_out == 0x05 && reg_cfg == 0xF8 &&
        (out_low & PCA9557_BIT_LCD_CS) == 0 && (out_high & PCA9557_BIT_LCD_CS) != 0) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "OUT=0x%02X CFG=0x%02X toggle OK", reg_out, reg_cfg);
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Reg mismatch OUT=0x%02X CFG=0x%02X", reg_out, reg_cfg);
    }
}

/* ============================================================================
 * 4. 2.0寸 IPS LCD (ST7789) 与 PWM 背光测试
 * ============================================================================ */
void test_lcd_and_backlight(test_result_t *res)
{
    res->name = "4. ST7789 LCD & BL";

    esp_err_t ret = bsp_lcd_init();
    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "LCD init failed (%s)", esp_err_to_name(ret));
        return;
    }

    /* 1) 背光 PWM 渐变测试 (0% -> 100%) */
    bsp_lcd_fill_screen(COLOR_WHITE);
    bsp_lcd_draw_string(40, 108, "BACKLIGHT PWM TEST", COLOR_BLACK, COLOR_WHITE, 2);
    for (int b = 0; b <= 100; b += 10) {
        bsp_lcd_set_backlight(b);
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    bsp_lcd_set_backlight(85);

    /* 2) 纯色刷屏检查坏点与 RGB 色序 */
    const uint16_t colors[] = { COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE };
    const char *names[] = { "RED", "GREEN", "BLUE", "WHITE" };
    for (int i = 0; i < 4; i++) {
        bsp_lcd_fill_screen(colors[i]);
        uint16_t txt_fg = (colors[i] == COLOR_WHITE || colors[i] == COLOR_GREEN) ? COLOR_BLACK : COLOR_WHITE;
        bsp_lcd_draw_string(120, 110, names[i], txt_fg, colors[i], 2);
        vTaskDelay(pdMS_TO_TICKS(350));
    }

    /* 3) 8色彩条显示 */
    const uint16_t bars[8] = {
        COLOR_WHITE, COLOR_YELLOW, COLOR_CYAN, COLOR_GREEN,
        COLOR_MAGENTA, COLOR_RED, COLOR_BLUE, COLOR_BLACK
    };
    int bar_w = BSP_LCD_H_RES / 8;
    for (int i = 0; i < 8; i++) {
        bsp_lcd_fill_rect(i * bar_w, 0, bar_w, BSP_LCD_V_RES, bars[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(600));

    res->status = TEST_STATUS_PASS;
    snprintf(res->detail, sizeof(res->detail), "320x240 RGB/Bars/PWM OK");
}

/* ============================================================================
 * 5. FT6336 电容触摸屏交互测试 (0x38, 兼容 FT5x06 驱动)
 * ============================================================================ */
void test_touch_ft6336(test_result_t *res)
{
    res->name = "5. FT6336 Touch";

    esp_lcd_panel_io_handle_t tp_io = NULL;
    esp_lcd_touch_handle_t tp = NULL;

    const esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    esp_err_t ret = esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_cfg, &tp_io);
    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Touch IO init err:%s", esp_err_to_name(ret));
        return;
    }

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_V_RES, /* 硬件原始坐标系为竖屏 240x320，swap_xy 后映射为 320x240 */
        .y_max = BSP_LCD_H_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 1,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };

    ret = esp_lcd_touch_new_i2c_ft5x06(tp_io, &tp_cfg, &tp);
    if (ret != ESP_OK) {
        esp_lcd_panel_io_del(tp_io);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "FT6336 init err:%s", esp_err_to_name(ret));
        return;
    }

    /* 验证一次 I2C 读寄存器是否正常 */
    ret = esp_lcd_touch_read_data(tp);
    if (ret != ESP_OK) {
        esp_lcd_touch_del(tp);
        esp_lcd_panel_io_del(tp_io);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "FT6336 read err:%s", esp_err_to_name(ret));
        return;
    }

    bsp_lcd_show_banner("TOUCH TEST (FT6336)", "Touch screen (6s)", "Press BOOT to continue");
    ESP_LOGI(TAG, "FT6336 ready. Please touch the screen within 6 seconds (press BOOT to skip)...");

    int touch_hits = 0;
    uint16_t last_x = 0, last_y = 0;
    int64_t end_us = esp_timer_get_time() + 6LL * 1000000LL;

    while (esp_timer_get_time() < end_us) {
        if (bsp_boot_key_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(150));
            break;
        }
        if (esp_lcd_touch_read_data(tp) == ESP_OK) {
            uint16_t tx[1] = { 0 };
            uint16_t ty[1] = { 0 };
            uint8_t cnt = 0;
            bool pressed = esp_lcd_touch_get_coordinates(tp, tx, ty, NULL, &cnt, 1);
            if (pressed && cnt > 0) {
                touch_hits++;
                last_x = tx[0];
                last_y = ty[0];
                if (touch_hits <= 5 || (touch_hits % 10) == 0) {
                    ESP_LOGI(TAG, "Touch #%d: x=%u, y=%u", touch_hits, last_x, last_y);
                }
                /* 在屏幕上画出触摸轨迹点并更新坐标显示 */
                bsp_lcd_fill_rect((int)last_x - 3, (int)last_y - 3, 7, 7, COLOR_YELLOW);
                char coord[48];
                snprintf(coord, sizeof(coord), "X=%3u  Y=%3u  hits=%d  ", last_x, last_y, touch_hits);
                bsp_lcd_draw_string(12, 218, coord, COLOR_GREEN, COLOR_DARK_BG, 1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    esp_lcd_touch_del(tp);
    esp_lcd_panel_io_del(tp_io);

    res->status = TEST_STATUS_PASS;
    if (touch_hits > 0) {
        snprintf(res->detail, sizeof(res->detail), "Touched %dx last(%u,%u)", touch_hits, last_x, last_y);
    } else {
        snprintf(res->detail, sizeof(res->detail), "Chip I2C OK (no touch)");
    }
}

/* ============================================================================
 * 6. QMI8658 6轴姿态传感器自检 (0x6A)
 * ============================================================================ */
void test_imu_qmi8658(test_result_t *res)
{
    res->name = "6. QMI8658 6-IMU";

    uint8_t who_am_i = 0;
    esp_err_t ret = bsp_qmi8658_init(&who_am_i);
    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Init err, ID=0x%02X", who_am_i);
        return;
    }

    bsp_lcd_show_banner("IMU TEST (QMI8658)", "Reading 6-axis data", "Tilt board to observe");

    bsp_qmi8658_data_t d = { 0 };
    int valid_samples = 0;
    float norm_sum = 0.0f;

    for (int i = 0; i < 12; i++) {
        if (bsp_qmi8658_read(&d) == ESP_OK) {
            float norm = sqrtf(d.acc_x_g * d.acc_x_g + d.acc_y_g * d.acc_y_g + d.acc_z_g * d.acc_z_g);
            norm_sum += norm;
            valid_samples++;

            ESP_LOGI(TAG, "IMU[%02d] Acc(g): %+.2f %+.2f %+.2f (|a|=%.2f) | Gyr(dps): %+.1f %+.1f %+.1f | P=%+.1f R=%+.1f",
                     i + 1, d.acc_x_g, d.acc_y_g, d.acc_z_g, norm,
                     d.gyr_x_dps, d.gyr_y_dps, d.gyr_z_dps, d.pitch_deg, d.roll_deg);

            char buf1[48], buf2[48], buf3[48];
            snprintf(buf1, sizeof(buf1), "ACC: %+.2f %+.2f %+.2f g ", d.acc_x_g, d.acc_y_g, d.acc_z_g);
            snprintf(buf2, sizeof(buf2), "GYR: %+.1f %+.1f %+.1f   ", d.gyr_x_dps, d.gyr_y_dps, d.gyr_z_dps);
            snprintf(buf3, sizeof(buf3), "Pitch:%+.1f  Roll:%+.1f  ", d.pitch_deg, d.roll_deg);
            bsp_lcd_draw_string(12, 130, buf1, COLOR_YELLOW, COLOR_DARK_BG, 1);
            bsp_lcd_draw_string(12, 150, buf2, COLOR_CYAN, COLOR_DARK_BG, 1);
            bsp_lcd_draw_string(12, 170, buf3, COLOR_GREEN, COLOR_DARK_BG, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    float avg_norm = (valid_samples > 0) ? (norm_sum / (float)valid_samples) : 0.0f;
    if (valid_samples >= 8 && avg_norm >= 0.5f && avg_norm <= 1.8f) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "ID=0x%02X |a|=%.2fg OK", who_am_i, avg_norm);
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Bad samples=%d |a|=%.2fg", valid_samples, avg_norm);
    }
}

/* ============================================================================
 * 7. TF 卡槽 1-bit SDMMC 读写测试 (不格式化用户卡，保护数据安全)
 * ============================================================================ */
void test_sdcard_sdmmc(test_result_t *res)
{
    res->name = "7. TF Card (SDMMC)";
    bsp_lcd_show_banner("TF CARD TEST", "Mounting 1-bit SDMMC", "Insert FAT32 TF card");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, /* 保护用户 TF 卡数据，绝不自动格式化 */
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = BSP_SD_CLK_IO;
    slot_config.cmd = BSP_SD_CMD_IO;
    slot_config.d0 = BSP_SD_D0_IO;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    sdmmc_card_t *card = NULL;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TF card mount failed (%s) - card absent or non-FAT32", esp_err_to_name(ret));
        res->status = TEST_STATUS_SKIP;
        snprintf(res->detail, sizeof(res->detail), "No FAT32 card (%s)", esp_err_to_name(ret));
        return;
    }

    uint64_t card_mb = ((uint64_t)card->csd.capacity * card->csd.sector_size) >> 20;
    ESP_LOGI(TAG, "TF Card mounted: name=%s, size=%" PRIu64 "MB, max_freq=%dkHz",
             card->cid.name, card_mb, card->max_freq_khz);

    /* 写入 -> 回读 -> 校验 -> 删除测试文件 */
    const char *test_path = BSP_SD_MOUNT_POINT "/_szpi_selftest.txt";
    const char *payload = "LCKFB-SZPI-ESP32-S3 SDMMC 1-bit RW OK!\n";
    bool rw_ok = false;

    FILE *f = fopen(test_path, "wb");
    if (f) {
        size_t wn = fwrite(payload, 1, strlen(payload), f);
        fclose(f);
        if (wn == strlen(payload)) {
            f = fopen(test_path, "rb");
            if (f) {
                char rbuf[64] = { 0 };
                size_t rn = fread(rbuf, 1, sizeof(rbuf) - 1, f);
                fclose(f);
                if (rn == strlen(payload) && memcmp(rbuf, payload, rn) == 0) {
                    rw_ok = true;
                }
            }
        }
        unlink(test_path);
    }

    esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, card);

    if (rw_ok) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "%" PRIu64 "MB %dMHz RW OK",
                 card_mb, card->max_freq_khz / 1000);
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "%" PRIu64 "MB mounted, RW FAIL", card_mb);
    }
}

/* ============================================================================
 * 13. BOOT 按键 (GPIO0) 与外拓多功能 GPIO10/11 电气自检
 * ============================================================================ */
static bool check_ext_pin_electrical(gpio_num_t pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    vTaskDelay(pdMS_TO_TICKS(5));
    int lvl_pu = gpio_get_level(pin);

    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&cfg);
    vTaskDelay(pdMS_TO_TICKS(5));
    int lvl_pd = gpio_get_level(pin);

    /* 恢复悬空输入，避免影响外部模块 */
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cfg);

    ESP_LOGI(TAG, "Ext GPIO%d check: PullUp=%d, PullDown=%d", (int)pin, lvl_pu, lvl_pd);
    return (lvl_pu == 1 && lvl_pd == 0);
}

void test_boot_key_and_ext_gpio(test_result_t *res)
{
    res->name = "13. Key & IO10/11";

    bool io10_ok = check_ext_pin_electrical(BSP_EXT_GPIO_10);
    bool io11_ok = check_ext_pin_electrical(BSP_EXT_GPIO_11);

    bsp_boot_key_init();
    bool idle_high = !bsp_boot_key_is_pressed();

    bsp_lcd_show_banner("KEY & EXT GPIO TEST", "Press BOOT button!", "Waiting 5s (IO0)...");
    ESP_LOGI(TAG, "Ext IO10=%s, IO11=%s. Press BOOT button (GPIO0) within 5s...",
             io10_ok ? "OK" : "WARN", io11_ok ? "OK" : "WARN");

    bool key_pressed = false;
    int64_t end_us = esp_timer_get_time() + 5LL * 1000000LL;
    int last_sec = -1;

    while (esp_timer_get_time() < end_us) {
        int rem_sec = (int)((end_us - esp_timer_get_time()) / 1000000LL) + 1;
        if (rem_sec != last_sec) {
            last_sec = rem_sec;
            char msg[64];
            snprintf(msg, sizeof(msg), "Countdown: %ds  IO10:%s IO11:%s",
                     rem_sec, io10_ok ? "OK" : "NG", io11_ok ? "OK" : "NG");
            bsp_lcd_draw_string(12, 145, msg, COLOR_YELLOW, COLOR_DARK_BG, 1);
        }
        if (bsp_boot_key_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(25)); /* 消抖 */
            if (bsp_boot_key_is_pressed()) {
                key_pressed = true;
                ESP_LOGI(TAG, "BOOT key press detected!");
                /* 等待松开 */
                while (bsp_boot_key_is_pressed()) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (!idle_high) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "BOOT stuck LOW! IO10:%d IO11:%d", io10_ok, io11_ok);
    } else if (key_pressed && io10_ok && io11_ok) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "BOOT OK, IO10/11 OK");
    } else if (!key_pressed && io10_ok && io11_ok) {
        res->status = TEST_STATUS_SKIP;
        snprintf(res->detail, sizeof(res->detail), "BOOT not pressed; IO10/11 OK");
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "BOOT:%s IO10:%s IO11:%s",
                 key_pressed ? "OK" : "NO", io10_ok ? "OK" : "ERR", io11_ok ? "OK" : "ERR");
    }
}
