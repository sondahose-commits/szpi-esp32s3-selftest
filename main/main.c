#include <stdio.h>
#include <string.h>

#include "board_bsp.h"
#include "board_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "selftest.h"

static const char *TAG = "szpi_selftest";

typedef void (*selftest_fn_t)(test_result_t *res);

static const selftest_fn_t s_tests[] = {
    test_system_memory,
    test_i2c_bus_scan,
    test_pca9557_expander,
    test_lcd_and_backlight,
    test_touch_ft6336,
    test_imu_qmi8658,
    test_sdcard_sdmmc,
    test_audio_es8311_speaker,
    test_audio_es7210_mic,
    test_camera_gc0308,
    test_wifi_scan,
    test_ble_scan,
    test_boot_key_and_ext_gpio,
};

#define NUM_TESTS (sizeof(s_tests) / sizeof(s_tests[0]))

static const char *status_str(test_status_t st)
{
    switch (st) {
    case TEST_STATUS_PASS: return "PASS";
    case TEST_STATUS_FAIL: return "FAIL";
    default:               return "SKIP";
    }
}

static uint16_t status_color(test_status_t st)
{
    switch (st) {
    case TEST_STATUS_PASS: return COLOR_GREEN;
    case TEST_STATUS_FAIL: return COLOR_RED;
    default:               return COLOR_YELLOW;
    }
}

static void print_summary_uart(const test_result_t *results, size_t count)
{
    int pass_cnt = 0, fail_cnt = 0, skip_cnt = 0;

    printf("\n");
    printf("========================================================================\n");
    printf("   立创·实战派 ESP32-S3 (LCKFB-SZPI-ESP32-S3-VA) 硬件功能自检报告\n");
    printf("========================================================================\n");
    printf("%-22s | %-6s | %s\n", "Test Item", "Result", "Details");
    printf("-----------------------+--------+---------------------------------------\n");

    for (size_t i = 0; i < count; i++) {
        if (results[i].status == TEST_STATUS_PASS) pass_cnt++;
        else if (results[i].status == TEST_STATUS_FAIL) fail_cnt++;
        else skip_cnt++;

        printf("%-22s | [%4s] | %s\n",
               results[i].name ? results[i].name : "Unknown",
               status_str(results[i].status),
               results[i].detail);
    }

    printf("========================================================================\n");
    printf("Summary: PASS=%d, FAIL=%d, SKIP=%d (Total=%u)\n",
           pass_cnt, fail_cnt, skip_cnt, (unsigned)count);
    printf("Press BOOT button (GPIO0) to re-run all tests.\n");
    printf("========================================================================\n\n");
}

static void draw_summary_lcd(const test_result_t *results, size_t count)
{
    if (!bsp_lcd_is_ready()) {
        return;
    }

    int pass_cnt = 0, fail_cnt = 0, skip_cnt = 0;
    for (size_t i = 0; i < count; i++) {
        if (results[i].status == TEST_STATUS_PASS) pass_cnt++;
        else if (results[i].status == TEST_STATUS_FAIL) fail_cnt++;
        else skip_cnt++;
    }

    bsp_lcd_fill_screen(COLOR_DARK_BG);
    uint16_t hdr_bg = (fail_cnt == 0) ? 0x03E0 : 0x8000; /* 全通过深绿 / 有失败深红 */
    bsp_lcd_fill_rect(0, 0, BSP_LCD_H_RES, 24, hdr_bg);

    char header[56];
    snprintf(header, sizeof(header), "SZPI-S3 TEST  P:%d F:%d S:%d", pass_cnt, fail_cnt, skip_cnt);
    bsp_lcd_draw_string(8, 8, header, COLOR_WHITE, hdr_bg, 1);

    /* 13 行结果，每行高 15px (y=28 .. 223) */
    for (size_t i = 0; i < count; i++) {
        int y = 28 + (int)i * 15;
        char line[56];
        snprintf(line, sizeof(line), "%-18.18s %-26.26s",
                 results[i].name ? results[i].name : "",
                 results[i].detail);
        bsp_lcd_draw_string(4, y, line, COLOR_WHITE, COLOR_DARK_BG, 1);
        bsp_lcd_draw_string(282, y, status_str(results[i].status),
                            status_color(results[i].status), COLOR_DARK_BG, 1);
    }

    bsp_lcd_fill_rect(0, 224, BSP_LCD_H_RES, 16, COLOR_HEADER_BG);
    bsp_lcd_draw_string(44, 228, "Press BOOT (IO0) to re-run tests", COLOR_YELLOW, COLOR_HEADER_BG, 1);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== LCKFB SZPI ESP32-S3 Hardware Self-Test Starting ===");
    bsp_boot_key_init();

    int round = 1;
    while (1) {
        ESP_LOGI(TAG, "---------- Self-Test Round #%d ----------", round);

        test_result_t results[NUM_TESTS];
        memset(results, 0, sizeof(results));

        for (size_t i = 0; i < NUM_TESTS; i++) {
            ESP_LOGI(TAG, ">>> Running test %u/%u ...", (unsigned)(i + 1), (unsigned)NUM_TESTS);
            s_tests[i](&results[i]);
            ESP_LOGI(TAG, "<<< [%s] %s: %s",
                     status_str(results[i].status),
                     results[i].name ? results[i].name : "",
                     results[i].detail);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        print_summary_uart(results, NUM_TESTS);
        draw_summary_lcd(results, NUM_TESTS);

        /* 等待用户按下 BOOT 按键重新执行整套测试 */
        while (!bsp_boot_key_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        while (bsp_boot_key_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        round++;
    }
}
