#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TEST_STATUS_PASS = 0,
    TEST_STATUS_FAIL,
    TEST_STATUS_SKIP,
} test_status_t;

typedef struct {
    const char *name;
    test_status_t status;
    char detail[96];
} test_result_t;

/* 各项硬件自检函数 */
void test_system_memory(test_result_t *res);
void test_i2c_bus_scan(test_result_t *res);
void test_pca9557_expander(test_result_t *res);
void test_lcd_and_backlight(test_result_t *res);
void test_touch_ft6336(test_result_t *res);
void test_imu_qmi8658(test_result_t *res);
void test_sdcard_sdmmc(test_result_t *res);
void test_audio_es8311_speaker(test_result_t *res);
void test_audio_es7210_mic(test_result_t *res);
void test_camera_gc0308(test_result_t *res);
void test_wifi_scan(test_result_t *res);
void test_ble_scan(test_result_t *res);
void test_boot_key_and_ext_gpio(test_result_t *res);

#ifdef __cplusplus
}
#endif
