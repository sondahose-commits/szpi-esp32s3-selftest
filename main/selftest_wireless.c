#include "selftest.h"
#include "board_bsp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#endif

static const char *TAG = "test_wireless";
static bool s_net_inited = false;

static esp_err_t ensure_nvs_and_netif(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    if (!s_net_inited) {
        ret = esp_netif_init();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            return ret;
        }
        ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            return ret;
        }
        s_net_inited = true;
    }
    return ESP_OK;
}

/* ============================================================================
 * 11. Wi-Fi 2.4GHz 射频与 STA 扫描自检
 * ============================================================================ */
void test_wifi_scan(test_result_t *res)
{
    res->name = "11. Wi-Fi 2.4G Scan";
    bsp_lcd_show_banner("WI-FI SCAN TEST", "Scanning 2.4GHz APs", "Please wait...");

    esp_err_t ret = ensure_nvs_and_netif();
    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "NVS/netif err:%s", esp_err_to_name(ret));
        return;
    }

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        if (sta_netif) {
            esp_netif_destroy_default_wifi(sta_netif);
        }
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "wifi_init err:%s", esp_err_to_name(ret));
        return;
    }

    uint16_t ap_num = 0;
    int8_t best_rssi = -127;
    char best_ssid[33] = { 0 };

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret == ESP_OK) {
        ret = esp_wifi_start();
    }
    if (ret == ESP_OK) {
        ret = esp_wifi_scan_start(NULL, true); /* 阻塞式主动扫描 */
    }

    if (ret == ESP_OK) {
        esp_wifi_scan_get_ap_num(&ap_num);
        uint16_t fetch = (ap_num > 16) ? 16 : ap_num;
        wifi_ap_record_t *ap_list = (fetch > 0) ? calloc(fetch, sizeof(wifi_ap_record_t)) : NULL;
        if (ap_list && esp_wifi_scan_get_ap_records(&fetch, ap_list) == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi scan found %u APs (showing top %u):", ap_num, fetch > 5 ? 5 : fetch);
            for (uint16_t i = 0; i < fetch; i++) {
                if (i == 0) {
                    best_rssi = ap_list[i].rssi;
                    strncpy(best_ssid, (const char *)ap_list[i].ssid, sizeof(best_ssid) - 1);
                }
                if (i < 5) {
                    ESP_LOGI(TAG, "  [%u] SSID=%-24s CH=%2d RSSI=%d dBm",
                             i + 1, (const char *)ap_list[i].ssid, ap_list[i].primary, ap_list[i].rssi);
                    char row[44];
                    snprintf(row, sizeof(row), "%u. %.18s (%ddBm)",
                             i + 1, (const char *)ap_list[i].ssid, ap_list[i].rssi);
                    bsp_lcd_draw_string(12, 128 + i * 16, row, COLOR_YELLOW, COLOR_DARK_BG, 1);
                }
            }
        }
        free(ap_list);
        vTaskDelay(pdMS_TO_TICKS(800));
    }

    esp_wifi_stop();
    esp_wifi_deinit();
    if (sta_netif) {
        esp_netif_destroy_default_wifi(sta_netif);
    }

    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Scan err:%s", esp_err_to_name(ret));
    } else if (ap_num > 0) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "%u APs %.10s(%d)", ap_num, best_ssid, best_rssi);
    } else {
        /* Wi-Fi 射频与协议栈初始化成功，但周围无 2.4GHz 热点 */
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "RF OK, 0 APs nearby");
    }
}

/* ============================================================================
 * 12. Bluetooth LE 5.0 (NimBLE) 扫描自检
 * ============================================================================ */
#if CONFIG_BT_NIMBLE_ENABLED

#define BLE_MAX_UNIQUE_DEVS 32

typedef struct {
    uint8_t addr[6];
} ble_seen_addr_t;

static SemaphoreHandle_t s_ble_scan_done_sem = NULL;
static SemaphoreHandle_t s_ble_task_exit_sem = NULL;
static ble_seen_addr_t s_seen_addrs[BLE_MAX_UNIQUE_DEVS];
static int s_seen_count = 0;
static int s_adv_packets = 0;
static int8_t s_best_ble_rssi = -127;
static int s_ble_start_err = 0;

static bool ble_record_unique_addr(const uint8_t addr[6])
{
    for (int i = 0; i < s_seen_count; i++) {
        if (memcmp(s_seen_addrs[i].addr, addr, 6) == 0) {
            return false;
        }
    }
    if (s_seen_count < BLE_MAX_UNIQUE_DEVS) {
        memcpy(s_seen_addrs[s_seen_count].addr, addr, 6);
        s_seen_count++;
    }
    return true;
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        s_adv_packets++;
        if (event->disc.rssi > s_best_ble_rssi) {
            s_best_ble_rssi = event->disc.rssi;
        }
        bool is_new = ble_record_unique_addr(event->disc.addr.val);
        if (is_new && s_seen_count <= 5) {
            struct ble_hs_adv_fields fields;
            char name_buf[24] = "(unnamed)";
            if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0 &&
                fields.name != NULL && fields.name_len > 0) {
                size_t copy_len = fields.name_len < sizeof(name_buf) - 1 ? fields.name_len : sizeof(name_buf) - 1;
                memcpy(name_buf, fields.name, copy_len);
                name_buf[copy_len] = '\0';
            }
            const uint8_t *a = event->disc.addr.val;
            ESP_LOGI(TAG, "  BLE[%d] %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d dBm name=%s",
                     s_seen_count, a[5], a[4], a[3], a[2], a[1], a[0], event->disc.rssi, name_buf);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (s_ble_scan_done_sem) {
            xSemaphoreGive(s_ble_scan_done_sem);
        }
        return 0;
    default:
        return 0;
    }
}

static void ble_on_sync(void)
{
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &own_addr_type);
    }
    if (rc != 0) {
        s_ble_start_err = rc;
        if (s_ble_scan_done_sem) {
            xSemaphoreGive(s_ble_scan_done_sem);
        }
        return;
    }

    struct ble_gap_disc_params disc_params = { 0 };
    disc_params.filter_duplicates = 0;
    disc_params.passive = 1;
    disc_params.itvl = 0x0050;   /* 50ms */
    disc_params.window = 0x0030; /* 30ms */

    rc = ble_gap_disc(own_addr_type, 3000, &disc_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        s_ble_start_err = rc;
        if (s_ble_scan_done_sem) {
            xSemaphoreGive(s_ble_scan_done_sem);
        }
    }
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    /* 注意: nimble_port_freertos_deinit() 内部会调用 vTaskDelete(NULL)，必须先释放信号量 */
    if (s_ble_task_exit_sem) {
        xSemaphoreGive(s_ble_task_exit_sem);
    }
    nimble_port_freertos_deinit();
}

void test_ble_scan(test_result_t *res)
{
    res->name = "12. BLE 5.0 Scan";
    bsp_lcd_show_banner("BLE 5.0 SCAN TEST", "Scanning BLE (3s)...", "NimBLE observer mode");

    esp_err_t ret = ensure_nvs_and_netif();
    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "NVS err:%s", esp_err_to_name(ret));
        return;
    }

    s_seen_count = 0;
    s_adv_packets = 0;
    s_best_ble_rssi = -127;
    s_ble_start_err = 0;

    s_ble_scan_done_sem = xSemaphoreCreateBinary();
    s_ble_task_exit_sem = xSemaphoreCreateBinary();
    if (!s_ble_scan_done_sem || !s_ble_task_exit_sem) {
        if (s_ble_scan_done_sem) vSemaphoreDelete(s_ble_scan_done_sem);
        if (s_ble_task_exit_sem) vSemaphoreDelete(s_ble_task_exit_sem);
        s_ble_scan_done_sem = NULL;
        s_ble_task_exit_sem = NULL;
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Sem alloc failed");
        return;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        vSemaphoreDelete(s_ble_scan_done_sem);
        vSemaphoreDelete(s_ble_task_exit_sem);
        s_ble_scan_done_sem = NULL;
        s_ble_task_exit_sem = NULL;
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "nimble_init:%s", esp_err_to_name(ret));
        return;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    /* 等待 3s 扫描完成（超时上限 4.5s） */
    bool completed = (xSemaphoreTake(s_ble_scan_done_sem, pdMS_TO_TICKS(4500)) == pdTRUE);

    /* 停止 NimBLE host 并释放控制器，确保支持多次重复测试 */
    if (nimble_port_stop() == 0) {
        xSemaphoreTake(s_ble_task_exit_sem, pdMS_TO_TICKS(2000));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    nimble_port_deinit();

    vSemaphoreDelete(s_ble_scan_done_sem);
    vSemaphoreDelete(s_ble_task_exit_sem);
    s_ble_scan_done_sem = NULL;
    s_ble_task_exit_sem = NULL;

    ESP_LOGI(TAG, "BLE scan done: completed=%d, err=%d, unique_devs=%d, adv_pkts=%d, best_rssi=%d",
             completed, s_ble_start_err, s_seen_count, s_adv_packets, s_best_ble_rssi);

    if (s_ble_start_err != 0 || !completed) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "BLE err=%d done=%d", s_ble_start_err, completed);
    } else if (s_seen_count > 0) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "%d devs (%d pkts %ddBm)",
                 s_seen_count, s_adv_packets, s_best_ble_rssi);
    } else {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "RF OK, 0 BLE nearby");
    }
}

#else

void test_ble_scan(test_result_t *res)
{
    res->name = "12. BLE 5.0 Scan";
    res->status = TEST_STATUS_SKIP;
    snprintf(res->detail, sizeof(res->detail), "CONFIG_BT_NIMBLE disabled");
}

#endif
