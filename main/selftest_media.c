#include "selftest.h"
#include "board_bsp.h"
#include "board_pins.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "es7210.h"
#include "es8311.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "test_media";

#define AUDIO_SAMPLE_RATE       16000
#define AUDIO_MCLK_MULTIPLE     I2S_MCLK_MULTIPLE_256
#define AUDIO_MCLK_FREQ_HZ      (AUDIO_SAMPLE_RATE * AUDIO_MCLK_MULTIPLE)

/* ============================================================================
 * 8. 音频输出自检: ES8311 DAC + NS4150B 功放 + 1W 喇叭
 *    动态合成 C5-E5-G5 三音和弦提示音，无需外部 PCM 文件
 * ============================================================================ */
void test_audio_es8311_speaker(test_result_t *res)
{
    res->name = "8. ES8311 Speaker";
    bsp_lcd_show_banner("AUDIO OUT (ES8311)", "Playing 3-note chime", "Listen to 1W speaker...");

    i2s_chan_handle_t tx_chan = NULL;
    es8311_handle_t es_handle = NULL;
    esp_err_t ret = ESP_OK;

    /* 1) 初始化 I2S0 TX (标准 I2S 立体声 16bit @ 16kHz) */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "I2S TX new err:%s", esp_err_to_name(ret));
        return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK_IO,
            .bclk = BSP_I2S_BCLK_IO,
            .ws = BSP_I2S_WS_IO,
            .dout = BSP_I2S_DOUT_IO,
            .din = GPIO_NUM_NC,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = AUDIO_MCLK_MULTIPLE;

    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret == ESP_OK) {
        ret = i2s_channel_enable(tx_chan);
    }
    if (ret != ESP_OK) {
        i2s_del_channel(tx_chan);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "I2S TX init err:%s", esp_err_to_name(ret));
        return;
    }

    /* 2) 配置 ES8311 编解码器 (I2C0 0x18) */
    es_handle = es8311_create(BSP_I2C_NUM, ES8311_ADDRRES_0);
    if (!es_handle) {
        i2s_channel_disable(tx_chan);
        i2s_del_channel(tx_chan);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "es8311_create failed");
        return;
    }

    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = AUDIO_MCLK_FREQ_HZ,
        .sample_frequency = AUDIO_SAMPLE_RATE,
    };

    ret = es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    if (ret == ESP_OK) {
        ret = es8311_sample_frequency_config(es_handle, AUDIO_MCLK_FREQ_HZ, AUDIO_SAMPLE_RATE);
    }
    if (ret == ESP_OK) {
        ret = es8311_voice_volume_set(es_handle, 72, NULL);
    }
    if (ret == ESP_OK) {
        ret = es8311_microphone_config(es_handle, false);
    }

    if (ret != ESP_OK) {
        es8311_delete(es_handle);
        i2s_channel_disable(tx_chan);
        i2s_del_channel(tx_chan);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "ES8311 cfg err:%s", esp_err_to_name(ret));
        return;
    }

    /* 3) 通过 PCA9557 打开 NS4150B 音频功放 */
    bsp_pca9557_set_pa_en(1);
    vTaskDelay(pdMS_TO_TICKS(30));

    /* 4) 播放三音测试提示音: C5 (523Hz) -> E5 (659Hz) -> G5 (784Hz) */
    const float freqs[3] = { 523.25f, 659.25f, 783.99f };
    const int samples_per_note = AUDIO_SAMPLE_RATE * 280 / 1000; /* 每音 280ms */
    int16_t chunk[256 * 2]; /* 256 立体声帧 */
    size_t total_written = 0;
    bool write_ok = true;

    for (int n = 0; n < 3 && write_ok; n++) {
        int pos = 0;
        while (pos < samples_per_note) {
            int count = (samples_per_note - pos < 256) ? (samples_per_note - pos) : 256;
            for (int i = 0; i < count; i++) {
                int idx = pos + i;
                /* 淡入淡出包络，消除爆破音 */
                float env = 1.0f;
                if (idx < 320) {
                    env = (float)idx / 320.0f;
                } else if (idx > samples_per_note - 480) {
                    env = (float)(samples_per_note - idx) / 480.0f;
                }
                float sample = sinf(2.0f * 3.14159265f * freqs[n] * (float)idx / (float)AUDIO_SAMPLE_RATE);
                int16_t pcm = (int16_t)(sample * env * 18000.0f);
                chunk[2 * i] = pcm;
                chunk[2 * i + 1] = pcm;
            }
            size_t bw = 0;
            if (i2s_channel_write(tx_chan, chunk, (size_t)count * 2 * sizeof(int16_t), &bw, pdMS_TO_TICKS(1000)) != ESP_OK) {
                write_ok = false;
                break;
            }
            total_written += bw;
            pos += count;
        }
    }

    /* 5) 关闭功放并彻底释放 I2S0，供后续 ES7210 麦克风测试使用 */
    vTaskDelay(pdMS_TO_TICKS(50));
    bsp_pca9557_set_pa_en(0);
    es8311_voice_volume_set(es_handle, 0, NULL);
    es8311_delete(es_handle);
    i2s_channel_disable(tx_chan);
    i2s_del_channel(tx_chan);

    ESP_LOGI(TAG, "ES8311 playback finished, written=%u bytes", (unsigned)total_written);
    if (write_ok && total_written > 0) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "Chime played (%u KB)", (unsigned)(total_written / 1024));
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "I2S TX write failed");
    }
}

/* ============================================================================
 * 9. 音频输入自检: ES7210 4路ADC + 双路 ZTS6216 麦克风 (I2S0 TDM 模式)
 * ============================================================================ */
void test_audio_es7210_mic(test_result_t *res)
{
    res->name = "9. ES7210 Dual-MIC";
    bsp_lcd_show_banner("AUDIO IN (ES7210)", "Speak or clap (2.5s)", "Monitoring MIC1 & MIC2...");

    i2s_chan_handle_t rx_chan = NULL;
    es7210_dev_handle_t es7210_handle = NULL;
    esp_err_t ret = ESP_OK;

    /* 1) 初始化 I2S0 RX 为 TDM 模式 (采集 MIC1/MIC2 两路) */
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM, I2S_ROLE_MASTER);
    ret = i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan);
    if (ret != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "I2S RX new err:%s", esp_err_to_name(ret));
        return;
    }

    i2s_tdm_config_t tdm_cfg = {
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_TDM_SLOT0 | I2S_TDM_SLOT1),
        .clk_cfg = {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .sample_rate_hz = AUDIO_SAMPLE_RATE,
            .mclk_multiple = AUDIO_MCLK_MULTIPLE,
        },
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK_IO,
            .bclk = BSP_I2S_BCLK_IO,
            .ws = BSP_I2S_WS_IO,
            .dout = GPIO_NUM_NC,
            .din = BSP_I2S_DIN_IO,
        },
    };

    ret = i2s_channel_init_tdm_mode(rx_chan, &tdm_cfg);
    if (ret != ESP_OK) {
        i2s_del_channel(rx_chan);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "I2S TDM init err:%s", esp_err_to_name(ret));
        return;
    }

    /* 2) 配置 ES7210 (I2C0 0x41) */
    es7210_i2c_config_t es7210_i2c_conf = {
        .i2c_port = BSP_I2C_NUM,
        .i2c_addr = BSP_I2C_ADDR_ES7210,
    };
    ret = es7210_new_codec(&es7210_i2c_conf, &es7210_handle);
    if (ret != ESP_OK) {
        i2s_del_channel(rx_chan);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "es7210_new err:%s", esp_err_to_name(ret));
        return;
    }

    es7210_codec_config_t codec_conf = {
        .i2s_format = ES7210_I2S_FMT_I2S,
        .mclk_ratio = AUDIO_MCLK_MULTIPLE,
        .sample_rate_hz = AUDIO_SAMPLE_RATE,
        .bit_width = (es7210_i2s_bits_t)I2S_DATA_BIT_WIDTH_16BIT,
        .mic_bias = ES7210_MIC_BIAS_2V87,
        .mic_gain = ES7210_MIC_GAIN_30DB,
        .flags.tdm_enable = true,
    };
    ret = es7210_config_codec(es7210_handle, &codec_conf);
    if (ret == ESP_OK) {
        ret = es7210_config_volume(es7210_handle, 0);
    }
    if (ret == ESP_OK) {
        ret = i2s_channel_enable(rx_chan);
    }

    if (ret != ESP_OK) {
        es7210_del_codec(es7210_handle);
        i2s_del_channel(rx_chan);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "ES7210 cfg err:%s", esp_err_to_name(ret));
        return;
    }

    /* 3) 持续采样 2.5 秒，统计 MIC1/MIC2 去直流后的交流 RMS 与峰值，并在屏幕画电平条 */
    int16_t *pcm = malloc(1024 * 2 * sizeof(int16_t));
    if (!pcm) {
        i2s_channel_disable(rx_chan);
        i2s_del_channel(rx_chan);
        es7210_del_codec(es7210_handle);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "OOM");
        return;
    }

    double max_rms1 = 0.0, max_rms2 = 0.0;
    int32_t peak1 = 0, peak2 = 0;
    bool read_ok = true;
    int frames_ok = 0;

    /* 丢弃前 2 帧上电瞬态 */
    for (int i = 0; i < 2; i++) {
        size_t br = 0;
        i2s_channel_read(rx_chan, pcm, 1024 * 2 * sizeof(int16_t), &br, pdMS_TO_TICKS(500));
    }

    int64_t end_us = esp_timer_get_time() + 2500000LL;
    while (esp_timer_get_time() < end_us) {
        size_t br = 0;
        if (i2s_channel_read(rx_chan, pcm, 1024 * 2 * sizeof(int16_t), &br, pdMS_TO_TICKS(1000)) != ESP_OK || br == 0) {
            read_ok = false;
            break;
        }
        int pairs = (int)(br / (2 * sizeof(int16_t)));
        int64_t sum1 = 0, sum2 = 0;
        for (int i = 0; i < pairs; i++) {
            sum1 += pcm[2 * i];
            sum2 += pcm[2 * i + 1];
        }
        double mean1 = (double)sum1 / pairs;
        double mean2 = (double)sum2 / pairs;
        double sq1 = 0.0, sq2 = 0.0;

        for (int i = 0; i < pairs; i++) {
            double v1 = (double)pcm[2 * i] - mean1;
            double v2 = (double)pcm[2 * i + 1] - mean2;
            sq1 += v1 * v1;
            sq2 += v2 * v2;
            int32_t a1 = (int32_t)fabs(v1);
            int32_t a2 = (int32_t)fabs(v2);
            if (a1 > peak1) peak1 = a1;
            if (a2 > peak2) peak2 = a2;
        }

        double rms1 = sqrt(sq1 / pairs);
        double rms2 = sqrt(sq2 / pairs);
        if (rms1 > max_rms1) max_rms1 = rms1;
        if (rms2 > max_rms2) max_rms2 = rms2;
        frames_ok++;

        /* 在 LCD 上实时刷新双路麦克风音量条 */
        int bar1 = (int)(rms1 / 25.0);
        int bar2 = (int)(rms2 / 25.0);
        if (bar1 > 220) bar1 = 220;
        if (bar2 > 220) bar2 = 220;
        bsp_lcd_draw_string(12, 138, "MIC1:", COLOR_YELLOW, COLOR_DARK_BG, 1);
        bsp_lcd_fill_rect(56, 136, bar1, 12, COLOR_GREEN);
        bsp_lcd_fill_rect(56 + bar1, 136, 220 - bar1, 12, COLOR_GRAY);

        bsp_lcd_draw_string(12, 162, "MIC2:", COLOR_CYAN, COLOR_DARK_BG, 1);
        bsp_lcd_fill_rect(56, 160, bar2, 12, COLOR_CYAN);
        bsp_lcd_fill_rect(56 + bar2, 160, 220 - bar2, 12, COLOR_GRAY);
    }

    free(pcm);
    i2s_channel_disable(rx_chan);
    i2s_del_channel(rx_chan);
    es7210_del_codec(es7210_handle);

    ESP_LOGI(TAG, "ES7210 done (%d blocks): MIC1 maxRMS=%.1f peak=%" PRId32 " | MIC2 maxRMS=%.1f peak=%" PRId32,
             frames_ok, max_rms1, peak1, max_rms2, peak2);

    /* 正常模拟麦克风即使在安静环境下也会有显著交流底噪 (RMS > 3.0)，而断线/卡死为 0 */
    if (read_ok && frames_ok > 0 && max_rms1 > 3.0 && max_rms2 > 3.0) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "M1:%.0f M2:%.0f (RMS)", max_rms1, max_rms2);
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "Low/No sig M1:%.1f M2:%.1f", max_rms1, max_rms2);
    }
}

/* ============================================================================
 * 10. GC0308 DVP 摄像头自检 (320x240 RGB565 实时画面推流到 LCD)
 * ============================================================================ */
void test_camera_gc0308(test_result_t *res)
{
    res->name = "10. GC0308 Camera";
    bsp_lcd_show_banner("CAMERA TEST (GC0308)", "Initializing DVP...", "Preview 5s (BOOT=skip)");

    /* 1) 拉低 PCA9557 IO2 (DVP_PWDN=0) 给摄像头上电唤醒 */
    if (bsp_pca9557_set_dvp_pwdn(0) != ESP_OK) {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "PCA9557 DVP_PWDN failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(30));

    camera_config_t config = {
        .pin_pwdn = -1,                 /* 已通过 PCA9557 IO2 控制 */
        .pin_reset = -1,
        .pin_xclk = BSP_CAM_XCLK_IO,
        .pin_sccb_sda = -1,             /* 复用已初始化的 I2C0 (GPIO1) */
        .pin_sccb_scl = BSP_I2C_SCL_IO,
        .sccb_i2c_port = BSP_I2C_NUM,

        .pin_d7 = BSP_CAM_D7_IO,
        .pin_d6 = BSP_CAM_D6_IO,
        .pin_d5 = BSP_CAM_D5_IO,
        .pin_d4 = BSP_CAM_D4_IO,
        .pin_d3 = BSP_CAM_D3_IO,
        .pin_d2 = BSP_CAM_D2_IO,
        .pin_d1 = BSP_CAM_D1_IO,
        .pin_d0 = BSP_CAM_D0_IO,
        .pin_vsync = BSP_CAM_VSYNC_IO,
        .pin_href = BSP_CAM_HREF_IO,
        .pin_pclk = BSP_CAM_PCLK_IO,

        .xclk_freq_hz = BSP_CAM_XCLK_FREQ_HZ,
        .ledc_timer = BSP_CAM_LEDC_TIMER,
        .ledc_channel = BSP_CAM_LEDC_CH,

        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QVGA,   /* 320x240，正好铺满 2.0 寸屏 */
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK) {
        bsp_pca9557_set_dvp_pwdn(1);
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "esp_camera_init:%s", esp_err_to_name(ret));
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    uint16_t pid = s ? s->id.PID : 0;
    if (s && pid == GC0308_PID) {
        s->set_hmirror(s, 1); /* 实战派开发板 GC0308 水平镜像校正 */
    }
    ESP_LOGI(TAG, "Camera detected PID=0x%04X (GC0308_PID=0x%04X)", pid, GC0308_PID);

    /* 2) 连续采帧并实时显示到 LCD 5 秒（按 BOOT 可提前结束） */
    int frame_cnt = 0;
    bool has_valid_image = false;
    int64_t start_us = esp_timer_get_time();
    int64_t end_us = start_us + 5LL * 1000000LL;

    while (esp_timer_get_time() < end_us) {
        if (bsp_boot_key_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(150));
            break;
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* 抽检像素是否非全黑/全白，排除 DVP 数据线接触不良 */
        if (!has_valid_image && fb->len >= 1024) {
            uint16_t first = ((const uint16_t *)fb->buf)[0];
            for (size_t i = 1; i < fb->len / 2; i += 67) {
                if (((const uint16_t *)fb->buf)[i] != first) {
                    has_valid_image = true;
                    break;
                }
            }
        }

        /* GC0308 RGB565 输出即为大端格式，直接刷屏无需交换字节序 */
        bsp_lcd_draw_bitmap_raw(0, 0, fb->width, fb->height, fb->buf);
        frame_cnt++;

        if ((frame_cnt % 10) == 0) {
            float fps = (float)frame_cnt * 1e6f / (float)(esp_timer_get_time() - start_us);
            char overlay[32];
            snprintf(overlay, sizeof(overlay), "CAM %dx%d %.1fFPS", fb->width, fb->height, fps);
            bsp_lcd_draw_string(6, 6, overlay, COLOR_YELLOW, COLOR_BLACK, 1);
        }

        esp_camera_fb_return(fb);
    }

    float elapsed_s = (float)(esp_timer_get_time() - start_us) / 1e6f;
    float fps = (elapsed_s > 0.05f) ? ((float)frame_cnt / elapsed_s) : 0.0f;

    /* 3) 释放摄像头并重新进入休眠 (DVP_PWDN=1) 省电 */
    esp_camera_deinit();
    bsp_pca9557_set_dvp_pwdn(1);

    ESP_LOGI(TAG, "Camera test done: PID=0x%04X, frames=%d, avg=%.1f FPS, image=%s",
             pid, frame_cnt, fps, has_valid_image ? "OK" : "BLANK");

    if (frame_cnt >= 5 && has_valid_image) {
        res->status = TEST_STATUS_PASS;
        snprintf(res->detail, sizeof(res->detail), "PID=0x%02X %dF %.1fFPS", pid, frame_cnt, fps);
    } else {
        res->status = TEST_STATUS_FAIL;
        snprintf(res->detail, sizeof(res->detail), "PID=0x%02X F=%d img=%d", pid, frame_cnt, has_valid_image);
    }
}
