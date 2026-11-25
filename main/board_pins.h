#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_common.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_bit_defs.h"

/* ============================================================================
 * 立创·实战派 ESP32-S3 (LCKFB-SZPI-ESP32-S3-VA) 硬件引脚与 I2C 地址定义
 * 模组: ESP32-S3-WROOM-1-N16R8 (16MB QIO Flash + 8MB Octal PSRAM)
 * ============================================================================ */

/* ---------- 1. 按键与外拓接口 ---------- */
#define BSP_BOOT_KEY_IO         GPIO_NUM_0      // 用户/BOOT 按键 (低电平有效，内部上拉)
#define BSP_EXT_GPIO_10         GPIO_NUM_10     // 5P GH1.25 多功能外拓口 IO10
#define BSP_EXT_GPIO_11         GPIO_NUM_11     // 5P GH1.25 多功能外拓口 IO11

/* ---------- 2. 共享 I2C0 总线 ---------- */
#define BSP_I2C_NUM             I2C_NUM_0
#define BSP_I2C_SDA_IO          GPIO_NUM_1
#define BSP_I2C_SCL_IO          GPIO_NUM_2
#define BSP_I2C_FREQ_HZ         100000          // 100kHz，兼容所有板载 I2C 芯片与 SCCB

/* 板载 I2C 7-bit 设备地址 */
#define BSP_I2C_ADDR_ES8311     0x18            // 音频 DAC (AD0=0 -> ES8311_ADDRRES_0)
#define BSP_I2C_ADDR_PCA9557    0x19            // 8位 IO 扩展芯片 (A2=0, A1=0, A0=1)
#define BSP_I2C_ADDR_GC0308     0x21            // 摄像头 SCCB (DVP_PWDN=0 上电后可见)
#define BSP_I2C_ADDR_FT6336     0x38            // 电容触摸芯片 FT6336 (兼容 FT5x06)
#define BSP_I2C_ADDR_ES7210     0x41            // 4路音频 ADC (AD0=1, AD1=0)
#define BSP_I2C_ADDR_QMI8658    0x6A            // 6轴姿态传感器 (加速度+陀螺仪)

/* ---------- 3. PCA9557 扩展 IO 寄存器与引脚 ---------- */
#define PCA9557_REG_INPUT       0x00
#define PCA9557_REG_OUTPUT      0x01
#define PCA9557_REG_POLARITY    0x02
#define PCA9557_REG_CONFIG      0x03

#define PCA9557_BIT_LCD_CS      BIT(0)          // IO0: LCD SPI 片选 (低电平有效)
#define PCA9557_BIT_PA_EN       BIT(1)          // IO1: NS4150B 音频功放使能 (高电平有效)
#define PCA9557_BIT_DVP_PWDN    BIT(2)          // IO2: GC0308 摄像头休眠控制 (低电平工作, 高电平休眠)

/* ---------- 4. 2.0寸 IPS LCD (ST7789, 320x240, SPI3) ---------- */
#define BSP_LCD_SPI_HOST        SPI3_HOST
#define BSP_LCD_H_RES           320
#define BSP_LCD_V_RES           240
#define BSP_LCD_PIXEL_CLK_HZ    (80 * 1000 * 1000)
#define BSP_LCD_SPI_MOSI_IO     GPIO_NUM_40
#define BSP_LCD_SPI_CLK_IO      GPIO_NUM_41
#define BSP_LCD_DC_IO           GPIO_NUM_39
#define BSP_LCD_BACKLIGHT_IO    GPIO_NUM_42     // 低电平点亮 (PMOS 控制，LEDC PWM 占空比反相)
#define BSP_LCD_BL_LEDC_TIMER   LEDC_TIMER_0
#define BSP_LCD_BL_LEDC_CH      LEDC_CHANNEL_0

/* ---------- 5. 音频 I2S0 (ES8311 DAC + ES7210 ADC 共用时钟线) ---------- */
#define BSP_I2S_NUM             I2S_NUM_0
#define BSP_I2S_MCLK_IO         GPIO_NUM_38
#define BSP_I2S_BCLK_IO         GPIO_NUM_14
#define BSP_I2S_WS_IO           GPIO_NUM_13
#define BSP_I2S_DOUT_IO         GPIO_NUM_45     // 输出到 ES8311 DAC -> NS4150B 喇叭
#define BSP_I2S_DIN_IO          GPIO_NUM_12     // 来自 ES7210 ADC <- 双路 ZTS6216 麦克风

/* ---------- 6. TF 卡槽 (SDMMC 1-bit SD 模式) ---------- */
#define BSP_SD_CLK_IO           GPIO_NUM_47
#define BSP_SD_CMD_IO           GPIO_NUM_48
#define BSP_SD_D0_IO            GPIO_NUM_21
#define BSP_SD_MOUNT_POINT      "/sdcard"

/* ---------- 7. 摄像头 (GC0308, 30万像素 DVP) ---------- */
#define BSP_CAM_XCLK_IO         GPIO_NUM_5
#define BSP_CAM_XCLK_FREQ_HZ    24000000
#define BSP_CAM_VSYNC_IO        GPIO_NUM_3
#define BSP_CAM_HREF_IO         GPIO_NUM_46     // 注意: IO46 为 Strapping 引脚，板载已下拉
#define BSP_CAM_PCLK_IO         GPIO_NUM_7
#define BSP_CAM_D7_IO           GPIO_NUM_9
#define BSP_CAM_D6_IO           GPIO_NUM_4
#define BSP_CAM_D5_IO           GPIO_NUM_6
#define BSP_CAM_D4_IO           GPIO_NUM_15
#define BSP_CAM_D3_IO           GPIO_NUM_17
#define BSP_CAM_D2_IO           GPIO_NUM_8
#define BSP_CAM_D1_IO           GPIO_NUM_18
#define BSP_CAM_D0_IO           GPIO_NUM_16
#define BSP_CAM_LEDC_TIMER      LEDC_TIMER_1    // 与 LCD 背光 LEDC_TIMER_0 错开
#define BSP_CAM_LEDC_CH         LEDC_CHANNEL_1
