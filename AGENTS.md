# AGENTS.md — 立创·实战派 ESP32-S3 (`LCKFB-SZPI-ESP32-S3-VA`) 工程开发规范

本文件记录经**真机验证通过**的硬件引脚、驱动时序与构建红线。修改任何外设驱动、重构 BSP 或新增功能前必须遵守以下约定。

---

## 1. 目标硬件与基础配置

- **开发板**：立创·实战派 ESP32-S3 (`LCKFB-SZPI-ESP32-S3-VA`)
- **模组**：`ESP32-S3-WROOM-1-N16R8`（16MB QIO Flash @ 80MHz + 8MB Octal PSRAM @ 80MHz）
- **开发框架**：ESP-IDF `v5.3.2`（`>=5.1.0,<6.0.0`），关键配置见 `sdkconfig.defaults` 与 `main/board_pins.h`

---

## 2. 经验证的硬件引脚与 I2C 地址速查

### 共享 I2C0 总线（`I2C_NUM_0`, `SDA=GPIO1`, `SCL=GPIO2`, `100kHz`）
| 芯片 | 7-bit 地址 | 说明 |
| :--- | :---: | :--- |
| `ES8311` | `0x18` | 音频 DAC (`ES8311_ADDRRES_0`)，驱动 NS4150B 喇叭 |
| `PCA9557` | `0x19` | 8 位 IO 扩展（上电初始化 `OUTPUT=0x05`, `CONFIG=0xF8`） |
| `GC0308` | `0x21` | DVP 摄像头 SCCB（仅在 `PCA9557 IO2 DVP_PWDN=0` 时可见） |
| `FT6336` | `0x38` | 电容触摸（兼容 `esp_lcd_touch_ft5x06`） |
| `ES7210` | `0x41` | 4 路音频 ADC，接双路 ZTS6216 麦克风 |
| `QMI8658` | `0x6A` | 6 轴 IMU（`WHO_AM_I=0x05`） |

### PCA9557 扩展 IO (`0x19`)
- `IO0` (`BIT(0)`): `LCD_CS` — ST7789 片选（**低电平有效**）
- `IO1` (`BIT(1)`): `PA_EN` — NS4150B 音频功放使能（**高电平有效**）
- `IO2` (`BIT(2)`): `DVP_PWDN` — GC0308 休眠控制（**0=工作，1=休眠**）

### 主要外设引脚
- **2.0" IPS LCD (ST7789, 320×240)**：`SPI3_HOST` @ `80MHz`，`MOSI=GPIO40`，`CLK=GPIO41`，`DC=GPIO39`，`CS=GPIO_NUM_NC`（PCA9557 IO0），`RST=GPIO_NUM_NC`，`BL=GPIO42`（低电平点亮，LEDC `TIMER_0/CH_0`）
- **音频 I2S0（ES8311 + ES7210 共用时钟）**：`MCLK=GPIO38`，`BCLK=GPIO14`，`WS=GPIO13`，`DOUT=GPIO45`，`DIN=GPIO12`
- **TF 卡（SDMMC 1-bit）**：`CLK=GPIO47`，`CMD=GPIO48`，`D0=GPIO21`
- **摄像头（GC0308 DVP）**：`XCLK=GPIO5`（24MHz，LEDC `TIMER_1/CH_1`），`VSYNC=GPIO3`，`HREF=GPIO46`（Strapping 引脚），`PCLK=GPIO7`，`D7..D0 = GPIO 9, 4, 6, 15, 17, 8, 18, 16`
- **按键与外拓口**：`BOOT=GPIO0`（低电平有效，内部上拉），外拓 IO `GPIO10`、`GPIO11`

---

## 3. 硬件驱动红线（禁止破坏）

1. **ST7789 + PCA9557 防黑屏初始化顺序（核心踩坑点）**
   - 必须使用 `SPI3_HOST`、`pclk_hz = 80MHz`、`spi_mode = 2`、`cs_gpio_num = GPIO_NUM_NC`。
   - 初始化与重初始化**必须严格按以下顺序**执行（见 `main/board_bsp.c` 中 `lcd_panel_configure_sequence()`）：
     1. `bsp_pca9557_set_lcd_cs(1)`（先取消片选）
     2. `esp_lcd_panel_reset(panel)`（利用首次 SPI 传输让 Mode 2 的 SCLK 稳定到空闲高电平）
     3. `bsp_pca9557_set_lcd_cs(0)`（再拉低片选并保持低电平）
     4. `esp_lcd_panel_init(panel)`
     5. `esp_lcd_panel_invert_color(panel, true)` → `swap_xy(true)` → `mirror(true, false)`
   - **禁止**在 `esp_lcd_panel_reset()` 之前就把 `LCD_CS` 拉低，否则首次 SCLK 上升沿会导致 ST7789 命令错位黑屏。

2. **LCD 背光反相与 LEDC 定时器隔离**
   - `GPIO42` 通过 PMOS 控制背光（**低电平点亮**），10-bit LEDC 占空比必须反相：`duty = 1023 * (100 - percent) / 100`；熄灭时为 `1023`。
   - LCD 背光固定使用 `LEDC_TIMER_0 / LEDC_CHANNEL_0`，摄像头 XCLK 固定使用 `LEDC_TIMER_1 / LEDC_CHANNEL_1`，禁止共用同一定时器。

3. **LCD SPI DMA 内部缓冲与字节序**
   - **严禁**把 PSRAM 中的图像帧（如 GC0308 `CAMERA_FB_IN_PSRAM`）直接传给 `esp_lcd_panel_draw_bitmap()`。
   - 必须先分条拷贝到内部 DMA 内存缓冲（`MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`，如 24 行 `s_line_buf`），并配合 `on_color_trans_done` ISR 二值信号量同步发送。
   - GC0308 输出的 `PIXFORMAT_RGB565` 已为大端字节序，直接推流无需 `swap16`；代码自绘 RGB565 像素需做 `swap16`。

4. **FT6336 触摸坐标系映射**
   - 硬件原始坐标系为竖屏 `240×320`，与横屏 `320×240` LCD 对齐时保持：`x_max = 240`, `y_max = 320`, `swap_xy = 1`, `mirror_x = 1`, `mirror_y = 0`。

5. **传统 I2C 驱动一致性（防 ESP-IDF v5.x 新旧驱动冲突）**
   - 本工程统一使用传统 `driver/i2c.h` (`I2C_NUM_0`) 配合 `esp32-camera`、`es8311 ~1.0.0`、`es7210 ~1.0.0`、`esp_lcd_touch_ft5x06 ~1.0.6`。
   - `sdkconfig.defaults` 必须保留：
     ```ini
     CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY=y
     CONFIG_SCCB_HARDWARE_I2C_PORT0=y
     ```
   - 摄像头配置中 `pin_sccb_sda = -1`、`sccb_i2c_port = I2C_NUM_0`（复用已初始化的 I2C0），检测到 `GC0308_PID` 时调用 `s->set_hmirror(s, 1)`。
   - 除非用户明确要求全量迁移所有外设组件，否则**禁止**混入新版 `driver/i2c_master.h`。

6. **音频 I2S0 通道复用**
   - `ES8311`（标准 I2S TX）与 `ES7210`（TDM RX）共用 `I2S_NUM_0`。若分别独立启停，切换前必须关闭功放 `PA_EN=0` 并彻底 `i2s_channel_disable` + `i2s_del_channel`。

7. **TF 卡数据安全红线**
   - 挂载 FATFS 时 `esp_vfs_fat_sdmmc_mount_config_t.format_if_mount_failed` **必须恒为 `false`**，绝不允许自动格式化用户存储卡。

8. **编译告警与缓冲区安全**
   - ESP-IDF 默认开启 `-Werror=format-truncation`，所有 `snprintf` 目标字符数组必须按最大可能输出长度留足余量。

---

## 4. 编译与合并固件规范

使用 ESP-IDF v5.3.2 容器编译：
```bash
docker run --rm -v "$PWD":/project -w /project espressif/idf:v5.3.2 \
  bash -c "idf.py set-target esp32s3 && idf.py build && idf.py merge-bin -o szpi_esp32s3_selftest_merged.bin"
```
- **注意**：`idf.py merge-bin` 在 `build/` 子目录内执行，`-o` 参数不要带 `build/` 前缀，产物路径为 `build/szpi_esp32s3_selftest_merged.bin`。
- 合并固件烧录起始地址为 **`0x0`**。
