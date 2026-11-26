---
name: lckfb-szpi-esp32s3-bsp
description: Verified ESP-IDF v5.x hardware pinout, I2C address map, PCA9557 control, ST7789 LCD SPI Mode 2 + DMA init guardrails, GC0308 camera, FT6336 touch, ES8311/ES7210 audio, QMI8658 IMU, and 1-bit SDMMC configuration for the 立创·实战派 ESP32-S3 (LCKFB-SZPI-ESP32-S3-VA / ESP32-S3-WROOM-1-N16R8) development board. Use whenever developing, debugging, or porting firmware for this board.
---
# 立创·实战派 ESP32-S3 (`LCKFB-SZPI-ESP32-S3-VA`) 板级开发指南 (ESP-IDF v5.x)

当任务涉及**立创·实战派 ESP32-S3**（模组 `ESP32-S3-WROOM-1-N16R8`，16MB QIO Flash + 8MB Octal PSRAM）的驱动编写、调试或移植时，严格遵循以下经真机验证通过的硬件参数与避坑规则。

---

## 1. 硬件引脚与 I2C 地址表

### 共享 I2C0 (`I2C_NUM_0`, `100kHz`)
- `SDA = GPIO1`, `SCL = GPIO2`（开启内部上拉）
- 设备 7-bit 地址：
  - `0x18`: `ES8311` 音频 DAC (`ES8311_ADDRRES_0`)
  - `0x19`: `PCA9557` 8 位 IO 扩展芯片
  - `0x21`: `GC0308` 摄像头 SCCB（仅 `PCA9557 IO2 DVP_PWDN = 0` 唤醒后可见）
  - `0x38`: `FT6336` 电容触摸（兼容 `espressif/esp_lcd_touch_ft5x06`）
  - `0x41`: `ES7210` 4 路音频 ADC（双路 ZTS6216 麦克风）
  - `0x6A`: `QMI8658` 6 轴 IMU（`WHO_AM_I` 寄存器 `0x00` 读值为 `0x05`）

### PCA9557 (`0x19`) 寄存器与引脚
- 寄存器：`INPUT=0x00`, `OUTPUT=0x01`, `POLARITY=0x02`, `CONFIG=0x03`
- 推荐默认初始化：`OUTPUT = 0x05`（`DVP_PWDN=1` 休眠, `PA_EN=0` 关功放, `LCD_CS=1` 未片选），`CONFIG = 0xF8`（`IO0..IO2` 输出，`IO3..IO7` 输入）
- 引脚位定义：
  - `BIT(0)` (`IO0`): `LCD_CS` — ST7789 片选，**低电平有效**
  - `BIT(1)` (`IO1`): `PA_EN` — NS4150B 音频功放使能，**高电平有效**
  - `BIT(2)` (`IO2`): `DVP_PWDN` — GC0308 休眠控制，**0=上电工作，1=休眠**

### 2.0" IPS LCD (`ST7789`, `320x240`)
- 总线：`SPI3_HOST` @ `80 MHz`，`spi_mode = 2`
- `MOSI = GPIO40`, `CLK = GPIO41`, `DC = GPIO39`, `CS = GPIO_NUM_NC`（由 PCA9557 IO0 控制）, `RST = GPIO_NUM_NC`
- 背光：`GPIO42`（PMOS 控制，**低电平点亮**），LEDC 使用 `LEDC_TIMER_0` / `LEDC_CHANNEL_0`

### 音频 (`I2S_NUM_0`)
- 共用时钟：`MCLK = GPIO38`, `BCLK = GPIO14`, `WS = GPIO13`
- `DOUT = GPIO45`（接 ES8311 DAC → NS4150B 喇叭，标准 I2S 立体声）
- `DIN = GPIO12`（接 ES7210 ADC ← 双麦克风，TDM 模式 `SLOT0 | SLOT1`）

### TF 卡 (`SDMMC 1-bit`)
- `CLK = GPIO47`, `CMD = GPIO48`, `D0 = GPIO21`（开启 `SDMMC_SLOT_FLAG_INTERNAL_PULLUP`）

### 摄像头 (`GC0308` DVP)
- `XCLK = GPIO5` (`24 MHz`, 使用 `LEDC_TIMER_1` / `LEDC_CHANNEL_1` 与背光错开)
- `VSYNC = GPIO3`, `HREF = GPIO46`（注意 GPIO46 是 Strapping 引脚，板载已下拉）, `PCLK = GPIO7`
- `D7..D0 = GPIO 9, 4, 6, 15, 17, 8, 18, 16`
- `pin_pwdn = -1`（由 PCA9557 IO2 控制）, `pin_reset = -1`, `pin_sccb_sda = -1`, `pin_sccb_scl = GPIO2`, `sccb_i2c_port = I2C_NUM_0`

### 按键与外拓口
- `BOOT = GPIO0`（低电平有效，内部上拉）
- 外拓 5P GH1.25 GPIO：`GPIO10`, `GPIO11`

---

## 2. 真机踩坑红线（务必遵守）

### 红线 1：ST7789 + PCA9557 初始化顺序（防黑屏）
由于 `LCD_CS` 挂在 PCA9557 IO0 上且为常低片选，而 SPI Mode 2 的 SCLK 空闲为高电平，ESP32-S3 首次 SPI 传输前 SCLK 默认为低电平。**必须先让 `CS=1` 跑一次 `reset` 把 SCLK 稳在高电平，再拉低 `CS=0` 执行 `init`**：

```c
ESP_RETURN_ON_ERROR(bsp_pca9557_set_lcd_cs(1), TAG, "CS high");
ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "reset");
ESP_RETURN_ON_ERROR(bsp_pca9557_set_lcd_cs(0), TAG, "CS low");
ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init");
ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, true), TAG, "invert");
ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panel, true), TAG, "swap_xy");
ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panel, true, false), TAG, "mirror");
```

### 红线 2：LCD SPI DMA 内部缓冲（防 PSRAM 刷屏崩溃/队列失败）
- 严禁把 PSRAM 缓冲区（例如摄像头 `CAMERA_FB_IN_PSRAM`）直接交给 `esp_lcd_panel_draw_bitmap()`。
- 必须分配内部 DMA RAM 分条缓冲（`heap_caps_malloc(320 * 24 * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)`），逐条 `memcpy` 后调用 `esp_lcd_panel_draw_bitmap()`，并注册 `on_color_trans_done` ISR 回调释放二值信号量实现同步发送。
- `GC0308` 的 `PIXFORMAT_RGB565` 输出已是大端格式，直接发送即可；CPU 自绘 RGB565 需先做高低字节交换 (`swap16`)。

### 红线 3：背光反相 PWM
- `GPIO42` 低电平有效：10-bit LEDC 占空比按 `duty = 1023 * (100 - percent) / 100` 计算；熄灭时 `duty = 1023`。

### 红线 4：FT6336 触摸方向对齐
- 使用 `esp_lcd_touch_new_i2c_ft5x06` 时，配置 `x_max = 240`, `y_max = 320`, `swap_xy = 1`, `mirror_x = 1`, `mirror_y = 0` 以匹配横屏 `320x240` LCD。

### 红线 5：I2C 新旧驱动冲突规避
- 使用 `espressif/esp32-camera` + `es8311 ~1.0.0` + `es7210 ~1.0.0` + `esp_lcd_touch_ft5x06 ~1.0.6` 时，统一用传统 `driver/i2c.h` (`I2C_NUM_0`)，并在 `sdkconfig.defaults` 固定开启：
  ```ini
  CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY=y
  CONFIG_SCCB_HARDWARE_I2C_PORT0=y
  ```
- 禁止在同一工程内混用 `driver/i2c.h` 与 `driver/i2c_master.h`。
- 摄像头初始化后若 `pid == GC0308_PID`，调用 `s->set_hmirror(s, 1)` 修正水平镜像。

### 红线 6：TF 卡数据安全
- 挂载 FATFS 时必须设 `.format_if_mount_failed = false`，禁止自动格式化用户 TF 卡。

---

## 3. `sdkconfig.defaults` 最小基线

```ini
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_BOOT_INIT=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_GC0308_SUPPORT=y
CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY=y
CONFIG_SCCB_HARDWARE_I2C_PORT0=y
```

## 4. 合并固件提醒
- 执行 `idf.py merge-bin -o <name>.bin` 时，该命令在 `build/` 目录内运行，`-o` 参数不要带 `build/` 前缀；合并后的单文件烧录地址为 `0x0`。