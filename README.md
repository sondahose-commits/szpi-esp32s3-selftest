# 立创·实战派 ESP32-S3 开发板全功能硬件自检工程 (ESP-IDF)

针对 **立创·实战派 ESP32-S3 (`LCKFB-SZPI-ESP32-S3-VA`)**（`ESP32-S3-WROOM-1-N16R8`）编写的一站式 ESP-IDF 硬件自检固件，自动检测板载全部外设，并在串口终端与 2.0 寸 IPS 液晶屏上同步输出测试结果汇总。

## 测试覆盖项 (共 13 项)

| 序号 | 测试项 | 硬件芯片 / 接口 | 检测内容 |
|---|---|---|---|
| 1 | MCU / Flash / PSRAM | ESP32-S3-WROOM-1-N16R8 | 芯片版本、双核、16MB Flash 容量、8MB Octal PSRAM 512KB 读写校验 |
| 2 | I2C0 总线扫描 | GPIO1 (SDA) / GPIO2 (SCL) | 扫描并验证 `0x18`(ES8311)、`0x19`(PCA9557)、`0x38`(FT6336)、`0x41`(ES7210)、`0x6A`(QMI8658) 全部应答 |
| 3 | IO 扩展芯片 | PCA9557 (`0x19`) | 寄存器配置回读及 `LCD_CS` 位翻转验证 |
| 4 | 2.0" IPS 屏幕与背光 | ST7789 (SPI3) + LEDC PWM | GPIO42 低电平有效背光 0%→100% 渐变、红/绿/蓝/白纯色及 8 色彩条显示 |
| 5 | 电容触摸屏 | FT6336 (`0x38`) | 芯片通信校验 + 6 秒屏幕手写触点绘制与坐标输出（按 BOOT 可跳过） |
| 6 | 6 轴姿态传感器 | QMI8658 (`0x6A`) | 校验 `WHO_AM_I=0x05`，连续采样加速度/陀螺仪/俯仰滚转角并校验重力模长 (~1g) |
| 7 | TF 卡槽 | 1-bit SDMMC (IO47/48/21) | 只读挂载 FAT32 TF 卡（**绝不自动格式化**），写入/回读/校验/删除临时测试文件 |
| 8 | 音频输出 (喇叭) | ES8311 (`0x18`) + NS4150B | 打开功放 `PA_EN`，代码实时合成 C5-E5-G5 三音和弦并通过 1W 喇叭播放 |
| 9 | 音频输入 (双麦) | ES7210 (`0x41`) + 2×ZTS6216 | I2S0 TDM 模式采集 MIC1/MIC2 2.5 秒，屏幕显示实时音量条并计算交流 RMS/峰值 |
| 10 | 摄像头 | GC0308 (DVP + SCCB) | 拉低 `DVP_PWDN` 唤醒，320×240 RGB565 实时画面推流到 LCD 5 秒并统计帧率 |
| 11 | Wi-Fi 2.4GHz | ESP32-S3 Wi-Fi | STA 模式主动扫描周边热点，输出前 5 个 AP 的 SSID 与 RSSI |
| 12 | BLE 5.0 蓝牙 | ESP32-S3 BLE (NimBLE) | Observer 模式扫描周边 BLE 广播 3 秒，统计设备数量、广播包数与最强 RSSI |
| 13 | 按键与外拓 GPIO | BOOT (IO0) + GH1.25 (IO10/11) | 校验外拓口 IO10/IO11 上下拉电气正常，交互检测 BOOT 按键按下与释放 |

## 编译与烧录步骤

> 推荐 **ESP-IDF v5.1 ~ v5.4**（Windows 需先安装 CH340K 串口驱动；若测试 TF 卡，请**断电状态下**插入 FAT32 格式 TF 卡，切勿带电热插拔）。

```bash
cd szpi_esp32s3_selftest

# 1. 设置目标芯片（会自动根据 sdkconfig.defaults 与 idf_component.yml 下载依赖组件）
idf.py set-target esp32s3

# 2. 编译、烧录并打开串口监视器（将 /dev/ttyUSB0 替换为你的实际串口，Windows 如 COM3）
idf.py -p /dev/ttyUSB0 build flash monitor
```

自检完成后，按下板载 **BOOT 按键 (`GPIO0`)** 可重新执行完整测试流程。
