# ESP32-S3健康监测手表开发实战（四）：软件I2C实现与OLED显示优化

> 本篇作为系列收官之作，讲解当硬件I2C资源不足时如何用GPIO模拟软件I2C协议，以及SSD1306 OLED驱动开发和界面优化技巧。

## 1. 为什么需要软件I2C？

### 1.1 ESP32-S3的I2C资源限制

ESP32-S3只有两路硬件I2C控制器：

| I2C端口 | 本项目使用 | 特点 |
|--------|-----------|------|
| I2C_NUM_0 | MPU6050（400kHz） | 硬件控制，CPU占用低 |
| I2C_NUM_1 | MAX30102（100kHz） | 硬件控制，支持中断 |
| 软件I2C | SSD1306 OLED | GPIO模拟，灵活 |

当两路硬件I2C已被占用，第三个I2C设备只能用软件模拟。

### 1.2 软件I2C的适用场景

**适合软件I2C的设备：**
- 只写设备（如OLED显示屏）：不需要读取数据
- 低速设备：对时序要求不严格
- 非实时设备：不需要高频率刷新

**不适合的场景：**
- 高速数据采集（如加速度计）
- 需要精确时序的设备
- 双向通信频繁的设备

### 1.3 本项目的配置

```c
// 软件I2C配置 (OLED)
#define APP_OLED_SDA_PIN        GPIO_NUM_10     // 任意GPIO
#define APP_OLED_SCL_PIN        GPIO_NUM_11     // 任意GPIO
#define APP_OLED_I2C_FREQ_KHZ   100             // 100kHz
```

## 2. I2C协议时序分析

### 2.1 I2C基础时序

I2C协议使用两根线进行通信：
- **SDA**：数据线（双向）
- **SCL**：时钟线（主机控制）

```
空闲状态：SDA=高, SCL=高
数据有效：SCL高电平期间SDA稳定
数据变化：SCL低电平期间SDA可变
```

### 2.2 起始条件（Start）

**定义**：SCL高电平期间，SDA从高变低

```
SDA  ───┐     ┌───
        │     │
        └─────┘

SCL  ─────────────
            │
            └─── SCL保持高

时间轴  →→→→→→→→→→→
```

**代码实现：**

```c
static void soft_i2c_start(soft_i2c_handle_t *handle)
{
    sda_set_output(handle);
    SDA_HIGH(handle);      // SDA先高
    I2C_DELAY(handle);
    SCL_HIGH(handle);      // SCL高
    I2C_DELAY(handle);
    SDA_LOW(handle);       // ★ SDA下降沿 = 起始信号
    I2C_DELAY(handle);
    SCL_LOW(handle);       // SCL拉低，准备传输
    I2C_DELAY(handle);
}
```

### 2.3 停止条件（Stop）

**定义**：SCL高电平期间，SDA从低变高

```
SDA       ┌─────────
          │
    ──────┘

SCL  ─────────────
          │
    ──────┘

时间轴  →→→→→→→→→→→
```

**代码实现：**

```c
static void soft_i2c_stop(soft_i2c_handle_t *handle)
{
    sda_set_output(handle);
    SDA_LOW(handle);       // SDA先低
    I2C_DELAY(handle);
    SCL_HIGH(handle);      // SCL高
    I2C_DELAY(handle);
    SDA_HIGH(handle);      // ★ SDA上升沿 = 停止信号
    I2C_DELAY(handle);
}
```

### 2.4 数据传输与ACK

**数据位传输**：MSB First，SCL上升沿采样

```
         D7    D6    D5    D4    D3    D2    D1    D0    ACK
SDA   ─╱──╲─╱──╲─╱──╲─╱──╲─╱──╲─╱──╲─╱──╲─╱──╲─╱──╲─
      │    │    │    │    │    │    │    │    │    │
SCL   ┴────┴────┴────┴────┴────┴────┴────┴────┴────┴─
        ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑
       采样  采样  采样  采样  采样  采样  采样  采样  读ACK
```

**ACK响应**：
- **ACK（确认）**：从机拉低SDA = 0
- **NACK（非确认）**：SDA保持高 = 1

## 3. GPIO模拟I2C实现

### 3.1 GPIO配置

```c
esp_err_t soft_i2c_init(soft_i2c_handle_t *handle, const soft_i2c_config_t *config)
{
    // 计算半周期延时
    // 100kHz → 周期10us → 半周期5us
    if (config->freq_khz > 0 && config->freq_khz <= 100) {
        handle->delay_us = 500 / config->freq_khz;
    } else {
        handle->delay_us = 10;  // 默认约50kHz
    }
    if (handle->delay_us < 5) {
        handle->delay_us = 5;   // 最小延时保护
    }

    // 配置SDA为开漏输出（标准I2C模式）
    gpio_config_t sda_conf = {
        .pin_bit_mask = (1ULL << config->sda_pin),
        .mode = GPIO_MODE_OUTPUT_OD,     // ★ 开漏输出
        .pull_up_en = GPIO_PULLUP_ENABLE,// 启用内部上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // 配置SCL为开漏输出
    gpio_config_t scl_conf = {
        .pin_bit_mask = (1ULL << config->scl_pin),
        .mode = GPIO_MODE_OUTPUT_OD,     // ★ 开漏输出
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&sda_conf);
    gpio_config(&scl_conf);

    // 初始化为空闲状态
    gpio_set_level(config->sda_pin, 1);
    gpio_set_level(config->scl_pin, 1);

    return ESP_OK;
}
```

### 3.2 为什么使用开漏输出？

I2C协议要求使用开漏（Open-Drain）输出：

```
推挽输出 vs 开漏输出：

推挽输出：          开漏输出：
  VCC                  VCC
   │                    │
  ┌┴┐               上拉电阻
  │P│                   │
  └┬┘                   ├────→ 输出
   ├────→ 输出          │
  ┌┴┐               ┌───┴───┐
  │N│               │   N   │
  └┬┘               └───┬───┘
   │                    │
  GND                  GND

推挽：可输出高/低电平    开漏：只能拉低，高电平靠上拉
```

**开漏输出的优势：**
1. 多设备共享总线时不会短路
2. 支持线与（Wired-AND）逻辑
3. 支持时钟拉伸（Clock Stretching）

### 3.3 字节发送实现

```c
static uint8_t soft_i2c_write_byte(soft_i2c_handle_t *handle, uint8_t data)
{
    uint8_t ack;
    sda_set_output(handle);

    // 发送8位数据，MSB First
    for (int i = 7; i >= 0; i--) {
        SCL_LOW(handle);                    // SCL低，可以改变SDA
        if (data & (1 << i)) {
            SDA_HIGH(handle);               // 发送1
        } else {
            SDA_LOW(handle);                // 发送0
        }
        I2C_DELAY(handle);
        SCL_HIGH(handle);                   // SCL高，数据稳定
        I2C_DELAY(handle);
    }
    SCL_LOW(handle);
    I2C_DELAY(handle);

    // 释放SDA，读取ACK
    SDA_HIGH(handle);                       // 释放SDA
    sda_set_input(handle);                  // 切换为输入模式
    I2C_DELAY(handle);
    I2C_DELAY(handle);                      // 等待从设备响应
    SCL_HIGH(handle);
    I2C_DELAY(handle);
    ack = SDA_READ(handle);                 // 读取ACK (0=ACK, 1=NACK)
    SCL_LOW(handle);
    I2C_DELAY(handle);

    sda_set_output(handle);
    SDA_HIGH(handle);

    return ack;
}
```

### 3.4 延时实现

```c
#include "esp_rom_sys.h"

#define I2C_DELAY(h)  esp_rom_delay_us((h)->delay_us)
```

ESP-IDF提供的`esp_rom_delay_us()`函数可以实现微秒级精确延时。

## 4. SSD1306 OLED驱动

### 4.1 SSD1306简介

SSD1306是一款常用的OLED驱动芯片：
- 分辨率：128×64 或 128×32 像素
- 接口：I2C（本项目）或 SPI
- 工作电压：3.3V或5V

### 4.2 显示内存布局

SSD1306使用**页地址模式**组织显示内存：

```
128×32 OLED内存布局：

        列 0   1   2   3   ...  125 126 127
      ┌─────┬───┬───┬───┬───┬───┬───┬───┐
Page0 │ B0  │B1 │B2 │B3 │...│   │   │   │ ← 8行像素
      ├─────┼───┼───┼───┼───┼───┼───┼───┤
Page1 │     │   │   │   │...│   │   │   │ ← 8行像素
      ├─────┼───┼───┼───┼───┼───┼───┼───┤
Page2 │     │   │   │   │...│   │   │   │ ← 8行像素
      ├─────┼───┼───┼───┼───┼───┼───┼───┤
Page3 │     │   │   │   │...│   │   │   │ ← 8行像素
      └─────┴───┴───┴───┴───┴───┴───┴───┘

每个字节控制8个垂直像素：
  B0 = [D0][D1][D2][D3][D4][D5][D6][D7]
        ↑                           ↓
       最上                        最下
```

### 4.3 初始化命令序列

```c
static const uint8_t s_init_cmds[] = {
    0xAE,           // 关闭显示
    0xD5, 0x80,     // 设置时钟分频因子
    0xA8, 0x1F,     // 设置驱动路数 (32行: 0x1F, 64行: 0x3F)
    0xD3, 0x00,     // 设置显示偏移
    0x40,           // 设置显示开始行
    0x8D, 0x14,     // 启用电荷泵 (内部DC-DC)
    0xA1,           // 段重映射 (左右翻转)
    0xC8,           // 扫描方向 (上下翻转)
    0xDA, 0x02,     // COM硬件配置 - 128x32专用
    0x81, 0x80,     // 对比度设置 (0x00-0xFF)
    0xD9, 0x1F,     // 预充电周期
    0xDB, 0x40,     // VCOM电压
    0xA4,           // 显示跟随RAM内容
    0xAF,           // 打开显示
};
```

### 4.4 I2C写入格式

SSD1306的I2C写入格式：

```
起始 → 设备地址(写) → 控制字节 → 数据... → 停止

控制字节：
  0x00 = 后续是命令
  0x40 = 后续是数据
```

**命令写入：**
```c
static esp_err_t oled_write_cmd(uint8_t cmd)
{
    return soft_i2c_mem_write(&s_i2c_handle, s_device_addr, 0x00, &cmd, 1);
    //                                                     ↑ 控制字节=命令
}
```

**数据写入：**
```c
static esp_err_t oled_write_data(uint8_t data)
{
    return soft_i2c_mem_write(&s_i2c_handle, s_device_addr, 0x40, &data, 1);
    //                                                     ↑ 控制字节=数据
}
```

### 4.5 设置显示位置

```c
void ssd1306_set_position(uint8_t x, uint8_t y)
{
    oled_write_cmd(0xB0 + y);                    // 设置页地址 (0xB0-0xB3)
    oled_write_cmd(((x & 0xF0) >> 4) | 0x10);    // 列地址高4位
    oled_write_cmd((x & 0x0F) | 0x00);           // 列地址低4位
}
```

### 4.6 字符显示

```c
void ssd1306_show_char(uint8_t x, uint8_t y, char ch, uint8_t fontsize)
{
    uint8_t c = ch - ' ';  // ASCII偏移

    if (fontsize == 16) {
        // 8×16字体（占用2个Page）
        ssd1306_set_position(x, y);
        for (uint8_t i = 0; i < 8; i++) {
            oled_write_data(F8X16[c * 16 + i]);      // 上半部分
        }
        ssd1306_set_position(x, y + 1);
        for (uint8_t i = 0; i < 8; i++) {
            oled_write_data(F8X16[c * 16 + i + 8]);  // 下半部分
        }
    } else {
        // 6×8字体（占用1个Page）
        ssd1306_set_position(x, y);
        for (uint8_t i = 0; i < 6; i++) {
            oled_write_data(F6X8[c][i]);
        }
    }
}
```

### 4.7 地址自动检测

SSD1306有两个可能的I2C地址：

```c
#define SSD1306_I2C_ADDR        0x78    // 0x3C << 1
#define SSD1306_I2C_ADDR_ALT    0x7A    // 0x3D << 1

static bool ssd1306_detect_address(void)
{
    uint8_t dummy = 0xAE;  // 安全的测试命令（显示关闭）

    // 尝试主地址 0x78
    if (soft_i2c_mem_write(&s_i2c_handle, SSD1306_I2C_ADDR, 0x00, &dummy, 1) == ESP_OK) {
        s_device_addr = SSD1306_I2C_ADDR;
        return true;
    }

    // 尝试备用地址 0x7A
    if (soft_i2c_mem_write(&s_i2c_handle, SSD1306_I2C_ADDR_ALT, 0x00, &dummy, 1) == ESP_OK) {
        s_device_addr = SSD1306_I2C_ADDR_ALT;
        return true;
    }

    return false;
}
```

## 5. 界面设计与优化

### 5.1 界面布局

```
┌────────────────────────────────────────────────────┐
│ HR:xxx BPM   SpO2:xx%     │  ← Page 0 (6x8字体)
│ Temp:xx.x C               │  ← Page 1
│ Status: OK                │  ← Page 2-3
│ 或 !! FALL DETECTED !!    │
└────────────────────────────────────────────────────┘
```

### 5.2 局部刷新优化

全屏刷新消耗大量I2C带宽，采用局部刷新策略：

```c
// 缓存上次的数据
static uint8_t s_last_hr = 0;
static float s_last_spo2 = 0;
static float s_last_temp = 0;

void app_oled_update(uint8_t heart_rate, float spo2, float temperature, bool fall_detected)
{
    // 只在数据变化时更新对应区域
    if (heart_rate != s_last_hr) {
        s_last_hr = heart_rate;
        ssd1306_set_position(18, 0);
        if (heart_rate > 0) {
            ssd1306_show_num(18, 0, heart_rate, 3, 8);
        } else {
            ssd1306_show_string(18, 0, "---", 8);
        }
    }

    if (spo2 != s_last_spo2) {
        s_last_spo2 = spo2;
        if (spo2 > 0) {
            ssd1306_show_num(108, 0, (uint32_t)spo2, 2, 8);
        } else {
            ssd1306_show_string(108, 0, "--", 8);
        }
    }

    if (temperature != s_last_temp) {
        s_last_temp = temperature;
        ssd1306_show_float(30, 1, temperature, 1, 8);
    }
}
```

**优化效果：**
- 避免重复绘制未变化的内容
- 减少I2C通信量
- 降低CPU占用

### 5.3 跌倒警报显示

```c
void app_oled_show_fall_alert(void)
{
    s_fall_alert_active = true;

    // 清屏并显示醒目的警报
    ssd1306_clear();
    ssd1306_show_string(16, 0, "!! WARNING !!", 8);   // 小字体
    ssd1306_show_string(32, 1, "FALL", 16);           // 大字体（8x16）
    ssd1306_show_string(16, 3, "DETECTED!", 8);
}

void app_oled_clear_fall_alert(void)
{
    s_fall_alert_active = false;

    // 恢复正常界面
    ssd1306_clear();
    ssd1306_show_string(0, 0, "HR:--- BPM", 8);
    ssd1306_show_string(78, 0, "SpO2:--%", 8);
    ssd1306_show_string(0, 1, "Temp:--.- C", 8);
    ssd1306_show_string(0, 2, "Status: OK", 8);

    // 重置缓存，强制下次更新
    s_last_hr = 0;
    s_last_spo2 = 0;
    s_last_temp = 0;
}
```

## 6. 完整系统架构

### 6.1 整体数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                        Smart Watch 系统架构                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌───────────┐   ┌───────────┐   ┌───────────┐   ┌─────────┐  │
│   │  MPU6050  │   │ MAX30102  │   │  DS18B20  │   │ SSD1306 │  │
│   │加速度+陀螺│   │  心率血氧  │   │   温度    │   │  OLED   │  │
│   └─────┬─────┘   └─────┬─────┘   └─────┬─────┘   └────┬────┘  │
│         │               │               │              │        │
│    I2C_NUM_0       I2C_NUM_1        OneWire      软件I2C       │
│    400kHz          100kHz           GPIO18       100kHz        │
│    GPIO8/9         GPIO5/15                      GPIO10/11     │
│         │               │               │              ↑        │
│         │               │               │              │        │
│         ▼               ▼               ▼              │        │
│   ┌─────────────────────────────────────────────┐     │        │
│   │              ESP32-S3 主程序                 │     │        │
│   │  ┌─────────────────────────────────────┐   │     │        │
│   │  │ app_main()                          │   │     │        │
│   │  │                                     │   │     │        │
│   │  │  20ms循环:                          │   │     │        │
│   │  │    ├─ 读取MPU6050 → 跌倒检测       │   │     │        │
│   │  │    │                               │   │     │        │
│   │  │  1秒循环:                           │   │     │        │
│   │  │    ├─ 读取温度                     │   │     │        │
│   │  │    └─ 读取心率血氧                  │   │     │        │
│   │  │                               │   │     │        │
│   │  │  500ms循环:                         │───┘     │        │
│   │  │    └─ 更新OLED显示  ────────────────────────┘        │
│   │  │                                     │                   │
│   │  └─────────────────────────────────────┘                   │
│   └─────────────────────────────────────────────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 关键技术总结

| 模块 | 技术要点 |
|-----|---------|
| MPU6050 | 硬件I2C(400kHz) + 互补滤波 + 三阶段状态机 |
| MAX30102 | 硬件I2C(100kHz) + GPIO中断 + PPG算法 |
| DS18B20 | OneWire协议 + RMT外设 |
| SSD1306 | 软件I2C + 局部刷新优化 |

## 7. 项目总结与扩展

### 7.1 完整项目回顾

本系列文章完整讲解了一个ESP32-S3健康监测智能手表的开发过程：

1. **第一篇**：项目架构设计，双硬件I2C总线配置
2. **第二篇**：MPU6050跌倒检测，互补滤波+状态机
3. **第三篇**：MAX30102心率血氧，PPG信号处理
4. **第四篇**：软件I2C实现，OLED显示优化

### 7.2 可扩展方向

1. **WiFi远程告警**
```c
// 检测到跌倒时发送HTTP请求
if (fall_detected) {
    esp_http_client_post("http://server/alert", "{\"type\":\"fall\"}");
}
```

2. **蓝牙数据同步**
```c
// 使用ESP32-S3的BLE功能
// 将健康数据发送到手机App
```

3. **数据存储**
```c
// 使用NVS或SD卡存储历史数据
nvs_set_blob("health_log", &data, sizeof(data));
```

4. **低功耗优化**
```c
// 使用ESP32-S3的Light Sleep模式
esp_sleep_enable_timer_wakeup(20000);  // 20ms唤醒
esp_light_sleep_start();
```

### 7.3 代码仓库结构

```
Smart_Watch/
├── main/
│   ├── main.c                    # 主程序
│   └── app/
│       ├── app_mpu6050.c/h       # 跌倒检测
│       ├── app_max30102.c/h      # 心率血氧
│       ├── app_ds18b20.c/h       # 温度
│       └── app_ssd1306_oled.c/h  # 显示
├── components/
│   ├── mpu6050/                  # MPU6050驱动
│   ├── max30102/                 # MAX30102驱动
│   └── ssd1306_oled/
│       ├── soft_i2c.c/h          # 软件I2C
│       └── ssd1306_oled.c/h      # OLED驱动
└── managed_components/
    └── espressif__ds18b20/       # DS18B20官方驱动
```

## 8. 总结

本篇讲解了软件I2C和OLED显示的实现：

1. **软件I2C**：GPIO模拟I2C协议，开漏输出+上拉电阻
2. **I2C时序**：起始/停止条件、字节发送、ACK检测
3. **SSD1306驱动**：页地址模式、初始化命令、字符显示
4. **界面优化**：局部刷新减少I2C通信，缓存比较避免重绘

---

**系列文章导航：**
- 第一篇：项目架构设计与双I2C总线实现
- 第二篇：MPU6050跌倒检测算法与状态机设计
- 第三篇：MAX30102心率血氧算法与PPG信号处理
- 本篇：软件I2C实现与OLED显示优化

---

> 作者：[你的名字]
> 开发环境：ESP-IDF 5.3.2 + VS Code
> 硬件平台：ESP32-S3 + SSD1306 OLED

---

## 附录：完整代码获取

本系列文章的完整代码可在以下位置获取：
- 项目路径：`Smart_Watch/`
- 博客文章：`Smart_Watch/blog/`

感谢阅读本系列文章，希望对你的ESP32-S3开发有所帮助！
