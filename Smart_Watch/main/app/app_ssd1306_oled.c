/**
 * @file app_ssd1306_oled.c
 * @brief SSD1306 OLED 应用层实现 - 健康数据显示
 *
 * 显示界面布局 (128x32 像素):
 * ┌────────────────────────────┐
 * │ HR:xxx BPM   SpO2:xx%     │  ← 第1行 (Page 0)
 * │ Temp:xx.x C               │  ← 第2行 (Page 1)
 * │ Status: OK                │  ← 第3-4行 (Page 2-3)
 * │ 或 !! FALL DETECTED !!    │
 * └────────────────────────────┘
 */

#include "app_ssd1306_oled.h"
#include "ssd1306_oled.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "APP_OLED";

/*=============================================================================
 * 内部变量
 *============================================================================*/
static bool s_initialized = false;
static bool s_fall_alert_active = false;

// 缓存上次的数据，用于局部刷新
static uint8_t s_last_hr = 0;
static float s_last_spo2 = 0;
static float s_last_temp = 0;

/*=============================================================================
 * API 实现
 *============================================================================*/

esp_err_t app_oled_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ssd1306_config_t config = {
        .sda_pin = APP_OLED_SDA_PIN,
        .scl_pin = APP_OLED_SCL_PIN,
        .freq_khz = APP_OLED_I2C_FREQ_KHZ,
    };

    esp_err_t ret = ssd1306_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 初始化失败");
        return ret;
    }

    s_initialized = true;
    s_fall_alert_active = false;

    // 测试：先填充白色，确认OLED工作正常
    ESP_LOGI(TAG, "测试：填充白色屏幕...");
    ssd1306_fill();
    vTaskDelay(pdMS_TO_TICKS(1000));  // 等待1秒观察

    // 显示初始界面
    ssd1306_clear();
    ssd1306_show_string(0, 0, "HR:--- BPM", 8);
    ssd1306_show_string(78, 0, "SpO2:--%", 8);
    ssd1306_show_string(0, 1, "Temp:--.- C", 8);
    ssd1306_show_string(0, 2, "Status: OK", 8);

    ESP_LOGI(TAG, "OLED 应用层初始化完成");
    return ESP_OK;
}

void app_oled_deinit(void)
{
    if (!s_initialized) {
        return;
    }
    ssd1306_deinit();
    s_initialized = false;
}

void app_oled_update(uint8_t heart_rate, float spo2, float temperature, bool fall_detected)
{
    if (!s_initialized) {
        return;
    }

    // 如果检测到跌倒，显示警报
    if (fall_detected && !s_fall_alert_active) {
        app_oled_show_fall_alert();
        return;
    }

    // 如果警报状态激活，不更新普通显示
    if (s_fall_alert_active) {
        return;
    }

    // 更新心率显示 (只在数据变化时更新)
    if (heart_rate != s_last_hr) {
        s_last_hr = heart_rate;
        ssd1306_set_position(18, 0);
        if (heart_rate > 0) {
            ssd1306_show_num(18, 0, heart_rate, 3, 8);
        } else {
            ssd1306_show_string(18, 0, "---", 8);
        }
    }

    // 更新血氧显示 (x=108 是 "SpO2:" 后面的位置: 78 + 5*6 = 108)
    if (spo2 != s_last_spo2) {
        s_last_spo2 = spo2;
        if (spo2 > 0) {
            ssd1306_show_num(108, 0, (uint32_t)spo2, 2, 8);
        } else {
            ssd1306_show_string(108, 0, "--", 8);
        }
    }

    // 更新温度显示：未佩戴/未初始化时 temperature<=0，显示占位符 "--.-"
    // 占位符宽度 4 字符，与正常体温(34~42°C)渲染宽度一致，可完整覆盖前值
    if (temperature != s_last_temp) {
        s_last_temp = temperature;
        if (temperature > 0.0f) {
            ssd1306_show_float(30, 1, temperature, 1, 8);
        } else {
            ssd1306_show_string(30, 1, "--.-", 8);
        }
    }
}

void app_oled_show_fall_alert(void)
{
    if (!s_initialized) {
        return;
    }

    s_fall_alert_active = true;

    // 清屏并显示警报
    ssd1306_clear();
    ssd1306_show_string(16, 0, "!! WARNING !!", 8);
    ssd1306_show_string(32, 1, "FALL", 16);
    ssd1306_show_string(16, 3, "DETECTED!", 8);

    ESP_LOGW(TAG, "显示跌倒警报");
}

void app_oled_clear_fall_alert(void)
{
    if (!s_initialized || !s_fall_alert_active) {
        return;
    }

    s_fall_alert_active = false;

    // 恢复正常显示
    ssd1306_clear();
    ssd1306_show_string(0, 0, "HR:--- BPM", 8);
    ssd1306_show_string(78, 0, "SpO2:--%", 8);
    ssd1306_show_string(0, 1, "Temp:--.- C", 8);
    ssd1306_show_string(0, 2, "Status: OK", 8);

    // 重置缓存，强制下次更新
    s_last_hr = 0;
    s_last_spo2 = 0;
    s_last_temp = 0;

    ESP_LOGI(TAG, "清除跌倒警报，恢复正常显示");
}
