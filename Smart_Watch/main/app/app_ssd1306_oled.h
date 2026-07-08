/**
 * @file app_ssd1306_oled.h
 * @brief SSD1306 OLED 应用层接口 - 健康数据显示
 */

#ifndef APP_SSD1306_OLED_H
#define APP_SSD1306_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

/*=============================================================================
 * 硬件配置 - 软件 I2C
 *============================================================================*/
#define APP_OLED_SDA_PIN        GPIO_NUM_10     // 软件 I2C SDA
#define APP_OLED_SCL_PIN        GPIO_NUM_11     // 软件 I2C SCL
#define APP_OLED_I2C_FREQ_KHZ   100             // 100kHz

/*=============================================================================
 * API 函数声明
 *============================================================================*/

/**
 * @brief 初始化 OLED 显示模块
 * @return ESP_OK 成功，其他失败
 */
esp_err_t app_oled_init(void);

/**
 * @brief 反初始化 OLED
 */
void app_oled_deinit(void);

/**
 * @brief 更新健康数据显示
 * @param heart_rate 心率 (BPM)，0 表示无效
 * @param spo2 血氧 (%)，0 表示无效
 * @param temperature 体温 (°C)
 * @param fall_detected 是否检测到跌倒
 */
void app_oled_update(uint8_t heart_rate, float spo2, float temperature, bool fall_detected);

/**
 * @brief 显示跌倒警报
 */
void app_oled_show_fall_alert(void);

/**
 * @brief 清除跌倒警报，恢复正常显示
 */
void app_oled_clear_fall_alert(void);

#ifdef __cplusplus
}
#endif

#endif // APP_SSD1306_OLED_H
