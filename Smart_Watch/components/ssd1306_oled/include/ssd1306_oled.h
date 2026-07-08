/**
 * @file ssd1306_oled.h
 * @brief SSD1306 OLED 驱动接口 (128x32)
 */

#ifndef SSD1306_OLED_H
#define SSD1306_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*=============================================================================
 * SSD1306 配置常量
 *============================================================================*/
#define SSD1306_I2C_ADDR        0x78    // 7位地址 0x3C 左移1位 (常见地址)
#define SSD1306_I2C_ADDR_ALT    0x7A    // 7位地址 0x3D 左移1位 (备用地址)
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          32
#define SSD1306_PAGES           4       // 32 / 8 = 4

/*=============================================================================
 * 配置结构
 *============================================================================*/
typedef struct {
    gpio_num_t sda_pin;         // SDA 引脚
    gpio_num_t scl_pin;         // SCL 引脚
    uint32_t freq_khz;          // I2C 频率 (kHz)，推荐 100-400
} ssd1306_config_t;

/*=============================================================================
 * API 函数声明
 *============================================================================*/

/**
 * @brief 初始化 SSD1306 OLED 显示屏
 * @param config 配置参数
 * @return ESP_OK 成功，其他失败
 */
esp_err_t ssd1306_init(const ssd1306_config_t *config);

/**
 * @brief 反初始化 SSD1306
 */
void ssd1306_deinit(void);

/**
 * @brief 清屏
 */
void ssd1306_clear(void);

/**
 * @brief 填满屏幕（全白）
 */
void ssd1306_fill(void);

/**
 * @brief 打开显示
 */
void ssd1306_display_on(void);

/**
 * @brief 关闭显示
 */
void ssd1306_display_off(void);

/**
 * @brief 设置光标位置
 * @param x X 坐标 (0-127)
 * @param y 页号 (0-3)
 */
void ssd1306_set_position(uint8_t x, uint8_t y);

/**
 * @brief 显示单个 ASCII 字符
 * @param x X 坐标
 * @param y 页号
 * @param ch 字符
 * @param fontsize 字体大小 (8 或 16)
 */
void ssd1306_show_char(uint8_t x, uint8_t y, char ch, uint8_t fontsize);

/**
 * @brief 显示 ASCII 字符串
 * @param x X 坐标
 * @param y 页号
 * @param str 字符串
 * @param fontsize 字体大小 (8 或 16)
 */
void ssd1306_show_string(uint8_t x, uint8_t y, const char *str, uint8_t fontsize);

/**
 * @brief 显示整数
 * @param x X 坐标
 * @param y 页号
 * @param num 数值
 * @param len 显示位数
 * @param fontsize 字体大小
 */
void ssd1306_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontsize);

/**
 * @brief 显示浮点数
 * @param x X 坐标
 * @param y 页号
 * @param num 数值
 * @param decimals 小数位数
 * @param fontsize 字体大小
 */
void ssd1306_show_float(uint8_t x, uint8_t y, float num, uint8_t decimals, uint8_t fontsize);

#ifdef __cplusplus
}
#endif

#endif // SSD1306_OLED_H
