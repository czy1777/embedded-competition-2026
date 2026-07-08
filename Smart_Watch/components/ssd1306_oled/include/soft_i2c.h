/**
 * @file soft_i2c.h
 * @brief 软件 I2C 驱动接口 (GPIO 模拟)
 */

#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

/**
 * @brief 软件 I2C 配置结构
 */
typedef struct {
    gpio_num_t sda_pin;     // SDA 引脚
    gpio_num_t scl_pin;     // SCL 引脚
    uint32_t freq_khz;      // I2C 频率 (kHz)，推荐 100-400
} soft_i2c_config_t;

/**
 * @brief 软件 I2C 句柄结构
 */
typedef struct {
    soft_i2c_config_t config;
    uint32_t delay_us;      // 半周期延时
    bool initialized;
} soft_i2c_handle_t;

/**
 * @brief 初始化软件 I2C
 * @param handle 句柄指针
 * @param config 配置参数
 * @return ESP_OK 成功，其他失败
 */
esp_err_t soft_i2c_init(soft_i2c_handle_t *handle, const soft_i2c_config_t *config);

/**
 * @brief 反初始化软件 I2C
 * @param handle 句柄指针
 */
void soft_i2c_deinit(soft_i2c_handle_t *handle);

/**
 * @brief 软件 I2C 内存写入 (等效于 HAL_I2C_Mem_Write)
 * @param handle 句柄指针
 * @param dev_addr 设备地址 (包含读写位的 8 位地址)
 * @param reg_addr 寄存器/控制字节地址
 * @param data 数据指针
 * @param len 数据长度
 * @return ESP_OK 成功，其他失败
 */
esp_err_t soft_i2c_mem_write(soft_i2c_handle_t *handle,
                              uint8_t dev_addr,
                              uint8_t reg_addr,
                              const uint8_t *data,
                              size_t len);

#ifdef __cplusplus
}
#endif

#endif // SOFT_I2C_H
