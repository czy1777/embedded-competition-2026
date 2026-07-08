/**
 * @file max30102.h
 * @brief MAX30102 心率血氧传感器底层驱动接口
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c.h"

/*=============================================================================
 * MAX30102 I2C 地址
 *============================================================================*/
#define MAX30102_I2C_ADDR           0x57

/*=============================================================================
 * MAX30102 寄存器地址定义
 *============================================================================*/
// 中断状态寄存器
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01

// 中断使能寄存器
#define MAX30102_REG_INTR_ENABLE_1  0x02
#define MAX30102_REG_INTR_ENABLE_2  0x03

// FIFO 寄存器
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_FIFO_OV_CNT    0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07

// 配置寄存器
#define MAX30102_REG_FIFO_CONFIG    0x08
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A

// LED 脉冲幅度寄存器
#define MAX30102_REG_LED1_PA        0x0C    // 红光 LED
#define MAX30102_REG_LED2_PA        0x0D    // 红外 LED
#define MAX30102_REG_PILOT_PA       0x10    // 导频 LED

// 温度寄存器
#define MAX30102_REG_TEMP_INT       0x1F
#define MAX30102_REG_TEMP_FRAC      0x20
#define MAX30102_REG_TEMP_CONFIG    0x21

// 器件ID寄存器
#define MAX30102_REG_PART_ID        0xFF
#define MAX30102_PART_ID_VAL        0x15    // MAX30102 器件ID值

/*=============================================================================
 * 配置常量
 *============================================================================*/
// 模式配置 (0x09)
#define MAX30102_MODE_RESET         0x40    // 软件重置
#define MAX30102_MODE_SPO2          0x03    // SpO2 模式

// 中断使能 (0x02)
#define MAX30102_INTR_A_FULL        0x80    // FIFO 几乎满中断
#define MAX30102_INTR_PPG_RDY       0x40    // 新数据就绪中断

// SpO2 配置 (0x0A)
// ADC范围8192, 采样率100Hz, 脉冲宽度411us (18位分辨率)
#define MAX30102_SPO2_CONFIG_DEFAULT 0x47

// LED 电流 (约16mA)
#define MAX30102_LED_CURRENT_DEFAULT 0x50

/*=============================================================================
 * API 函数声明
 *============================================================================*/

/**
 * @brief 初始化 MAX30102 传感器
 * @param i2c_port I2C 端口号 (复用已初始化的I2C)
 * @return ESP_OK 成功，其他失败
 */
esp_err_t max30102_init(i2c_port_t i2c_port);

/**
 * @brief 读取 FIFO 数据 (红光和红外)
 * @param[out] red 红光通道原始值 (18位)
 * @param[out] ir 红外通道原始值 (18位)
 * @return ESP_OK 成功，其他失败
 */
esp_err_t max30102_read_fifo(uint32_t *red, uint32_t *ir);

/**
 * @brief 读取芯片内部温度
 * @param[out] temp 温度值 (摄氏度)
 * @return ESP_OK 成功，其他失败
 */
esp_err_t max30102_read_temp(float *temp);

/**
 * @brief 清除 FIFO 指针和中断标志
 * @return ESP_OK 成功，其他失败
 */
esp_err_t max30102_clear_fifo(void);

/**
 * @brief 清除中断状态 (读取中断状态寄存器)
 * @return ESP_OK 成功，其他失败
 */
esp_err_t max30102_clear_interrupt(void);

/**
 * @brief 读取寄存器
 * @param reg_addr 寄存器地址
 * @param[out] data 数据缓冲区
 * @param len 读取长度
 * @return ESP_OK 成功，其他失败
 */
esp_err_t max30102_read_reg(uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief 写入寄存器
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @return ESP_OK 成功，其他失败
 */
esp_err_t max30102_write_reg(uint8_t reg_addr, uint8_t data);

#ifdef __cplusplus
}
#endif
