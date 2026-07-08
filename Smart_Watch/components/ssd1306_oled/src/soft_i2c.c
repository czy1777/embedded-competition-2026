/**
 * @file soft_i2c.c
 * @brief 软件 I2C 驱动实现 (GPIO 模拟)
 *
 * 注意：此实现使用推挽输出模式以提高驱动能力
 * 适用于只有写入操作的 I2C 设备（如 OLED）
 */

#include "soft_i2c.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "SOFT_I2C";

/*=============================================================================
 * 内部宏定义
 *============================================================================*/
#define SDA_HIGH(h)   gpio_set_level((h)->config.sda_pin, 1)
#define SDA_LOW(h)    gpio_set_level((h)->config.sda_pin, 0)
#define SCL_HIGH(h)   gpio_set_level((h)->config.scl_pin, 1)
#define SCL_LOW(h)    gpio_set_level((h)->config.scl_pin, 0)
#define SDA_READ(h)   gpio_get_level((h)->config.sda_pin)

// 延时宏
#define I2C_DELAY(h)  esp_rom_delay_us((h)->delay_us)

/*=============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 设置 SDA 为输出模式 (开漏)
 */
static void sda_set_output(soft_i2c_handle_t *handle)
{
    gpio_set_direction(handle->config.sda_pin, GPIO_MODE_OUTPUT_OD);
}

/**
 * @brief 设置 SDA 为输入模式 (用于读取 ACK)
 */
static void sda_set_input(soft_i2c_handle_t *handle)
{
    gpio_set_direction(handle->config.sda_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(handle->config.sda_pin, GPIO_PULLUP_ONLY);
}

/**
 * @brief I2C 起始条件
 * SCL 高电平期间，SDA 从高变低
 */
static void soft_i2c_start(soft_i2c_handle_t *handle)
{
    sda_set_output(handle);
    SDA_HIGH(handle);
    I2C_DELAY(handle);
    SCL_HIGH(handle);
    I2C_DELAY(handle);
    SDA_LOW(handle);
    I2C_DELAY(handle);
    SCL_LOW(handle);
    I2C_DELAY(handle);
}

/**
 * @brief I2C 停止条件
 * SCL 高电平期间，SDA 从低变高
 */
static void soft_i2c_stop(soft_i2c_handle_t *handle)
{
    sda_set_output(handle);
    SDA_LOW(handle);
    I2C_DELAY(handle);
    SCL_HIGH(handle);
    I2C_DELAY(handle);
    SDA_HIGH(handle);
    I2C_DELAY(handle);
}

/**
 * @brief 发送一个字节
 * @return 0=收到ACK, 1=收到NACK
 */
static uint8_t soft_i2c_write_byte(soft_i2c_handle_t *handle, uint8_t data)
{
    uint8_t ack;

    sda_set_output(handle);

    // 发送 8 位数据，MSB first
    for (int i = 7; i >= 0; i--) {
        SCL_LOW(handle);
        if (data & (1 << i)) {
            SDA_HIGH(handle);
        } else {
            SDA_LOW(handle);
        }
        I2C_DELAY(handle);
        SCL_HIGH(handle);
        I2C_DELAY(handle);
    }
    SCL_LOW(handle);
    I2C_DELAY(handle);

    // 释放 SDA，读取 ACK
    SDA_HIGH(handle);  // 先拉高
    sda_set_input(handle);  // 切换为输入
    I2C_DELAY(handle);
    I2C_DELAY(handle);  // 等待从设备响应
    SCL_HIGH(handle);
    I2C_DELAY(handle);
    ack = SDA_READ(handle);
    SCL_LOW(handle);
    I2C_DELAY(handle);

    sda_set_output(handle);
    SDA_HIGH(handle);

    return ack;
}

/*=============================================================================
 * API 实现
 *============================================================================*/

esp_err_t soft_i2c_init(soft_i2c_handle_t *handle, const soft_i2c_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->config = *config;

    // 计算半周期延时 (微秒)
    // 使用较长的延时以确保稳定性，最小 10us (约 50kHz)
    if (config->freq_khz > 0 && config->freq_khz <= 100) {
        handle->delay_us = 500 / config->freq_khz;
    } else {
        handle->delay_us = 10;  // 默认 10us，约 50kHz
    }
    // 确保最小延时
    if (handle->delay_us < 5) {
        handle->delay_us = 5;
    }

    // 配置 SDA 为开漏输出（标准I2C模式）
    gpio_config_t sda_conf = {
        .pin_bit_mask = (1ULL << config->sda_pin),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // 配置 SCL 为开漏输出（标准I2C模式）
    gpio_config_t scl_conf = {
        .pin_bit_mask = (1ULL << config->scl_pin),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&sda_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SDA GPIO 配置失败");
        return ret;
    }

    ret = gpio_config(&scl_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCL GPIO 配置失败");
        return ret;
    }

    // 初始化为空闲状态（SDA 和 SCL 均为高）
    gpio_set_level(config->sda_pin, 1);
    gpio_set_level(config->scl_pin, 1);

    // 等待总线稳定
    esp_rom_delay_us(100);

    handle->initialized = true;
    ESP_LOGI(TAG, "软件 I2C 初始化完成 (SDA=GPIO%d, SCL=GPIO%d, delay=%ldus)",
             config->sda_pin, config->scl_pin, handle->delay_us);

    return ESP_OK;
}

void soft_i2c_deinit(soft_i2c_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return;
    }

    gpio_reset_pin(handle->config.sda_pin);
    gpio_reset_pin(handle->config.scl_pin);
    handle->initialized = false;
}

esp_err_t soft_i2c_mem_write(soft_i2c_handle_t *handle,
                              uint8_t dev_addr,
                              uint8_t reg_addr,
                              const uint8_t *data,
                              size_t len)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL && len > 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 发送起始条件
    soft_i2c_start(handle);

    // 发送设备地址 (写)
    if (soft_i2c_write_byte(handle, dev_addr) != 0) {
        soft_i2c_stop(handle);
        return ESP_ERR_TIMEOUT;  // NACK
    }

    // 发送寄存器/控制字节地址
    if (soft_i2c_write_byte(handle, reg_addr) != 0) {
        soft_i2c_stop(handle);
        return ESP_ERR_TIMEOUT;
    }

    // 发送数据
    for (size_t i = 0; i < len; i++) {
        if (soft_i2c_write_byte(handle, data[i]) != 0) {
            soft_i2c_stop(handle);
            return ESP_ERR_TIMEOUT;
        }
    }

    // 发送停止条件
    soft_i2c_stop(handle);

    return ESP_OK;
}
