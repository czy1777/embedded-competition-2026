/**
 * @file max30102.c
 * @brief MAX30102 心率血氧传感器底层驱动实现
 */

#include "max30102.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAX30102";

/*=============================================================================
 * 内部变量
 *============================================================================*/
static i2c_port_t s_i2c_port = I2C_NUM_0;

// I2C 超时时间
#define MAX30102_I2C_TIMEOUT_MS     1000

/*=============================================================================
 * 底层 I2C 读写函数
 *============================================================================*/

esp_err_t max30102_read_reg(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        s_i2c_port,
        MAX30102_I2C_ADDR,
        &reg_addr,
        1,
        data,
        len,
        pdMS_TO_TICKS(MAX30102_I2C_TIMEOUT_MS)
    );
}

esp_err_t max30102_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(
        s_i2c_port,
        MAX30102_I2C_ADDR,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(MAX30102_I2C_TIMEOUT_MS)
    );
}

/*=============================================================================
 * API 函数实现
 *============================================================================*/

esp_err_t max30102_init(i2c_port_t i2c_port)
{
    esp_err_t ret;
    uint8_t part_id;

    // 保存 I2C 端口号 (复用已初始化的I2C)
    s_i2c_port = i2c_port;

    // 验证器件 ID
    ret = max30102_read_reg(MAX30102_REG_PART_ID, &part_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取器件ID失败: %s", esp_err_to_name(ret));
        return ret;
    }
    if (part_id != MAX30102_PART_ID_VAL) {
        ESP_LOGE(TAG, "器件ID不匹配 (读取=0x%02X, 期望=0x%02X)", part_id, MAX30102_PART_ID_VAL);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "器件ID验证成功 (0x%02X)", part_id);

    // 软件重置
    ret = max30102_write_reg(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "软件重置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // 等待重置完成

    // 配置中断使能: A_FULL + PPG_RDY
    ret = max30102_write_reg(MAX30102_REG_INTR_ENABLE_1, 0xC0);
    if (ret != ESP_OK) return ret;

    // 配置温度就绪中断
    ret = max30102_write_reg(MAX30102_REG_INTR_ENABLE_2, 0x02);
    if (ret != ESP_OK) return ret;

    // 清除 FIFO 指针
    ret = max30102_clear_fifo();
    if (ret != ESP_OK) return ret;

    // FIFO 配置: SMP_AVE=0 (无平均), FIFO_A_FULL=15
    ret = max30102_write_reg(MAX30102_REG_FIFO_CONFIG, 0x0F);
    if (ret != ESP_OK) return ret;

    // 模式配置: SpO2 模式
    ret = max30102_write_reg(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2);
    if (ret != ESP_OK) return ret;

    // SpO2 配置: ADC范围8192, 采样率100Hz, 脉冲宽度411us, 18位分辨率
    ret = max30102_write_reg(MAX30102_REG_SPO2_CONFIG, MAX30102_SPO2_CONFIG_DEFAULT);
    if (ret != ESP_OK) return ret;

    // LED 脉冲幅度配置 (约16mA)
    ret = max30102_write_reg(MAX30102_REG_LED1_PA, MAX30102_LED_CURRENT_DEFAULT);  // 红光
    if (ret != ESP_OK) return ret;
    ret = max30102_write_reg(MAX30102_REG_LED2_PA, MAX30102_LED_CURRENT_DEFAULT);  // 红外
    if (ret != ESP_OK) return ret;
    ret = max30102_write_reg(MAX30102_REG_PILOT_PA, MAX30102_LED_CURRENT_DEFAULT); // 导频
    if (ret != ESP_OK) return ret;

    // 清除中断标志
    ret = max30102_clear_interrupt();
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MAX30102 初始化完成");
    return ESP_OK;
}

esp_err_t max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    if (red == NULL || ir == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t fifo_data[6];
    esp_err_t ret = max30102_read_reg(MAX30102_REG_FIFO_DATA, fifo_data, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // 解析 18 位数据
    // byte[0-2]: 红光通道
    // byte[3-5]: 红外通道
    *red = ((uint32_t)fifo_data[0] << 16 | (uint32_t)fifo_data[1] << 8 | fifo_data[2]) & 0x03FFFF;
    *ir  = ((uint32_t)fifo_data[3] << 16 | (uint32_t)fifo_data[4] << 8 | fifo_data[5]) & 0x03FFFF;

    return ESP_OK;
}

esp_err_t max30102_read_temp(float *temp)
{
    if (temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 启动温度测量
    ret = max30102_write_reg(MAX30102_REG_TEMP_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;

    // 等待测量完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取温度
    uint8_t temp_int, temp_frac;
    ret = max30102_read_reg(MAX30102_REG_TEMP_INT, &temp_int, 1);
    if (ret != ESP_OK) return ret;
    ret = max30102_read_reg(MAX30102_REG_TEMP_FRAC, &temp_frac, 1);
    if (ret != ESP_OK) return ret;

    // 计算温度 (整数部分 + 小数部分 * 0.0625)
    *temp = (float)(int8_t)temp_int + (float)temp_frac * 0.0625f;

    return ESP_OK;
}

esp_err_t max30102_clear_fifo(void)
{
    esp_err_t ret;

    ret = max30102_write_reg(MAX30102_REG_FIFO_WR_PTR, 0x00);
    if (ret != ESP_OK) return ret;

    ret = max30102_write_reg(MAX30102_REG_FIFO_OV_CNT, 0x00);
    if (ret != ESP_OK) return ret;

    ret = max30102_write_reg(MAX30102_REG_FIFO_RD_PTR, 0x00);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t max30102_clear_interrupt(void)
{
    uint8_t status;
    esp_err_t ret;

    // 读取中断状态寄存器以清除标志
    ret = max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &status, 1);
    if (ret != ESP_OK) return ret;

    ret = max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &status, 1);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}
