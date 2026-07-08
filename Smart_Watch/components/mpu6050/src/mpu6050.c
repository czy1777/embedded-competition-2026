/**
 * @file mpu6050.c
 * @brief MPU6050 底层驱动实现
 */

#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "esp_log.h"

/*=============================================================================
 * 内部常量
 *============================================================================*/
static const char *TAG = "MPU6050";

// 互补滤波参数
#define ALPHA           0.99f       // 陀螺仪权重（0.95-0.99）
#define RAD_TO_DEG      57.2957795f // 弧度转角度

// MPU6050 寄存器地址
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/*=============================================================================
 * 内部数据结构
 *============================================================================*/
typedef struct {
    i2c_port_t i2c_port;        // I2C 端口
    uint16_t dev_addr;          // 设备地址（已左移1位）
    uint32_t sample_count;      // 采样计数（用于互补滤波初始化）
    float dt;                   // 采样时间间隔（秒）
    struct timeval last_time;   // 上次采样时间
    float gyro_offset_x;        // 陀螺仪零偏（dps）
    float gyro_offset_y;
    float gyro_offset_z;
} mpu6050_dev_t;

/*=============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief I2C 写入函数
 */
static esp_err_t mpu6050_write_reg(mpu6050_dev_t *dev, uint8_t reg,
                                   const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev->dev_addr | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd,
                                          pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief I2C 读取函数
 */
static esp_err_t mpu6050_read_reg(mpu6050_dev_t *dev, uint8_t reg,
                                  uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev->dev_addr | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev->dev_addr | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd,
                                          pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief 获取加速度计灵敏度（LSB/g）
 */
static float get_acce_sensitivity(mpu6050_dev_t *dev)
{
    uint8_t config;
    mpu6050_read_reg(dev, REG_ACCEL_CONFIG, &config, 1);
    uint8_t fs = (config >> 3) & 0x03;

    switch (fs) {
        case MPU6050_ACCE_FS_2G:  return 16384.0f;
        case MPU6050_ACCE_FS_4G:  return 8192.0f;
        case MPU6050_ACCE_FS_8G:  return 4096.0f;
        case MPU6050_ACCE_FS_16G: return 2048.0f;
        default: return 16384.0f;
    }
}

/**
 * @brief 获取陀螺仪灵敏度（LSB/(度/秒)）
 */
static float get_gyro_sensitivity(mpu6050_dev_t *dev)
{
    uint8_t config;
    mpu6050_read_reg(dev, REG_GYRO_CONFIG, &config, 1);
    uint8_t fs = (config >> 3) & 0x03;

    switch (fs) {
        case MPU6050_GYRO_FS_250DPS:  return 131.0f;
        case MPU6050_GYRO_FS_500DPS:  return 65.5f;
        case MPU6050_GYRO_FS_1000DPS: return 32.8f;
        case MPU6050_GYRO_FS_2000DPS: return 16.4f;
        default: return 131.0f;
    }
}

/*=============================================================================
 * API 函数实现
 *============================================================================*/

mpu6050_handle_t mpu6050_create(i2c_port_t i2c_port, uint16_t dev_addr)
{
    mpu6050_dev_t *dev = calloc(1, sizeof(mpu6050_dev_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "内存分配失败");
        return NULL;
    }

    dev->i2c_port = i2c_port;
    dev->dev_addr = dev_addr << 1;  // I2C 地址左移1位
    dev->sample_count = 0;
    dev->dt = 0;

    return (mpu6050_handle_t)dev;
}

void mpu6050_delete(mpu6050_handle_t handle)
{
    if (handle) {
        free(handle);
    }
}

esp_err_t mpu6050_get_device_id(mpu6050_handle_t handle, uint8_t *device_id)
{
    if (handle == NULL || device_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    return mpu6050_read_reg(dev, REG_WHO_AM_I, device_id, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    uint8_t tmp;

    esp_err_t ret = mpu6050_read_reg(dev, REG_PWR_MGMT_1, &tmp, 1);
    if (ret != ESP_OK) return ret;

    tmp &= ~(1 << 6);  // 清除 SLEEP 位
    return mpu6050_write_reg(dev, REG_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6050_sleep(mpu6050_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    uint8_t tmp;

    esp_err_t ret = mpu6050_read_reg(dev, REG_PWR_MGMT_1, &tmp, 1);
    if (ret != ESP_OK) return ret;

    tmp |= (1 << 6);  // 设置 SLEEP 位
    return mpu6050_write_reg(dev, REG_PWR_MGMT_1, &tmp, 1);
}

esp_err_t mpu6050_config(mpu6050_handle_t handle,
                         mpu6050_acce_fs_t acce_fs,
                         mpu6050_gyro_fs_t gyro_fs)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;

    uint8_t gyro_config = gyro_fs << 3;
    uint8_t acce_config = acce_fs << 3;

    esp_err_t ret = mpu6050_write_reg(dev, REG_GYRO_CONFIG, &gyro_config, 1);
    if (ret != ESP_OK) return ret;

    return mpu6050_write_reg(dev, REG_ACCEL_CONFIG, &acce_config, 1);
}

esp_err_t mpu6050_set_dlpf(mpu6050_handle_t handle, mpu6050_dlpf_t bw)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    /* CONFIG 寄存器：bit[2:0] = DLPF_CFG；其余位保持默认 0 */
    uint8_t cfg = (uint8_t)(bw & 0x07);
    return mpu6050_write_reg(dev, REG_CONFIG, &cfg, 1);
}

esp_err_t mpu6050_set_sample_rate(mpu6050_handle_t handle, uint16_t rate_hz)
{
    if (handle == NULL || rate_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;

    /* 本驱动默认启用 DLPF（DLPF_CFG=1~6），gyro 输出率为 1kHz */
    uint32_t div = (1000u / rate_hz);
    if (div == 0) div = 1;
    if (div > 256) div = 256;
    uint8_t smplrt_div = (uint8_t)(div - 1);
    return mpu6050_write_reg(dev, REG_SMPLRT_DIV, &smplrt_div, 1);
}

esp_err_t mpu6050_calibrate_gyro(mpu6050_handle_t handle, uint16_t samples)
{
    if (handle == NULL || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;

    /* 暂存并清零旧偏置，避免递归扣除 */
    float saved_x = dev->gyro_offset_x;
    float saved_y = dev->gyro_offset_y;
    float saved_z = dev->gyro_offset_z;
    dev->gyro_offset_x = 0.0f;
    dev->gyro_offset_y = 0.0f;
    dev->gyro_offset_z = 0.0f;

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint16_t valid = 0;
    for (uint16_t i = 0; i < samples; i++) {
        mpu6050_gyro_t g;
        if (mpu6050_get_gyro(handle, &g) == ESP_OK) {
            sum_x += g.x;
            sum_y += g.y;
            sum_z += g.z;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (valid < (samples >> 1)) {
        /* 有效采样不足一半，恢复旧偏置并返回错误 */
        dev->gyro_offset_x = saved_x;
        dev->gyro_offset_y = saved_y;
        dev->gyro_offset_z = saved_z;
        ESP_LOGE(TAG, "陀螺仪校准失败: 有效样本 %u/%u", valid, samples);
        return ESP_FAIL;
    }

    dev->gyro_offset_x = (float)(sum_x / valid);
    dev->gyro_offset_y = (float)(sum_y / valid);
    dev->gyro_offset_z = (float)(sum_z / valid);

    ESP_LOGI(TAG, "陀螺仪零偏: x=%.2f y=%.2f z=%.2f dps (N=%u)",
             dev->gyro_offset_x, dev->gyro_offset_y, dev->gyro_offset_z, valid);
    return ESP_OK;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t handle, mpu6050_acce_t *acce)
{
    if (handle == NULL || acce == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    uint8_t data[6];

    esp_err_t ret = mpu6050_read_reg(dev, REG_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) return ret;

    float sensitivity = get_acce_sensitivity(dev);

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    acce->x = raw_x / sensitivity;
    acce->y = raw_y / sensitivity;
    acce->z = raw_z / sensitivity;

    return ESP_OK;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t handle, mpu6050_gyro_t *gyro)
{
    if (handle == NULL || gyro == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;
    uint8_t data[6];

    esp_err_t ret = mpu6050_read_reg(dev, REG_GYRO_XOUT_H, data, 6);
    if (ret != ESP_OK) return ret;

    float sensitivity = get_gyro_sensitivity(dev);

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);

    gyro->x = raw_x / sensitivity - dev->gyro_offset_x;
    gyro->y = raw_y / sensitivity - dev->gyro_offset_y;
    gyro->z = raw_z / sensitivity - dev->gyro_offset_z;

    return ESP_OK;
}

esp_err_t mpu6050_calc_angle(mpu6050_handle_t handle,
                             const mpu6050_acce_t *acce,
                             const mpu6050_gyro_t *gyro,
                             mpu6050_angle_t *angle)
{
    if (handle == NULL || acce == NULL || gyro == NULL || angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_dev_t *dev = (mpu6050_dev_t *)handle;

    // 从加速度计计算角度
    float acce_roll  = atan2f(acce->y, acce->z) * RAD_TO_DEG;
    float acce_pitch = atan2f(acce->x, acce->z) * RAD_TO_DEG;

    dev->sample_count++;

    // 第一次采样，直接使用加速度计角度
    if (dev->sample_count == 1) {
        angle->roll = acce_roll;
        angle->pitch = acce_pitch;
        gettimeofday(&dev->last_time, NULL);
        return ESP_OK;
    }

    // 计算时间间隔
    struct timeval now, dt_tv;
    gettimeofday(&now, NULL);
    timersub(&now, &dev->last_time, &dt_tv);
    dev->dt = (float)dt_tv.tv_sec + (float)dt_tv.tv_usec / 1000000.0f;
    dev->last_time = now;

    // 防止时间间隔过大或过小
    if (dev->dt <= 0 || dev->dt > 1.0f) {
        dev->dt = 0.02f;  // 默认 50Hz
    }

    // 陀螺仪角度增量
    float gyro_roll  = gyro->x * dev->dt;
    float gyro_pitch = gyro->y * dev->dt;

    // 互补滤波融合
    angle->roll  = ALPHA * (angle->roll + gyro_roll) +
                   (1.0f - ALPHA) * acce_roll;
    angle->pitch = ALPHA * (angle->pitch + gyro_pitch) +
                   (1.0f - ALPHA) * acce_pitch;

    return ESP_OK;
}
