/**
 * @file mpu6050.h
 * @brief MPU6050 底层驱动接口 - 精简版（用于跌倒检测）
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "esp_err.h"

/*=============================================================================
 * 常量定义
 *============================================================================*/
#define MPU6050_I2C_ADDR            0x68    // AD0接地时的I2C地址
#define MPU6050_WHO_AM_I_VAL        0x68    // 器件ID验证值

/*=============================================================================
 * 量程枚举
 *============================================================================*/
typedef enum {
    MPU6050_ACCE_FS_2G  = 0,    // +/- 2g
    MPU6050_ACCE_FS_4G  = 1,    // +/- 4g
    MPU6050_ACCE_FS_8G  = 2,    // +/- 8g (推荐用于跌倒检测)
    MPU6050_ACCE_FS_16G = 3,    // +/- 16g
} mpu6050_acce_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,    // +/- 250 度/秒
    MPU6050_GYRO_FS_500DPS  = 1,    // +/- 500 度/秒
    MPU6050_GYRO_FS_1000DPS = 2,    // +/- 1000 度/秒
    MPU6050_GYRO_FS_2000DPS = 3,    // +/- 2000 度/秒
} mpu6050_gyro_fs_t;

// DLPF_CFG（寄存器 0x1A 的低 3 位）——数字低通滤波带宽
typedef enum {
    MPU6050_DLPF_260HZ = 0,   // 加速度 260Hz/0ms,   陀螺仪 256Hz/0.98ms（默认）
    MPU6050_DLPF_184HZ = 1,   // 加速度 184Hz/2.0ms, 陀螺仪 188Hz/1.9ms
    MPU6050_DLPF_94HZ  = 2,   // 加速度  94Hz/3.0ms, 陀螺仪  98Hz/2.8ms
    MPU6050_DLPF_44HZ  = 3,   // 加速度  44Hz/4.9ms, 陀螺仪  42Hz/4.8ms (推荐)
    MPU6050_DLPF_21HZ  = 4,   // 加速度  21Hz/8.5ms, 陀螺仪  20Hz/8.3ms
    MPU6050_DLPF_10HZ  = 5,   // 加速度  10Hz/13.8ms,陀螺仪  10Hz/13.4ms
    MPU6050_DLPF_5HZ   = 6,   // 加速度   5Hz/19.0ms,陀螺仪   5Hz/18.6ms
} mpu6050_dlpf_t;

/*=============================================================================
 * 数据结构
 *============================================================================*/

// 加速度数据（单位：g）
typedef struct {
    float x;
    float y;
    float z;
} mpu6050_acce_t;

// 陀螺仪数据（单位：度/秒）
typedef struct {
    float x;
    float y;
    float z;
} mpu6050_gyro_t;

// 姿态角数据（单位：度）
typedef struct {
    float roll;     // 横滚角
    float pitch;    // 俯仰角
} mpu6050_angle_t;

// 设备句柄（不透明指针）
typedef void *mpu6050_handle_t;

/*=============================================================================
 * API 函数声明
 *============================================================================*/

/**
 * @brief 创建 MPU6050 设备句柄
 * @param i2c_port I2C 端口号 (I2C_NUM_0 或 I2C_NUM_1)
 * @param dev_addr I2C 设备地址 (通常为 MPU6050_I2C_ADDR)
 * @return 成功返回句柄，失败返回 NULL
 */
mpu6050_handle_t mpu6050_create(i2c_port_t i2c_port, uint16_t dev_addr);

/**
 * @brief 释放 MPU6050 设备句柄
 * @param handle 设备句柄
 */
void mpu6050_delete(mpu6050_handle_t handle);

/**
 * @brief 获取器件 ID（用于验证通信）
 * @param handle 设备句柄
 * @param[out] device_id 器件 ID（应为 0x68）
 * @return ESP_OK 成功，其他失败
 */
esp_err_t mpu6050_get_device_id(mpu6050_handle_t handle, uint8_t *device_id);

/**
 * @brief 唤醒 MPU6050（退出睡眠模式）
 * @param handle 设备句柄
 * @return ESP_OK 成功，其他失败
 */
esp_err_t mpu6050_wake_up(mpu6050_handle_t handle);

/**
 * @brief 使 MPU6050 进入睡眠模式
 * @param handle 设备句柄
 * @return ESP_OK 成功，其他失败
 */
esp_err_t mpu6050_sleep(mpu6050_handle_t handle);

/**
 * @brief 配置加速度计和陀螺仪量程
 * @param handle 设备句柄
 * @param acce_fs 加速度计量程
 * @param gyro_fs 陀螺仪量程
 * @return ESP_OK 成功，其他失败
 */
esp_err_t mpu6050_config(mpu6050_handle_t handle,
                         mpu6050_acce_fs_t acce_fs,
                         mpu6050_gyro_fs_t gyro_fs);

/**
 * @brief 配置数字低通滤波器（DLPF_CFG，寄存器 0x1A 低 3 位）
 *
 * 抗混叠关键：50Hz 采样下 Nyquist=25Hz，建议设 44Hz 或更低带宽。
 *
 * @param handle 设备句柄
 * @param bw DLPF 带宽枚举
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_set_dlpf(mpu6050_handle_t handle, mpu6050_dlpf_t bw);

/**
 * @brief 设置采样率（SMPLRT_DIV，寄存器 0x19）
 *
 * 内部公式：SMPLRT_DIV = gyro_output_rate / rate_hz - 1
 *   - 启用 DLPF 时 gyro_output_rate = 1kHz
 *   - 未启用 DLPF 时 gyro_output_rate = 8kHz（本驱动默认启用 DLPF）
 *
 * @param handle 设备句柄
 * @param rate_hz 期望采样率（Hz），合理范围 4~1000
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_set_sample_rate(mpu6050_handle_t handle, uint16_t rate_hz);

/**
 * @brief 陀螺仪静态零偏校准
 *
 * 采样 samples 次取平均，作为陀螺仪三轴零偏；之后 mpu6050_get_gyro
 * 会自动扣除零偏。调用前应保持设备静止。
 *
 * @param handle 设备句柄
 * @param samples 采样数（推荐 100，间隔 10ms）
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_calibrate_gyro(mpu6050_handle_t handle, uint16_t samples);

/**
 * @brief 读取加速度数据（已转换为 g）
 * @param handle 设备句柄
 * @param[out] acce 加速度数据
 * @return ESP_OK 成功，其他失败
 */
esp_err_t mpu6050_get_acce(mpu6050_handle_t handle, mpu6050_acce_t *acce);

/**
 * @brief 读取陀螺仪数据（已转换为 度/秒）
 * @param handle 设备句柄
 * @param[out] gyro 陀螺仪数据
 * @return ESP_OK 成功，其他失败
 */
esp_err_t mpu6050_get_gyro(mpu6050_handle_t handle, mpu6050_gyro_t *gyro);

/**
 * @brief 互补滤波计算姿态角
 * @param handle 设备句柄
 * @param acce 加速度数据
 * @param gyro 陀螺仪数据
 * @param[in,out] angle 姿态角（输入上次值，输出更新后的值）
 * @return ESP_OK 成功，其他失败
 */
esp_err_t mpu6050_calc_angle(mpu6050_handle_t handle,
                             const mpu6050_acce_t *acce,
                             const mpu6050_gyro_t *gyro,
                             mpu6050_angle_t *angle);

#ifdef __cplusplus
}
#endif
