/**
 * @file app_mpu6050.h
 * @brief MPU6050 应用层接口 - 跌倒检测功能
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"
#include "mpu6050.h"

/*=============================================================================
 * I2C 硬件配置
 *============================================================================*/
#define APP_I2C_PORT        I2C_NUM_0
#define APP_I2C_SDA_PIN     8       // GPIO8
#define APP_I2C_SCL_PIN     9       // GPIO9
#define APP_I2C_FREQ_HZ     400000  // 400kHz

/*=============================================================================
 * 跌倒检测参数（4 阶段状态机：NORMAL → FREE_FALL → IMPACT → POSTURE → DETECTED）
 *============================================================================*/

/* 阶段 1：自由落体 —— 加速度模值塌陷 */
#define FALL_FREE_FALL_TH           0.6f    /* acce_mag 下限 (g) */
#define FALL_FREE_FALL_MIN_FRAMES   2       /* 连续达阈值帧数（20ms/帧） */
#define FALL_FREE_FALL_TIMEOUT      1000    /* 自由落体最大停留 (ms) */

/* 阶段 2：撞击 + 角速度峰值 */
#define FALL_IMPACT_TH              2.5f    /* 撞击 acce_mag 下限 (g) */
#define FALL_IMPACT_WINDOW_MS       200     /* 撞击后捕捉 gyro 峰值窗口 (ms) */
#define FALL_IMPACT_GYRO_MIN_TH     120.0f  /* 角速度模值峰值下限 (dps) */

/* 阶段 3：姿态保持 + 静止确认 */
#define FALL_POSTURE_ANGLE_TH       60.0f   /* |roll|或|pitch| 阈值 (度) */
#define FALL_POSTURE_MIN_FRAMES     25      /* 姿态保持帧数（25≈500ms） */
#define FALL_POSTURE_TIMEOUT        2500    /* 姿态阶段最大停留 (ms) */

/* 静止判据：近 N 帧 mean(|acce_mag-1.0|) < 阈值 */
#define FALL_STILL_WINDOW_N         15      /* 静止判据窗口帧数（15≈300ms） */
#define FALL_STILL_THRESHOLD        0.15f   /* 平均偏离上限 (g) */

/* 陀螺仪静态零偏校准 */
#define GYRO_CALIB_SAMPLES          100     /* 校准采样数（100×10ms = 1s） */
#define GYRO_CALIB_SETTLE_MS        300     /* 校准前静置时间 (ms) */

/* DLPF & 采样率 */
#define MPU6050_DEFAULT_DLPF        MPU6050_DLPF_44HZ
#define MPU6050_DEFAULT_RATE_HZ     50

/*=============================================================================
 * 数据结构
 *============================================================================*/

// 传感器综合数据
typedef struct {
    mpu6050_acce_t  acce;       // 加速度 (g)
    mpu6050_gyro_t  gyro;       // 角速度 (度/秒)
    mpu6050_angle_t angle;      // 姿态角 (度)
    float           acce_mag;   // 加速度模值 (g)
} app_mpu6050_data_t;

// 跌倒检测状态
typedef enum {
    FALL_STATE_NORMAL = 0,      // 正常状态
    FALL_STATE_FREE_FALL,       // 自由落体检测中
    FALL_STATE_IMPACT,          // 撞击特征采集中（观察窗内捕捉 gyro 峰值）
    FALL_STATE_POSTURE,         // 姿态保持 + 静止确认中
    FALL_STATE_DETECTED,        // 跌倒已检测到
} app_fall_state_t;

/*=============================================================================
 * API 函数声明
 *============================================================================*/

/**
 * @brief 初始化 MPU6050 传感器和 I2C 总线
 * @return ESP_OK 成功，其他失败
 */
esp_err_t app_mpu6050_init(void);

/**
 * @brief 反初始化 MPU6050
 */
void app_mpu6050_deinit(void);

/**
 * @brief 读取传感器数据并更新姿态角
 * @param[out] data 传感器综合数据
 * @return ESP_OK 成功，其他失败
 */
esp_err_t app_mpu6050_read(app_mpu6050_data_t *data);

/**
 * @brief 跌倒检测（基于当前传感器数据）
 * @param data 传感器数据
 * @return true 检测到跌倒，false 未检测到
 */
bool app_mpu6050_detect_fall(const app_mpu6050_data_t *data);

/**
 * @brief 获取当前跌倒检测状态
 * @return 当前状态
 */
app_fall_state_t app_mpu6050_get_fall_state(void);

/**
 * @brief 重置跌倒检测状态（确认后调用）
 */
void app_mpu6050_reset_fall_state(void);

#ifdef __cplusplus
}
#endif
