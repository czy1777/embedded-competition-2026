/**
 * @file app_ds18b20.h
 * @brief DS18B20 温度传感器应用层接口
 *
 * 本模块面向"手环腕部测温"场景：
 *  - 异步非阻塞状态机（不再阻塞主循环 800ms）
 *  - 腕温 → 核心体温映射（线性两参数）
 *  - 滑动平均滤波 + 异常值/幻值/离群剔除
 *  - 佩戴状态识别（迟滞 + 连续确认）
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "ds18b20_types.h"

/*=============================================================================
 * 硬件配置
 *============================================================================*/
#define APP_DS18B20_GPIO            18                      /* OneWire 数据引脚 */
#define APP_DS18B20_RESOLUTION      DS18B20_RESOLUTION_12B  /* 量化精度 0.0625°C */

/*=============================================================================
 * 时序参数
 *============================================================================*/
#define CONVERT_WAIT_MS             780   /* 12-bit 规格 max 750ms，留 30ms 裕量 */
#define SAMPLE_INTERVAL_MS          1000  /* 两次采样最小间隔 */

/*=============================================================================
 * 滤波与异常值参数
 *============================================================================*/
#define FILTER_WINDOW_SIZE          5     /* 滑动平均窗口（约 5 秒） */

#define DS18B20_RAW_SANE_MIN       (-10.0f)
#define DS18B20_RAW_SANE_MAX       ( 60.0f)
#define OUTLIER_DELTA               5.0f  /* |raw - mean| > 此值视作离群 */
#define OUTLIER_MAX_STREAK          3     /* 连续偏差达此次数视为真实跳变 */
#define MAX_CONSECUTIVE_ERRORS      5     /* 连续失败达此次数置 sensor_fault */

/*=============================================================================
 * 腕温 → 核心体温映射（线性两参数：body = skin * GAIN + OFFSET）
 *============================================================================*/
#define WRIST_GAIN                  0.30f
#define WRIST_OFFSET                27.5f
#define BODY_TEMP_MIN               34.0f
#define BODY_TEMP_MAX               42.0f

/*=============================================================================
 * 佩戴状态识别（阈值 + 迟滞 + 连续确认）
 *============================================================================*/
#define WEAR_ON_THRESHOLD_C         29.5f  /* skin_filtered 高于此值 → 可能佩戴 */
#define WEAR_OFF_THRESHOLD_C        28.0f  /* skin_filtered 低于此值 → 可能脱下（迟滞） */
#define WEAR_CONFIRM_SAMPLES        5      /* 连续确认样本数 */

/*=============================================================================
 * 对外数据类型
 *============================================================================*/
typedef enum {
    APP_DS18B20_WEAR_UNKNOWN = 0,  /* 启动阶段 ring 未填满，或 sensor_fault */
    APP_DS18B20_WEAR_OFF,
    APP_DS18B20_WEAR_ON,
} app_ds18b20_wear_state_t;

typedef struct {
    float    skin_temp_raw;        /* 最近一次原始读数（未滤波） */
    float    skin_temp_filtered;   /* 滑动平均后皮温 */
    float    body_temp;            /* 映射后的核心体温估计（仅 WORN 有效） */
    bool     valid;                /* body_temp 是否可用 */
    bool     sensor_fault;         /* 连续失败 → true */
    app_ds18b20_wear_state_t wear_state;
    uint32_t last_update_tick;     /* xTaskGetTickCount() 时间戳 */
} app_ds18b20_data_t;

/*=============================================================================
 * API
 *============================================================================*/

/** @brief 初始化 DS18B20（含总线、设备、分辨率设置与状态清零） */
esp_err_t app_ds18b20_init(void);

/** @brief 反初始化 */
void app_ds18b20_deinit(void);

/**
 * @brief 推进状态机（应在主循环每次迭代调用，非阻塞）
 *
 * 单次调用最坏耗时 ≈ 一次 OneWire reset + 少量字节读写（~1.5ms），
 * 远低于 20ms 主循环预算。
 */
void app_ds18b20_tick(void);

/**
 * @brief 读取最新快照
 * @param[out] out 数据结构
 * @return ESP_OK 成功；ESP_ERR_INVALID_ARG out 为 NULL
 */
esp_err_t app_ds18b20_read(app_ds18b20_data_t *out);

/**
 * @brief 读取核心体温（向后兼容接口）
 * @param[out] temperature 映射后的体温估计
 * @return ESP_OK 数据有效；ESP_ERR_INVALID_STATE 未佩戴/故障/启动未完成
 */
esp_err_t app_ds18b20_read_temperature(float *temperature);

#ifdef __cplusplus
}
#endif
