/**
 * @file app_spp.h
 * @brief 应用层蓝牙接口
 *
 * 封装BLE数据打包和发送逻辑，提供健康数据传输API。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * 数据包定义
 *============================================================================*/

#define PACKET_HEADER   0xAA
#define PACKET_TAIL     0x55

/* 数据包类型 */
#define PACKET_TYPE_HEALTH      0x01    /* 健康数据 */
#define PACKET_TYPE_FALL_ALERT  0x02    /* 跌倒警报 */

/* 健康数据包 (12字节固定长度)
 *
 * 协议约定：
 *  - 所有多字节字段按 小端序（ESP32 本地序）。
 *  - heart_rate == 0：MAX30102 未在 COOLDOWN 态 / 测量无效。
 *  - spo2       == 0：同上。
 *  - temperature == 0：未佩戴 / DS18B20 sensor_fault / 启动滤波未填满。
 *                     正常值范围 3400-4200（对应 34.00-42.00°C）。
 *  - fall_detected：
 *      type=0x01 健康包：反映 MPU6050 当前是否处于 DETECTED 态
 *                       （跌倒确认后持续 ~5s 为 1，复位后自动归 0）。
 *      type=0x02 告警包：固定 1，一次性边沿事件。
 *  - checksum：覆盖 偏移 0..9（header 到 reserved 末尾）的 XOR。
 */
typedef struct __attribute__((packed)) {
    uint8_t  header;        /* 帧头: 0xAA */
    uint8_t  type;          /* 数据类型 */
    uint8_t  heart_rate;    /* 心率 BPM (0-255) */
    uint8_t  spo2;          /* 血氧百分比 (0-100) */
    int16_t  temperature;   /* 温度 x100 (3650 = 36.50°C) */
    uint8_t  fall_detected; /* 跌倒状态: 0=正常, 1=跌倒 */
    uint8_t  reserved[3];   /* 保留字段 */
    uint8_t  checksum;      /* XOR校验和 */
    uint8_t  tail;          /* 帧尾: 0x55 */
} ble_health_packet_t;

/*=============================================================================
 * API
 *============================================================================*/

/**
 * @brief 初始化蓝牙应用层（内部调用ble_spp_init）
 */
esp_err_t app_spp_init(void);

/**
 * @brief 发送健康数据包
 * @param heart_rate   心率 BPM
 * @param spo2         血氧百分比
 * @param temperature  温度（摄氏度）
 * @param fall_detected 是否跌倒
 * @return ESP_OK 成功
 */
esp_err_t app_spp_send_health_data(uint8_t heart_rate, uint8_t spo2,
                                   float temperature, bool fall_detected);

/**
 * @brief 发送跌倒警报（紧急）
 */
esp_err_t app_spp_send_fall_alert(void);

/**
 * @brief 检查蓝牙是否已连接
 */
bool app_spp_is_connected(void);

#ifdef __cplusplus
}
#endif
