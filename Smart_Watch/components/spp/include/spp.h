/**
 * @file spp.h
 * @brief BLE SPP 底层驱动接口
 *
 * 基于NimBLE协议栈实现BLE GATT服务，提供数据透传功能。
 * ESP32-S3作为BLE Peripheral，等待Central（如JDY-18）连接。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BLE 设备名称 */
#define BLE_DEVICE_NAME             "SmartWatch-S3"

/* 自定义健康数据服务UUID (16-bit, 使用FFE0匹配JDY-18默认透传UUID) */
#define BLE_SVC_HEALTH_UUID16       0xFFE0
/* 健康数据特征UUID (16-bit, 使用FFE1匹配JDY-18默认透传UUID) */
#define BLE_CHR_HEALTH_DATA_UUID16  0xFFE1

/* BLE连接状态 */
typedef enum {
    BLE_STATE_IDLE = 0,         /* 空闲（未初始化） */
    BLE_STATE_ADVERTISING,      /* 广播中，等待连接 */
    BLE_STATE_CONNECTED,        /* 已连接 */
} ble_conn_state_t;

/* 连接状态变更回调 */
typedef void (*ble_conn_callback_t)(ble_conn_state_t state);

/**
 * @brief 初始化BLE SPP服务
 * @param conn_cb 连接状态回调，可为NULL
 * @return ESP_OK 成功
 */
esp_err_t ble_spp_init(ble_conn_callback_t conn_cb);

/**
 * @brief 反初始化BLE
 */
void ble_spp_deinit(void);

/**
 * @brief 通过BLE Notification发送数据
 * @param data 数据缓冲区
 * @param len  数据长度
 * @return ESP_OK 成功, ESP_ERR_INVALID_STATE 未连接或未订阅
 */
esp_err_t ble_spp_send(const uint8_t *data, uint16_t len);

/**
 * @brief 获取当前连接状态
 */
ble_conn_state_t ble_spp_get_state(void);

/**
 * @brief 检查是否已连接
 */
bool ble_spp_is_connected(void);

#ifdef __cplusplus
}
#endif
