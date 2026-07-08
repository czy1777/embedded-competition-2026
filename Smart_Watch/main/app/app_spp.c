/**
 * @file app_spp.c
 * @brief 应用层蓝牙实现
 *
 * 负责健康数据打包和BLE发送。
 */

#include "app_spp.h"
#include "spp.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "APP_SPP";

/*=============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 计算XOR校验和（从header到checksum前一字节）
 */
static uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief BLE连接状态变更回调
 */
static void ble_conn_state_cb(ble_conn_state_t state)
{
    switch (state) {
    case BLE_STATE_CONNECTED:
        ESP_LOGI(TAG, "蓝牙已连接");
        break;
    case BLE_STATE_ADVERTISING:
        ESP_LOGI(TAG, "蓝牙广播中，等待连接...");
        break;
    default:
        break;
    }
}

/*=============================================================================
 * 公共接口
 *============================================================================*/

esp_err_t app_spp_init(void)
{
    ESP_LOGI(TAG, "初始化蓝牙应用层");
    return ble_spp_init(ble_conn_state_cb);
}

esp_err_t app_spp_send_health_data(uint8_t heart_rate, uint8_t spo2,
                                   float temperature, bool fall_detected)
{
    if (!ble_spp_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_health_packet_t pkt = {
        .header        = PACKET_HEADER,
        .type          = PACKET_TYPE_HEALTH,
        .heart_rate    = heart_rate,
        .spo2          = spo2,
        .temperature   = (int16_t)(temperature * 100),
        .fall_detected = fall_detected ? 1 : 0,
        .reserved      = {0},
        .tail          = PACKET_TAIL,
    };

    /* 校验和覆盖 header 到 reserved 末尾 (偏移0~9, 共10字节) */
    pkt.checksum = calculate_checksum((uint8_t *)&pkt,
                                      offsetof(ble_health_packet_t, checksum));

    esp_err_t ret = ble_spp_send((uint8_t *)&pkt, sizeof(pkt));
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "发送健康数据: HR=%d SpO2=%d T=%.1f Fall=%d",
                 heart_rate, spo2, temperature, fall_detected);
    }
    return ret;
}

esp_err_t app_spp_send_fall_alert(void)
{
    if (!ble_spp_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_health_packet_t pkt = {
        .header        = PACKET_HEADER,
        .type          = PACKET_TYPE_FALL_ALERT,
        .heart_rate    = 0,
        .spo2          = 0,
        .temperature   = 0,
        .fall_detected = 1,
        .reserved      = {0},
        .tail          = PACKET_TAIL,
    };

    pkt.checksum = calculate_checksum((uint8_t *)&pkt,
                                      offsetof(ble_health_packet_t, checksum));

    ESP_LOGW(TAG, "发送跌倒警报!");
    return ble_spp_send((uint8_t *)&pkt, sizeof(pkt));
}

bool app_spp_is_connected(void)
{
    return ble_spp_is_connected();
}
