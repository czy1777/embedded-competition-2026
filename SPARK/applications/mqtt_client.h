/**
 * @file    mqtt_client.h
 * @brief   SPARK 上云 MQTT 客户端 (基于 kawaii-mqtt + RW007 WiFi)
 */

#ifndef APPLICATIONS_MQTT_CLIENT_H
#define APPLICATIONS_MQTT_CLIENT_H

#include "mqttclient.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动云端服务 (WiFi + MQTT + 发布线程)
 * @return RT_EOK 成功,负数失败
 *
 *   会自动通过 INIT_APP_EXPORT 在系统启动后调起,无需手动调用。
 *   该函数只 spawn 一个 cloud_thread,实际 WiFi/MQTT 连接在线程内异步完成,
 *   不阻塞 sensor_threads_init。
 */
int cloud_start(void);

/**
 * @brief 获取 mqtt client 句柄供 cloud_pub.c 使用
 * @return mqtt_client_t * 或 RT_NULL (尚未连接)
 */
mqtt_client_t *cloud_mqtt_get(void);

/**
 * @brief 查询云端是否就绪 (WiFi+MQTT 都连上)
 */
rt_bool_t cloud_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_MQTT_CLIENT_H */
