/**
 * @file    cloud_config.h
 * @brief   云端上行配置 (WiFi / MQTT broker / device id)
 *
 *  烧录前必须修改:CLOUD_WIFI_SSID / CLOUD_WIFI_PASSWORD 改为你的实际 WiFi。
 *  其余项除非要换 broker 或换 device id,否则保持默认即可。
 */

#ifndef APPLICATIONS_CLOUD_CONFIG_H
#define APPLICATIONS_CLOUD_CONFIG_H

/* ------- WiFi (RW007 STA 模式) ------- */
#ifndef CLOUD_WIFI_SSID
#define CLOUD_WIFI_SSID         "YOUR_WIFI_SSID"        /* TODO: 改成你的 SSID */
#endif
#ifndef CLOUD_WIFI_PASSWORD
#define CLOUD_WIFI_PASSWORD     "YOUR_WIFI_PASSWORD"    /* TODO: 改成你的密码  */
#endif

/* ------- MQTT broker (公网 EMQX,无 TLS) ------- */
#ifndef CLOUD_MQTT_HOST
#define CLOUD_MQTT_HOST         "broker.emqx.io"
#endif
#ifndef CLOUD_MQTT_PORT
#define CLOUD_MQTT_PORT         "1883"
#endif

/* ------- 设备身份 ------- */
#ifndef CLOUD_DEVICE_ID
#define CLOUD_DEVICE_ID         "spark-walker-001"      /* 多设备时各自改唯一名 */
#endif
#ifndef CLOUD_TOPIC_PREFIX
#define CLOUD_TOPIC_PREFIX      "walker/" CLOUD_DEVICE_ID
#endif

/* MQTT client id (broker 上的连接身份,避免与其他 client 冲突) */
#ifndef CLOUD_MQTT_CLIENT_ID
#define CLOUD_MQTT_CLIENT_ID    "spark-" CLOUD_DEVICE_ID
#endif

/* ------- 发布周期 (毫秒) ------- */
#define CLOUD_TICK_MS           100         /* 调度基准 (对齐 K230 10Hz) */
#define CLOUD_PUB_KP_MS         100         /* K230 步态 10Hz */
#define CLOUD_PUB_ATTITUDE_MS   200
#define CLOUD_PUB_HEALTH_MS     500
#define CLOUD_PUB_GPS_MS        1000
#define CLOUD_PUB_HEARTBEAT_MS  1000
#define CLOUD_PUB_ENV_MS        2000

/* ------- 线程参数 ------- */
#define CLOUD_THREAD_NAME       "cloud"
#define CLOUD_THREAD_STACK_SIZE 4096
#define CLOUD_THREAD_PRIORITY   17          /* 低于 sensor 线程,不抢占 BLE/relay */
#define CLOUD_THREAD_TIMESLICE  10

#endif /* APPLICATIONS_CLOUD_CONFIG_H */
