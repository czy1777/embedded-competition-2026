/**
 * @file    mqtt_client.c
 * @brief   SPARK 上云客户端 (RW007 WiFi + kawaii-mqtt + 周期发布线程)
 *
 *   架构:
 *     INIT_APP_EXPORT(cloud_start)
 *        └─ rt_thread_create(cloud_thread)
 *              ├─ Step1: 连 WiFi (rt_wlan_connect + wait ready)
 *              ├─ Step2: 连 MQTT (mqtt_lease + mqtt_connect)
 *              └─ Step3: 250ms tick 循环,按各 channel 周期调 cloud_pub_*
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <wlan_mgnt.h>

#include "mqttclient.h"
#include "cloud_config.h"
#include "mqtt_client.h"
#include "cloud_pub.h"

#define LOG_I(fmt, ...) rt_kprintf("[cloud] " fmt "\n", ##__VA_ARGS__)
#define LOG_W(fmt, ...) rt_kprintf("[cloud][W] " fmt "\n", ##__VA_ARGS__)
#define LOG_E(fmt, ...) rt_kprintf("[cloud][E] " fmt "\n", ##__VA_ARGS__)

/* ============== 内部状态 ============== */
static mqtt_client_t *g_mqtt = RT_NULL;
static volatile rt_bool_t g_ready = RT_FALSE;
static rt_thread_t g_thread = RT_NULL;

/* ============== 对外查询 ============== */
mqtt_client_t *cloud_mqtt_get(void)
{
    return g_mqtt;
}

rt_bool_t cloud_is_ready(void)
{
    return g_ready;
}

/* ============== WiFi 连接 (阻塞,带超时与重试) ============== */
static int wifi_connect_blocking(void)
{
    int retry = 0;

    LOG_I("connecting WiFi: SSID=%s", CLOUD_WIFI_SSID);

    while (!rt_wlan_is_ready())
    {
        int err = rt_wlan_connect(CLOUD_WIFI_SSID, CLOUD_WIFI_PASSWORD);
        if (err == RT_EOK)
        {
            /* 等 DHCP 拿到 IP,最多 15s */
            int wait = 0;
            while (!rt_wlan_is_ready() && wait < 30)
            {
                rt_thread_mdelay(500);
                wait++;
            }
            if (rt_wlan_is_ready())
            {
                LOG_I("WiFi ready");
                return RT_EOK;
            }
        }
        retry++;
        LOG_W("WiFi connect failed (retry %d), wait 5s...", retry);
        rt_thread_mdelay(5000);
    }
    return RT_EOK;
}

/* ============== MQTT 连接 (阻塞,带重试) ============== */
static int mqtt_connect_blocking(void)
{
    g_mqtt = mqtt_lease();
    if (g_mqtt == RT_NULL)
    {
        LOG_E("mqtt_lease failed");
        return -RT_ERROR;
    }

    mqtt_set_host(g_mqtt, CLOUD_MQTT_HOST);
    mqtt_set_port(g_mqtt, CLOUD_MQTT_PORT);
    mqtt_set_client_id(g_mqtt, CLOUD_MQTT_CLIENT_ID);
    mqtt_set_clean_session(g_mqtt, 1);

    LOG_I("connecting MQTT: %s:%s clientid=%s",
          CLOUD_MQTT_HOST, CLOUD_MQTT_PORT, CLOUD_MQTT_CLIENT_ID);

    int retry = 0;
    while (1)
    {
        int rc = mqtt_connect(g_mqtt);
        if (rc == 0)
        {
            LOG_I("MQTT connected");
            return RT_EOK;
        }
        retry++;
        LOG_W("mqtt_connect rc=%d (retry %d), wait 5s...", rc, retry);
        rt_thread_mdelay(5000);
    }
}

/* ============== 主线程入口 ============== */
static void cloud_thread_entry(void *param)
{
    (void)param;

    /* 1. 等 WiFi 就绪 */
    if (wifi_connect_blocking() != RT_EOK)
    {
        LOG_E("WiFi never ready, cloud thread exit");
        return;
    }

    /* 2. 连 MQTT broker */
    if (mqtt_connect_blocking() != RT_EOK)
    {
        LOG_E("MQTT connect failed, cloud thread exit");
        return;
    }

    g_ready = RT_TRUE;
    LOG_I("cloud bridge online, start publishing");

    /* 3. 周期发布
     *   tick: 100ms 步长 (与 K230 10Hz 对齐)。各 channel 错开 tick % N 发布,避免聚集。
     *     kp        每 1 tick  (100ms)    tick % 1 == 0  ← 10Hz, 内部按 seq 去重
     *     attitude  每 2 tick  (200ms)    tick % 2 == 0
     *     health    每 5 tick  (500ms)    tick % 5 == 1
     *     gps       每 10 tick (1000ms)   tick % 10 == 3
     *     heartbeat 每 10 tick (1000ms)   tick % 10 == 7
     *     env       每 20 tick (2000ms)   tick % 20 == 4
     */
    rt_uint32_t tick = 0;
    while (1)
    {
        if (g_ready)
        {
            cloud_pub_kp();
            if ((tick % 2)  == 0) cloud_pub_attitude();
            if ((tick % 5)  == 1) cloud_pub_health();
            if ((tick % 10) == 3) cloud_pub_gps();
            if ((tick % 10) == 7) cloud_pub_heartbeat();
            if ((tick % 20) == 4) cloud_pub_env();
        }
        tick++;
        rt_thread_mdelay(CLOUD_TICK_MS);
    }
}

/* ============== 公开启动函数 ============== */
int cloud_start(void)
{
    g_thread = rt_thread_create(
        CLOUD_THREAD_NAME,
        cloud_thread_entry,
        RT_NULL,
        CLOUD_THREAD_STACK_SIZE,
        CLOUD_THREAD_PRIORITY,
        CLOUD_THREAD_TIMESLICE
    );
    if (g_thread == RT_NULL)
    {
        LOG_E("cloud thread create failed");
        return -RT_ERROR;
    }
    rt_thread_startup(g_thread);
    return RT_EOK;
}

INIT_APP_EXPORT(cloud_start);
