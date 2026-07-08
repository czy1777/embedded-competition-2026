/**
 * @file    cloud_pub.c
 * @brief   5 类遥测的 MQTT 发布 (snprintf 拼 JSON)
 *
 *   字段命名与前端 web/public/app.js 的 socket.on('xxx', m => ...) 严格对齐。
 *   浮点用整数 + 取模的 "%d.%0Nd" 模式输出,规避 newlib 浮点 printf 依赖。
 */

#include <rtthread.h>
#include <stdlib.h>
#include <string.h>

#include "mqttclient.h"
#include "cloud_config.h"
#include "mqtt_client.h"
#include "cloud_pub.h"

#include "JYD-18.h"          /* jdy18_peek_latest / jdy18_is_connected */
#include "pm2.5.h"           /* gp2y1014_get_pm25 */
#include "aht20_app.h"       /* aht20_get_th */
#include "icm20608_app.h"    /* icm20608_get_pitch_deg / icm20608_get_slope_state */
#include "atgm336h.h"        /* Save_Data / g_LatAndLongData */
#include "jdy18_skystar.h"   /* spark_kp_peek / spark_kp_t */

#define LOG_W(fmt, ...) rt_kprintf("[cloud.pub][W] " fmt "\n", ##__VA_ARGS__)

#define MAX_PAYLOAD 256        /* kp 帧 JSON 约 200 字节,需要更大 */
#define MAX_TOPIC   64

/* 设备 uptime 毫秒数,用于 ts 字段 */
static rt_uint32_t uptime_ms(void)
{
    return (rt_uint32_t)((rt_tick_get() * 1000UL) / RT_TICK_PER_SECOND);
}

/* 通用发包 */
static int publish_to(const char *channel, const char *json, int len)
{
    if (!cloud_is_ready()) return 0;
    mqtt_client_t *c = cloud_mqtt_get();
    if (!c) return 0;

    char topic[MAX_TOPIC];
    rt_snprintf(topic, sizeof(topic), CLOUD_TOPIC_PREFIX "/%s", channel);

    mqtt_message_t msg = {0};
    msg.qos = QOS0;
    msg.retained = 0;
    msg.payload = (void *)json;
    msg.payloadlen = len;

    int rc = mqtt_publish(c, topic, &msg);
    if (rc != 0)
    {
        LOG_W("publish %s rc=%d", channel, rc);
    }
    return rc;
}

/* =========================================================================
 *  health: { hr, spo2, body_temp, fall, ts }
 *  数据源: jdy18_peek_latest (UART4 BLE Smart_Watch 透传)
 * ========================================================================= */
int cloud_pub_health(void)
{
    ble_health_packet_t hp;
    if (!jdy18_peek_latest(&hp)) return 0;

    int t_x100 = hp.temperature;
    int t_neg = t_x100 < 0;
    int t_abs = t_neg ? -t_x100 : t_x100;

    char buf[MAX_PAYLOAD];
    int len = rt_snprintf(buf, sizeof(buf),
        "{\"hr\":%u,\"spo2\":%u,\"body_temp\":%s%d.%02d,\"fall\":%u,\"ts\":%u}",
        (unsigned)hp.heart_rate,
        (unsigned)hp.spo2,
        t_neg ? "-" : "", t_abs / 100, t_abs % 100,
        (unsigned)hp.fall_detected,
        (unsigned)uptime_ms());

    return publish_to("health", buf, len);
}

/* =========================================================================
 *  env: { pm25, temp, humidity, ts }
 *  数据源: gp2y1014_get_pm25 + aht20_get_th
 * ========================================================================= */
int cloud_pub_env(void)
{
    float pm25 = gp2y1014_get_pm25();
    float t = 0.0f, h = 0.0f;
    aht20_get_th(&t, &h);

    if (pm25 < 0.0f) pm25 = 0.0f;
    if (pm25 > 6553.5f) pm25 = 6553.5f;
    int pm_x10 = (int)(pm25 * 10.0f + 0.5f);

    int t_x100 = (int)(t * 100.0f);
    int t_neg = t_x100 < 0;
    int t_abs = t_neg ? -t_x100 : t_x100;

    if (h < 0.0f) h = 0.0f;
    if (h > 100.0f) h = 100.0f;
    int h_x100 = (int)(h * 100.0f + 0.5f);

    char buf[MAX_PAYLOAD];
    int len = rt_snprintf(buf, sizeof(buf),
        "{\"pm25\":%d.%d,\"temp\":%s%d.%02d,\"humidity\":%d.%02d,\"ts\":%u}",
        pm_x10 / 10, pm_x10 % 10,
        t_neg ? "-" : "", t_abs / 100, t_abs % 100,
        h_x100 / 100, h_x100 % 100,
        (unsigned)uptime_ms());

    return publish_to("env", buf, len);
}

/* =========================================================================
 *  attitude: { pitch, slope, ts }
 *  slope ∈ {"flat","uphill","downhill"} (前端期望字符串)
 * ========================================================================= */
int cloud_pub_attitude(void)
{
    float pitch = icm20608_get_pitch_deg();
    int p_x100 = (int)(pitch * 100.0f);
    int p_neg = p_x100 < 0;
    int p_abs = p_neg ? -p_x100 : p_x100;

    slope_state_t s = icm20608_get_slope_state();
    const char *slope = (s == SLOPE_UPHILL) ? "uphill"
                       : (s == SLOPE_DOWNHILL) ? "downhill" : "flat";

    char buf[MAX_PAYLOAD];
    int len = rt_snprintf(buf, sizeof(buf),
        "{\"pitch\":%s%d.%02d,\"slope\":\"%s\",\"ts\":%u}",
        p_neg ? "-" : "", p_abs / 100, p_abs % 100,
        slope,
        (unsigned)uptime_ms());

    return publish_to("attitude", buf, len);
}

/* =========================================================================
 *  gps: { lat, lon, fix, utc, ts }
 *  utc 格式 "hh:mm:ss" (从 Save_Data.UTCTime "hhmmss.ss" 解析)
 * ========================================================================= */
int cloud_pub_gps(void)
{
    rt_uint8_t fix = Save_Data.isUsefull ? 1 : 0;
    double lat = g_LatAndLongData.latitude;
    double lon = g_LatAndLongData.longitude;
    if (g_LatAndLongData.N_S == 'S') lat = -lat;
    if (g_LatAndLongData.E_W == 'W') lon = -lon;

    long lat_e6 = (long)(lat * 1000000.0);
    long lon_e6 = (long)(lon * 1000000.0);
    int lat_neg = lat_e6 < 0;
    int lon_neg = lon_e6 < 0;
    long lat_abs = lat_neg ? -lat_e6 : lat_e6;
    long lon_abs = lon_neg ? -lon_e6 : lon_e6;

    char utc[12] = "--:--:--";
    const char *u = Save_Data.UTCTime;
    if (u[0] >= '0' && u[0] <= '9' && u[1] >= '0' && u[1] <= '9' &&
        u[2] >= '0' && u[2] <= '9' && u[3] >= '0' && u[3] <= '9' &&
        u[4] >= '0' && u[4] <= '9' && u[5] >= '0' && u[5] <= '9')
    {
        utc[0] = u[0]; utc[1] = u[1]; utc[2] = ':';
        utc[3] = u[2]; utc[4] = u[3]; utc[5] = ':';
        utc[6] = u[4]; utc[7] = u[5]; utc[8] = '\0';
    }

    char buf[MAX_PAYLOAD];
    int len = rt_snprintf(buf, sizeof(buf),
        "{\"lat\":%s%ld.%06ld,\"lon\":%s%ld.%06ld,\"fix\":%u,\"utc\":\"%s\",\"ts\":%u}",
        lat_neg ? "-" : "", lat_abs / 1000000, lat_abs % 1000000,
        lon_neg ? "-" : "", lon_abs / 1000000, lon_abs % 1000000,
        (unsigned)fix,
        utc,
        (unsigned)uptime_ms());

    return publish_to("gps", buf, len);
}

/* =========================================================================
 *  heartbeat: { uptime, watch_connected, wifi_rssi, ts }
 *  uptime 单位:秒
 * ========================================================================= */
int cloud_pub_heartbeat(void)
{
    rt_uint32_t up_s = rt_tick_get() / RT_TICK_PER_SECOND;
    rt_uint8_t watch = jdy18_is_connected() ? 1 : 0;

    extern int rt_wlan_get_rssi(void);   /* wlan_mgnt.h, RT_USING_WIFI */
    int rssi = rt_wlan_get_rssi();

    char buf[MAX_PAYLOAD];
    int len = rt_snprintf(buf, sizeof(buf),
        "{\"uptime\":%u,\"watch_connected\":%u,\"wifi_rssi\":%d,\"ts\":%u}",
        (unsigned)up_s,
        (unsigned)watch,
        rssi,
        (unsigned)uptime_ms());

    return publish_to("heartbeat", buf, len);
}

/* =========================================================================
 *  kp: { seq, points: [[x,y,c]×6], ts }
 *  数据源: SkyStar 通过 BLE 反向 36B 长帧 -> spark_kp_peek
 *  去重: seq 与上次发布相同则跳过 (K230 没出新帧)
 * ========================================================================= */
int cloud_pub_kp(void)
{
    static rt_uint8_t last_seq = 0;
    static rt_bool_t  has_last = RT_FALSE;

    spark_kp_t kp;
    if (!spark_kp_peek(&kp)) return 0;
    if (has_last && kp.seq == last_seq) return 0;

    char buf[MAX_PAYLOAD];
    int len = rt_snprintf(buf, sizeof(buf),
        "{\"seq\":%u,\"points\":[[%u,%u,%u],[%u,%u,%u],[%u,%u,%u],[%u,%u,%u],[%u,%u,%u],[%u,%u,%u]],\"ts\":%u}",
        (unsigned)kp.seq,
        (unsigned)kp.kp[0][0], (unsigned)kp.kp[0][1], (unsigned)kp.conf[0],
        (unsigned)kp.kp[1][0], (unsigned)kp.kp[1][1], (unsigned)kp.conf[1],
        (unsigned)kp.kp[2][0], (unsigned)kp.kp[2][1], (unsigned)kp.conf[2],
        (unsigned)kp.kp[3][0], (unsigned)kp.kp[3][1], (unsigned)kp.conf[3],
        (unsigned)kp.kp[4][0], (unsigned)kp.kp[4][1], (unsigned)kp.conf[4],
        (unsigned)kp.kp[5][0], (unsigned)kp.kp[5][1], (unsigned)kp.conf[5],
        (unsigned)uptime_ms());

    int rc = publish_to("kp", buf, len);
    last_seq = kp.seq;
    has_last = RT_TRUE;
    return rc;
}
