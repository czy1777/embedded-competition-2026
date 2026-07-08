/*
 * xiaozhi_link.c
 * UART5 @115200, 把 JDY-18 收到的健康/跌倒/环境/GPS 单向透传给 xiaozhi。
 *
 * 协议复用 JYD-18.h 的 type 编号 (0x01/0x02/0x03/0x05/0x06/0x07), xiaozhi 端
 * SkyStarLink 直接镜像本工程 JYD-18.c 的解析逻辑。
 *
 * 周期 1Hz 轮发 HEALTH/ENV/GPS_COORD/GPS_META/HEARTBEAT, fall_ts 变化时
 * 立即多发一帧 FALL_ALERT (0x02), 避免 1s 周期把告警延迟拖大。
 */
#include <rtthread.h>
#include <string.h>
#include "xiaozhi_link.h"
#include "uart_app.h"
#include "JYD-18.h"

#define DBG_TAG "xz_link"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define XZ_UART_PORT         UART_PORT_5
#define XZ_TX_PERIOD_MS      1000

/* 与 xiaozhi 端 / SkyStar JYD-18 完全一致的常量。这里不复用 JYD-18.h 的宏
 * 是因为下方那些常量没在 JYD-18.h 暴露; 重新声明 PACKET_HEADER/TAIL/SIZE。 */
#define XZ_PKT_HEADER        0xAA
#define XZ_PKT_TAIL          0x55
#define XZ_PKT_SIZE          12

/* ============== 工具 ============== */

static void seal_packet(rt_uint8_t pkt[XZ_PKT_SIZE], uint8_t type)
{
    pkt[0] = XZ_PKT_HEADER;
    pkt[1] = type;
    rt_uint8_t xor_calc = 0;
    for (int i = 0; i < XZ_PKT_SIZE - 2; i++) xor_calc ^= pkt[i];
    pkt[XZ_PKT_SIZE - 2] = xor_calc;
    pkt[XZ_PKT_SIZE - 1] = XZ_PKT_TAIL;
}

static inline void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void wr_i16(uint8_t *p, int16_t v) { wr_u16(p, (uint16_t)v); }
static inline void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8)  & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline void wr_i32(uint8_t *p, int32_t v) { wr_u32(p, (uint32_t)v); }

/* ============== 打包: 各 type 一个 builder ============== */

static void build_pkt_health(rt_uint8_t pkt[XZ_PKT_SIZE], const telemetry_t *s)
{
    pkt[2] = s->hr;
    pkt[3] = s->spo2;
    wr_i16(&pkt[4], s->body_temp_x100);
    pkt[6] = s->fall_active;
    pkt[7] = 0;
    pkt[8] = 0;
    pkt[9] = 0;
    seal_packet(pkt, PACKET_TYPE_HEALTH);
}

static void build_pkt_fall_alert(rt_uint8_t pkt[XZ_PKT_SIZE])
{
    rt_memset(&pkt[2], 0, 8);
    seal_packet(pkt, PACKET_TYPE_FALL_ALERT);
}

static void build_pkt_env(rt_uint8_t pkt[XZ_PKT_SIZE], const telemetry_t *s)
{
    wr_u16(&pkt[2], s->pm25_x10);
    wr_i16(&pkt[4], s->env_temp_x100);
    wr_u16(&pkt[6], s->humi_x100);
    pkt[8] = 0;
    pkt[9] = 0;
    seal_packet(pkt, PACKET_TYPE_ENV);
}

static void build_pkt_gps_coord(rt_uint8_t pkt[XZ_PKT_SIZE], const telemetry_t *s)
{
    wr_i32(&pkt[2], s->lat_e6);
    wr_i32(&pkt[6], s->lon_e6);
    seal_packet(pkt, PACKET_TYPE_GPS_COORD);
}

static void build_pkt_gps_meta(rt_uint8_t pkt[XZ_PKT_SIZE], const telemetry_t *s)
{
    pkt[2] = s->gps_fix_valid;
    pkt[3] = s->utc_h;
    pkt[4] = s->utc_m;
    pkt[5] = s->utc_s;
    wr_u16(&pkt[6], s->utc_ms);
    pkt[8] = 0;
    pkt[9] = 0;
    seal_packet(pkt, PACKET_TYPE_GPS_META);
}

static void build_pkt_heartbeat(rt_uint8_t pkt[XZ_PKT_SIZE], const telemetry_t *s)
{
    wr_u32(&pkt[2], s->spark_uptime_s);
    pkt[6] = s->watch_connected;
    pkt[7] = 0;
    pkt[8] = 0;
    pkt[9] = 0;
    seal_packet(pkt, PACKET_TYPE_HEARTBEAT);
}

/* ============== TX 线程 ============== */

static void xz_tx_thread(void *p)
{
    (void)p;
    rt_uint8_t pkt[XZ_PKT_SIZE];
    telemetry_t snap;
    rt_tick_t last_fall_ts_seen = 0;
    rt_tick_t next_tick = rt_tick_get();

    while (1) {
        rt_tick_t now = rt_tick_get();
        rt_int32_t remain = (rt_int32_t)(next_tick - now);
        if (remain > 0) {
            rt_thread_mdelay((rt_uint32_t)remain * 1000 / RT_TICK_PER_SECOND);
        }

        jdy18_get_telemetry(&snap);

        /* 跌倒沿: fall_ts 出现新值时立即多发一帧 FALL_ALERT, 让 xiaozhi
         * 端在 1s 周期外感知, 缩短告警延迟。 */
        if (snap.fall_active && snap.fall_ts != last_fall_ts_seen) {
            last_fall_ts_seen = snap.fall_ts;
            build_pkt_fall_alert(pkt);
            uart_send(XZ_UART_PORT, pkt, XZ_PKT_SIZE);
        }

        build_pkt_health(pkt, &snap);
        uart_send(XZ_UART_PORT, pkt, XZ_PKT_SIZE);

        build_pkt_env(pkt, &snap);
        uart_send(XZ_UART_PORT, pkt, XZ_PKT_SIZE);

        build_pkt_gps_coord(pkt, &snap);
        uart_send(XZ_UART_PORT, pkt, XZ_PKT_SIZE);

        build_pkt_gps_meta(pkt, &snap);
        uart_send(XZ_UART_PORT, pkt, XZ_PKT_SIZE);

        build_pkt_heartbeat(pkt, &snap);
        uart_send(XZ_UART_PORT, pkt, XZ_PKT_SIZE);

        next_tick += rt_tick_from_millisecond(XZ_TX_PERIOD_MS);
        /* 落后太多就重新对齐, 避免补发雪崩 */
        if ((rt_int32_t)(rt_tick_get() - next_tick) > rt_tick_from_millisecond(XZ_TX_PERIOD_MS)) {
            next_tick = rt_tick_get() + rt_tick_from_millisecond(XZ_TX_PERIOD_MS);
        }
    }
}

/* ============== 自动注册 ============== */

int xiaozhi_link_init(void)
{
    /* 链路是单向上行, 不注册 UART5 RX 回调 (默认 handler 仅打印, 无害)。 */

    rt_thread_t t = rt_thread_create("xz_tx", xz_tx_thread, RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     10);
    if (t == RT_NULL) {
        LOG_E("tx thread create FAIL");
        return -RT_ERROR;
    }
    rt_thread_startup(t);

    LOG_I("up on UART5 @115200");
    return RT_EOK;
}
INIT_APP_EXPORT(xiaozhi_link_init);
