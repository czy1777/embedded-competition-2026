/**
 * @file    JYD-18.c
 * @brief   SkyStar 端 JDY-18：UART6 接收 SPARK 中转的遥测数据
 *
 * 解析模式照搬 SPARK 端字节级状态机，只是写 g_telemetry 而非简单打印。
 * 全局缓存用 mutex 保护：写者 = UART6 处理线程，读者 = LVGL lv_timer 线程。
 */
#include "JYD-18.h"
#include "uart_app.h"
#include <string.h>

/*============================================================================
 *                             私有定义
 *============================================================================*/

typedef enum {
    PARSE_WAIT_HEADER = 0,
    PARSE_RECEIVING
} parse_state_t;

/*============================================================================
 *                             私有变量
 *============================================================================*/

static parse_state_t parse_state = PARSE_WAIT_HEADER;
static rt_uint8_t    parse_buf[PACKET_SIZE];
static rt_uint8_t    parse_index = 0;

static telemetry_t      g_telemetry;
static struct rt_mutex  g_telemetry_mutex;
static uint32_t         g_rx_count = 0;
static rt_bool_t        g_initialized = RT_FALSE;

/* ---- 调试统计：定位"UART 收没收到字节" vs "解析失败" ---- */
static volatile uint32_t dbg_byte_count    = 0;     /* UART6 进来的总字节数 */
static volatile uint32_t dbg_pkt_drop_tail = 0;     /* 帧尾不是 0x55 */
static volatile uint32_t dbg_pkt_drop_xor  = 0;     /* XOR 不对 */
static volatile uint32_t dbg_pkt_drop_type = 0;     /* 未知 TYPE */
static volatile uint8_t  dbg_last_byte     = 0;     /* 最近一字节，调试用 */

/*============================================================================
 *                             小端读取工具
 *============================================================================*/

static inline uint16_t rd_u16(const rt_uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t  rd_i16(const rt_uint8_t *p) { return (int16_t)rd_u16(p); }
static inline uint32_t rd_u32(const rt_uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}
static inline int32_t  rd_i32(const rt_uint8_t *p) { return (int32_t)rd_u32(p); }

static rt_uint8_t calc_xor(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint8_t s = 0;
    for (rt_size_t i = 0; i < len; i++) s ^= data[i];
    return s;
}

/*============================================================================
 *                             包处理
 *============================================================================*/

static void handle_health(const rt_uint8_t *buf)
{
    /* byte 2 hr | byte 3 spo2 | byte 4..5 temp_x100 | byte 6 fall */
    g_telemetry.hr              = buf[2];
    g_telemetry.spo2            = buf[3];
    g_telemetry.body_temp_x100  = rd_i16(buf + 4);
    uint8_t fall = buf[6];
    g_telemetry.fall_active = fall;
    if (fall) {
        g_telemetry.fall_ts = rt_tick_get();
    }
}

static void handle_fall_alert(const rt_uint8_t *buf)
{
    (void)buf;
    g_telemetry.fall_active = 1;
    g_telemetry.fall_ts = rt_tick_get();
}

static void handle_env(const rt_uint8_t *buf)
{
    g_telemetry.pm25_x10      = rd_u16(buf + 2);
    g_telemetry.env_temp_x100 = rd_i16(buf + 4);
    g_telemetry.humi_x100     = rd_u16(buf + 6);
}

static void handle_attitude(const rt_uint8_t *buf)
{
    g_telemetry.pitch_x100  = rd_i16(buf + 2);
    g_telemetry.slope_state = buf[4];
}

static void handle_gps_coord(const rt_uint8_t *buf)
{
    g_telemetry.lat_e6 = rd_i32(buf + 2);
    g_telemetry.lon_e6 = rd_i32(buf + 6);
}

static void handle_gps_meta(const rt_uint8_t *buf)
{
    g_telemetry.gps_fix_valid = buf[2];
    g_telemetry.utc_h         = buf[3];
    g_telemetry.utc_m         = buf[4];
    g_telemetry.utc_s         = buf[5];
    g_telemetry.utc_ms        = rd_u16(buf + 6);
}

static void handle_heartbeat(const rt_uint8_t *buf)
{
    g_telemetry.spark_uptime_s  = rd_u32(buf + 2);
    g_telemetry.watch_connected = buf[6];
}

static void process_packet(const rt_uint8_t *buf)
{
    if (buf[PACKET_SIZE - 1] != PACKET_TAIL) { dbg_pkt_drop_tail++; return; }
    if (buf[10] != calc_xor(buf, 10))        { dbg_pkt_drop_xor++;  return; }

    uint8_t type = buf[1];

    rt_mutex_take(&g_telemetry_mutex, RT_WAITING_FOREVER);
    switch (type)
    {
    case PACKET_TYPE_HEALTH:     handle_health(buf);     break;
    case PACKET_TYPE_FALL_ALERT: handle_fall_alert(buf); break;
    case PACKET_TYPE_ENV:        handle_env(buf);        break;
    case PACKET_TYPE_ATTITUDE:   handle_attitude(buf);   break;
    case PACKET_TYPE_GPS_COORD:  handle_gps_coord(buf);  break;
    case PACKET_TYPE_GPS_META:   handle_gps_meta(buf);   break;
    case PACKET_TYPE_HEARTBEAT:  handle_heartbeat(buf);  break;
    default:
        dbg_pkt_drop_type++;
        rt_mutex_release(&g_telemetry_mutex);
        return;     /* 未知 TYPE 忽略 */
    }
    if (type < 8) g_telemetry.last_pkt_tick[type] = rt_tick_get();
    g_rx_count++;
    rt_mutex_release(&g_telemetry_mutex);
}

static void parse_byte(rt_uint8_t b)
{
    switch (parse_state)
    {
    case PARSE_WAIT_HEADER:
        if (b == PACKET_HEADER)
        {
            parse_buf[0] = b;
            parse_index = 1;
            parse_state = PARSE_RECEIVING;
        }
        break;
    case PARSE_RECEIVING:
        parse_buf[parse_index++] = b;
        if (parse_index >= PACKET_SIZE)
        {
            process_packet(parse_buf);
            parse_state = PARSE_WAIT_HEADER;
            parse_index = 0;
        }
        break;
    }
}

/*============================================================================
 *                             UART 回调
 *============================================================================*/

static void jdy18_rx_callback(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    (void)port;
    dbg_byte_count += size;
    if (size > 0) dbg_last_byte = data[size - 1];
    for (rt_size_t i = 0; i < size; i++) parse_byte(data[i]);
}

/*============================================================================
 *                             公开 API
 *============================================================================*/

int jdy18_init(void)
{
    /* 一次性资源初始化:缓存、互斥锁、解析状态机 */
    if (!g_initialized) {
        memset(&g_telemetry, 0, sizeof(g_telemetry));
        rt_mutex_init(&g_telemetry_mutex, "telmtx", RT_IPC_FLAG_PRIO);

        parse_state = PARSE_WAIT_HEADER;
        parse_index = 0;
        g_rx_count = 0;

        g_initialized = RT_TRUE;
        rt_kprintf("[JDY18] SkyStar slave initialized on UART6\n");
    }

    /* 回调每次都重新绑定:防御 uart_app_init 在同优先级 INIT_APP_EXPORT
     * 阶段后跑、把 UART6 回调重置为 uart6_default_rx_handler 的竞争 */
    uart_set_rx_callback(JDY18_UART_PORT, jdy18_rx_callback);
    return 0;
}

void jdy18_get_telemetry(telemetry_t *out)
{
    if (out == RT_NULL) return;
    rt_mutex_take(&g_telemetry_mutex, RT_WAITING_FOREVER);
    memcpy(out, &g_telemetry, sizeof(telemetry_t));
    rt_mutex_release(&g_telemetry_mutex);
}

uint32_t jdy18_get_rx_count(void)
{
    return g_rx_count;
}

/* 启动期自动注册；main() 再显式调一次也安全 */
INIT_APP_EXPORT(jdy18_init);

/*============================================================================
 *                             MSH 调试命令
 *============================================================================*/

static void cmd_tele_dump(void)
{
    telemetry_t t;
    jdy18_get_telemetry(&t);
    rt_kprintf("=== Telemetry snapshot (rx_count=%u) ===\n", (unsigned)g_rx_count);
    rt_kprintf("HR=%u SpO2=%u BodyT=%d.%02d Fall=%u\n",
               t.hr, t.spo2,
               t.body_temp_x100 / 100,
               (t.body_temp_x100 >= 0 ? t.body_temp_x100 : -t.body_temp_x100) % 100,
               t.fall_active);
    rt_kprintf("PM2.5=%u.%u  EnvT=%d.%02d  Hum=%u.%02u\n",
               t.pm25_x10 / 10, t.pm25_x10 % 10,
               t.env_temp_x100 / 100,
               (t.env_temp_x100 >= 0 ? t.env_temp_x100 : -t.env_temp_x100) % 100,
               t.humi_x100 / 100, t.humi_x100 % 100);
    rt_kprintf("Pitch=%d.%02d Slope=%u\n",
               t.pitch_x100 / 100,
               (t.pitch_x100 >= 0 ? t.pitch_x100 : -t.pitch_x100) % 100,
               t.slope_state);
    rt_kprintf("GPS fix=%u lat_e6=%d lon_e6=%d  UTC=%02u:%02u:%02u.%03u\n",
               t.gps_fix_valid, (int)t.lat_e6, (int)t.lon_e6,
               t.utc_h, t.utc_m, t.utc_s, t.utc_ms);
    rt_kprintf("SPARK uptime=%us  Watch conn=%u\n",
               (unsigned)t.spark_uptime_s, t.watch_connected);
}
MSH_CMD_EXPORT_ALIAS(cmd_tele_dump, tele_dump, Dump current telemetry snapshot);

/* 持续观察收包速率：每秒打印 字节/有效包/各类丢包，定位失败原因 */
static void cmd_tele_watch(int argc, char **argv)
{
    int sec = (argc > 1) ? atoi(argv[1]) : 10;
    if (sec < 1)  sec = 1;
    if (sec > 60) sec = 60;

    uint32_t prev_bytes = dbg_byte_count;
    uint32_t prev_ok    = jdy18_get_rx_count();
    uint32_t prev_tail  = dbg_pkt_drop_tail;
    uint32_t prev_xor   = dbg_pkt_drop_xor;
    uint32_t prev_type  = dbg_pkt_drop_type;

    rt_kprintf("[tele_watch] %d s; byte=进 UART6 字节数, ok=有效包, drop_*=丢包原因\n", sec);
    rt_kprintf("baseline byte=%u ok=%u drop_tail=%u drop_xor=%u drop_type=%u last=0x%02X\n",
               prev_bytes, prev_ok, prev_tail, prev_xor, prev_type, dbg_last_byte);

    for (int i = 1; i <= sec; i++)
    {
        rt_thread_mdelay(1000);
        uint32_t b  = dbg_byte_count;
        uint32_t o  = jdy18_get_rx_count();
        uint32_t dt = dbg_pkt_drop_tail;
        uint32_t dx = dbg_pkt_drop_xor;
        uint32_t dp = dbg_pkt_drop_type;
        rt_kprintf("[t=%2ds] +byte=%u  +ok=%u  +drop(tail=%u xor=%u type=%u)  last=0x%02X\n",
                   i, b - prev_bytes, o - prev_ok,
                   dt - prev_tail, dx - prev_xor, dp - prev_type,
                   dbg_last_byte);
        prev_bytes = b; prev_ok = o; prev_tail = dt; prev_xor = dx; prev_type = dp;
    }
    rt_kprintf("[tele_watch] done\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_tele_watch, tele_watch, Watch rx packet rate for N seconds (default 10));
