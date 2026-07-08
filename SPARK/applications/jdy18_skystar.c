/**
 * @file    jdy18_skystar.c
 * @brief   第二只 JDY-18：SPARK <-> SkyStar 中转 BLE 链路 (UART2)
 *
 * 设计上完全独立于 JYD-18.c（UART4 -> Smart_Watch 链路），
 * 包括解析状态机、AT 模式标志、连接状态、统计计数都各自一份。
 * 这样两路 UART 中断回调互不踩对方的静态变量。
 */

#include "jdy18_skystar.h"
#include "uart_app.h"

/*============================================================================
 *                             私有定义
 *============================================================================*/

typedef enum {
    SKY_PARSE_WAIT_HEADER = 0,
    SKY_PARSE_RECEIVING
} sky_parse_state_t;

/*============================================================================
 *                             私有变量
 *============================================================================*/

/* 反向长帧 (K230 步态,SkyStar -> SPARK):
 *   [0]=0xAA  [1]=0x10  [2]=31  [3]=seq
 *   [4..27]=6×(x_lo,x_hi,y_lo,y_hi)  [28..33]=6×conf
 *   [34]=XOR(buf[0..33])  [35]=0x55
 */
#define SKY_TYPE_KP_LONG     0x10
#define SKY_KP_LONG_LEN      36
#define SKY_PARSE_BUF_SIZE   SKY_KP_LONG_LEN   /* 兼容现有 12B 短帧 */

static sky_parse_state_t sky_parse_state = SKY_PARSE_WAIT_HEADER;
static rt_uint8_t        sky_parse_buf[SKY_PARSE_BUF_SIZE];
static rt_uint8_t        sky_parse_index = 0;
static rt_size_t         sky_expected_len = PACKET_SIZE;  /* 见到 type 后动态调整 */

static rt_tick_t         sky_last_rx_tick = 0;
static rt_uint32_t       sky_rx_count = 0;
static rt_uint32_t       sky_kp_long_count = 0;

static volatile rt_bool_t sky_at_mode = RT_FALSE;

/* K230 步态最新一帧 (写者: sky_handle_kp_long;读者: cloud_pub_kp) */
static volatile spark_kp_t g_spark_kp;

/*============================================================================
 *                             私有函数
 *============================================================================*/

static rt_uint8_t sky_xor(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint8_t s = 0;
    for (rt_size_t i = 0; i < len; i++) s ^= data[i];
    return s;
}

/* K230 步态长帧解析 -> 写入 g_spark_kp */
static void sky_handle_kp_long(const rt_uint8_t *buf)
{
    g_spark_kp.seq = buf[3];
    for (int i = 0; i < 6; i++)
    {
        rt_uint16_t x = (rt_uint16_t)buf[4 + i * 4]
                      | ((rt_uint16_t)buf[4 + i * 4 + 1] << 8);
        rt_uint16_t y = (rt_uint16_t)buf[4 + i * 4 + 2]
                      | ((rt_uint16_t)buf[4 + i * 4 + 3] << 8);
        g_spark_kp.kp[i][0] = x;
        g_spark_kp.kp[i][1] = y;
        g_spark_kp.conf[i]  = buf[28 + i];
    }
    g_spark_kp.tick = rt_tick_get();
    g_spark_kp.has_data = 1;
    sky_kp_long_count++;
}

/* 通用反向包分发 (type ≥ 0x10) */
static void sky_handle_reverse(const rt_uint8_t *buf, rt_size_t len)
{
    rt_uint8_t type = buf[1];
    if (type == SKY_TYPE_KP_LONG && len == SKY_KP_LONG_LEN)
    {
        sky_handle_kp_long(buf);
        return;
    }
    rt_kprintf("[JDY-Sky] rx reverse type=0x%02X len=%u (dropped)\n",
               type, (unsigned)len);
}

static void sky_process_packet(const rt_uint8_t *buf, rt_size_t len)
{
    /* 帧尾 */
    if (buf[len - 1] != PACKET_TAIL) return;

    /* XOR 校验:覆盖 [0, len-2),期望值在 [len-2] */
    if (buf[len - 2] != sky_xor(buf, len - 2)) return;

    sky_last_rx_tick = rt_tick_get();
    sky_rx_count++;

    rt_uint8_t type = buf[1];
    if (type >= 0x10)
    {
        sky_handle_reverse(buf, len);
    }
    else
    {
        /* SkyStar 端不会主动发 0x01..0x0F，收到忽略 */
    }
}

static void sky_parse_byte(rt_uint8_t b)
{
    switch (sky_parse_state)
    {
    case SKY_PARSE_WAIT_HEADER:
        if (b == PACKET_HEADER)
        {
            sky_parse_buf[0] = b;
            sky_parse_index = 1;
            sky_expected_len = PACKET_SIZE;   /* 默认假设 12B,见到 type 再修 */
            sky_parse_state = SKY_PARSE_RECEIVING;
        }
        break;
    case SKY_PARSE_RECEIVING:
        sky_parse_buf[sky_parse_index++] = b;

        /* 刚收到 type 字段:决定本帧实际长度 */
        if (sky_parse_index == 2)
        {
            sky_expected_len = (b == SKY_TYPE_KP_LONG)
                             ? SKY_KP_LONG_LEN
                             : PACKET_SIZE;
        }

        if (sky_parse_index >= sky_expected_len)
        {
            sky_process_packet(sky_parse_buf, sky_expected_len);
            sky_parse_state = SKY_PARSE_WAIT_HEADER;
            sky_parse_index = 0;
        }
        break;
    }
}

static void sky_rx_callback(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    if (sky_at_mode)
    {
        if (size < UART_BUFFER_SIZE) data[size] = '\0';
        rt_kprintf("[JDY-Sky] AT resp: %s\n", data);
        return;
    }
    for (rt_size_t i = 0; i < size; i++) sky_parse_byte(data[i]);
}

static void sky_send_at(const char *cmd)
{
    static const rt_uint8_t crlf[2] = {'\r', '\n'};
    if (!cmd) return;
    uart_send(JDY18_SKY_UART_PORT, (const rt_uint8_t *)cmd, rt_strlen(cmd));
    uart_send(JDY18_SKY_UART_PORT, crlf, 2);
}

/*============================================================================
 *                             公开函数
 *============================================================================*/

void jdy18_sky_init(void)
{
    uart_set_rx_callback(JDY18_SKY_UART_PORT, sky_rx_callback);

    sky_parse_state = SKY_PARSE_WAIT_HEADER;
    sky_parse_index = 0;
    sky_last_rx_tick = 0;
    sky_rx_count = 0;

    rt_kprintf("[JDY-Sky] Initialized on UART2 (9600 baud)\n");
}

void jdy18_sky_configure(void)
{
    rt_kprintf("[JDY-Sky] Starting AT configuration...\n");
    sky_at_mode = RT_TRUE;

    sky_send_at("AT+ROLE1");        rt_thread_mdelay(200);
    sky_send_at("AT+SVRUUIDFFE0");  rt_thread_mdelay(200);
    sky_send_at("AT+CHRUUIDFFE1");  rt_thread_mdelay(200);
    sky_send_at("AT+BAUD4");        rt_thread_mdelay(200);
    sky_send_at("AT+STARTEN0");     rt_thread_mdelay(200);
    sky_send_at("AT+ENLOG0");       rt_thread_mdelay(200);
    sky_send_at("AT+RESET");        rt_thread_mdelay(1000);

    sky_at_mode = RT_FALSE;
    rt_kprintf("[JDY-Sky] Configuration done. Bind with: jdy18_sky_bind <MAC>\n");
}

rt_size_t jdy18_sky_send(const rt_uint8_t pkt[PACKET_SIZE])
{
    if (pkt == RT_NULL) return 0;
    return uart_send(JDY18_SKY_UART_PORT, pkt, PACKET_SIZE);
}

rt_bool_t jdy18_sky_is_alive(void)
{
    if (sky_last_rx_tick == 0) return RT_FALSE;
    rt_tick_t now = rt_tick_get();
    return ((now - sky_last_rx_tick) < rt_tick_from_millisecond(5000)) ? RT_TRUE : RT_FALSE;
}

rt_uint32_t jdy18_sky_rx_count(void)
{
    return sky_rx_count;
}

rt_uint32_t jdy18_sky_kp_count(void)
{
    return sky_kp_long_count;
}

rt_bool_t spark_kp_peek(spark_kp_t *out)
{
    if (out == RT_NULL || !g_spark_kp.has_data) return RT_FALSE;
    rt_base_t level = rt_hw_interrupt_disable();
    out->has_data = g_spark_kp.has_data;
    out->seq      = g_spark_kp.seq;
    for (int i = 0; i < 6; i++)
    {
        out->kp[i][0] = g_spark_kp.kp[i][0];
        out->kp[i][1] = g_spark_kp.kp[i][1];
        out->conf[i]  = g_spark_kp.conf[i];
    }
    out->tick = g_spark_kp.tick;
    rt_hw_interrupt_enable(level);
    return RT_TRUE;
}

static int spark_kp_show(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("[SPARK-KP] rx_long=%u\n", (unsigned)sky_kp_long_count);
    spark_kp_t kp;
    if (!spark_kp_peek(&kp))
    {
        rt_kprintf("[SPARK-KP] no frame yet\n");
        return 0;
    }
    rt_uint32_t age_ms = (rt_tick_get() - kp.tick) * 1000UL / RT_TICK_PER_SECOND;
    rt_kprintf("[SPARK-KP] seq=%u age=%ums\n", (unsigned)kp.seq, (unsigned)age_ms);
    const char *name[6] = { "L-Hip ", "R-Hip ", "L-Knee", "R-Knee", "L-Ankle", "R-Ankle" };
    for (int i = 0; i < 6; i++)
    {
        rt_kprintf("  %s  x=%-4u y=%-4u c=%u\n",
                   name[i],
                   (unsigned)kp.kp[i][0],
                   (unsigned)kp.kp[i][1],
                   (unsigned)kp.conf[i]);
    }
    return 0;
}
MSH_CMD_EXPORT(spark_kp_show, show latest K230 frame received from SkyStar);

/*============================================================================
 *                             MSH 命令
 *============================================================================*/

static void cmd_jdy18_sky_config(void)
{
    jdy18_sky_configure();
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_sky_config, jdy18_sky_config,
                    Configure SPARK->SkyStar JDY-18 module);

static void cmd_jdy18_sky_at(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: jdy18_sky_at <command>\n");
        return;
    }
    sky_at_mode = RT_TRUE;
    sky_send_at(argv[1]);
    rt_thread_mdelay(500);
    sky_at_mode = RT_FALSE;
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_sky_at, jdy18_sky_at,
                    Send AT cmd to SkyStar-side JDY-18);

static void cmd_jdy18_sky_bind(int argc, char **argv)
{
    char cmd[32];
    if (argc < 2)
    {
        rt_kprintf("Usage: jdy18_sky_bind <MAC>\n");
        return;
    }
    sky_at_mode = RT_TRUE;
    rt_snprintf(cmd, sizeof(cmd), "AT+BAND%s", argv[1]);
    sky_send_at(cmd);
    rt_thread_mdelay(500);
    sky_send_at("AT+RESET");
    rt_thread_mdelay(1000);
    sky_at_mode = RT_FALSE;
    rt_kprintf("[JDY-Sky] Bound to %s, resetting...\n", argv[1]);
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_sky_bind, jdy18_sky_bind,
                    Bind SkyStar-side JDY-18 to MAC);

static void cmd_jdy18_sky_ping(void)
{
    rt_uint8_t pkt[PACKET_SIZE];
    build_pkt_heartbeat(pkt, (rt_uint32_t)(rt_tick_get() / RT_TICK_PER_SECOND), 0);
    rt_size_t n = jdy18_sky_send(pkt);
    rt_kprintf("[JDY-Sky] ping sent %d bytes, rx_count=%u\n",
               (int)n, (unsigned)sky_rx_count);
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_sky_ping, jdy18_sky_ping,
                    Send a heartbeat packet for link smoke test);
