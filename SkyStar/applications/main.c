#include <bsp_system.h>
#include "Emm_driver.h"
#include "uart_app.h"
#include "JYD-18.h"

Emm_Motor_t motor_test;
Emm_Motor_t motor_test2;

void emm_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size) {
    if (port == UART_PORT_2) {
        Emm_ProcessBuffer(&motor_test, data, size);
    } else if (port == UART_PORT_3) {
        Emm_ProcessBuffer(&motor_test2, data, size);
    }
}

/* ====== K230 关键点帧接收 + 解析 ======
 * 协议: $KP,seq,n,x,y,c,...*XX\r\n
 *   seq: 0-255 (取 % 256)
 *   n  : 关键点数 (本系统固定 6 = COCO 11/12/13/14/15/16, 即 双髋/双膝/双踝)
 *   x,y: 像素坐标 (0-320, 整数)
 *   c  : 置信度 (0-99, 整数)
 *   XX : 从 'K' 到 '*' 前一字节的 XOR (2 位大写 HEX)
 * n=0 帧表示"未识别到人",不更新 g_last_kp 仅累加计数。
 */

/* === 全局缓存:最新一帧解析结果 ===
 *   写者: kp_rx_handler (UART RX 回调上下文,单写)
 *   读者: kp_show MSH 命令 / 实施 4 中的 BLE 反向发送线程
 *   volatile 保证读者拿到最新值;字段无依赖关系,读取过程被打断只会拿到混合的新旧,
 *   单字段语义独立,不会损坏数据结构,无需上锁。
 */
typedef struct {
    rt_uint8_t  has_data;    /* 是否曾经收到过至少一帧有效解析 */
    rt_uint8_t  seq;
    rt_uint16_t kp[6][2];    /* 6 点 (x, y) */
    rt_uint8_t  conf[6];     /* 6 点 confidence (0-99) */
    rt_tick_t   tick;        /* 最近一次更新时刻 */
} kp_frame_t;

static volatile kp_frame_t g_last_kp;
static volatile rt_uint32_t g_kp_total = 0;      /* 收到过的所有有效校验帧 */
static volatile rt_uint32_t g_kp_parsed = 0;     /* 成功解析为 6 点的帧 */
static volatile rt_uint32_t g_kp_nodetect = 0;   /* n=0 "无人" 帧 */
static volatile rt_uint32_t g_kp_bad_format = 0; /* 校验过但字段不符 */

static rt_uint8_t hex_to_u8(rt_uint8_t c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0xFF;
}

/* 从 (*pp) 指向的位置就地解析一个十进制整数,可带前导 '-'。
 *   成功:*pp 推进到首个非数字字符,*out = 数值,返回 0
 *   失败(没有任何数字字符):返回 -1
 */
static int parse_one_int(const rt_uint8_t **pp, const rt_uint8_t *end, int *out)
{
    const rt_uint8_t *p = *pp;
    int v = 0, sign = 1, got = 0;
    if (p < end && *p == '-') { sign = -1; p++; }
    while (p < end && *p >= '0' && *p <= '9') {
        v = v * 10 + (*p - '0');
        got = 1;
        p++;
    }
    if (!got) return -1;
    *out = v * sign;
    *pp = p;
    return 0;
}

/* 解析 [body, body_end) 范围内的 "KP,seq,n[,x,y,c]*n" 内容。
 *   返回 0 = 成功(6 点),1 = n=0 无人帧,负值 = 解析错。
 */
static int parse_kp_payload(const rt_uint8_t *body, const rt_uint8_t *body_end)
{
    const rt_uint8_t *p = body;

    if (p + 3 > body_end || p[0] != 'K' || p[1] != 'P' || p[2] != ',') return -1;
    p += 3;

    int seq = 0, n = 0;
    if (parse_one_int(&p, body_end, &seq) != 0) return -2;
    if (p >= body_end || *p != ',') return -3;
    p++;
    if (parse_one_int(&p, body_end, &n) != 0) return -4;

    if (n == 0) return 1;       /* 无人帧 */
    if (n != 6) return -5;      /* 非预期点数 */

    rt_uint16_t tmp_xy[6][2];
    rt_uint8_t  tmp_c[6];

    for (int i = 0; i < 6; i++) {
        int x, y, c;
        if (p >= body_end || *p != ',') return -10 - i * 3;
        p++;
        if (parse_one_int(&p, body_end, &x) != 0) return -11 - i * 3;
        if (p >= body_end || *p != ',') return -12 - i * 3;
        p++;
        if (parse_one_int(&p, body_end, &y) != 0) return -13 - i * 3;
        if (p >= body_end || *p != ',') return -14 - i * 3;
        p++;
        if (parse_one_int(&p, body_end, &c) != 0) return -15 - i * 3;

        if (x < 0) x = 0;       if (x > 65535) x = 65535;
        if (y < 0) y = 0;       if (y > 65535) y = 65535;
        if (c < 0) c = 0;       if (c > 99)    c = 99;

        tmp_xy[i][0] = (rt_uint16_t)x;
        tmp_xy[i][1] = (rt_uint16_t)y;
        tmp_c[i]     = (rt_uint8_t)c;
    }

    /* 一次性提交,降低读者拿到部分更新的概率 */
    for (int i = 0; i < 6; i++) {
        g_last_kp.kp[i][0] = tmp_xy[i][0];
        g_last_kp.kp[i][1] = tmp_xy[i][1];
        g_last_kp.conf[i]  = tmp_c[i];
    }
    g_last_kp.seq      = (rt_uint8_t)seq;
    g_last_kp.tick     = rt_tick_get();
    g_last_kp.has_data = 1;
    return 0;
}

static void kp_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    (void)port;
    if (size < 8 || data[0] != '$' || data[1] != 'K' || data[2] != 'P') {
        return;
    }
    rt_size_t star = 0;
    for (rt_size_t i = 1; i < size; i++) {
        if (data[i] == '*') { star = i; break; }
    }
    if (star == 0 || star + 2 >= size) return;

    rt_uint8_t hi = hex_to_u8(data[star + 1]);
    rt_uint8_t lo = hex_to_u8(data[star + 2]);
    if (hi == 0xFF || lo == 0xFF) return;
    rt_uint8_t expect = (hi << 4) | lo;

    rt_uint8_t calc = 0;
    for (rt_size_t i = 1; i < star; i++) calc ^= data[i];

    if (calc != expect) {
        uart1_printf("[KP] BAD CRC exp=%02X got=%02X\r\n", expect, calc);
        return;
    }

    g_kp_total++;
    int pr = parse_kp_payload(&data[1], &data[star]);
    if (pr == 0)        g_kp_parsed++;
    else if (pr == 1)   g_kp_nodetect++;
    else                g_kp_bad_format++;

    /* 仍保留原 UART1 转发,便于继续观察原始 K230 帧 */
    uart_send(UART_PORT_1, data, size);
}

/* ========= MSH 命令:打印最新一帧 + 计数器 ========= */
static int kp_show(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("[KP] total=%u parsed=%u nodetect=%u bad=%u\n",
               (unsigned)g_kp_total, (unsigned)g_kp_parsed,
               (unsigned)g_kp_nodetect, (unsigned)g_kp_bad_format);
    if (!g_last_kp.has_data) {
        rt_kprintf("[KP] no frame yet\n");
        return 0;
    }
    rt_uint32_t age_ms = (rt_tick_get() - g_last_kp.tick) * 1000UL / RT_TICK_PER_SECOND;
    rt_kprintf("[KP] seq=%u age=%ums\n", (unsigned)g_last_kp.seq, (unsigned)age_ms);
    const char *name[6] = { "L-Hip ", "R-Hip ", "L-Knee", "R-Knee", "L-Ankle", "R-Ankle" };
    for (int i = 0; i < 6; i++) {
        rt_kprintf("  %s  x=%-4u y=%-4u c=%u\n",
                   name[i],
                   (unsigned)g_last_kp.kp[i][0],
                   (unsigned)g_last_kp.kp[i][1],
                   (unsigned)g_last_kp.conf[i]);
    }
    return 0;
}
MSH_CMD_EXPORT(kp_show, show latest K230 keypoint frame);

/* ========= K230 → SPARK 反向 BLE 发送线程 =========
 *   每 100ms 从 g_last_kp 抓快照,seq 变化才发,打包成 36 字节长帧:
 *     [0]=0xAA [1]=0x10 [2]=31 [3]=seq [4..27]=xy(LE) [28..33]=conf [34]=XOR [35]=0x55
 *   通过 UART6 (JDY-18 从模块) 反向透传给 SPARK。
 */
#define KP_LONG_TYPE      0x10
#define KP_LONG_FRAME_LEN 36
#define KP_TX_PERIOD_MS   100

static rt_thread_t kp_tx_thread_handle = RT_NULL;
static volatile rt_uint32_t g_kp_tx_count = 0;

static void kp_tx_thread_entry(void *param)
{
    (void)param;
    rt_uint8_t buf[KP_LONG_FRAME_LEN];
    rt_uint8_t last_seq = 0;
    rt_bool_t  first = RT_TRUE;

    rt_kprintf("[KP-TX] thread started, period=%dms\n", KP_TX_PERIOD_MS);

    while (1)
    {
        rt_thread_mdelay(KP_TX_PERIOD_MS);

        if (!g_last_kp.has_data) continue;

        /* 抓快照 — 中断屏蔽期间一次拷贝所有字段,防止跨写读取 */
        rt_base_t level = rt_hw_interrupt_disable();
        rt_uint8_t  seq = g_last_kp.seq;
        rt_uint16_t xy[6][2];
        rt_uint8_t  cf[6];
        for (int i = 0; i < 6; i++)
        {
            xy[i][0] = g_last_kp.kp[i][0];
            xy[i][1] = g_last_kp.kp[i][1];
            cf[i]    = g_last_kp.conf[i];
        }
        rt_hw_interrupt_enable(level);

        /* K230 没出新帧 → 跳过,避免重复占用 BLE 带宽 */
        if (!first && seq == last_seq) continue;

        buf[0] = 0xAA;
        buf[1] = KP_LONG_TYPE;
        buf[2] = 31;
        buf[3] = seq;
        for (int i = 0; i < 6; i++)
        {
            buf[4 + i * 4]     = (rt_uint8_t)(xy[i][0] & 0xFF);
            buf[4 + i * 4 + 1] = (rt_uint8_t)((xy[i][0] >> 8) & 0xFF);
            buf[4 + i * 4 + 2] = (rt_uint8_t)(xy[i][1] & 0xFF);
            buf[4 + i * 4 + 3] = (rt_uint8_t)((xy[i][1] >> 8) & 0xFF);
        }
        for (int i = 0; i < 6; i++)
        {
            buf[28 + i] = cf[i];
        }

        rt_uint8_t xor_v = 0;
        for (int i = 0; i < KP_LONG_FRAME_LEN - 2; i++) xor_v ^= buf[i];
        buf[KP_LONG_FRAME_LEN - 2] = xor_v;
        buf[KP_LONG_FRAME_LEN - 1] = 0x55;

        uart_send(UART_PORT_6, buf, KP_LONG_FRAME_LEN);
        g_kp_tx_count++;
        last_seq = seq;
        first = RT_FALSE;
    }
}

void kp_tx_start(void)
{
    if (kp_tx_thread_handle != RT_NULL) return;
    kp_tx_thread_handle = rt_thread_create("kp_tx", kp_tx_thread_entry,
                                           RT_NULL, 1024, 17, 10);
    if (kp_tx_thread_handle)
    {
        rt_thread_startup(kp_tx_thread_handle);
    }
    else
    {
        rt_kprintf("[KP-TX] thread create failed\n");
    }
}

/* MSH: 看反向发送统计 */
static int kp_tx_stat(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("[KP-TX] sent=%u (frames pushed to UART6)\n", (unsigned)g_kp_tx_count);
    return 0;
}
MSH_CMD_EXPORT(kp_tx_stat, show K230 -> SPARK BLE reverse tx count);

int main(void)
{
    // 电机1初始化 - UART2
    if (Emm_Create(&motor_test, UART_PORT_2, 0x01, 1000) == 0) {
        uart_set_rx_callback(UART_PORT_2, emm_rx_handler);
        rt_kprintf("Motor1 init success!\n");
    }

    // 电机2初始化 - UART3
    if (Emm_Create(&motor_test2, UART_PORT_3, 0x01, 1000) == 0) {
        uart_set_rx_callback(UART_PORT_3, emm_rx_handler);
        rt_kprintf("Motor2 init success!\n");
    }

    // K230 关键点接收 - UART4
    uart_set_rx_callback(UART_PORT_4, kp_rx_handler);
    rt_kprintf("KP RX on UART4 ready\n");

    // BLE 遥测接收 - UART6 (SPARK 中转)
    // 已通过 INIT_APP_EXPORT 自动注册，此处再调一次便于日志确认
    jdy18_init();

    // K230 步态反向发送线程 (UART6 BLE -> SPARK, 36B 长帧 type=0x10)
    kp_tx_start();

    // 电机使能
    Emm_SetEnableControl(&motor_test, true, false);
    Emm_SetEnableControl(&motor_test2, true, false);

    /* 电机控制由 rc_drive 线程 (INIT_APP_EXPORT 自动注册) 接管 */
    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return 0;
}
