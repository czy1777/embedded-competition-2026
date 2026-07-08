/*
 * nrf24l01_app.c
 * 应用层: 自动扫频 + 锁定 + RTC 备份寄存器持久化信道 + 协议解析
 */
#include <rtthread.h>
#include "nrf24l01.h"
#include "nrf24l01_app.h"
#include "uart_app.h"

#define LIMIT(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

#define DWELL_MS        80          /* 扫频时每个信道驻留 */
#define LOST_MS         1500        /* 锁定后多久没新包判定丢失 */

/* RTC 备份寄存器 BKP0 持久化保存锁定信道.
 * 高 16 位写魔数 0xA50A 用于校验有效性, 低 8 位为信道值 (0~125) */
#define CH_MAGIC        0xA50A0000U
#define CH_MASK         0x000000FFU

_st_Remote Remote = {
    .thr = 1500, .yaw = 1500, .roll = 1500, .pitch = 1500,
    .AUX1 = 1500, .AUX2 = 1500, .AUX3 = 1500, .AUX4 = 1500,
    .AUX5 = 1500, .AUX6 = 1500
};

volatile uint8_t  nrf_link_ok = 0;
volatile uint32_t nrf_pkt_cnt = 0;

static volatile uint8_t locked_ch = 0xFF;

static void ch_save(uint8_t ch)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    RTC->BKP0R = CH_MAGIC | (ch & CH_MASK);
}

/* 返回 0~125 = 有效已存信道, 0xFF = 无记录 */
static uint8_t ch_load(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    uint32_t v = RTC->BKP0R;
    if ((v & 0xFFFF0000U) != CH_MAGIC) return 0xFF;
    uint8_t ch = v & CH_MASK;
    return (ch <= 125) ? ch : 0xFF;
}

void ANO_DT_Data_Receive_Anl(uint8_t *buf, uint8_t num)
{
    uint8_t sum = 0, i;
    if (num < 5) return;
    for (i = 0; i < num - 1; i++) sum += buf[i];
    if (sum != buf[num - 1]) return;
    if (buf[0] != 0xAA || buf[1] != 0xAF) return;
    if (buf[2] != 0x03)    return;

    Remote.thr   = LIMIT(((int16_t)(buf[4]  << 8) | buf[5]),  1000, 2000);
    Remote.yaw   = LIMIT(((int16_t)(buf[6]  << 8) | buf[7]),  1000, 2000);
    Remote.roll  = LIMIT(((int16_t)(buf[8]  << 8) | buf[9]),  1000, 2000);
    Remote.pitch = LIMIT(((int16_t)(buf[10] << 8) | buf[11]), 1000, 2000);
    Remote.AUX1  = LIMIT(((int16_t)(buf[12] << 8) | buf[13]), 1000, 2000);
    Remote.AUX2  = LIMIT(((int16_t)(buf[14] << 8) | buf[15]), 1000, 2000);
    Remote.AUX3  = LIMIT(((int16_t)(buf[16] << 8) | buf[17]), 1000, 2000);
    Remote.AUX4  = LIMIT(((int16_t)(buf[18] << 8) | buf[19]), 1000, 2000);
    Remote.AUX5  = LIMIT(((int16_t)(buf[20] << 8) | buf[21]), 1000, 2000);
    Remote.AUX6  = LIMIT(((int16_t)(buf[22] << 8) | buf[23]), 1000, 2000);
    nrf_pkt_cnt++;
}

static void rx_thread(void *p)
{
    uint8_t   scan_ch     = 0;
    rt_tick_t dwell_start = 0;
    uint32_t  last_pkts   = 0;
    rt_tick_t last_pkt_tick = rt_tick_get();
    uint8_t   saved_ch    = ch_load();
    (void)p;

    /* 优先尝试上次锁定的信道 */
    if (saved_ch != 0xFF) {
        scan_ch = saved_ch;
        // uart1_printf("[NRF] resume saved CH=%d\r\n", saved_ch);
    }
    ANO_NRF_SetChannel(scan_ch);
    dwell_start = rt_tick_get();

    while (1) {
        ANO_NRF_Check_Event();
        if (nrf_data_ready) {
            uint8_t pkt_len = NRF24L01_RXDATA[3] + 5;
            ANO_DT_Data_Receive_Anl(NRF24L01_RXDATA, pkt_len);
            nrf_data_ready = 0;
        }

        if (locked_ch == 0xFF) {
            if (nrf_pkt_cnt > last_pkts) {
                locked_ch = scan_ch;
                last_pkts = nrf_pkt_cnt;
                last_pkt_tick = rt_tick_get();
                nrf_link_ok = 1;
                ch_save(scan_ch);
                // uart1_printf("[NRF] >>> LOCKED CH=%d (saved) <<<\r\n", scan_ch);
            } else if (rt_tick_get() - dwell_start >
                       rt_tick_from_millisecond(DWELL_MS)) {
                if (++scan_ch > 125) scan_ch = 0;
                ANO_NRF_SetChannel(scan_ch);
                dwell_start = rt_tick_get();
            }
        } else {
            if (nrf_pkt_cnt > last_pkts) {
                last_pkts = nrf_pkt_cnt;
                last_pkt_tick = rt_tick_get();
                nrf_link_ok = 1;
            }
            if (rt_tick_get() - last_pkt_tick >
                rt_tick_from_millisecond(LOST_MS)) {
                // uart1_printf("[NRF] !!! lost CH=%d, rescan\r\n", locked_ch);
                locked_ch = 0xFF;
                scan_ch = 0;
                nrf_link_ok = 0;
                ANO_NRF_SetChannel(scan_ch);
                dwell_start = rt_tick_get();
            }
        }
        rt_thread_mdelay(2);
    }
}

// 关闭 NRF 在 UART1 上的周期调试输出, 避免与 K230 关键点帧互相污染
// static void dbg_thread(void *p)
// {
//     (void)p;
//     rt_thread_mdelay(2000);
//     while (1) {
//         uart1_printf("[NRF] link=%d lock=%d pkts=%lu | THR=%4d YAW=%4d ROL=%4d PIT=%4d "
//                      "A1=%4d A2=%4d A3=%4d A4=%4d A5=%4d A6=%4d\r\n",
//                      nrf_link_ok, locked_ch, nrf_pkt_cnt,
//                      Remote.thr, Remote.yaw, Remote.roll, Remote.pitch,
//                      Remote.AUX1, Remote.AUX2, Remote.AUX3,
//                      Remote.AUX4, Remote.AUX5, Remote.AUX6);
//         rt_thread_mdelay(500);
//     }
// }

int nrf_app_init(void)
{
    uint8_t retry = 0;

    NRF24L01_Configuration();
    rt_thread_mdelay(20);

    while (NRF24L01_Check() != 0) {
        rt_kprintf("[NRF] check FAIL retry=%d\n", ++retry);
        rt_thread_mdelay(200);
        if (retry >= 10) return -RT_ERROR;
    }
    rt_kprintf("[NRF] check OK\n");

    ANO_NRF_Init(0);   /* 信道由扫频线程动态设置 */

    rt_thread_t t;
    t = rt_thread_create("nrf_rx", rx_thread, RT_NULL,
                         1024, RT_THREAD_PRIORITY_MAX / 2 - 1, 5);
    if (t) rt_thread_startup(t);
    // 关闭 nrf_dbg 线程, 避免空函数被调度
    // t = rt_thread_create("nrf_dbg", dbg_thread, RT_NULL,
    //                      1024, RT_THREAD_PRIORITY_MAX / 2 + 2, 10);
    // if (t) rt_thread_startup(t);
    return RT_EOK;
}
INIT_APP_EXPORT(nrf_app_init);
