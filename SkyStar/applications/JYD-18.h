/*
 * JYD-18.h
 * SkyStar 端 JDY-18 BLE 从机（UART6）：接收来自 SPARK 中转站的遥测数据。
 *
 * 协议与 SPARK 端镜像：12 字节定长包，header 0xAA / tail 0x55 / XOR(byte 0..9)。
 * 收到的数据落到模块内 g_telemetry 全局缓存，UI 线程通过 lv_timer 定时刷新。
 */
#ifndef APPLICATIONS_JYD_18_H_
#define APPLICATIONS_JYD_18_H_

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 协议常量（与 SPARK 端 JYD-18.h / telemetry_pack.h 一致） ---- */
#define PACKET_HEADER           0xAA
#define PACKET_TAIL             0x55
#define PACKET_SIZE             12

#define PACKET_TYPE_HEALTH      0x01
#define PACKET_TYPE_FALL_ALERT  0x02
#define PACKET_TYPE_ENV         0x03
#define PACKET_TYPE_ATTITUDE    0x04
#define PACKET_TYPE_GPS_COORD   0x05
#define PACKET_TYPE_GPS_META    0x06
#define PACKET_TYPE_HEARTBEAT   0x07

/* JDY-18 串口 */
#define JDY18_UART_PORT         UART_PORT_6

/* ---- 遥测缓存 ---- */
typedef struct {
    /* health (0x01 / 0x02) */
    uint8_t   hr;
    uint8_t   spo2;
    int16_t   body_temp_x100;
    uint8_t   fall_active;
    rt_tick_t fall_ts;          /* 最近一次 fall=1 的 tick，UI 用于 5s 横幅 */

    /* environment (0x03) */
    uint16_t  pm25_x10;
    int16_t   env_temp_x100;
    uint16_t  humi_x100;

    /* attitude (0x04) */
    int16_t   pitch_x100;
    uint8_t   slope_state;      /* 0=FLAT 1=UPHILL 2=DOWNHILL */

    /* GPS (0x05 / 0x06) */
    int32_t   lat_e6;
    int32_t   lon_e6;
    uint8_t   gps_fix_valid;
    uint8_t   utc_h, utc_m, utc_s;
    uint16_t  utc_ms;

    /* heartbeat (0x07) */
    uint32_t  spark_uptime_s;
    uint8_t   watch_connected;

    /* 各 TYPE 最近一次更新 tick，UI 可据此显示「不活跃」 */
    rt_tick_t last_pkt_tick[8];
} telemetry_t;

/**
 * @brief 模块初始化（注册 UART6 回调、清零缓存）
 *        多次调用安全（内部已去重）
 */
int jdy18_init(void);

/**
 * @brief 拷贝当前 g_telemetry 快照（加锁，UI 用）
 */
void jdy18_get_telemetry(telemetry_t *out);

/**
 * @brief 累计收包数（调试用）
 */
uint32_t jdy18_get_rx_count(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_JYD_18_H_ */
