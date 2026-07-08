/*
 * jdy18_skystar.h
 * 第二只 JDY-18：SPARK 通过 UART2 连接 SkyStar 端 JDY-18 从机。
 * 与 JYD-18.c 完全独立（各自一套静态解析状态机），不共享变量。
 *
 * 角色：本端 = ROLE1 主机；对端（SkyStar）= ROLE0 从机。
 * 链路通后由 sensor_threads.c 的 relay 线程周期下发遥测包。
 */
#ifndef APPLICATIONS_JDY18_SKYSTAR_H_
#define APPLICATIONS_JDY18_SKYSTAR_H_

#include <rtthread.h>
#include "JYD-18.h"          /* 复用 PACKET_SIZE 等宏 */
#include "telemetry_pack.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 中转链路用的串口 */
#define JDY18_SKY_UART_PORT  UART_PORT_2

/**
 * @brief  初始化：注册 UART2 接收回调，重置状态机
 *         不发送 AT 指令；首轮配置请用 finsh 命令 jdy18_sky_config
 */
void jdy18_sky_init(void);

/**
 * @brief  一次性 AT 配置（同 JDY-18 主机：ROLE1 + FFE0/FFE1 + 9600 + 关 log）
 *         配置参数掉电保存，只需执行一次
 */
void jdy18_sky_configure(void);

/**
 * @brief  发送一个完整的 12 字节遥测包
 * @return 实际发送字节数
 */
rt_size_t jdy18_sky_send(const rt_uint8_t pkt[PACKET_SIZE]);

/**
 * @brief  对端（SkyStar）链路是否活跃（基于最近一次收到包的时间）
 */
rt_bool_t jdy18_sky_is_alive(void);

/**
 * @brief  累计有效收包数（含 12B 短帧 + 36B 长帧)
 */
rt_uint32_t jdy18_sky_rx_count(void);

/**
 * @brief  累计 K230 步态长帧成功解析数 (反向 type=0x10)
 */
rt_uint32_t jdy18_sky_kp_count(void);

/* ====== K230 步态数据 (SkyStar -> SPARK 反向通道) ====== */

typedef struct {
    rt_uint8_t  has_data;
    rt_uint8_t  seq;
    rt_uint16_t kp[6][2];   /* L-Hip, R-Hip, L-Knee, R-Knee, L-Ankle, R-Ankle */
    rt_uint8_t  conf[6];    /* 置信度 0-99 */
    rt_tick_t   tick;       /* 接收时刻 */
} spark_kp_t;

/**
 * @brief  抓取最新一帧 K230 步态数据 (一次性快照,内部用中断屏蔽保证字段一致)
 * @return RT_TRUE 有数据并已写入 *out, RT_FALSE 从未收到过帧
 */
rt_bool_t spark_kp_peek(spark_kp_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_JDY18_SKYSTAR_H_ */
