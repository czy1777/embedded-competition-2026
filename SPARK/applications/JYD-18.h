#ifndef APPLICATIONS_JYD_18_H_
#define APPLICATIONS_JYD_18_H_

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *                             常量定义
 *============================================================================*/

/* JDY-18 使用的串口端口 (UART4) */
#define JDY18_UART_PORT         UART_PORT_4

/* 健康数据包定义 (与 ESP32-S3 端 app_spp.h 完全一致) */
#define PACKET_HEADER           0xAA
#define PACKET_TAIL             0x55
#define PACKET_SIZE             12

/* 数据包类型 */
#define PACKET_TYPE_HEALTH      0x01    /* 健康数据 */
#define PACKET_TYPE_FALL_ALERT  0x02    /* 跌倒警报 */

/* AT 指令相关 */
#define JDY18_AT_RESP_TIMEOUT   1000    /* AT 响应超时 (ms) */
#define JDY18_AT_RESP_BUF_SIZE  128     /* AT 响应缓冲区大小 */

/*============================================================================
 *                             类型定义
 *============================================================================*/

/* JDY-18 连接状态 */
typedef enum {
    JDY18_STATE_UNINIT = 0,     /* 未初始化 */
    JDY18_STATE_DISCONNECTED,   /* 已断开 */
    JDY18_STATE_CONNECTED       /* 已连接 */
} jdy18_state_t;

/* 健康数据包结构体 (12 字节, packed, 与 ESP32-S3 一致) */
#pragma pack(push, 1)
typedef struct {
    rt_uint8_t  header;         /* 帧头: 0xAA */
    rt_uint8_t  type;           /* 数据类型 */
    rt_uint8_t  heart_rate;     /* 心率 BPM (0-255) */
    rt_uint8_t  spo2;           /* 血氧百分比 (0-100) */
    rt_int16_t  temperature;    /* 温度 x100 (3650 = 36.50 C) */
    rt_uint8_t  fall_detected;  /* 跌倒状态: 0=正常, 1=跌倒 */
    rt_uint8_t  reserved[3];    /* 保留字段 */
    rt_uint8_t  checksum;       /* XOR 校验和 (字节 0~9) */
    rt_uint8_t  tail;           /* 帧尾: 0x55 */
} ble_health_packet_t;
#pragma pack(pop)

/*============================================================================
 *                             函数声明
 *============================================================================*/

/**
 * @brief  初始化 JDY-18 蓝牙模块
 *         注册 UART4 接收回调，启动数据包解析
 */
void jdy18_init(void);

/**
 * @brief  发送 AT 指令 (自动追加 \r\n)
 * @param  cmd AT 指令字符串, 如 "AT+ROLE1"
 */
void jdy18_send_at(const char *cmd);

/**
 * @brief  一次性配置 JDY-18 (主机模式、UUID、波特率等)
 *         配置参数掉电保存，只需执行一次
 */
void jdy18_configure(void);

/**
 * @brief  扫描并绑定 ESP32-S3
 *         发送 AT+INQ 扫描，找到 SmartWatch-S3 后绑定 MAC
 */
void jdy18_scan_and_bind(void);

/**
 * @brief  获取最新的健康数据
 * @param  pkt 数据包输出指针
 * @return RT_TRUE 有新数据, RT_FALSE 无数据
 */
rt_bool_t jdy18_get_health_data(ble_health_packet_t *pkt);

/**
 * @brief  无副作用读取最新一帧（不清新数据标志）
 *         供中转线程使用，让 jdy18_task 继续负责日志/超时
 * @return RT_TRUE 已收到过任意有效包；RT_FALSE 从未收到包
 */
rt_bool_t jdy18_peek_latest(ble_health_packet_t *pkt);

/**
 * @brief  查询连接状态
 * @return 当前连接状态
 */
jdy18_state_t jdy18_get_state(void);

/**
 * @brief  检查是否已连接
 */
rt_bool_t jdy18_is_connected(void);

/**
 * @brief  获取最后一次有效数据的系统 tick
 */
rt_tick_t jdy18_get_last_update_tick(void);

/**
 * @brief  JDY-18 周期任务 (在 BLE 线程中调用)
 *         监控连接状态，打印调试信息
 */
void jdy18_task(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_JYD_18_H_ */
