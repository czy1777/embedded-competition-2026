/*
 * telemetry_pack.h
 * SPARK -> SkyStar 中转协议的统一打包工具。
 *
 * 信封沿用 JYD-18.h 里的 12 字节 ble_health_packet_t 思路：
 *   byte 0      : header   = 0xAA
 *   byte 1      : type
 *   byte 2..9   : payload (8B)
 *   byte 10     : XOR(byte 0..9)
 *   byte 11     : tail     = 0x55
 *
 * 每个 TYPE 一个 build_pkt_xxx 函数，写满 12 字节到调用者提供的 out[]，
 * 由调用方拿去 uart_send。所有多字节字段小端序。
 */
#ifndef APPLICATIONS_TELEMETRY_PACK_H_
#define APPLICATIONS_TELEMETRY_PACK_H_

#include <rtthread.h>
#include "JYD-18.h"   /* 复用 PACKET_HEADER / PACKET_TAIL / PACKET_SIZE / ble_health_packet_t */

#ifdef __cplusplus
extern "C" {
#endif

/* TYPE 编码 (0x01/0x02 沿用 JYD-18.h，已定义) */
#define PACKET_TYPE_ENV         0x03    /* 环境: PM2.5 + 温 + 湿  */
#define PACKET_TYPE_ATTITUDE    0x04    /* 姿态: pitch + slope    */
#define PACKET_TYPE_GPS_COORD   0x05    /* GPS 坐标               */
#define PACKET_TYPE_GPS_META    0x06    /* GPS 时间 + fix         */
#define PACKET_TYPE_HEARTBEAT   0x07    /* SPARK 心跳             */

/* 0x10+ 预留 SkyStar -> SPARK 方向（本期 accept-and-drop） */
#define PACKET_TYPE_SKY_ACK     0x10
#define PACKET_TYPE_SKY_CMD     0x11

/* ---------------- 通用工具 ---------------- */

/* 写入 header/tail 并计算 XOR 校验，调用前 out[1..9] 必须已填好 */
void telemetry_seal(rt_uint8_t out[PACKET_SIZE], rt_uint8_t type);

/* ---------------- 各 TYPE 的 builder ---------------- */

/* 0x01 Health: 直接转发 SmartWatch 来的 12 字节包（已自带 checksum，原样 memcpy） */
void build_pkt_passthrough(rt_uint8_t out[PACKET_SIZE], const ble_health_packet_t *pkt);

/* 0x03 Environment */
void build_pkt_env(rt_uint8_t out[PACKET_SIZE],
                   rt_uint16_t pm25_x10,
                   rt_int16_t  temp_x100,
                   rt_uint16_t humi_x100);

/* 0x04 Attitude: pitch×100 (deg); slope: 0=FLAT 1=UPHILL 2=DOWNHILL */
void build_pkt_attitude(rt_uint8_t out[PACKET_SIZE],
                        rt_int16_t pitch_x100,
                        rt_uint8_t slope_state);

/* 0x05 GPS-coord */
void build_pkt_gps_coord(rt_uint8_t out[PACKET_SIZE],
                         rt_int32_t lat_e6,
                         rt_int32_t lon_e6);

/* 0x06 GPS-meta */
void build_pkt_gps_meta(rt_uint8_t out[PACKET_SIZE],
                        rt_uint8_t fix_valid,
                        rt_uint8_t utc_h,
                        rt_uint8_t utc_m,
                        rt_uint8_t utc_s,
                        rt_uint16_t utc_ms);

/* 0x07 Heartbeat */
void build_pkt_heartbeat(rt_uint8_t out[PACKET_SIZE],
                         rt_uint32_t uptime_s,
                         rt_uint8_t  watch_connected);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_TELEMETRY_PACK_H_ */
