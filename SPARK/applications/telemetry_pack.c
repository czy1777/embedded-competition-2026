#include "telemetry_pack.h"
#include <string.h>

/* ---- 小端写入工具 ---- */
static inline void wr_u16(rt_uint8_t *p, rt_uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void wr_i16(rt_uint8_t *p, rt_int16_t  v) { wr_u16(p, (rt_uint16_t)v); }
static inline void wr_u32(rt_uint8_t *p, rt_uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline void wr_i32(rt_uint8_t *p, rt_int32_t  v) { wr_u32(p, (rt_uint32_t)v); }

void telemetry_seal(rt_uint8_t out[PACKET_SIZE], rt_uint8_t type)
{
    out[0] = PACKET_HEADER;
    out[1] = type;

    rt_uint8_t xor_sum = 0;
    for (int i = 0; i < 10; i++) xor_sum ^= out[i];
    out[10] = xor_sum;
    out[11] = PACKET_TAIL;
}

void build_pkt_passthrough(rt_uint8_t out[PACKET_SIZE], const ble_health_packet_t *pkt)
{
    memcpy(out, pkt, PACKET_SIZE);
}

void build_pkt_env(rt_uint8_t out[PACKET_SIZE],
                   rt_uint16_t pm25_x10,
                   rt_int16_t  temp_x100,
                   rt_uint16_t humi_x100)
{
    memset(out + 2, 0, 8);          /* 清 payload */
    wr_u16(out + 2, pm25_x10);
    wr_i16(out + 4, temp_x100);
    wr_u16(out + 6, humi_x100);
    /* out[8..9] reserved = 0 */
    telemetry_seal(out, PACKET_TYPE_ENV);
}

void build_pkt_attitude(rt_uint8_t out[PACKET_SIZE],
                        rt_int16_t pitch_x100,
                        rt_uint8_t slope_state)
{
    memset(out + 2, 0, 8);
    wr_i16(out + 2, pitch_x100);
    out[4] = slope_state;
    telemetry_seal(out, PACKET_TYPE_ATTITUDE);
}

void build_pkt_gps_coord(rt_uint8_t out[PACKET_SIZE],
                         rt_int32_t lat_e6,
                         rt_int32_t lon_e6)
{
    memset(out + 2, 0, 8);
    wr_i32(out + 2, lat_e6);
    wr_i32(out + 6, lon_e6);
    telemetry_seal(out, PACKET_TYPE_GPS_COORD);
}

void build_pkt_gps_meta(rt_uint8_t out[PACKET_SIZE],
                        rt_uint8_t fix_valid,
                        rt_uint8_t utc_h,
                        rt_uint8_t utc_m,
                        rt_uint8_t utc_s,
                        rt_uint16_t utc_ms)
{
    memset(out + 2, 0, 8);
    out[2] = fix_valid;
    out[3] = utc_h;
    out[4] = utc_m;
    out[5] = utc_s;
    wr_u16(out + 6, utc_ms);
    telemetry_seal(out, PACKET_TYPE_GPS_META);
}

void build_pkt_heartbeat(rt_uint8_t out[PACKET_SIZE],
                         rt_uint32_t uptime_s,
                         rt_uint8_t  watch_connected)
{
    memset(out + 2, 0, 8);
    wr_u32(out + 2, uptime_s);
    out[6] = watch_connected;
    telemetry_seal(out, PACKET_TYPE_HEARTBEAT);
}
