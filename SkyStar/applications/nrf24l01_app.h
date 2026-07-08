/*
 * nrf24l01_app.h
 * 应用层: 协议解析 (AA AF 03 ...) + 全局通道值 Remote
 */
#ifndef APPLICATIONS_NRF24L01_APP_H_
#define APPLICATIONS_NRF24L01_APP_H_

#include <stdint.h>

/* 10 路通道值, 范围 1000~2000, 中点 1500 (与遥控器一致) */
typedef struct {
    int16_t thr;
    int16_t yaw;
    int16_t roll;
    int16_t pitch;
    int16_t AUX1;
    int16_t AUX2;
    int16_t AUX3;
    int16_t AUX4;
    int16_t AUX5;
    int16_t AUX6;
} _st_Remote;

extern _st_Remote Remote;

extern volatile uint8_t  nrf_link_ok;       /* 1=2.4G 已连通, 0=断连 */
extern volatile uint32_t nrf_pkt_cnt;       /* 累计有效包数 */

/* 协议解析 (帧头 AA AF + 命令字 0x03 + 10*2B 通道 + sum) */
void ANO_DT_Data_Receive_Anl(uint8_t *buf, uint8_t num);

/* 自动初始化 (RT-Thread INIT_APP_EXPORT 注册) */
int nrf_app_init(void);

#endif /* APPLICATIONS_NRF24L01_APP_H_ */
