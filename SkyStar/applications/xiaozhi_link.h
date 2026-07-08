/*
 * xiaozhi_link.h
 * UART5 (PC12 TX / PD2 RX) @115200, 把 JDY-18 收到的健康/环境/GPS/跌倒
 * 单向上行透传给 ESP32 xiaozhi 端。
 *
 * 协议沿用 JYD-18.h 那套 12B 定长帧 (type 0x01/0x02/0x03/0x05/0x06/0x07),
 * 没有反向链路。本头文件保留只是为了 INIT_APP_EXPORT 自动注册的对外签名;
 * 业务代码无需引用任何东西。
 */
#ifndef APPLICATIONS_XIAOZHI_LINK_H_
#define APPLICATIONS_XIAOZHI_LINK_H_

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int xiaozhi_link_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_XIAOZHI_LINK_H_ */
