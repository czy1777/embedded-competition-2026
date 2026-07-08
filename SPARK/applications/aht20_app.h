#ifndef APPLICATIONS_AHT20_APP_H_
#define APPLICATIONS_AHT20_H_

#include <bsp_system.h>

#ifdef __cplusplus
extern "C" {
#endif

int  aht20_app_init(void);

/* 由 sensor_threads.c 的 aht20 线程在采样后调用，刷新模块内缓存。 */
void aht20_update_th(float temperature, float humidity);

/* 中转线程读取温湿度快照；尚未填充时 t/h 写 0。 */
void aht20_get_th(float *t, float *h);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_AHT20_APP_H_ */
