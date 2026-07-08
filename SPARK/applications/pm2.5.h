/**
 * @file    pm2.5.h
 * @brief   GP2Y1014AU0F 灰尘传感器驱动头文件
 */

#ifndef APPLICATIONS_PM2_5_H_
#define APPLICATIONS_PM2_5_H_

#include <bsp_system.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化GP2Y1014传感器
 * @return RT_EOK 成功, RT_ERROR 失败
 */
int gp2y1014_init(void);

/**
 * @brief  GP2Y1014任务函数，读取传感器并通过串口2输出
 */
void gp2y1014_task(void);

/**
 * @brief  获取当前PM2.5浓度值
 * @return PM2.5浓度 (ug/m3)
 */
float gp2y1014_get_pm25(void);

/**
 * @brief  获取当前ADC原始值
 * @return ADC原始值 (0-4095)
 */
rt_uint16_t gp2y1014_get_raw(void);

/**
 * @brief  获取当前电压值
 * @return 电压值 (mV)
 */
rt_uint16_t gp2y1014_get_voltage(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_PM2_5_H_ */
