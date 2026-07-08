#ifndef APPLICATIONS_SENSOR_THREADS_H_
#define APPLICATIONS_SENSOR_THREADS_H_

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *                             线程配置宏
 *============================================================================*/

/* PM2.5 线程配置 - 最高优先级（时序敏感） */
#define PM25_THREAD_NAME        "pm25"
#define PM25_THREAD_STACK_SIZE  1024
#define PM25_THREAD_PRIORITY    8
#define PM25_THREAD_TIMESLICE   5
#define PM25_SAMPLE_PERIOD_MS   200

/* GPS 线程配置 */
#define GPS_THREAD_NAME         "gps"
#define GPS_THREAD_STACK_SIZE   2048
#define GPS_THREAD_PRIORITY     15
#define GPS_THREAD_TIMESLICE    10
#define GPS_SAMPLE_PERIOD_MS    100

/* AHT20 线程配置 - 较低优先级（阻塞时间长） */
#define AHT20_THREAD_NAME       "aht20"
#define AHT20_THREAD_STACK_SIZE 1024
#define AHT20_THREAD_PRIORITY   18
#define AHT20_THREAD_TIMESLICE  10
#define AHT20_SAMPLE_PERIOD_MS  2000

/* BLE (JDY-18) 线程配置 */
#define BLE_THREAD_NAME         "ble"
#define BLE_THREAD_STACK_SIZE   2048
#define BLE_THREAD_PRIORITY     12
#define BLE_THREAD_TIMESLICE    10
#define BLE_SAMPLE_PERIOD_MS    500

/* IMU (ICM20608) 线程配置 - 介于 BLE 与 GPS 之间, 高于 AHT20 防 I2C3 阻塞 */
#define IMU_THREAD_NAME         "imu"
#define IMU_THREAD_STACK_SIZE   1024
#define IMU_THREAD_PRIORITY     14
#define IMU_THREAD_TIMESLICE    5
#define IMU_SAMPLE_PERIOD_MS    50

/* Relay (SPARK->SkyStar 中转) 线程配置 - 13 介于 BLE=12 与 IMU=14 之间.
 * 每 250ms 一次, 5 槽轮转一种 TYPE; 12B/9600 ≈ 12.5ms, 不会塞爆 UART2. */
#define RELAY_THREAD_NAME       "relay"
#define RELAY_THREAD_STACK_SIZE 2048
#define RELAY_THREAD_PRIORITY   13
#define RELAY_THREAD_TIMESLICE  10
#define RELAY_SAMPLE_PERIOD_MS  250

/*============================================================================
 *                             函数声明
 *============================================================================*/

/**
 * @brief  传感器线程初始化
 * @return RT_EOK 成功
 */
int sensor_threads_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_SENSOR_THREADS_H_ */
