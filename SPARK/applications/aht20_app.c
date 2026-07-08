#include <aht20_app.h>

#define AHT_I2C_BUS_NAME "i2c3"

/* 文件作用域缓存：sensor_threads 的 aht20 线程负责写，中转线程负责读。
 * 单生产单消费 + float 在 STM32F4 上 4B 对齐写入是原子的，足够这里使用。 */
static float g_temperature = 0.0f;
static float g_humidity    = 0.0f;

int aht20_app_init(void)
{
    /* 实际的传感器轮询由 sensor_threads.c 的 aht20_thread_entry 接管。
     * 此处只保留旧入口的签名，避免老代码 include 失败。 */
    return 0;
}

void aht20_update_th(float temperature, float humidity)
{
    g_temperature = temperature;
    g_humidity    = humidity;
}

void aht20_get_th(float *t, float *h)
{
    if (t) *t = g_temperature;
    if (h) *h = g_humidity;
}
