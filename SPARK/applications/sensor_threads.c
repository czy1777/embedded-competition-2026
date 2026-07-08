/**
 * @file    sensor_threads.c
 * @brief   传感器多线程管理
 * @note    为 AHT20、GPS、PM2.5 创建独立线程
 */

#include <bsp_system.h>
#include "jdy18_skystar.h"
#include "telemetry_pack.h"

/*============================================================================
 *                             私有变量
 *============================================================================*/

/* 线程控制块指针 */
static rt_thread_t pm25_thread = RT_NULL;
static rt_thread_t gps_thread = RT_NULL;
static rt_thread_t aht20_thread = RT_NULL;
static rt_thread_t ble_thread = RT_NULL;
static rt_thread_t imu_thread = RT_NULL;
static rt_thread_t relay_thread = RT_NULL;

/* AHT20 设备句柄 */
static aht10_device_t aht20_dev = RT_NULL;

/*============================================================================
 *                             线程入口函数
 *============================================================================*/

/**
 * @brief PM2.5 线程入口
 * @note  优先级最高，保证时序精度
 */
static void pm25_thread_entry(void *parameter)
{
    /* 初始化 PM2.5 传感器 */
    if (gp2y1014_init() != RT_EOK)
    {
        rt_kprintf("[PM25] Init failed!\n");
        return;
    }
    rt_kprintf("[PM25] Thread started, priority=%d\n", PM25_THREAD_PRIORITY);

    while (1)
    {
        /* 执行采样任务 */
        gp2y1014_task();

        /* 周期延时 */
        rt_thread_mdelay(PM25_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief GPS 线程入口
 * @note  解析由 uart_app 中断回调存入的 GPS 数据
 */
static void gps_thread_entry(void *parameter)
{
    /* 初始化 GPS 模块（设置 UART3 回调） */
    atgm336h_init();
    rt_kprintf("[GPS] Thread started, priority=%d\n", GPS_THREAD_PRIORITY);

    while (1)
    {
        /* 解析并处理 GPS 数据 */
        atgm336h_task();

        /* 周期延时 */
        rt_thread_mdelay(GPS_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief AHT20 线程入口
 * @note  优先级较低，因为有较长的阻塞延时
 */
static void aht20_thread_entry(void *parameter)
{
    float temperature, humidity;

    /* 初始化 AHT20 传感器 */
    aht20_dev = aht10_init("i2c3");
    if (aht20_dev == RT_NULL)
    {
        rt_kprintf("[AHT20] Init failed!\n");
        return;
    }
    rt_kprintf("[AHT20] Thread started, priority=%d\n", AHT20_THREAD_PRIORITY);

    while (1)
    {
        /* 读取温湿度数据 */
        humidity = aht10_read_humidity(aht20_dev);
        temperature = aht10_read_temperature(aht20_dev);

        /* 推给中转线程使用的缓存 */
        aht20_update_th(temperature, humidity);

        /* 周期延时 */
        rt_thread_mdelay(AHT20_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief BLE (JDY-18) 线程入口
 * @note  接收 ESP32-S3 通过 JDY-18 透传的健康数据
 */
static void ble_thread_entry(void *parameter)
{
    /* 初始化 JDY-18 模块 (注册 UART4 回调) */
    jdy18_init();
    rt_kprintf("[BLE] Thread started, priority=%d\n", BLE_THREAD_PRIORITY);

    while (1)
    {
        /* 执行 JDY-18 周期任务 (检查连接/打印数据) */
        jdy18_task();

        /* 周期延时 */
        rt_thread_mdelay(BLE_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief IMU (ICM20608) 线程入口
 * @note  采集加速度, 计算 pitch, 输出上下坡 / 平地三态
 */
static void imu_thread_entry(void *parameter)
{
    if (icm20608_app_init() != RT_EOK)
    {
        rt_kprintf("[IMU] Init failed!\n");
        return;
    }
    rt_kprintf("[IMU] Thread started, priority=%d\n", IMU_THREAD_PRIORITY);

    while (1)
    {
        icm20608_app_task();
        rt_thread_mdelay(IMU_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief 中转线程入口 (SPARK -> SkyStar)
 * @note  5 槽轮转：health passthrough / env / attitude / gps_coord / gps_meta+heartbeat
 *        12B/9600 ≈ 12.5ms 单包；250ms 周期一槽，对 UART2 极轻负载
 */
static void relay_thread_entry(void *parameter)
{
    rt_uint8_t pkt[PACKET_SIZE];
    int slot = 0;
    ble_health_packet_t hp;
    rt_tick_t boot_tick;

    jdy18_sky_init();
    boot_tick = rt_tick_get();
    rt_kprintf("[Relay] Thread started, priority=%d\n", RELAY_THREAD_PRIORITY);

    while (1)
    {
        switch (slot)
        {
        case 0:  /* 透传 Smart_Watch 的健康/告警包 (type 0x01 或 0x02) */
            if (jdy18_peek_latest(&hp))
            {
                build_pkt_passthrough(pkt, &hp);
                jdy18_sky_send(pkt);
            }
            break;

        case 1:  /* 环境: PM2.5 + AHT20 温湿度 */
        {
            float t = 0.0f, h = 0.0f;
            aht20_get_th(&t, &h);
            float pm25 = gp2y1014_get_pm25();

            rt_uint16_t pm25_x10 = (pm25 <= 0.0f) ? 0
                                  : (pm25 >= 6553.5f) ? 65535
                                  : (rt_uint16_t)(pm25 * 10.0f + 0.5f);
            rt_int16_t  t_x100 = (rt_int16_t)(t * 100.0f);
            rt_uint16_t h_x100 = (h <= 0.0f) ? 0
                                : (h >= 655.35f) ? 65535
                                : (rt_uint16_t)(h * 100.0f + 0.5f);

            build_pkt_env(pkt, pm25_x10, t_x100, h_x100);
            jdy18_sky_send(pkt);
            break;
        }

        case 2:  /* 姿态: pitch + slope */
        {
            float pitch = icm20608_get_pitch_deg();
            rt_int16_t pitch_x100 = (rt_int16_t)(pitch * 100.0f);
            slope_state_t s = icm20608_get_slope_state();
            rt_uint8_t slope_u8 = (s == SLOPE_UPHILL) ? 1
                                : (s == SLOPE_DOWNHILL) ? 2 : 0;
            build_pkt_attitude(pkt, pitch_x100, slope_u8);
            jdy18_sky_send(pkt);
            break;
        }

        case 3:  /* GPS 坐标 (无效定位时跳过, meta 槽仍发) */
            if (Save_Data.isUsefull)
            {
                double lat = g_LatAndLongData.latitude;
                double lon = g_LatAndLongData.longitude;
                if (g_LatAndLongData.N_S == 'S') lat = -lat;
                if (g_LatAndLongData.E_W == 'W') lon = -lon;
                rt_int32_t lat_e6 = (rt_int32_t)(lat * 1000000.0);
                rt_int32_t lon_e6 = (rt_int32_t)(lon * 1000000.0);
                build_pkt_gps_coord(pkt, lat_e6, lon_e6);
                jdy18_sky_send(pkt);
            }
            break;

        case 4:  /* GPS 时间 + 心跳 (同槽发两个包) */
        {
            rt_uint8_t fix = Save_Data.isUsefull ? 1 : 0;
            rt_uint8_t hh = 0, mm = 0, ss = 0;
            rt_uint16_t ms = 0;
            const char *u = Save_Data.UTCTime;
            if (u[0] >= '0' && u[0] <= '9' && u[1] >= '0' && u[1] <= '9' &&
                u[2] >= '0' && u[2] <= '9' && u[3] >= '0' && u[3] <= '9' &&
                u[4] >= '0' && u[4] <= '9' && u[5] >= '0' && u[5] <= '9')
            {
                hh = (u[0] - '0') * 10 + (u[1] - '0');
                mm = (u[2] - '0') * 10 + (u[3] - '0');
                ss = (u[4] - '0') * 10 + (u[5] - '0');
                if (u[6] == '.' && u[7] >= '0' && u[7] <= '9'
                    && u[8] >= '0' && u[8] <= '9' && u[9] >= '0' && u[9] <= '9')
                {
                    ms = (u[7] - '0') * 100 + (u[8] - '0') * 10 + (u[9] - '0');
                }
            }
            build_pkt_gps_meta(pkt, fix, hh, mm, ss, ms);
            jdy18_sky_send(pkt);

            rt_uint32_t uptime_s = (rt_tick_get() - boot_tick) / RT_TICK_PER_SECOND;
            build_pkt_heartbeat(pkt, uptime_s, jdy18_is_connected() ? 1 : 0);
            jdy18_sky_send(pkt);
            break;
        }
        }

        slot = (slot + 1) % 5;
        rt_thread_mdelay(RELAY_SAMPLE_PERIOD_MS);
    }
}

/*============================================================================
 *                             公开函数
 *============================================================================*/

/**
 * @brief  传感器线程初始化
 * @return RT_EOK 成功
 */
int sensor_threads_init(void)
{
    rt_kprintf("=== Sensor Threads Initializing ===\n");

    /* 1. 创建 PM2.5 线程（最高优先级） */
    pm25_thread = rt_thread_create(
        PM25_THREAD_NAME,
        pm25_thread_entry,
        RT_NULL,
        PM25_THREAD_STACK_SIZE,
        PM25_THREAD_PRIORITY,
        PM25_THREAD_TIMESLICE
    );
    if (pm25_thread != RT_NULL)
    {
        rt_thread_startup(pm25_thread);
    }
    else
    {
        rt_kprintf("[PM25] Thread create failed!\n");
        return -RT_ERROR;
    }

    /* 2. 创建 GPS 线程 */
    gps_thread = rt_thread_create(
        GPS_THREAD_NAME,
        gps_thread_entry,
        RT_NULL,
        GPS_THREAD_STACK_SIZE,
        GPS_THREAD_PRIORITY,
        GPS_THREAD_TIMESLICE
    );
    if (gps_thread != RT_NULL)
    {
        rt_thread_startup(gps_thread);
    }
    else
    {
        rt_kprintf("[GPS] Thread create failed!\n");
        return -RT_ERROR;
    }

    /* 3. 创建 AHT20 线程（最低优先级） */
    aht20_thread = rt_thread_create(
        AHT20_THREAD_NAME,
        aht20_thread_entry,
        RT_NULL,
        AHT20_THREAD_STACK_SIZE,
        AHT20_THREAD_PRIORITY,
        AHT20_THREAD_TIMESLICE
    );
    if (aht20_thread != RT_NULL)
    {
        rt_thread_startup(aht20_thread);
    }
    else
    {
        rt_kprintf("[AHT20] Thread create failed!\n");
        return -RT_ERROR;
    }

    /* 4. 创建 BLE (JDY-18) 线程 */
    ble_thread = rt_thread_create(
        BLE_THREAD_NAME,
        ble_thread_entry,
        RT_NULL,
        BLE_THREAD_STACK_SIZE,
        BLE_THREAD_PRIORITY,
        BLE_THREAD_TIMESLICE
    );
    if (ble_thread != RT_NULL)
    {
        rt_thread_startup(ble_thread);
    }
    else
    {
        rt_kprintf("[BLE] Thread create failed!\n");
        return -RT_ERROR;
    }

    /* 5. 创建 IMU (ICM20608) 线程 */
    imu_thread = rt_thread_create(
        IMU_THREAD_NAME,
        imu_thread_entry,
        RT_NULL,
        IMU_THREAD_STACK_SIZE,
        IMU_THREAD_PRIORITY,
        IMU_THREAD_TIMESLICE
    );
    if (imu_thread != RT_NULL)
    {
        rt_thread_startup(imu_thread);
    }
    else
    {
        rt_kprintf("[IMU] Thread create failed!\n");
        return -RT_ERROR;
    }

    /* 6. 创建中转线程 (SPARK -> SkyStar) */
    relay_thread = rt_thread_create(
        RELAY_THREAD_NAME,
        relay_thread_entry,
        RT_NULL,
        RELAY_THREAD_STACK_SIZE,
        RELAY_THREAD_PRIORITY,
        RELAY_THREAD_TIMESLICE
    );
    if (relay_thread != RT_NULL)
    {
        rt_thread_startup(relay_thread);
    }
    else
    {
        rt_kprintf("[Relay] Thread create failed!\n");
        return -RT_ERROR;
    }

    rt_kprintf("=== All Sensor Threads Started ===\n");
    return RT_EOK;
}

/* 导出到自动初始化（应用层初始化阶段） */
INIT_APP_EXPORT(sensor_threads_init);
