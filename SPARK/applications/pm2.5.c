/**
 * @file    pm2.5.c
 * @brief   GP2Y1014AU0F 灰尘传感器驱动
 * @note    移植自STM32F103测试例程，适配RT-Thread
 */

#include "pm2.5.h"

/*============================================================================
 *                             宏定义
 *============================================================================*/
#define GP2Y_ADC_DEV_NAME   "adc1"
#define GP2Y_ADC_CHANNEL    6               /* PA6, ADC1通道6 */
#define GP2Y_LED_PIN        GET_PIN(G, 6)   /* PG6 控制传感器内部LED */

/* 校准参数 */
#define NO_DUST_VOLTAGE     400             /* 无尘电压阈值(mV) */
#define COV_RATIO           0.20f           /* 浓度转换系数 */
#define REFER_VOLTAGE       3300            /* 参考电压(mV) */
#define CONVERT_BITS        4096            /* 12位ADC分辨率 */
#define VOLTAGE_DIVIDER     11              /* 分压电阻系数 */

/*============================================================================
 *                             全局变量
 *============================================================================*/
static rt_adc_device_t gp2y_adc_dev = RT_NULL;
static rt_uint16_t gp2y_adc_raw = 0;        /* 原始ADC值 (0-4095) */
static rt_uint16_t gp2y_voltage_mv = 0;     /* 测量的电压 (mV) */
static float gp2y_dust_density = 0;         /* PM2.5浓度 (ug/m3) */

/*============================================================================
 *                             滤波函数
 *============================================================================*/
/**
 * @brief  10点移动平均滤波
 * @param  new_val 新的ADC采样值
 * @return 滤波后的值
 */
static rt_uint16_t filter_adc(rt_uint16_t new_val)
{
    static rt_uint16_t buf[10];
    static rt_uint8_t i = 0;
    static rt_uint32_t sum = 0;
    static rt_uint8_t is_first = 1;

    /* 第一次调用时，用当前值填充整个缓冲区 */
    if (is_first)
    {
        rt_uint8_t k;
        for (k = 0; k < 10; k++)
        {
            buf[k] = new_val;
        }
        sum = (rt_uint32_t)new_val * 10;
        is_first = 0;
        return new_val;
    }

    /* 滑动平均：减去旧值，加入新值 */
    sum -= buf[i];
    buf[i] = new_val;
    sum += buf[i];

    i++;
    if (i >= 10) i = 0;

    return (rt_uint16_t)(sum / 10);
}

/*============================================================================
 *                             核心读取函数
 *============================================================================*/
/**
 * @brief  执行一次完整的GP2Y1014读取时序
 * @note   时序要求：点亮LED->等待280us->ADC采集->等待40us->熄灭LED->等待9.68ms
 */
static void gp2y1014_read(void)
{
    rt_uint32_t adc_value;
    rt_uint16_t filter_out;
    rt_base_t level;

    /* 关闭中断以保证微秒级时序精度 */
    level = rt_hw_interrupt_disable();

    /* 1. 点亮传感器内部LED (高电平有效) */
    rt_pin_write(GP2Y_LED_PIN, PIN_HIGH);

    /* 2. 等待280us (采样点前延时) */
    rt_hw_us_delay(280);

    /* 3. ADC采集 */
    adc_value = rt_adc_read(gp2y_adc_dev, GP2Y_ADC_CHANNEL);

    /* 4. 等待40us */
    rt_hw_us_delay(40);

    /* 5. 熄灭LED (低电平) */
    rt_pin_write(GP2Y_LED_PIN, PIN_LOW);

    /* 恢复中断 */
    rt_hw_interrupt_enable(level);

    /* 6. 等待脉冲周期完成 (9.68ms) */
    rt_hw_us_delay(9600);

    /* 保存原始值 */
    gp2y_adc_raw = (rt_uint16_t)adc_value;

    /* 7. 滤波处理 */
    filter_out = filter_adc(gp2y_adc_raw);

    /* 8. 电压计算 (含分压电阻系数) */
    gp2y_voltage_mv = (rt_uint16_t)(((rt_uint32_t)filter_out * REFER_VOLTAGE / CONVERT_BITS) * VOLTAGE_DIVIDER);

    /* 9. PM2.5浓度计算 */
    if (gp2y_voltage_mv > NO_DUST_VOLTAGE)
    {
        gp2y_dust_density = (float)(gp2y_voltage_mv - NO_DUST_VOLTAGE) * COV_RATIO;
    }
    else
    {
        gp2y_dust_density = 0;
    }
}

/*============================================================================
 *                             公共接口函数
 *============================================================================*/
/**
 * @brief  初始化GP2Y1014传感器
 * @return RT_EOK 成功, RT_ERROR 失败
 */
int gp2y1014_init(void)
{
    rt_err_t ret;

    /* 1. 查找ADC设备 */
    gp2y_adc_dev = (rt_adc_device_t)rt_device_find(GP2Y_ADC_DEV_NAME);
    if (gp2y_adc_dev == RT_NULL)
    {
        rt_kprintf("[GP2Y1014] Error: ADC device %s not found!\n", GP2Y_ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 2. 使能ADC通道 */
    ret = rt_adc_enable(gp2y_adc_dev, GP2Y_ADC_CHANNEL);
    if (ret != RT_EOK)
    {
        rt_kprintf("[GP2Y1014] Error: ADC channel %d enable failed!\n", GP2Y_ADC_CHANNEL);
        return RT_ERROR;
    }

    /* 3. 配置LED控制引脚为输出模式 */
    rt_pin_mode(GP2Y_LED_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(GP2Y_LED_PIN, PIN_LOW);  /* 默认关闭LED */

    return RT_EOK;
}

/**
 * @brief  获取当前PM2.5浓度值
 * @return PM2.5浓度 (ug/m3)
 */
float gp2y1014_get_pm25(void)
{
    return gp2y_dust_density;
}

/**
 * @brief  获取当前ADC原始值
 * @return ADC原始值 (0-4095)
 */
rt_uint16_t gp2y1014_get_raw(void)
{
    return gp2y_adc_raw;
}

/**
 * @brief  获取当前电压值
 * @return 电压值 (mV)
 */
rt_uint16_t gp2y1014_get_voltage(void)
{
    return gp2y_voltage_mv;
}

/**
 * @brief  GP2Y1014任务函数，读取传感器并刷新内部状态
 *         数值通过 gp2y1014_get_pm25 / get_raw / get_voltage 获取
 */
void gp2y1014_task(void)
{
    gp2y1014_read();
}
