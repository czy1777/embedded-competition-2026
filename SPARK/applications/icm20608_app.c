/*
 * ICM20608 应用层: 加速度 -> pitch -> 上下坡 / 平地 三态
 *
 * 数据流:
 *   icm20608_get_accel (LSB) -> /16384 转 g -> 一阶 IIR 滤波
 *   -> atan2 求 pitch (deg) -> 带迟滞状态机 -> SLOPE_*
 *
 * 仅在状态切换时打印一行日志, 减少串口噪音; 上层通过 getter 主动查询。
 */

#include <math.h>
#include <stdlib.h>
#include <icm20608.h>
#include "icm20608_app.h"

/* ---- 静态状态 ---- */
static icm20608_device_t  icm_dev = RT_NULL;
static rt_bool_t          icm_ready = RT_FALSE;
static float              ax_g = 0.0f, ay_g = 0.0f, az_g = 1.0f;
static float              pitch_deg = 0.0f;
static slope_state_t      cur_state = SLOPE_FLAT;

#define RAD2DEG_F  57.29578f

/* ---- API ---- */

int icm20608_app_init(void)
{
    rt_int16_t rx, ry, rz;

    icm_dev = icm20608_init(ICM_I2C_BUS_NAME);
    if (icm_dev == RT_NULL)
    {
        rt_kprintf("[ICM] init failed (bus=%s)\n", ICM_I2C_BUS_NAME);
        return -RT_ERROR;
    }

    /* 装机水平静止时一次性零位补偿; 若上电时不水平, 把 ICM_CALIB_TIMES 改为 0 */
    if (ICM_CALIB_TIMES > 0)
    {
        icm20608_calib_level(icm_dev, ICM_CALIB_TIMES);
    }

    /* 预热 IIR, 避免起始几帧从 0 缓慢爬升 */
    if (icm20608_get_accel(icm_dev, &rx, &ry, &rz) == RT_EOK)
    {
        ax_g = rx / ICM_ACCEL_LSB_PER_G;
        ay_g = ry / ICM_ACCEL_LSB_PER_G;
        az_g = rz / ICM_ACCEL_LSB_PER_G;
    }

    cur_state = SLOPE_FLAT;
    pitch_deg = 0.0f;
    icm_ready = RT_TRUE;

    rt_kprintf("[ICM] init OK\n");
    return RT_EOK;
}

void icm20608_app_task(void)
{
    rt_int16_t rx, ry, rz;
    float fx, fy, fz;
    float enter, exit_th;
    slope_state_t new_state;

    if (!icm_ready) return;

    if (icm20608_get_accel(icm_dev, &rx, &ry, &rz) != RT_EOK) return;

    fx = rx / ICM_ACCEL_LSB_PER_G;
    fy = ry / ICM_ACCEL_LSB_PER_G;
    fz = rz / ICM_ACCEL_LSB_PER_G;

    ax_g = ICM_LPF_ALPHA * fx + (1.0f - ICM_LPF_ALPHA) * ax_g;
    ay_g = ICM_LPF_ALPHA * fy + (1.0f - ICM_LPF_ALPHA) * ay_g;
    az_g = ICM_LPF_ALPHA * fz + (1.0f - ICM_LPF_ALPHA) * az_g;

#if ICM_FORWARD_AXIS_Y
    /* +Y 朝前: 车头抬起 -> ay 减小 -> -ay 增大 -> pitch>0 */
    pitch_deg = atan2f(-ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * RAD2DEG_F;
#else
    /* +X 朝前 */
    pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD2DEG_F;
#endif

    enter   = ICM_PITCH_THRESH_DEG;
    exit_th = ICM_PITCH_THRESH_DEG - ICM_PITCH_HYST_DEG;

    new_state = cur_state;
    switch (cur_state)
    {
    case SLOPE_FLAT:
        if      (pitch_deg >  enter) new_state = SLOPE_UPHILL;
        else if (pitch_deg < -enter) new_state = SLOPE_DOWNHILL;
        break;
    case SLOPE_UPHILL:
        if (pitch_deg < exit_th) new_state = SLOPE_FLAT;
        break;
    case SLOPE_DOWNHILL:
        if (pitch_deg > -exit_th) new_state = SLOPE_FLAT;
        break;
    }

    if (new_state != cur_state)
    {
        static const char *name[] = { "FLAT", "UPHILL", "DOWNHILL" };
        int p_int = (int)pitch_deg;
        int p_dec = abs((int)((pitch_deg - p_int) * 100));
        rt_kprintf("[ICM] state: %s pitch=%d.%02d deg\n",
                   name[new_state], p_int, p_dec);
        cur_state = new_state;
    }
}

slope_state_t icm20608_get_slope_state(void)
{
    return cur_state;
}

float icm20608_get_pitch_deg(void)
{
    return pitch_deg;
}
