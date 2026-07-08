#ifndef APPLICATIONS_ICM20608_APP_H_
#define APPLICATIONS_ICM20608_APP_H_

#include <bsp_system.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ICM20608 应用层: 上坡 / 下坡 / 平地 姿态识别
 *
 * 安装姿态约定:
 *   +Y 朝向前进方向 (车头)
 *   +Z 朝上
 *   pitch > 0 表示车头抬起 = 上坡
 *
 * 若实际安装方向不同, 改 ICM_FORWARD_AXIS_Y 即可切换前进轴。
 */

/* ---- 调参宏 ---- */
#define ICM_I2C_BUS_NAME       "i2c2"
#define ICM_SAMPLE_PERIOD_MS   50
#define ICM_FORWARD_AXIS_Y     1
#define ICM_PITCH_THRESH_DEG   5.0f
#define ICM_PITCH_HYST_DEG     1.0f
#define ICM_LPF_ALPHA          0.2f
#define ICM_ACCEL_LSB_PER_G    16384.0f
#define ICM_CALIB_TIMES        50

typedef enum {
    SLOPE_FLAT = 0,
    SLOPE_UPHILL,
    SLOPE_DOWNHILL
} slope_state_t;

int           icm20608_app_init(void);
void          icm20608_app_task(void);
slope_state_t icm20608_get_slope_state(void);
float         icm20608_get_pitch_deg(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_ICM20608_APP_H_ */
