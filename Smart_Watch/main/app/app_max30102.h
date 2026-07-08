/**
 * @file app_max30102.h
 * @brief MAX30102 应用层接口 - 按需手指测量 + 低功耗
 *
 * 工作流程：
 *   上电 → POWER_DOWN（芯片 shutdown，LED 灭）
 *        ↓ 每 PROBE_INTERVAL_MS 短暂 probe 一次
 *   用户把手指放上去
 *        ↓ 连续 PROBE_HIT_REQUIRED 次 probe 检测到 IR ≥ FINGER_ON_TH
 *   MEASURING（亮灯测 MEASURING_DURATION_MS，期间掉指会中止）
 *        ↓
 *   COOLDOWN（芯片 shutdown，结果展示 COOLDOWN_MS）
 *        ↓
 *   回到 POWER_DOWN
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*=============================================================================
 * 硬件配置
 *============================================================================*/
#define APP_MAX30102_I2C_PORT       I2C_NUM_1
#define APP_MAX30102_I2C_SDA_PIN    GPIO_NUM_5
#define APP_MAX30102_I2C_SCL_PIN    GPIO_NUM_15
#define APP_MAX30102_I2C_FREQ_HZ    50000       /* 50kHz：偏慢但对长杜邦线/面包板更稳 */
#define APP_MAX30102_INT_PIN        GPIO_NUM_3  /* FIFO_A_FULL 下降沿触发 */

/*=============================================================================
 * 状态机参数
 *============================================================================*/
#define PROBE_INTERVAL_MS           2000    /* POWER_DOWN 周期性 probe 间隔 */
#define PROBE_WAKE_MS               300     /* 每次 probe 唤醒持续时长 */
#define PROBE_LED_CURRENT           0x30    /* ~10mA，弱光下手指反射仍可识别 */
#define PROBE_HIT_REQUIRED          2       /* 连续命中几次才进 MEASURING */

#define MEASURING_DURATION_MS       15000   /* 测量态持续时长 */
#define MEASURING_LED_CURRENT       0x50    /* ~16mA，正常测量电流 */

#define COOLDOWN_MS                 30000   /* 结果展示时长，之后回 POWER_DOWN */

/*=============================================================================
 * 信号判据
 *============================================================================*/
#define FINGER_ON_TH                30000u  /* IR 均值 ≥ 此值视为手指在位 */
#define FINGER_OFF_TH               20000u  /* IR 均值 < 此值累计判为掉指 */
#define FINGER_OFF_CONSEC_N         8       /* 连续多少帧判掉指 */

/*=============================================================================
 * 采样缓冲 & 算法触发
 *============================================================================*/
#define MAX30102_SAMPLE_RATE_HZ     25      /* 实际 FIFO 输出速率 = 100Hz ADC / SMP_AVE(4) */
#define MAX30102_BUFFER_SIZE        125     /* 5s @ 25Hz */
#define MAX30102_MIN_SAMPLES_FOR_CALC 75    /* 启动算法最小样本（3s @ 25Hz） */
#define MAX30102_CALC_INTERVAL_MS   500     /* MEASURING 中每 500ms 算一次 */

/*=============================================================================
 * 芯片寄存器配置
 *============================================================================*/
#define MAX30102_FIFO_CONFIG_VAL    0x4F    /* SMP_AVE=4（4样本硬件平均, 有效25SPS）, FIFO_ROLLOVER=0, A_FULL=15 */
#define MAX30102_MODE_SPO2_VAL      0x03
#define MAX30102_MODE_SHUTDOWN_VAL  0x80

/*=============================================================================
 * 数据结构
 *============================================================================*/
typedef enum {
    MAX30102_STATE_POWER_DOWN = 0,  /* 芯片 shutdown，LED 灭 */
    MAX30102_STATE_PROBING,         /* 低电流短脉冲检测手指 */
    MAX30102_STATE_MEASURING,       /* 正常测量中 */
    MAX30102_STATE_COOLDOWN,        /* 测量完成，结果保留展示 */
} max30102_state_t;

typedef struct {
    uint32_t red_raw;               /* 最新原始值（仅 MEASURING 过程更新） */
    uint32_t ir_raw;
    float    spo2;                  /* 测量结果（COOLDOWN 时有效） */
    uint8_t  heart_rate;
    bool     finger_on;             /* 当前是否检测到手指 */
    bool     data_valid;            /* spo2/heart_rate 是否有效 */
    max30102_state_t state;
} app_max30102_data_t;

/*=============================================================================
 * API
 *============================================================================*/

/** @brief 初始化（立即进入 POWER_DOWN 态，启动 probe 定时器） */
esp_err_t app_max30102_init(void);

/** @brief 反初始化 */
void app_max30102_deinit(void);

/** @brief 读取最新数据快照（非阻塞） */
esp_err_t app_max30102_read(app_max30102_data_t *data);

/** @brief 检查是否有新数据（COOLDOWN 态置位，读后清除） */
bool app_max30102_data_ready(void);

/** @brief 读取芯片温度（会短暂唤醒芯片） */
esp_err_t app_max30102_read_temp(float *temp);

/** @brief 手动触发一次测量（绕过 PROBING，直接进 MEASURING） */
esp_err_t app_max30102_start_once(void);

/** @brief 中止当前测量并回 POWER_DOWN */
esp_err_t app_max30102_stop(void);

/** @brief 当前状态 */
max30102_state_t app_max30102_get_state(void);

#ifdef __cplusplus
}
#endif
