/**
 * @file max30102_algorithm.h
 * @brief MAX30102 心率/血氧算法（Maxim AN6409 / SparkFun_MAX3010x 风格）
 *
 * 纯计算模块，不依赖 I2C / FreeRTOS，便于单元测试。
 * 假设采样率为 100Hz。
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*=============================================================================
 * 算法参数（可调）
 *============================================================================*/
#define MAX30102_ALGO_SAMPLE_RATE_HZ    25    /* 匹配 SMP_AVE=4 时的实际 FIFO 输出速率 */
#define MAX30102_ALGO_MA4_SIZE          4     /* DC 去除 4 抽头移动平均 */
#define MAX30102_ALGO_PEAK_MIN_DISTANCE 4     /* 峰值最小间距（样本） */
#define MAX30102_ALGO_PEAK_MAX_COUNT    15    /* 最多识别峰值数 */

#define MAX30102_ALGO_HR_MIN            30    /* BPM */
#define MAX30102_ALGO_HR_MAX            220   /* BPM */
#define MAX30102_ALGO_SPO2_MIN          70    /* % */
#define MAX30102_ALGO_SPO2_MAX          100   /* % */

/*=============================================================================
 * 计算结果
 *============================================================================*/
typedef struct {
    int32_t heart_rate;     /* BPM，无效时为 -1 */
    int32_t spo2;           /* %，无效时为 -1 */
    bool    hr_valid;
    bool    spo2_valid;
    uint8_t peaks_count;    /* 调试用：本次识别到的峰值数 */
} max30102_algo_result_t;

/*=============================================================================
 * API
 *============================================================================*/

/**
 * @brief 基于 Maxim AN6409 的 HR + SpO2 联合计算
 *
 * 算法流程：
 *   1. DC 去除（4 抽头 MA 高通近似）
 *   2. 信号翻转（IR 波谷 → 极大值）
 *   3. 自适应阈值峰值检测（局部极大 + 最小间距）
 *   4. HR = SAMPLE_RATE * 60 / avg_peak_interval
 *   5. 逐心跳周期计算 R 值，SpO2 取中位数抗伪影
 *
 * @param ir_buffer  IR 通道样本缓冲（至少 100 样本，推荐 500）
 * @param red_buffer RED 通道样本缓冲（与 ir_buffer 等长）
 * @param n_samples  样本数
 * @param out        [out] 计算结果
 */
void max30102_algo_calc(const uint32_t *ir_buffer,
                        const uint32_t *red_buffer,
                        int32_t n_samples,
                        max30102_algo_result_t *out);

#ifdef __cplusplus
}
#endif
