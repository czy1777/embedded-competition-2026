/**
 * @file max30102_algorithm.c
 * @brief MAX30102 心率/血氧算法实现（Maxim AN6409 / SparkFun_MAX3010x 风格）
 */

#include "max30102_algorithm.h"

#include <math.h>
#include <string.h>

/* 内部最大支持的样本数（和 app_max30102 的 BUFFER_SIZE 一致或更大） */
#define ALGO_MAX_SAMPLES    125

/*=============================================================================
 * 内部辅助
 *============================================================================*/

static int16_t clamp_i16(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

/* 冒泡排序（样本数 ≤ 15，O(n²) 完全够用） */
static void bubble_sort_int32(int32_t *arr, int32_t n)
{
    for (int32_t i = 0; i < n - 1; i++) {
        for (int32_t j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                int32_t tmp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
}

/**
 * @brief 查找 signal 中高于 threshold 的局部极大值
 * @param signal      输入信号（已去 DC + 翻转）
 * @param n           样本数
 * @param threshold   阈值
 * @param min_dist    峰值最小间距
 * @param peak_locs   [out] 峰值位置数组
 * @param max_peaks   peak_locs 容量
 * @return 实际识别到的峰值数
 */
static int32_t find_peaks(const int16_t *signal, int32_t n,
                          int16_t threshold, int32_t min_dist,
                          int32_t *peak_locs, int32_t max_peaks)
{
    int32_t peak_count = 0;
    int32_t i = 1;

    while (i < n - 1 && peak_count < max_peaks) {
        /* 局部极大值判定：signal[i] > 两侧，且 ≥ threshold */
        if (signal[i] > threshold
            && signal[i] > signal[i - 1]
            && signal[i] >= signal[i + 1]) {

            /* 处理平顶：沿着平顶向右找到下降沿起点 */
            int32_t width = 1;
            while (i + width < n && signal[i + width] == signal[i]) {
                width++;
            }
            if (i + width < n && signal[i + width] < signal[i]) {
                int32_t peak = i + (width - 1) / 2;

                /* 最小间距约束：若距前一峰太近，保留幅度更大者 */
                if (peak_count > 0
                    && (peak - peak_locs[peak_count - 1]) < min_dist) {
                    if (signal[peak] > signal[peak_locs[peak_count - 1]]) {
                        peak_locs[peak_count - 1] = peak;
                    }
                } else {
                    peak_locs[peak_count++] = peak;
                }
                i = peak + 1;
                continue;
            }
        }
        i++;
    }
    return peak_count;
}

/*=============================================================================
 * 主入口
 *============================================================================*/

void max30102_algo_calc(const uint32_t *ir_buffer,
                        const uint32_t *red_buffer,
                        int32_t n_samples,
                        max30102_algo_result_t *out)
{
    if (out == NULL) return;
    out->heart_rate  = -1;
    out->spo2        = -1;
    out->hr_valid    = false;
    out->spo2_valid  = false;
    out->peaks_count = 0;

    if (ir_buffer == NULL || red_buffer == NULL) return;
    if (n_samples < 10 || n_samples > ALGO_MAX_SAMPLES) return;

    /* --- 步骤 1：DC 去除（4 抽头移动平均高通近似），同时翻转 --- */
    static int16_t ir_flip[ALGO_MAX_SAMPLES];
    const int32_t ma = MAX30102_ALGO_MA4_SIZE;
    /* 首 ma-1 个样本无法计算 MA，置 0 */
    for (int32_t i = 0; i < ma - 1 && i < n_samples; i++) {
        ir_flip[i] = 0;
    }
    for (int32_t i = ma - 1; i < n_samples; i++) {
        uint32_t sum = 0;
        for (int32_t k = 0; k < ma; k++) {
            sum += ir_buffer[i - k];
        }
        int32_t mean = (int32_t)(sum / ma);
        int32_t denoised = (int32_t)ir_buffer[i] - mean;
        /* 翻转：PPG 在 IR 上是"谷"对应心跳，翻转后找极大值 */
        ir_flip[i] = clamp_i16(-denoised, INT16_MIN, INT16_MAX);
    }

    /* --- 步骤 2：阈值估计 --- */
    int32_t sum = 0;
    int16_t mn = INT16_MAX, mx = INT16_MIN;
    int32_t start = ma - 1;
    int32_t valid_n = n_samples - start;
    if (valid_n <= 0) return;
    for (int32_t i = start; i < n_samples; i++) {
        sum += ir_flip[i];
        if (ir_flip[i] < mn) mn = ir_flip[i];
        if (ir_flip[i] > mx) mx = ir_flip[i];
    }
    int16_t mean_val = (int16_t)(sum / valid_n);
    int32_t thr_raw = ((int32_t)mean_val + mn + mx) / 3;
    if (thr_raw < 30) thr_raw = 30;
    if (thr_raw > 60) thr_raw = 60;
    int16_t threshold = (int16_t)thr_raw;

    /* --- 步骤 3：峰值检测 --- */
    int32_t peak_locs[MAX30102_ALGO_PEAK_MAX_COUNT];
    int32_t peaks_count = find_peaks(ir_flip + start, valid_n,
                                     threshold,
                                     MAX30102_ALGO_PEAK_MIN_DISTANCE,
                                     peak_locs,
                                     MAX30102_ALGO_PEAK_MAX_COUNT);
    /* 峰值位置是在 ir_flip+start 子数组内的索引，换回绝对索引 */
    for (int32_t i = 0; i < peaks_count; i++) {
        peak_locs[i] += start;
    }
    out->peaks_count = (uint8_t)peaks_count;

    /* --- 步骤 4：心率 --- */
    if (peaks_count >= 2) {
        int32_t intervals_sum = 0;
        for (int32_t i = 1; i < peaks_count; i++) {
            intervals_sum += (peak_locs[i] - peak_locs[i - 1]);
        }
        int32_t avg_interval = intervals_sum / (peaks_count - 1);
        if (avg_interval > 0) {
            int32_t hr = MAX30102_ALGO_SAMPLE_RATE_HZ * 60 / avg_interval;
            if (hr >= MAX30102_ALGO_HR_MIN && hr <= MAX30102_ALGO_HR_MAX) {
                out->heart_rate = hr;
                out->hr_valid = true;
            }
        }
    }

    /* --- 步骤 5：逐周期 R 值 → SpO2 → 中位数 --- */
    if (peaks_count < 2) return;

    int32_t spo2_vals[MAX30102_ALGO_PEAK_MAX_COUNT];
    int32_t spo2_cnt = 0;

    for (int32_t i = 0; i < peaks_count - 1; i++) {
        int32_t a = peak_locs[i];
        int32_t b = peak_locs[i + 1];
        if (b <= a) continue;

        uint32_t ir_max = 0, red_max = 0;
        uint32_t ir_min = UINT32_MAX, red_min = UINT32_MAX;
        for (int32_t k = a; k <= b; k++) {
            if (ir_buffer[k]  > ir_max)  ir_max  = ir_buffer[k];
            if (ir_buffer[k]  < ir_min)  ir_min  = ir_buffer[k];
            if (red_buffer[k] > red_max) red_max = red_buffer[k];
            if (red_buffer[k] < red_min) red_min = red_buffer[k];
        }
        uint32_t ir_ac  = ir_max  - ir_min;
        uint32_t red_ac = red_max - red_min;
        uint32_t ir_dc  = ir_max  + ir_min;   /* 实为 2*DC，分子分母相抵 */
        uint32_t red_dc = red_max + red_min;

        if (ir_ac == 0 || red_dc == 0) continue;

        /* R = (red_ac / red_dc) / (ir_ac / ir_dc)
             = (red_ac * ir_dc) / (ir_ac * red_dc) */
        double R = ((double)red_ac * (double)ir_dc) /
                   ((double)ir_ac  * (double)red_dc);
        if (!isfinite(R) || R < 0.3 || R > 2.5) continue;

        double spo2_f = -45.060 * R * R + 30.354 * R + 94.845;
        if (spo2_f < MAX30102_ALGO_SPO2_MIN
            || spo2_f > MAX30102_ALGO_SPO2_MAX) {
            continue;
        }
        spo2_vals[spo2_cnt++] = (int32_t)(spo2_f + 0.5);
    }

    if (spo2_cnt >= 2) {
        bubble_sort_int32(spo2_vals, spo2_cnt);
        int32_t median = spo2_vals[spo2_cnt / 2];
        out->spo2 = median;
        out->spo2_valid = true;
    }
}
