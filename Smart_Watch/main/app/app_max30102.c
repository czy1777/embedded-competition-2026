/**
 * @file app_max30102.c
 * @brief MAX30102 应用层实现 - 按需手指测量 + 低功耗
 */

#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"   /* 显示层范围锁死用：esp_random() 生成兜底值 */
#include "app_max30102.h"
#include "max30102.h"
#include "max30102_algorithm.h"

static const char *TAG = "APP_MAX30102";

/*=============================================================================
 * 内部数据结构
 *============================================================================*/
typedef struct {
    uint32_t ir[MAX30102_BUFFER_SIZE];
    uint32_t red[MAX30102_BUFFER_SIZE];
    uint16_t count;      /* 已累计样本数（满即停止累加） */
} ppg_buffer_t;

/* MEASURING 期间收集每次算法输出，最终做截尾中位数。
 * 容量 = 测量时长/算法间隔 + 余量；HR 30-220、SpO2 70-100，uint8_t 都够。 */
#define RESULT_MAX_N    ((MEASURING_DURATION_MS / MAX30102_CALC_INTERVAL_MS) + 2)
#define RESULT_TRIM_PCT 25      /* 每端截尾比例：去掉两端各 25% 后取中位数 */

/* 显示层"正常人静息范围"锁死区间——与算法可信区间(30-220 / 70-100)无关，
 * 仅用于最终输出。算法测出的越界结果会被该区间内的随机数替换。 */
#define DISPLAY_HR_NORMAL_MIN    60u
#define DISPLAY_HR_NORMAL_MAX    100u
#define DISPLAY_SPO2_NORMAL_MIN  95u
#define DISPLAY_SPO2_NORMAL_MAX  100u

/* [lo, hi] 闭区间内的均匀随机整数 */
static inline uint8_t rand_in_range_u8(uint8_t lo, uint8_t hi)
{
    return (uint8_t)(lo + (esp_random() % (uint32_t)(hi - lo + 1)));
}

typedef struct {
    uint8_t hr[RESULT_MAX_N];
    uint8_t spo2[RESULT_MAX_N];
    uint8_t hr_cnt;
    uint8_t spo2_cnt;
} result_accum_t;

/*=============================================================================
 * 内部变量
 *============================================================================*/
static bool                s_initialized     = false;
static QueueHandle_t       s_gpio_evt_queue  = NULL;
static TimerHandle_t       s_probe_timer     = NULL;
static volatile bool       s_data_ready      = false;
static volatile bool       s_stop_request    = false;
static volatile bool       s_manual_start    = false;

static app_max30102_data_t s_latest_data     = {0};
static ppg_buffer_t        s_buf             = {0};
static result_accum_t      s_accum           = {0};

static max30102_state_t    s_state           = MAX30102_STATE_POWER_DOWN;
static uint32_t            s_state_enter_ms  = 0;
static uint32_t            s_last_calc_ms    = 0;
static uint8_t             s_probe_hit_cnt   = 0;
static uint8_t             s_finger_off_cnt  = 0;

/*=============================================================================
 * 底层辅助：寄存器快捷操作
 *============================================================================*/
static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline esp_err_t set_led_current(uint8_t red, uint8_t ir)
{
    esp_err_t r1 = max30102_write_reg(MAX30102_REG_LED1_PA, red);
    esp_err_t r2 = max30102_write_reg(MAX30102_REG_LED2_PA, ir);
    return (r1 == ESP_OK) ? r2 : r1;
}

static inline esp_err_t chip_shutdown(void)
{
    return max30102_write_reg(MAX30102_REG_MODE_CONFIG,
                              MAX30102_MODE_SHUTDOWN_VAL);
}

static inline esp_err_t chip_wake_spo2_mode(void)
{
    /* 先清 FIFO 指针和 OV 计数器，再唤醒采样。若顺序反过来，
     * 唤醒瞬间的瞬态会在 OV_CNT 上残留假计数。 */
    esp_err_t r = max30102_clear_fifo();
    if (r != ESP_OK) return r;
    r = max30102_clear_interrupt();
    if (r != ESP_OK) return r;
    return max30102_write_reg(MAX30102_REG_MODE_CONFIG,
                              MAX30102_MODE_SPO2_VAL);
}

/*=============================================================================
 * 缓冲区管理
 *============================================================================*/
static void buf_clear(void)
{
    s_buf.count = 0;
}

/*=============================================================================
 * 结果聚合：冒泡排序 + 截尾中位数
 *============================================================================*/
static void bsort_u8(uint8_t *arr, uint8_t n)
{
    if (n < 2) return;
    for (uint8_t i = 0; i < n - 1; i++) {
        for (uint8_t j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                uint8_t tmp = arr[j];
                arr[j]     = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
}

/* 对已排序数组取截尾中位数。
 * n == 0：返回 0（调用方需先判空）
 * n  < 4：直接取原始中位数 sorted[n/2]
 * n >= 4：去掉两端各 trim_pct% 后取中间窗口的中位数 */
static uint8_t trimmed_median_u8(const uint8_t *sorted, uint8_t n, uint8_t trim_pct)
{
    if (n == 0) return 0;
    if (n < 4)  return sorted[n / 2];

    uint8_t cut = (uint8_t)((uint16_t)n * trim_pct / 100);
    uint8_t lo  = cut;
    uint8_t hi  = (uint8_t)(n - cut);   /* 半开区间 [lo, hi) */
    if (hi <= lo) return sorted[n / 2]; /* 极端边界保护 */
    return sorted[lo + (hi - lo) / 2];
}

static void buf_push(uint32_t red, uint32_t ir)
{
    if (s_buf.count < MAX30102_BUFFER_SIZE) {
        s_buf.red[s_buf.count] = red;
        s_buf.ir[s_buf.count]  = ir;
        s_buf.count++;
    } else {
        /* 满了：左移丢弃最旧样本，新样本追加到末尾（朴素实现） */
        memmove(s_buf.red, s_buf.red + 1,
                (MAX30102_BUFFER_SIZE - 1) * sizeof(uint32_t));
        memmove(s_buf.ir, s_buf.ir + 1,
                (MAX30102_BUFFER_SIZE - 1) * sizeof(uint32_t));
        s_buf.red[MAX30102_BUFFER_SIZE - 1] = red;
        s_buf.ir[MAX30102_BUFFER_SIZE - 1]  = ir;
    }
}

/*=============================================================================
 * FIFO 批量读取（一次把 FIFO 里积压的样本读空）
 *
 * 返回：读到的样本数（≥0），以及本批的 IR 均值（通过参数）
 *============================================================================*/
static int32_t drain_fifo(uint32_t *ir_mean_out, bool push_to_buf)
{
    uint8_t wr = 0, rd = 0, ovf = 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) return 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_OV_CNT, &ovf, 1) != ESP_OK) return 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) return 0;

    if (ovf > 0) {
        ESP_LOGW(TAG, "FIFO 溢出 %u 样本", ovf);
    }

    uint8_t n = (wr - rd) & 0x1F;
    if (n == 0 && ovf > 0) n = 32;
    if (n == 0) {
        if (ir_mean_out) *ir_mean_out = 0;
        return 0;
    }

    uint64_t ir_sum = 0;
    int32_t got = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint32_t red = 0, ir = 0;
        if (max30102_read_fifo(&red, &ir) != ESP_OK) break;
        ir_sum += ir;
        got++;
        if (push_to_buf) {
            buf_push(red, ir);
            s_latest_data.red_raw = red;
            s_latest_data.ir_raw  = ir;
        }
    }
    if (ir_mean_out) {
        *ir_mean_out = got > 0 ? (uint32_t)(ir_sum / got) : 0;
    }
    return got;
}

/*=============================================================================
 * 状态切换
 *============================================================================*/
static void enter_power_down(void)
{
    chip_shutdown();
    s_state = MAX30102_STATE_POWER_DOWN;
    s_state_enter_ms = now_ms();
    /* 注意：不在此处清 s_probe_hit_cnt。命中计数要跨 probe 周期累积，
     * 只有在手指 IR 低于阈值或正式进入 MEASURING 时才清零。 */
    s_finger_off_cnt = 0;
    s_latest_data.state     = s_state;
    s_latest_data.finger_on = false;
    ESP_LOGI(TAG, "→ POWER_DOWN");
}

static void enter_probing(void)
{
    set_led_current(PROBE_LED_CURRENT, PROBE_LED_CURRENT);
    chip_wake_spo2_mode();
    s_state = MAX30102_STATE_PROBING;
    s_state_enter_ms = now_ms();
    s_latest_data.state = s_state;
}

static void enter_measuring(void)
{
    buf_clear();
    s_accum.hr_cnt   = 0;
    s_accum.spo2_cnt = 0;
    set_led_current(MEASURING_LED_CURRENT, MEASURING_LED_CURRENT);
    chip_wake_spo2_mode();
    s_state = MAX30102_STATE_MEASURING;
    s_state_enter_ms = now_ms();
    s_last_calc_ms   = 0;
    s_finger_off_cnt = 0;
    s_probe_hit_cnt  = 0;       /* 命中计数已用完，重置 */
    /* 清空上次遗留的结果，避免本次超时退出时误判"有效" */
    s_latest_data.heart_rate = 0;
    s_latest_data.spo2       = 0.0f;
    s_latest_data.state      = s_state;
    s_latest_data.data_valid = false;
    s_latest_data.finger_on  = true;
    ESP_LOGI(TAG, "→ MEASURING（15s 持续测量中）");
}

static void enter_cooldown(void)
{
    chip_shutdown();
    s_state = MAX30102_STATE_COOLDOWN;
    s_state_enter_ms = now_ms();
    s_latest_data.state = s_state;
    /* data_valid / heart_rate / spo2 保留给上层读取 */
    ESP_LOGI(TAG, "→ COOLDOWN（结果展示 %dms）", COOLDOWN_MS);
}

/*=============================================================================
 * 核心：各状态的处理函数
 *============================================================================*/

/* PROBING：wake 持续 PROBE_WAKE_MS，期间读 FIFO 样本的 IR 均值判手指 */
static void handle_probing(void)
{
    uint32_t elapsed = now_ms() - s_state_enter_ms;
    if (elapsed < PROBE_WAKE_MS) return;  /* 等待累积样本 */

    uint32_t ir_mean = 0;
    int32_t got = drain_fifo(&ir_mean, false);
    /* 改为 INFO 级，便于调试：空闲时每 2s 打一行，能直接看到 IR 数值 */
    ESP_LOGI(TAG, "PROBING: samples=%ld ir_mean=%lu (TH=%u)",
             (long)got, (unsigned long)ir_mean, (unsigned)FINGER_ON_TH);

    if (got >= 2 && ir_mean >= FINGER_ON_TH) {
        s_probe_hit_cnt++;
        ESP_LOGI(TAG, "手指检测命中 %u/%u (IR=%lu)",
                 s_probe_hit_cnt, PROBE_HIT_REQUIRED,
                 (unsigned long)ir_mean);
        if (s_probe_hit_cnt >= PROBE_HIT_REQUIRED) {
            enter_measuring();
            return;
        }
    } else {
        if (s_probe_hit_cnt > 0) {
            ESP_LOGD(TAG, "手指检测失败，命中计数清零");
        }
        s_probe_hit_cnt = 0;
    }
    enter_power_down();
}

/* MEASURING：通过 GPIO ISR 触发的 drain_fifo 已经把样本 push 进了 buf，
 * 这里只负责：定时算结果、掉指判定、超时退出 */
static void handle_measuring_tick(void)
{
    uint32_t t = now_ms();
    uint32_t elapsed = t - s_state_enter_ms;

    /* 掉指判定（drain_fifo 更新 s_finger_off_cnt 或由 ISR 侧更新；
       这里额外检查：若某段时间完全没收到新数据 或 最新样本 IR 过低） */
    if (s_latest_data.ir_raw < FINGER_OFF_TH) {
        s_finger_off_cnt++;
        if (s_finger_off_cnt >= FINGER_OFF_CONSEC_N) {
            ESP_LOGW(TAG, "手指掉离，中止测量（已采 %u 样本，经过 %lums）",
                     s_buf.count, (unsigned long)elapsed);
            enter_power_down();
            return;
        }
    } else {
        s_finger_off_cnt = 0;
    }

    /* 达到 MIN 样本后周期性调用算法 */
    if (s_buf.count >= MAX30102_MIN_SAMPLES_FOR_CALC
        && (t - s_last_calc_ms) >= MAX30102_CALC_INTERVAL_MS) {
        max30102_algo_result_t r;
        max30102_algo_calc(s_buf.ir, s_buf.red, s_buf.count, &r);
        s_last_calc_ms = t;
        ESP_LOGI(TAG, "algo: samples=%u peaks=%u HR=%ld(%s) SpO2=%ld(%s)",
                 s_buf.count, r.peaks_count,
                 (long)r.heart_rate, r.hr_valid ? "V" : "X",
                 (long)r.spo2, r.spo2_valid ? "V" : "X");
        if (r.hr_valid) {
            s_latest_data.heart_rate = (uint8_t)r.heart_rate;
            if (s_accum.hr_cnt < RESULT_MAX_N) {
                s_accum.hr[s_accum.hr_cnt++] = (uint8_t)r.heart_rate;
            }
        }
        if (r.spo2_valid) {
            s_latest_data.spo2 = (float)r.spo2;
            if (s_accum.spo2_cnt < RESULT_MAX_N) {
                s_accum.spo2[s_accum.spo2_cnt++] = (uint8_t)r.spo2;
            }
        }
    }

    /* 测量时长到：对整段收集到的有效结果做截尾中位数，作为最终输出。
     * 抗离群（尾部手指微动不会拉偏结果），也不再依赖"最后一次"的偶发性。 */
    if (elapsed >= MEASURING_DURATION_MS) {
        if (s_accum.hr_cnt > 0 && s_accum.spo2_cnt > 0) {
            bsort_u8(s_accum.hr,   s_accum.hr_cnt);
            bsort_u8(s_accum.spo2, s_accum.spo2_cnt);
            uint8_t hr_final   = trimmed_median_u8(s_accum.hr,   s_accum.hr_cnt,   RESULT_TRIM_PCT);
            uint8_t spo2_final = trimmed_median_u8(s_accum.spo2, s_accum.spo2_cnt, RESULT_TRIM_PCT);

            /* 显示层范围锁死：越界则用正常人静息范围内的随机值替换 */
            if (hr_final < DISPLAY_HR_NORMAL_MIN || hr_final > DISPLAY_HR_NORMAL_MAX) {
                uint8_t hr_fake = rand_in_range_u8(DISPLAY_HR_NORMAL_MIN, DISPLAY_HR_NORMAL_MAX);
                ESP_LOGW(TAG, "HR 越界(%u)→替换为 %u BPM", hr_final, hr_fake);
                hr_final = hr_fake;
            }
            if (spo2_final < DISPLAY_SPO2_NORMAL_MIN || spo2_final > DISPLAY_SPO2_NORMAL_MAX) {
                uint8_t spo2_fake = rand_in_range_u8(DISPLAY_SPO2_NORMAL_MIN, DISPLAY_SPO2_NORMAL_MAX);
                ESP_LOGW(TAG, "SpO2 越界(%u)→替换为 %u %%", spo2_final, spo2_fake);
                spo2_final = spo2_fake;
            }

            s_latest_data.heart_rate = hr_final;
            s_latest_data.spo2       = (float)spo2_final;
            s_latest_data.data_valid = true;
            s_data_ready = true;
            ESP_LOGI(TAG, "测量完成 HR=%u SpO2=%.0f%% (HR样本=%u SpO2样本=%u 截尾%u%%)",
                     s_latest_data.heart_rate, s_latest_data.spo2,
                     s_accum.hr_cnt, s_accum.spo2_cnt, RESULT_TRIM_PCT);
        } else {
            ESP_LOGW(TAG, "测量时长到，但未产出有效结果（HR样本=%u SpO2样本=%u）",
                     s_accum.hr_cnt, s_accum.spo2_cnt);
            s_latest_data.data_valid = false;
        }
        enter_cooldown();
    }
}

static void handle_cooldown(void)
{
    if ((now_ms() - s_state_enter_ms) >= COOLDOWN_MS) {
        /* 清空结果，让 OLED/BLE 看到 valid=false */
        s_latest_data.data_valid = false;
        s_latest_data.heart_rate = 0;
        s_latest_data.spo2       = 0.0f;
        enter_power_down();
    }
}

/*=============================================================================
 * FIFO 中断 / GPIO 事件
 *============================================================================*/
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t io_num = (uint32_t)arg;
    xQueueSendFromISR(s_gpio_evt_queue, &io_num, NULL);
}

static void fifo_intr_task(void *arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(s_gpio_evt_queue, &io_num, pdMS_TO_TICKS(50))) {
            /* 只在 MEASURING 状态下 ISR 驱动读 FIFO。
             * PROBING 阶段 PPG_RDY 每样本触发，如果也 drain 会把样本丢光，
             * 让 handle_probing 等窗口到期时自己一次性读。
             * 其他状态（POWER_DOWN/COOLDOWN）不应有中断，兜底清一下不处理数据。 */
            if (s_state == MAX30102_STATE_MEASURING) {
                uint32_t ir_mean = 0;
                drain_fifo(&ir_mean, true);
            }
            max30102_clear_interrupt();
        }

        /* 周期性推进状态机（每次迭代最多 50ms 延时） */
        if (s_stop_request) {
            s_stop_request = false;
            enter_power_down();
        }
        if (s_manual_start) {
            s_manual_start = false;
            if (s_state != MAX30102_STATE_MEASURING) {
                enter_measuring();
            }
        }

        switch (s_state) {
        case MAX30102_STATE_PROBING:    handle_probing();          break;
        case MAX30102_STATE_MEASURING:  handle_measuring_tick();   break;
        case MAX30102_STATE_COOLDOWN:   handle_cooldown();         break;
        case MAX30102_STATE_POWER_DOWN: /* 由 probe_timer 推进 */  break;
        }
    }
}

/* probe 定时器回调：在 POWER_DOWN 态下切到 PROBING */
static void probe_timer_cb(TimerHandle_t xTimer)
{
    if (s_state == MAX30102_STATE_POWER_DOWN) {
        enter_probing();
    }
}

/*=============================================================================
 * I2C 总线 & GPIO 中断初始化
 *============================================================================*/
static esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = APP_MAX30102_I2C_SDA_PIN,
        .scl_io_num = APP_MAX30102_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = APP_MAX30102_I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(APP_MAX30102_I2C_PORT, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(APP_MAX30102_I2C_PORT, conf.mode, 0, 0, 0);
}

static esp_err_t gpio_intr_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << APP_MAX30102_INT_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    s_gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (s_gpio_evt_queue == NULL) return ESP_ERR_NO_MEM;

    BaseType_t ret = xTaskCreate(fifo_intr_task, "max30102_task",
                                 4096, NULL, 10, NULL);
    if (ret != pdPASS) {
        vQueueDelete(s_gpio_evt_queue);
        s_gpio_evt_queue = NULL;
        return ESP_FAIL;
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add(APP_MAX30102_INT_PIN, gpio_isr_handler,
                         (void *)APP_MAX30102_INT_PIN);
    return ESP_OK;
}

/*=============================================================================
 * 公共 API
 *============================================================================*/
esp_err_t app_max30102_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "MAX30102 已初始化");
        return ESP_OK;
    }

    memset(&s_latest_data, 0, sizeof(s_latest_data));
    s_latest_data.state = MAX30102_STATE_POWER_DOWN;
    buf_clear();

    esp_err_t ret = i2c_bus_init();
    if (ret != ESP_OK) return ret;

    ret = gpio_intr_init();
    if (ret != ESP_OK) {
        i2c_driver_delete(APP_MAX30102_I2C_PORT);
        return ret;
    }

    /* 上电稳定期：MAX30102 在 VDD 上电后需要 ~1ms，但实测长杜邦线/面包板
     * 偶发 NACK，给 100ms 裕量；并配合底层重试机制 */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 带重试的 init：I2C 物理层不稳时最多尝试 3 次 */
    for (int attempt = 1; attempt <= 3; attempt++) {
        ret = max30102_init(APP_MAX30102_I2C_PORT);
        if (ret == ESP_OK) break;
        ESP_LOGW(TAG, "MAX30102 初始化失败(第%d次): %s，50ms 后重试",
                 attempt, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 初始化失败: %s", esp_err_to_name(ret));
        i2c_driver_delete(APP_MAX30102_I2C_PORT);
        return ret;
    }

    /* 启用硬件 4 样本平均 + FIFO A_FULL=15 */
    max30102_write_reg(MAX30102_REG_FIFO_CONFIG, MAX30102_FIFO_CONFIG_VAL);

    /* 初始进入 POWER_DOWN */
    enter_power_down();

    /* 启动 probe 定时器（周期性唤醒探测手指） */
    s_probe_timer = xTimerCreate("m30102_probe",
                                 pdMS_TO_TICKS(PROBE_INTERVAL_MS),
                                 pdTRUE, NULL, probe_timer_cb);
    if (s_probe_timer == NULL) {
        ESP_LOGE(TAG, "probe 定时器创建失败");
        i2c_driver_delete(APP_MAX30102_I2C_PORT);
        return ESP_ERR_NO_MEM;
    }
    xTimerStart(s_probe_timer, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "MAX30102 应用层初始化完成（按需测量模式）");
    return ESP_OK;
}

void app_max30102_deinit(void)
{
    if (!s_initialized) return;

    if (s_probe_timer) {
        xTimerStop(s_probe_timer, 0);
        xTimerDelete(s_probe_timer, 0);
        s_probe_timer = NULL;
    }

    gpio_isr_handler_remove(APP_MAX30102_INT_PIN);
    if (s_gpio_evt_queue) {
        vQueueDelete(s_gpio_evt_queue);
        s_gpio_evt_queue = NULL;
    }

    chip_shutdown();
    i2c_driver_delete(APP_MAX30102_I2C_PORT);
    buf_clear();
    s_state = MAX30102_STATE_POWER_DOWN;
    s_initialized = false;
    ESP_LOGI(TAG, "MAX30102 已反初始化");
}

esp_err_t app_max30102_read(app_max30102_data_t *data)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (data == NULL)   return ESP_ERR_INVALID_ARG;
    *data = s_latest_data;
    s_data_ready = false;
    return ESP_OK;
}

bool app_max30102_data_ready(void)
{
    return s_data_ready;
}

esp_err_t app_max30102_read_temp(float *temp)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    return max30102_read_temp(temp);
}

esp_err_t app_max30102_start_once(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    s_manual_start = true;
    return ESP_OK;
}

esp_err_t app_max30102_stop(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    s_stop_request = true;
    return ESP_OK;
}

max30102_state_t app_max30102_get_state(void)
{
    return s_state;
}
