/**
 * @file app_ds18b20.c
 * @brief DS18B20 温度传感器应用层实现（异步状态机 + 腕温→体温映射）
 */

#include "app_ds18b20.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "onewire_bus.h"
#include "onewire_cmd.h"
#include "ds18b20.h"
#include "esp_log.h"

static const char *TAG = "APP_DS18B20";

/*=============================================================================
 * 内部状态机
 *============================================================================*/
typedef enum {
    ST_IDLE = 0,
    ST_CONVERTING,
} ds_state_t;

/*=============================================================================
 * 模块内部变量
 *============================================================================*/
static onewire_bus_handle_t     s_bus           = NULL;
static ds18b20_device_handle_t  s_ds18b20       = NULL;

static ds_state_t               s_state         = ST_IDLE;
static TickType_t               s_convert_start = 0;
static TickType_t               s_last_sample   = 0;

/* 滑动平均环形缓冲 */
static float                    s_ring[FILTER_WINDOW_SIZE];
static uint8_t                  s_ring_idx      = 0;
static uint8_t                  s_ring_cnt      = 0;
static float                    s_mean          = 0.0f;

/* 异常计数 */
static uint8_t                  s_err_count     = 0;
static uint8_t                  s_outlier_streak= 0;

/* 佩戴状态机计数 */
static uint8_t                  s_wear_on_cnt   = 0;
static uint8_t                  s_wear_off_cnt  = 0;

/* 对外暴露的最新快照 */
static app_ds18b20_data_t       s_data;

/*=============================================================================
 * 内部辅助函数
 *============================================================================*/

static inline float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool is_bogus(float r)
{
    /* -127°C：断线；+85°C：power-on default（未真正转换） */
    if (r <= -126.9f) return true;
    if (r >= 84.9f && r <= 85.1f) return true;
    if (r < DS18B20_RAW_SANE_MIN || r > DS18B20_RAW_SANE_MAX) return true;
    return false;
}

static void reset_filter(void)
{
    s_ring_idx = 0;
    s_ring_cnt = 0;
    s_mean     = 0.0f;
    s_outlier_streak = 0;
}

static float ring_push_mean(float v)
{
    s_ring[s_ring_idx] = v;
    s_ring_idx = (s_ring_idx + 1) % FILTER_WINDOW_SIZE;
    if (s_ring_cnt < FILTER_WINDOW_SIZE) {
        s_ring_cnt++;
    }
    float sum = 0.0f;
    for (uint8_t i = 0; i < s_ring_cnt; i++) {
        sum += s_ring[i];
    }
    return sum / (float)s_ring_cnt;
}

static void record_error(void)
{
    if (s_err_count < 0xFF) s_err_count++;
    if (s_err_count >= MAX_CONSECUTIVE_ERRORS) {
        if (!s_data.sensor_fault) {
            ESP_LOGW(TAG, "连续 %u 次读取失败，标记 sensor_fault", s_err_count);
        }
        s_data.sensor_fault = true;
        s_data.valid        = false;
        s_data.body_temp    = 0.0f;
        s_data.wear_state   = APP_DS18B20_WEAR_UNKNOWN;
        reset_filter();
        s_wear_on_cnt  = 0;
        s_wear_off_cnt = 0;
    }
}

static void clear_error(void)
{
    if (s_data.sensor_fault) {
        ESP_LOGI(TAG, "sensor_fault 已恢复");
    }
    s_err_count         = 0;
    s_data.sensor_fault = false;
}

/* 非阻塞触发温度转换：手发 reset + SKIP_ROM + CONVERT_T，不调用驱动的 vTaskDelay */
static esp_err_t trigger_convert_nowait(void)
{
    esp_err_t ret = onewire_bus_reset(s_bus);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t tx[2] = { ONEWIRE_CMD_SKIP_ROM, 0x44 /* DS18B20 CONVERT_T */ };
    return onewire_bus_write_bytes(s_bus, tx, sizeof(tx));
}

static float map_skin_to_body(float skin)
{
    float body = skin * WRIST_GAIN + WRIST_OFFSET;
    return clamp_f(body, BODY_TEMP_MIN, BODY_TEMP_MAX);
}

/* 佩戴状态更新：阈值 + 迟滞 + 连续确认 */
static void update_wear_state(float skin_filtered)
{
    if (s_data.wear_state == APP_DS18B20_WEAR_ON) {
        if (skin_filtered < WEAR_OFF_THRESHOLD_C) {
            s_wear_off_cnt++;
            s_wear_on_cnt = 0;
            if (s_wear_off_cnt >= WEAR_CONFIRM_SAMPLES) {
                s_data.wear_state = APP_DS18B20_WEAR_OFF;
                s_wear_off_cnt = 0;
                ESP_LOGI(TAG, "佩戴状态: 已摘下 (skin=%.2f°C)", skin_filtered);
            }
        } else {
            s_wear_off_cnt = 0;
        }
        return;
    }

    /* WEAR_OFF 或 WEAR_UNKNOWN 都视作"非佩戴"起点 */
    if (skin_filtered > WEAR_ON_THRESHOLD_C) {
        s_wear_on_cnt++;
        s_wear_off_cnt = 0;
        if (s_wear_on_cnt >= WEAR_CONFIRM_SAMPLES) {
            s_data.wear_state = APP_DS18B20_WEAR_ON;
            s_wear_on_cnt = 0;
            ESP_LOGI(TAG, "佩戴状态: 已佩戴 (skin=%.2f°C)", skin_filtered);
        }
    } else {
        s_wear_on_cnt = 0;
        if (s_data.wear_state == APP_DS18B20_WEAR_UNKNOWN
            && s_ring_cnt == FILTER_WINDOW_SIZE) {
            s_data.wear_state = APP_DS18B20_WEAR_OFF;
        }
    }
}

/*=============================================================================
 * 单次采样结果处理
 *============================================================================*/
static void process_new_raw(float raw)
{
    s_data.skin_temp_raw = raw;

    /* 离群检测：ring 已满时，偏差超过阈值先计数 */
    if (s_ring_cnt == FILTER_WINDOW_SIZE
        && fabsf(raw - s_mean) > OUTLIER_DELTA) {
        s_outlier_streak++;
        if (s_outlier_streak < OUTLIER_MAX_STREAK) {
            ESP_LOGW(TAG, "离群值已丢弃: raw=%.2f mean=%.2f streak=%u",
                     raw, s_mean, s_outlier_streak);
            return;
        }
        /* 连续多次偏差 → 认可为真实跳变，重建基线 */
        ESP_LOGW(TAG, "连续离群，重建滤波基线: raw=%.2f", raw);
        reset_filter();
    }
    s_outlier_streak = 0;

    s_mean = ring_push_mean(raw);
    s_data.skin_temp_filtered = s_mean;

    update_wear_state(s_mean);

    if (s_data.wear_state == APP_DS18B20_WEAR_ON
        && s_ring_cnt == FILTER_WINDOW_SIZE) {
        s_data.body_temp = map_skin_to_body(s_mean);
        s_data.valid     = true;
    } else {
        s_data.body_temp = 0.0f;
        s_data.valid     = false;
    }
}

/*=============================================================================
 * API
 *============================================================================*/

esp_err_t app_ds18b20_init(void)
{
    esp_err_t ret;

    memset(&s_data, 0, sizeof(s_data));
    s_data.wear_state = APP_DS18B20_WEAR_UNKNOWN;

    /* 1. OneWire 总线 */
    onewire_bus_config_t bus_cfg = {
        .bus_gpio_num = APP_DS18B20_GPIO,
    };
    onewire_bus_rmt_config_t rmt_cfg = {
        .max_rx_bytes = 10,
    };
    ret = onewire_new_bus_rmt(&bus_cfg, &rmt_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OneWire 总线初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "OneWire 总线初始化完成 (GPIO%d)", APP_DS18B20_GPIO);

    /* 2. DS18B20 设备（单设备模式） */
    ds18b20_config_t ds_cfg = {};
    ret = ds18b20_new_device_from_bus(s_bus, &ds_cfg, &s_ds18b20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS18B20 设备创建失败: %s", esp_err_to_name(ret));
        onewire_bus_del(s_bus);
        s_bus = NULL;
        return ret;
    }

    /* 3. 设置分辨率（决定转换时间：12-bit = 750ms） */
    ret = ds18b20_set_resolution(s_ds18b20, APP_DS18B20_RESOLUTION);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置分辨率失败: %s（保持默认）", esp_err_to_name(ret));
    }

    s_state         = ST_IDLE;
    s_last_sample   = 0;
    s_convert_start = 0;
    reset_filter();
    s_err_count    = 0;
    s_wear_on_cnt  = 0;
    s_wear_off_cnt = 0;

    ESP_LOGI(TAG, "DS18B20 初始化完成（异步模式）");
    return ESP_OK;
}

void app_ds18b20_deinit(void)
{
    if (s_ds18b20) {
        ds18b20_del_device(s_ds18b20);
        s_ds18b20 = NULL;
    }
    if (s_bus) {
        onewire_bus_del(s_bus);
        s_bus = NULL;
    }
    s_state = ST_IDLE;
    ESP_LOGI(TAG, "DS18B20 已反初始化");
}

void app_ds18b20_tick(void)
{
    if (s_ds18b20 == NULL || s_bus == NULL) {
        return;
    }

    TickType_t now = xTaskGetTickCount();

    switch (s_state) {
    case ST_IDLE: {
        if ((now - s_last_sample) < pdMS_TO_TICKS(SAMPLE_INTERVAL_MS)
            && s_last_sample != 0) {
            break;
        }
        esp_err_t r = trigger_convert_nowait();
        if (r == ESP_OK) {
            s_convert_start = now;
            s_state = ST_CONVERTING;
        } else {
            ESP_LOGD(TAG, "触发转换失败: %s", esp_err_to_name(r));
            s_last_sample = now;
            record_error();
        }
        break;
    }

    case ST_CONVERTING: {
        if ((now - s_convert_start) < pdMS_TO_TICKS(CONVERT_WAIT_MS)) {
            break;
        }
        float raw = 0.0f;
        esp_err_t r = ds18b20_get_temperature(s_ds18b20, &raw);
        s_last_sample = now;
        s_state = ST_IDLE;

        if (r != ESP_OK) {
            ESP_LOGD(TAG, "读取温度失败: %s", esp_err_to_name(r));
            record_error();
            break;
        }
        if (is_bogus(raw)) {
            ESP_LOGW(TAG, "幻值丢弃: raw=%.2f°C", raw);
            record_error();
            break;
        }
        clear_error();
        process_new_raw(raw);
        s_data.last_update_tick = now;
        break;
    }
    }
}

esp_err_t app_ds18b20_read(app_ds18b20_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_data;
    return ESP_OK;
}

esp_err_t app_ds18b20_read_temperature(float *temperature)
{
    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_data.valid) {
        *temperature = 0.0f;
        return ESP_ERR_INVALID_STATE;
    }
    *temperature = s_data.body_temp;
    return ESP_OK;
}
