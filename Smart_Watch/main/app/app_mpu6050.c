/**
 * @file app_mpu6050.c
 * @brief MPU6050 应用层实现 - 跌倒检测
 */

#include "sdkconfig.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "app_mpu6050.h"

static const char *TAG = "APP_MPU6050";

/*=============================================================================
 * 内部变量
 *============================================================================*/
static mpu6050_handle_t s_mpu6050 = NULL;
static mpu6050_angle_t s_angle = {0};           // 持续更新的姿态角
static app_fall_state_t s_fall_state = FALL_STATE_NORMAL;
static uint32_t s_fall_timestamp = 0;           // 当前状态进入时间戳

/* 阶段内部状态 */
static uint8_t  s_freefall_frames = 0;          // 自由落体连续帧计数
static uint8_t  s_posture_frames  = 0;          // 姿态保持帧计数
static float    s_gyro_peak       = 0.0f;       // IMPACT 窗内 gyro 峰值 (dps)

/* 静止判据：加速度模值环形缓冲 */
static float    s_mag_buf[FALL_STILL_WINDOW_N];
static uint8_t  s_mag_idx = 0;
static uint8_t  s_mag_cnt = 0;

/*=============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 初始化 I2C 总线
 */
static esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = APP_I2C_SDA_PIN,
        .scl_io_num = APP_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = APP_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(APP_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(APP_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C 总线初始化完成 (SDA=GPIO%d, SCL=GPIO%d, %dkHz)",
             APP_I2C_SDA_PIN, APP_I2C_SCL_PIN, APP_I2C_FREQ_HZ / 1000);
    return ESP_OK;
}

/**
 * @brief 计算加速度模值
 */
static float calc_acce_magnitude(const mpu6050_acce_t *acce)
{
    return sqrtf(acce->x * acce->x + acce->y * acce->y + acce->z * acce->z);
}

/**
 * @brief 静止判据：近 N 帧 mean(|mag - 1g|) 是否小于阈值
 */
static bool is_still_enough(void)
{
    if (s_mag_cnt < FALL_STILL_WINDOW_N) return false;
    float sum_dev = 0.0f;
    for (uint8_t i = 0; i < FALL_STILL_WINDOW_N; i++) {
        sum_dev += fabsf(s_mag_buf[i] - 1.0f);
    }
    return (sum_dev / (float)FALL_STILL_WINDOW_N) < FALL_STILL_THRESHOLD;
}

/*=============================================================================
 * API 函数实现
 *============================================================================*/

esp_err_t app_mpu6050_init(void)
{
    esp_err_t ret;

    // 1. 初始化 I2C 总线
    ret = i2c_bus_init();
    if (ret != ESP_OK) {
        return ret;
    }

    // 2. 创建 MPU6050 设备句柄
    s_mpu6050 = mpu6050_create(APP_I2C_PORT, MPU6050_I2C_ADDR);
    if (s_mpu6050 == NULL) {
        ESP_LOGE(TAG, "创建 MPU6050 句柄失败");
        i2c_driver_delete(APP_I2C_PORT);
        return ESP_FAIL;
    }

    // 3. 验证器件 ID
    uint8_t device_id;
    ret = mpu6050_get_device_id(s_mpu6050, &device_id);
    if (ret != ESP_OK || device_id != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "器件验证失败 (ID=0x%02X, 期望=0x%02X)",
                 device_id, MPU6050_WHO_AM_I_VAL);
        mpu6050_delete(s_mpu6050);
        s_mpu6050 = NULL;
        i2c_driver_delete(APP_I2C_PORT);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MPU6050 器件验证成功 (ID=0x%02X)", device_id);

    // 4. 唤醒传感器
    ret = mpu6050_wake_up(s_mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "唤醒传感器失败: %s", esp_err_to_name(ret));
        mpu6050_delete(s_mpu6050);
        s_mpu6050 = NULL;
        i2c_driver_delete(APP_I2C_PORT);
        return ret;
    }

    // 5. 配置量程（跌倒检测推荐 8g 和 1000dps）
    ret = mpu6050_config(s_mpu6050, MPU6050_ACCE_FS_8G, MPU6050_GYRO_FS_1000DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置量程失败: %s", esp_err_to_name(ret));
        mpu6050_delete(s_mpu6050);
        s_mpu6050 = NULL;
        i2c_driver_delete(APP_I2C_PORT);
        return ret;
    }
    ESP_LOGI(TAG, "量程配置: 加速度 +/-8g, 陀螺仪 +/-1000dps");

    // 6. 数字低通滤波（抗混叠关键）
    ret = mpu6050_set_dlpf(s_mpu6050, MPU6050_DEFAULT_DLPF);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置 DLPF 失败: %s（继续使用默认带宽）", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DLPF 已配置: 44Hz 带宽");
    }

    // 7. 采样率（与主循环 50Hz 对齐）
    ret = mpu6050_set_sample_rate(s_mpu6050, MPU6050_DEFAULT_RATE_HZ);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置采样率失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "采样率已配置: %dHz", MPU6050_DEFAULT_RATE_HZ);
    }

    // 8. 陀螺仪静态零偏校准
    ESP_LOGI(TAG, "陀螺仪静态校准中，请保持设备平稳 ~1 秒...");
    vTaskDelay(pdMS_TO_TICKS(GYRO_CALIB_SETTLE_MS));
    ret = mpu6050_calibrate_gyro(s_mpu6050, GYRO_CALIB_SAMPLES);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "陀螺仪校准失败: %s（将使用未校准数据）", esp_err_to_name(ret));
    }

    // 9. 重置状态
    s_angle.roll = 0;
    s_angle.pitch = 0;
    s_fall_state = FALL_STATE_NORMAL;

    ESP_LOGI(TAG, "MPU6050 初始化完成");
    return ESP_OK;
}

void app_mpu6050_deinit(void)
{
    if (s_mpu6050) {
        mpu6050_sleep(s_mpu6050);
        mpu6050_delete(s_mpu6050);
        s_mpu6050 = NULL;
    }
    i2c_driver_delete(APP_I2C_PORT);
    ESP_LOGI(TAG, "MPU6050 已反初始化");
}

esp_err_t app_mpu6050_read(app_mpu6050_data_t *data)
{
    if (s_mpu6050 == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 读取加速度
    ret = mpu6050_get_acce(s_mpu6050, &data->acce);
    if (ret != ESP_OK) return ret;

    // 读取陀螺仪
    ret = mpu6050_get_gyro(s_mpu6050, &data->gyro);
    if (ret != ESP_OK) return ret;

    // 更新姿态角（互补滤波）
    ret = mpu6050_calc_angle(s_mpu6050, &data->acce, &data->gyro, &s_angle);
    if (ret != ESP_OK) return ret;

    data->angle = s_angle;

    // 计算加速度模值
    data->acce_mag = calc_acce_magnitude(&data->acce);

    return ESP_OK;
}

bool app_mpu6050_detect_fall(const app_mpu6050_data_t *data)
{
    if (data == NULL) return false;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float mag     = data->acce_mag;
    float gmag    = sqrtf(data->gyro.x * data->gyro.x
                        + data->gyro.y * data->gyro.y
                        + data->gyro.z * data->gyro.z);
    float ang_max = fmaxf(fabsf(data->angle.roll), fabsf(data->angle.pitch));

    /* 更新静止判据环形缓冲 */
    s_mag_buf[s_mag_idx] = mag;
    s_mag_idx = (s_mag_idx + 1) % FALL_STILL_WINDOW_N;
    if (s_mag_cnt < FALL_STILL_WINDOW_N) s_mag_cnt++;

    switch (s_fall_state) {
        case FALL_STATE_NORMAL:
            if (mag < FALL_FREE_FALL_TH) {
                if (++s_freefall_frames >= FALL_FREE_FALL_MIN_FRAMES) {
                    s_fall_state     = FALL_STATE_FREE_FALL;
                    s_fall_timestamp = now;
                    s_gyro_peak      = gmag;
                    s_freefall_frames = 0;
                    ESP_LOGD(TAG, "阶段1: 自由落体 mag=%.2fg", mag);
                }
            } else {
                s_freefall_frames = 0;
            }
            break;

        case FALL_STATE_FREE_FALL:
            if (gmag > s_gyro_peak) s_gyro_peak = gmag;
            if (mag > FALL_IMPACT_TH) {
                s_fall_state     = FALL_STATE_IMPACT;
                s_fall_timestamp = now;
                ESP_LOGD(TAG, "阶段2: 撞击 mag=%.2fg gyro_peak=%.0fdps",
                         mag, s_gyro_peak);
            } else if ((now - s_fall_timestamp) > FALL_FREE_FALL_TIMEOUT) {
                ESP_LOGD(TAG, "阶段1 超时，无撞击，回 NORMAL");
                s_fall_state = FALL_STATE_NORMAL;
            }
            break;

        case FALL_STATE_IMPACT:
            if (gmag > s_gyro_peak) s_gyro_peak = gmag;
            if ((now - s_fall_timestamp) >= FALL_IMPACT_WINDOW_MS) {
                if (s_gyro_peak < FALL_IMPACT_GYRO_MIN_TH) {
                    ESP_LOGD(TAG, "阶段2 拒: 角速度峰值不足 %.0fdps < %.0fdps",
                             s_gyro_peak, FALL_IMPACT_GYRO_MIN_TH);
                    s_fall_state = FALL_STATE_NORMAL;
                } else {
                    s_fall_state     = FALL_STATE_POSTURE;
                    s_fall_timestamp = now;
                    s_posture_frames = 0;
                    ESP_LOGD(TAG, "阶段3: 进入姿态确认 gyro_peak=%.0fdps",
                             s_gyro_peak);
                }
            }
            break;

        case FALL_STATE_POSTURE:
            if (ang_max > FALL_POSTURE_ANGLE_TH) {
                s_posture_frames++;
            } else {
                s_posture_frames = 0;
            }
            if (s_posture_frames >= FALL_POSTURE_MIN_FRAMES
                && is_still_enough()) {
                s_fall_state = FALL_STATE_DETECTED;
                ESP_LOGW(TAG, "跌倒确认: gyro_peak=%.0fdps ang=%.1f still=ok",
                         s_gyro_peak, ang_max);
                return true;
            }
            if ((now - s_fall_timestamp) > FALL_POSTURE_TIMEOUT) {
                ESP_LOGD(TAG, "阶段3 超时，姿态不稳定，回 NORMAL");
                s_fall_state = FALL_STATE_NORMAL;
            }
            break;

        case FALL_STATE_DETECTED:
            return true;

        default:
            s_fall_state = FALL_STATE_NORMAL;
            break;
    }

    return false;
}

app_fall_state_t app_mpu6050_get_fall_state(void)
{
    return s_fall_state;
}

void app_mpu6050_reset_fall_state(void)
{
    s_fall_state       = FALL_STATE_NORMAL;
    s_fall_timestamp   = 0;
    s_freefall_frames  = 0;
    s_posture_frames   = 0;
    s_gyro_peak        = 0.0f;
    s_mag_idx          = 0;
    s_mag_cnt          = 0;
    ESP_LOGI(TAG, "跌倒状态已重置");
}
