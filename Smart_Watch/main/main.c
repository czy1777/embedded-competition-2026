/**
 * @file main.c
 * @brief Smart Watch 主程序 - MPU6050 跌倒检测 + BLE 蓝牙通信
 */

#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "app_mpu6050.h"
#include "app_ds18b20.h"
#include "app_max30102.h"
#include "app_ssd1306_oled.h"
#include "app_spp.h"

static const char *TAG = "MAIN";

// 采样周期 (ms)
#define SAMPLE_PERIOD_MS    20  // 50Hz 采样率

// 蓝牙发送间隔 (循环次数, 25次 = 500ms)
#define BLE_SEND_INTERVAL   25

void app_main(void)
{
    ESP_LOGI(TAG, "Smart Watch 启动");

    // 初始化 NVS (蓝牙配对信息存储必需)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化 OLED (优先初始化，用于显示启动状态)
    ret = app_oled_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED 初始化失败");
    }

    // 初始化 MPU6050
    ret = app_mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 初始化失败，程序退出");
        return;
    }

    // 初始化 DS18B20
    ret = app_ds18b20_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS18B20 初始化失败，程序退出");
        return;
    }

    // 初始化 MAX30102 (独立 I2C_NUM_1: GPIO5-SDA, GPIO15-SCL)
    ret = app_max30102_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 初始化失败");
    }

    // 初始化蓝牙
    ret = app_spp_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙初始化失败");
    }

    ESP_LOGI(TAG, "开始跌倒检测...");

    app_mpu6050_data_t sensor_data;
    app_max30102_data_t hr_data;
    app_ds18b20_data_t temp_data = {0};
    float temperature = 0;
    uint32_t health_log_counter = 0;
    uint32_t display_update_counter = 0;
    uint32_t ble_send_counter = 0;

    // 当前健康数据
    uint8_t current_hr = 0;
    float current_spo2 = 0;
    bool hr_valid = false;

    while (1) {
        // 推进 DS18B20 异步状态机（非阻塞，内部按 SAMPLE_INTERVAL_MS 限速）
        app_ds18b20_tick();

        // 读取传感器数据
        ret = app_mpu6050_read(&sensor_data);
        if (ret == ESP_OK) {
            // 跌倒检测
            bool fall_detected = app_mpu6050_detect_fall(&sensor_data);

            if (fall_detected) {
                ESP_LOGW(TAG, "========================================");
                ESP_LOGW(TAG, "*** 检测到跌倒! ***");
                ESP_LOGW(TAG, "加速度: X=%.2fg Y=%.2fg Z=%.2fg (模值=%.2fg)",
                         sensor_data.acce.x, sensor_data.acce.y,
                         sensor_data.acce.z, sensor_data.acce_mag);
                ESP_LOGW(TAG, "角速度: X=%.1f Y=%.1f Z=%.1f dps",
                         sensor_data.gyro.x, sensor_data.gyro.y,
                         sensor_data.gyro.z);
                ESP_LOGW(TAG, "姿态角: Roll=%.1f Pitch=%.1f",
                         sensor_data.angle.roll, sensor_data.angle.pitch);
                ESP_LOGW(TAG, "========================================");

                // 显示跌倒警报
                app_oled_show_fall_alert();

                // 蓝牙发送跌倒警报
                app_spp_send_fall_alert();

                // 等待一段时间后重置
                vTaskDelay(pdMS_TO_TICKS(5000));
                app_mpu6050_reset_fall_state();
                app_oled_clear_fall_alert();
                ESP_LOGI(TAG, "继续监测...");
            }
        } else {
            ESP_LOGE(TAG, "传感器读取失败: %s", esp_err_to_name(ret));
        }

        // 每50次循环（约1秒）读取温度快照与心率，输出日志
        if (++health_log_counter >= 50) {
            health_log_counter = 0;

            // 读取温度快照（tick 已异步更新）
            app_ds18b20_read(&temp_data);
            temperature = temp_data.valid ? temp_data.body_temp : 0.0f;

            const char *wear_str =
                temp_data.sensor_fault              ? "FAULT" :
                temp_data.wear_state == APP_DS18B20_WEAR_ON  ? "WORN"  :
                temp_data.wear_state == APP_DS18B20_WEAR_OFF ? "OFF"   : "UNKNOWN";
            ESP_LOGI(TAG, "温度: raw=%.2f°C filtered=%.2f°C body=%.2f°C [%s]",
                     temp_data.skin_temp_raw,
                     temp_data.skin_temp_filtered,
                     temp_data.body_temp,
                     wear_str);

            // 读取心率血氧数据
            if (app_max30102_read(&hr_data) == ESP_OK) {
                if (hr_data.data_valid) {
                    current_hr = hr_data.heart_rate;
                    current_spo2 = hr_data.spo2;
                    hr_valid = true;
                    ESP_LOGI(TAG, "心率: %d BPM, 血氧: %.1f%%",
                             hr_data.heart_rate, hr_data.spo2);
                } else {
                    hr_valid = false;
                }
            }
        }

        // 每25次循环更新一次显示（约500ms）
        if (++display_update_counter >= 25) {
            display_update_counter = 0;
            app_oled_update(
                hr_valid ? current_hr : 0,
                hr_valid ? current_spo2 : 0,
                temperature,
                false  // 跌倒状态由 detect_fall 触发
            );
        }

        // 每25次循环发送一次蓝牙数据（约500ms）
        if (++ble_send_counter >= BLE_SEND_INTERVAL) {
            ble_send_counter = 0;
            if (app_spp_is_connected()) {
                app_spp_send_health_data(
                    hr_valid ? current_hr : 0,
                    hr_valid ? (uint8_t)current_spo2 : 0,
                    temperature,
                    app_mpu6050_get_fall_state() == FALL_STATE_DETECTED
                );
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
