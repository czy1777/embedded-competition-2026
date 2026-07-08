#include "skystar_link.h"

#ifdef CONFIG_ENABLE_SKYSTAR_LINK

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <esp_log.h>

#define TAG "SkyStar"

namespace {
constexpr int kRxBufSize  = 1024;
constexpr int kPacketSize = 12;
constexpr uint8_t kHeader = 0xAA;
constexpr uint8_t kTail   = 0x55;

// 与 SkyStar 端 JYD-18.h 一致
constexpr uint8_t kTypeHealth    = 0x01;
constexpr uint8_t kTypeFallAlert = 0x02;
constexpr uint8_t kTypeEnv       = 0x03;
constexpr uint8_t kTypeGpsCoord  = 0x05;
constexpr uint8_t kTypeGpsMeta   = 0x06;
constexpr uint8_t kTypeHeartbeat = 0x07;

// 健康/环境异常阈值。生理学常识值, 不暴露给运行时配置。
constexpr uint8_t kHrLow      = 50;
constexpr uint8_t kHrHigh     = 120;
constexpr uint8_t kSpo2Low    = 92;
constexpr float   kTempLow    = 35.0f;
constexpr float   kTempHigh   = 38.0f;
constexpr float   kPm25High   = 75.0f;

// 同一项连续 3 次异常才触发回调; 触发后 60s 内不重复, 避免轰炸。
constexpr int        kAnomalyConsecutive = 3;
constexpr TickType_t kAnomalyCooldown    = pdMS_TO_TICKS(60000);

// 异常状态机, 跑在 RxTaskLoop 里 (单线程), 不需要加锁。
struct AnomalyCounter {
    int        consecutive = 0;
    TickType_t last_fire   = 0;
};
AnomalyCounter g_hr_low_cnt, g_hr_high_cnt, g_spo2_low_cnt;
AnomalyCounter g_temp_low_cnt, g_temp_high_cnt, g_pm25_high_cnt;

// 跌倒沿检测: 用 fall_tick 的"新值"判定, 配合 fall_active=1。
TickType_t g_last_fall_tick_seen = 0;
}  // namespace

bool SkyStarLink::Initialize(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud_rate) {
    if (initialized_) return true;
    initialized_ = true;
    uart_port_ = port;

    uart_config_t cfg = {};
    cfg.baud_rate = baud_rate;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    if (uart_driver_install(port, kRxBufSize, 0, 0, nullptr, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return false;
    }
    uart_param_config(port, &cfg);
    uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY);

    xTaskCreate(&SkyStarLink::RxTaskEntry, "skystar_rx", 4096, this, 5, &rx_task_);
    ESP_LOGI(TAG, "SkyStar UART link up: port=%d tx=%d rx=%d baud=%d",
             port, tx, rx, baud_rate);
    return true;
}

void SkyStarLink::RxTaskLoop() {
    uint8_t buf[kPacketSize];
    int pos = 0;

    while (true) {
        uint8_t ch;
        int n = uart_read_bytes(uart_port_, &ch, 1, pdMS_TO_TICKS(200));
        if (n <= 0) continue;

        if (pos == 0) {
            if (ch != kHeader) continue;
            buf[pos++] = ch;
            continue;
        }
        buf[pos++] = ch;
        if (pos < kPacketSize) continue;

        if (buf[kPacketSize - 1] != kTail) {
            ESP_LOGW(TAG, "bad tail 0x%02X", buf[kPacketSize - 1]);
            pos = 0;
            continue;
        }
        uint8_t xor_calc = 0;
        for (int i = 0; i < 10; ++i) xor_calc ^= buf[i];
        if (xor_calc != buf[10]) {
            ESP_LOGW(TAG, "bad XOR: calc=0x%02X got=0x%02X", xor_calc, buf[10]);
            pos = 0;
            continue;
        }
        HandlePacket(buf);
        pos = 0;
    }
}

void SkyStarLink::HandlePacket(const uint8_t* buf) {
    uint8_t type = buf[1];

    // ===== 临时调试: 抓到一帧就打一行, 验证通过后删掉这行 =====
    ESP_LOGI(TAG, "RX type=0x%02X payload=%02X %02X %02X %02X %02X %02X %02X %02X",
             type, buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);

    if (type == kTypeHealth) {
        uint8_t hr   = buf[2];
        uint8_t spo2 = buf[3];
        int16_t bt   = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
        uint8_t fall = buf[6];

        bool need_fall_edge = false;
        portENTER_CRITICAL(&lock_);
        bool prev_fall = snap_.fall_active;
        snap_.hr           = hr;
        snap_.spo2         = spo2;
        snap_.body_temp_c  = bt / 100.0f;
        snap_.fall_active  = (fall != 0);
        if (snap_.fall_active && !prev_fall) {
            snap_.fall_tick = xTaskGetTickCount();
            need_fall_edge  = true;
        }
        snap_tick_ = xTaskGetTickCount();
        has_snap_  = true;
        HealthSnapshot copy = snap_;
        portEXIT_CRITICAL(&lock_);

        ESP_LOGD(TAG, "HEALTH hr=%d spo2=%d temp=%.2f fall=%d", hr, spo2, bt/100.0f, fall);

        if (need_fall_edge && copy.fall_tick != g_last_fall_tick_seen) {
            g_last_fall_tick_seen = copy.fall_tick;
            if (on_fall_edge_) on_fall_edge_(copy);
        }
        CheckAnomaly();
        return;
    }

    if (type == kTypeFallAlert) {
        portENTER_CRITICAL(&lock_);
        snap_.fall_active = true;
        snap_.fall_tick   = xTaskGetTickCount();
        snap_tick_        = xTaskGetTickCount();
        has_snap_         = true;
        HealthSnapshot copy = snap_;
        portEXIT_CRITICAL(&lock_);

        ESP_LOGW(TAG, "FALL_ALERT received");
        if (copy.fall_tick != g_last_fall_tick_seen) {
            g_last_fall_tick_seen = copy.fall_tick;
            if (on_fall_edge_) on_fall_edge_(copy);
        }
        return;
    }

    if (type == kTypeEnv) {
        uint16_t pm25_x10 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        int16_t  t_x100   = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
        uint16_t h_x100   = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

        portENTER_CRITICAL(&lock_);
        snap_.pm25         = pm25_x10 / 10.0f;
        snap_.env_temp_c   = t_x100   / 100.0f;
        snap_.env_humi_pct = h_x100   / 100.0f;
        snap_tick_         = xTaskGetTickCount();
        has_snap_          = true;
        portEXIT_CRITICAL(&lock_);

        ESP_LOGD(TAG, "ENV pm25=%.1f temp=%.2f humi=%.2f",
                 pm25_x10/10.0f, t_x100/100.0f, h_x100/100.0f);
        CheckAnomaly();
        return;
    }

    if (type == kTypeGpsCoord) {
        int32_t lat_raw = (int32_t)((uint32_t)buf[2] |
                                    ((uint32_t)buf[3] << 8) |
                                    ((uint32_t)buf[4] << 16) |
                                    ((uint32_t)buf[5] << 24));
        int32_t lng_raw = (int32_t)((uint32_t)buf[6] |
                                    ((uint32_t)buf[7] << 8) |
                                    ((uint32_t)buf[8] << 16) |
                                    ((uint32_t)buf[9] << 24));
        double lat = lat_raw / 1.0e6;
        double lng = lng_raw / 1.0e6;

        portENTER_CRITICAL(&lock_);
        lat_      = lat;
        lng_      = lng;
        fix_tick_ = xTaskGetTickCount();
        has_fix_  = true;
        portEXIT_CRITICAL(&lock_);

        ESP_LOGD(TAG, "GPS %.6f,%.6f", lat, lng);
        return;
    }

    if (type == kTypeGpsMeta) {
        // 协议: buf[2]=fix, buf[3]=h, buf[4]=m, buf[5]=s, buf[6..7]=ms
        portENTER_CRITICAL(&lock_);
        snap_.utc_h = buf[3];
        snap_.utc_m = buf[4];
        snap_.utc_s = buf[5];
        snap_tick_  = xTaskGetTickCount();
        has_snap_   = true;
        portEXIT_CRITICAL(&lock_);
        ESP_LOGD(TAG, "GPS_META %02u:%02u:%02u", buf[3], buf[4], buf[5]);
        return;
    }

    if (type == kTypeHeartbeat) {
        uint32_t up = (uint32_t)buf[2]
                    | ((uint32_t)buf[3] << 8)
                    | ((uint32_t)buf[4] << 16)
                    | ((uint32_t)buf[5] << 24);
        portENTER_CRITICAL(&lock_);
        snap_.uptime_s        = up;
        snap_.watch_connected = (buf[6] != 0);
        portEXIT_CRITICAL(&lock_);
        ESP_LOGD(TAG, "HB uptime=%lus watch=%d", (unsigned long)up, buf[6]);
        return;
    }

    ESP_LOGD(TAG, "ignore packet type=0x%02X", type);
}

namespace {
// 通用单项异常步进: cnt 满足连续阈值且超过 cooldown 则返回 true 并刷新 last_fire。
bool StepAnomaly(AnomalyCounter& cnt, bool tripped) {
    if (!tripped) {
        cnt.consecutive = 0;
        return false;
    }
    cnt.consecutive++;
    if (cnt.consecutive < kAnomalyConsecutive) return false;
    TickType_t now = xTaskGetTickCount();
    if (cnt.last_fire != 0 && (now - cnt.last_fire) < kAnomalyCooldown) {
        // 仍在 cooldown, 不触发, 但保持 consecutive 不清零, 让 cooldown 结束后立即再发
        return false;
    }
    cnt.last_fire = now;
    return true;
}
}  // namespace

void SkyStarLink::CheckAnomaly() {
    HealthSnapshot s;
    portENTER_CRITICAL(&lock_);
    s = snap_;
    portEXIT_CRITICAL(&lock_);

    char buf[64];

    if (s.hr > 0) {  // hr=0 视为无数据, 跳过
        if (StepAnomaly(g_hr_low_cnt, s.hr < kHrLow)) {
            std::snprintf(buf, sizeof(buf), "心率偏低 %u", s.hr);
            if (on_anomaly_) on_anomaly_(s, buf);
        }
        if (StepAnomaly(g_hr_high_cnt, s.hr > kHrHigh)) {
            std::snprintf(buf, sizeof(buf), "心率偏高 %u", s.hr);
            if (on_anomaly_) on_anomaly_(s, buf);
        }
    }
    if (s.spo2 > 0) {
        if (StepAnomaly(g_spo2_low_cnt, s.spo2 < kSpo2Low)) {
            std::snprintf(buf, sizeof(buf), "血氧偏低 %u%%", s.spo2);
            if (on_anomaly_) on_anomaly_(s, buf);
        }
    }
    if (s.body_temp_c > 0.0f) {
        if (StepAnomaly(g_temp_low_cnt, s.body_temp_c < kTempLow)) {
            std::snprintf(buf, sizeof(buf), "体温偏低 %.1f°C", s.body_temp_c);
            if (on_anomaly_) on_anomaly_(s, buf);
        }
        if (StepAnomaly(g_temp_high_cnt, s.body_temp_c > kTempHigh)) {
            std::snprintf(buf, sizeof(buf), "体温偏高 %.1f°C", s.body_temp_c);
            if (on_anomaly_) on_anomaly_(s, buf);
        }
    }
    if (s.pm25 > 0.0f) {
        if (StepAnomaly(g_pm25_high_cnt, s.pm25 > kPm25High)) {
            std::snprintf(buf, sizeof(buf), "空气质量差 PM2.5=%.0f", s.pm25);
            if (on_anomaly_) on_anomaly_(s, buf);
        }
    }
}

bool SkyStarLink::GetLatestLocation(double& lat, double& lng, uint32_t& age_ms) {
    portENTER_CRITICAL(&lock_);
    bool ok = has_fix_;
    double cached_lat = lat_;
    double cached_lng = lng_;
    TickType_t cached_tick = fix_tick_;
    portEXIT_CRITICAL(&lock_);

    if (!ok) return false;
    lat = cached_lat;
    lng = cached_lng;
    age_ms = (uint32_t)((xTaskGetTickCount() - cached_tick) * portTICK_PERIOD_MS);
    return true;
}

bool SkyStarLink::GetHealthSnapshot(HealthSnapshot& out, uint32_t& age_ms) {
    portENTER_CRITICAL(&lock_);
    bool ok = has_snap_;
    HealthSnapshot cached = snap_;
    TickType_t cached_tick = snap_tick_;
    portEXIT_CRITICAL(&lock_);

    if (!ok) return false;
    out = cached;
    age_ms = (uint32_t)((xTaskGetTickCount() - cached_tick) * portTICK_PERIOD_MS);
    return true;
}

#endif  // CONFIG_ENABLE_SKYSTAR_LINK
