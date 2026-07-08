#ifndef SKYSTAR_LINK_H
#define SKYSTAR_LINK_H

#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_SKYSTAR_LINK

#include <cstdint>
#include <functional>
#include <string>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 接收 SkyStar 通过 UART5 转发的健康/环境/GPS 上行帧。
// 协议 (与 SkyStar 端 JYD-18.c / xiaozhi_link.c 镜像):
//   12B 定长 [0]=0xAA [1]=type [2..9]=payload [10]=XOR(0..9) [11]=0x55
//   0x01 HEALTH       hr(u8) spo2(u8) body_temp_x100(i16) fall(u8) 3B rsv
//   0x02 FALL_ALERT   全 0
//   0x03 ENV          pm25_x10(u16) env_temp_x100(i16) humi_x100(u16) 2B rsv
//   0x05 GPS_COORD    lat_e6(i32) lon_e6(i32)
//   0x06 GPS_META     fix(u8) h(u8) m(u8) s(u8) ms(u16) 2B rsv
//   0x07 HEARTBEAT    uptime_s(u32) watch_connected(u8) 3B rsv
class SkyStarLink {
public:
    struct HealthSnapshot {
        // 0x01 / 0x02
        uint8_t  hr             = 0;
        uint8_t  spo2           = 0;
        float    body_temp_c    = 0.0f;
        bool     fall_active    = false;
        TickType_t fall_tick    = 0;
        // 0x03
        float    pm25           = 0.0f;
        float    env_temp_c     = 0.0f;
        float    env_humi_pct   = 0.0f;
        // 0x06
        uint8_t  utc_h          = 0;
        uint8_t  utc_m          = 0;
        uint8_t  utc_s          = 0;
        // 0x07
        uint32_t uptime_s       = 0;
        bool     watch_connected = false;
    };

    using OnFallEdgeCallback     = std::function<void(const HealthSnapshot&)>;
    using OnHealthAnomalyCallback = std::function<void(const HealthSnapshot&, const std::string& what)>;

    static SkyStarLink& GetInstance() {
        static SkyStarLink instance;
        return instance;
    }

    bool Initialize(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud_rate);

    // 取最近一次解析成功的 GPS 坐标 (e6 -> 度), age_ms 是距收到该帧的毫秒数。
    // amap_route 路径规划模块依赖此接口。
    bool GetLatestLocation(double& lat, double& lng, uint32_t& age_ms);

    // 取最近一次健康/环境快照。从未收到有效帧返回 false。
    bool GetHealthSnapshot(HealthSnapshot& out, uint32_t& age_ms);

    // 注册"跌倒沿事件"回调: fall_active 从 0 翻 1 (或来 0x02 FALL_ALERT) 时调用。
    // 回调跑在 RxTask 线程, 务必只做轻量动作 (例如 Application::Schedule)。
    void RegisterOnFallEdge(OnFallEdgeCallback cb) { on_fall_edge_ = std::move(cb); }

    // 注册"健康/环境异常"回调: 连续 3 次越阈值且距上次告警 >60s 时调用。
    // 第二参是已格式化的中文描述, 例如 "心率偏低 45"。
    void RegisterOnHealthAnomaly(OnHealthAnomalyCallback cb) { on_anomaly_ = std::move(cb); }

private:
    SkyStarLink() = default;
    ~SkyStarLink() = default;
    SkyStarLink(const SkyStarLink&) = delete;
    SkyStarLink& operator=(const SkyStarLink&) = delete;

    void RxTaskLoop();
    void HandlePacket(const uint8_t* buf);
    void CheckAnomaly();

    static void RxTaskEntry(void* arg) {
        static_cast<SkyStarLink*>(arg)->RxTaskLoop();
    }

    uart_port_t uart_port_ = UART_NUM_2;
    bool initialized_ = false;
    TaskHandle_t rx_task_ = nullptr;

    // 缓存与回调。 spinlock 保护写、读用 critical section; 回调读取时先拷贝再调,
    // 避免回调内重入或者持锁过久。
    portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

    // GPS (0x05)
    bool       has_fix_  = false;
    double     lat_      = 0.0;
    double     lng_      = 0.0;
    TickType_t fix_tick_ = 0;

    // 健康/环境/心跳 (0x01/0x03/0x06/0x07)
    bool           has_snap_  = false;
    HealthSnapshot snap_{};
    TickType_t     snap_tick_ = 0;

    OnFallEdgeCallback      on_fall_edge_;
    OnHealthAnomalyCallback on_anomaly_;
};

#endif  // CONFIG_ENABLE_SKYSTAR_LINK

#endif  // SKYSTAR_LINK_H
