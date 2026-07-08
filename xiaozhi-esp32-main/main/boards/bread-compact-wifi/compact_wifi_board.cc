#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "a7670_sms.h"
#include "a7670_sms_mcp_tool.h"
#include "amap_route_mcp_tool.h"
#include "skystar_link.h"
#include "skystar_link_mcp_tool.h"
#include <atomic>

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        // SSD1306 config
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = 400 * 1000,
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // 物联网初始化，逐步迁移到 MCP 协议
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
        InitializeSmsModule();
#ifdef CONFIG_ENABLE_SKYSTAR_LINK
        SkyStarLink::GetInstance().Initialize(
            SKYSTAR_UART_NUM, SKYSTAR_TX_PIN, SKYSTAR_RX_PIN, SKYSTAR_BAUD_RATE);

        // 跌倒沿事件 → 等设备 idle → 主动唤醒, 让服务端 AI 询问是否联系紧急联系人。
        // watcher 模式照搬 SMS notify 那段, 避免打断当前对话。
        SkyStarLink::GetInstance().RegisterOnFallEdge(
            [](const SkyStarLink::HealthSnapshot&) {
                ESP_LOGW(TAG, "fall edge detected, scheduling AI prompt");
                static std::atomic<bool> watcher{false};
                bool expected = false;
                if (!watcher.compare_exchange_strong(expected, true)) {
                    ESP_LOGI(TAG, "fall watcher pending, skip duplicate");
                    return;
                }
                xTaskCreate([](void* arg) {
                    auto* flag = static_cast<std::atomic<bool>*>(arg);
                    auto& app = Application::GetInstance();
                    for (int i = 0; i < 150; ++i) {
                        if (app.GetDeviceState() == kDeviceStateIdle) {
                            app.WakeWordInvoke("用户跌倒了, 请询问是否需要联系紧急联系人");
                            break;
                        }
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                    flag->store(false);
                    vTaskDelete(nullptr);
                }, "fall_notify", 3072, &watcher, 4, nullptr);
            });

        // 健康/环境异常 → 等设备 idle → 把异常描述喂给 AI, 让它用人话播报。
        SkyStarLink::GetInstance().RegisterOnHealthAnomaly(
            [](const SkyStarLink::HealthSnapshot&, const std::string& what) {
                ESP_LOGW(TAG, "health anomaly: %s", what.c_str());
                static std::atomic<bool> watcher{false};
                bool expected = false;
                if (!watcher.compare_exchange_strong(expected, true)) {
                    ESP_LOGI(TAG, "anomaly watcher pending, skip duplicate");
                    return;
                }
                struct Arg { std::atomic<bool>* flag; std::string msg; };
                auto* a = new Arg{&watcher, "健康提醒: " + what};
                xTaskCreate([](void* p) {
                    auto* arg = static_cast<Arg*>(p);
                    auto& app = Application::GetInstance();
                    for (int i = 0; i < 150; ++i) {
                        if (app.GetDeviceState() == kDeviceStateIdle) {
                            app.WakeWordInvoke(arg->msg);
                            break;
                        }
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                    arg->flag->store(false);
                    delete arg;
                    vTaskDelete(nullptr);
                }, "anom_notify", 4096, a, 3, nullptr);
            });

        RegisterSkyStarLinkMcpTools(&SkyStarLink::GetInstance());
#endif
#ifdef CONFIG_ENABLE_AMAP_ROUTE
        RegisterAmapRouteMcpTools();
#endif
    }

    void InitializeSmsModule() {
        auto& sms = A7670Sms::GetInstance();
        sms.Initialize(A7670_UART_NUM, A7670_TX_PIN, A7670_RX_PIN, A7670_BAUD_RATE);
        sms.RegisterOnNewSms([](const A7670Sms::SmsMessage& msg) {
            (void)msg;
            // 设备非 idle 时直接 WakeWordInvoke 会掐掉当前对话，
            // 起一个 watcher 任务等到回 idle 再唤醒；同时刻只跑一个。
            static std::atomic<bool> watcher_running{false};
            bool expected = false;
            if (!watcher_running.compare_exchange_strong(expected, true)) {
                ESP_LOGI(TAG, "SMS notify watcher already pending, skip duplicate");
                return;
            }
            xTaskCreate([](void* arg) {
                auto* flag = static_cast<std::atomic<bool>*>(arg);
                auto& app = Application::GetInstance();
                bool fired = false;
                bool nudged = false;
                // 总计最多 30 秒：前 5 秒留给用户自然结束当前对话；之后强制关停 listening 让设备回 idle
                for (int i = 0; i < 150; ++i) {
                    auto state = app.GetDeviceState();
                    if (state == kDeviceStateIdle) {
                        ESP_LOGI(TAG, "SMS notify: device idle, invoking wake word");
                        app.WakeWordInvoke("收到新短信");
                        fired = true;
                        break;
                    }
                    if (i == 25 && state == kDeviceStateListening && !nudged) {
                        // 5 秒还在 listening：用户对话已结束但 session 没关，主动 nudge 一下
                        ESP_LOGW(TAG, "SMS notify: listening stalled 5s, nudging close");
                        app.WakeWordInvoke("收到新短信");  // 仅关闭通道，不会真触发提示
                        nudged = true;
                    }
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                if (!fired) {
                    ESP_LOGW(TAG, "SMS notify: timed out, no prompt shown");
                }
                flag->store(false);
                vTaskDelete(nullptr);
            }, "sms_notify", 3072, &watcher_running, 3, nullptr);
        });
        RegisterA7670SmsMcpTools(&sms);
        ESP_LOGI(TAG, "A7670 SMS tools registered");
    }

public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiBoard);
