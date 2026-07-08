#ifndef A7670_SMS_H
#define A7670_SMS_H

#include <string>
#include <functional>
#include <mutex>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

class A7670Sms {
public:
    struct SmsMessage {
        std::string sender;
        std::string content;
        std::string timestamp;
    };

    using OnNewSmsCallback = std::function<void(const SmsMessage& msg)>;

    static A7670Sms& GetInstance() {
        static A7670Sms instance;
        return instance;
    }

    bool Initialize(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud_rate);

    bool SendSms(const std::string& recipient_utf8, const std::string& content_utf8);

    // 取走最近一条主动上报但未消费的短信（被 LLM 工具读走后清空）
    bool TakePendingIncoming(SmsMessage& out);

    void RegisterOnNewSms(OnNewSmsCallback cb) { on_new_sms_ = std::move(cb); }

    static std::string Utf8ToUcs2Hex(const std::string& utf8);
    static std::string Ucs2HexToUtf8(const std::string& hex);

private:
    A7670Sms() = default;
    ~A7670Sms() = default;
    A7670Sms(const A7670Sms&) = delete;
    A7670Sms& operator=(const A7670Sms&) = delete;

    bool SendAt(const std::string& cmd, std::string& response,
                int timeout_ms = 2000, const std::string& success_token = "OK");
    bool WaitPrompt(int timeout_ms);
    void RxTaskLoop();
    void InitTaskLoop();
    void DispatchLine(const std::string& line);

    static void RxTaskEntry(void* arg) {
        static_cast<A7670Sms*>(arg)->RxTaskLoop();
    }
    static void InitTaskEntry(void* arg) {
        static_cast<A7670Sms*>(arg)->InitTaskLoop();
    }

    uart_port_t uart_port_ = UART_NUM_1;
    int user_baud_ = 115200;
    bool ready_ = false;
    bool initialized_ = false;

    std::mutex cmd_mutex_;
    QueueHandle_t line_queue_ = nullptr;
    OnNewSmsCallback on_new_sms_;
    TaskHandle_t rx_task_ = nullptr;

    // +CMT URC 是两行的："+CMT: "<oa_hex>","",<scts>" 之后跟 body_hex 一行
    bool pending_cmt_ = false;
    SmsMessage pending_msg_;

    SmsMessage pending_incoming_;
    bool has_pending_incoming_ = false;
};

#endif
