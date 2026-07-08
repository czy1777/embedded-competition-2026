#include "a7670_sms.h"

#include <cstring>
#include <cstdio>
#include <algorithm>

#include <esp_log.h>

#define TAG "A7670Sms"

#define A7670_RX_BUF_SIZE 2048
#define A7670_LINE_MAX    512
#define A7670_QUEUE_DEPTH 16

namespace {

struct LineEvent {
    char data[A7670_LINE_MAX];
    int  len;
};

bool ExtractQuoted(const std::string& s, int nth, std::string& out) {
    int count = 0;
    size_t i = 0;
    while (i < s.size()) {
        if (s[i] == '"') {
            size_t j = s.find('"', i + 1);
            if (j == std::string::npos) return false;
            if (count == nth) {
                out = s.substr(i + 1, j - i - 1);
                return true;
            }
            ++count;
            i = j + 1;
        } else {
            ++i;
        }
    }
    return false;
}

int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

}  // namespace

bool A7670Sms::Initialize(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud_rate) {
    if (initialized_) return ready_;
    initialized_ = true;
    uart_port_ = port;
    user_baud_ = baud_rate;

    uart_config_t cfg = {};
    cfg.baud_rate = baud_rate;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    if (uart_driver_install(port, A7670_RX_BUF_SIZE * 2, 0, 0, nullptr, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return false;
    }
    uart_param_config(port, &cfg);
    uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // RX 内部上拉：A7670 TXD 高电平约 2.1V，ESP32-S3 输入门限 ~2.475V，
    // 不开上拉可能识别不稳。
    gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY);

    line_queue_ = xQueueCreate(A7670_QUEUE_DEPTH, sizeof(LineEvent));
    if (!line_queue_) return false;

    xTaskCreate(&A7670Sms::RxTaskEntry,   "a7670_rx",   4096, this, 5, &rx_task_);
    xTaskCreate(&A7670Sms::InitTaskEntry, "a7670_init", 8192, this, 4, nullptr);
    return true;
}

void A7670Sms::InitTaskLoop() {
    // A7670 上电后通常 3-8 秒发出 RDY；先静等 3 秒避免吃掉早期乱码
    ESP_LOGI(TAG, "Waiting 3s for modem boot, then probing AT @ %d bps...", user_baud_);
    vTaskDelay(pdMS_TO_TICKS(3000));
    LineEvent stale;
    while (xQueueReceive(line_queue_, &stale, 0) == pdTRUE) {}

    std::string resp;
    bool ok = false;
    for (int i = 0; i < 15 && !ok; ++i) {
        if (SendAt("AT", resp, 1500)) { ok = true; break; }
        ESP_LOGW(TAG, "AT no resp, retry %d/15", i + 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!ok) {
        ESP_LOGE(TAG, "A7670 not responding @ %d bps. Check wiring/power/PWRKEY.", user_baud_);
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "AT OK @ %d bps", user_baud_);

    SendAt("ATE0", resp);
    if (!SendAt("AT+CPIN?", resp, 5000) || resp.find("READY") == std::string::npos) {
        ESP_LOGE(TAG, "SIM not ready: %s", resp.c_str());
        vTaskDelete(nullptr);
        return;
    }
    SendAt("AT+CSQ", resp, 2000);
    ESP_LOGI(TAG, "Signal: %s", resp.c_str());
    SendAt("AT+CREG?", resp, 3000);
    ESP_LOGI(TAG, "Registration: %s", resp.c_str());

    SendAt("AT+CMGF=1", resp);
    SendAt("AT+CSCS=\"UCS2\"", resp);
    SendAt("AT+CSMP=17,167,0,8", resp);
    SendAt("AT+CNMI=2,2,0,0", resp);

    ready_ = true;
    ESP_LOGI(TAG, "A7670 SMS ready");
    vTaskDelete(nullptr);
}

void A7670Sms::RxTaskLoop() {
    static char buf[A7670_LINE_MAX];
    int pos = 0;
    while (true) {
        uint8_t ch;
        int n = uart_read_bytes(uart_port_, &ch, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        if (ch == '\r') continue;
        if (ch == '\n') {
            if (pos > 0) {
                buf[pos] = 0;
                DispatchLine(std::string(buf, pos));
                pos = 0;
            }
            continue;
        }
        // ">" 提示符无换行，单独处理
        if (ch == '>' && pos == 0) {
            DispatchLine(">");
            continue;
        }
        if (pos < A7670_LINE_MAX - 1) {
            buf[pos++] = static_cast<char>(ch);
        } else {
            pos = 0;  // 行过长，丢弃
        }
    }
}

void A7670Sms::DispatchLine(const std::string& line) {
    ESP_LOGD(TAG, "RX: %s", line.c_str());

    // +CMT URC 是两行的：第一行收到后置位，第二行解为 UCS2 hex 正文
    if (pending_cmt_) {
        pending_cmt_ = false;
        pending_msg_.content = Ucs2HexToUtf8(line);
        pending_incoming_ = pending_msg_;
        has_pending_incoming_ = true;
        if (on_new_sms_) on_new_sms_(pending_msg_);
        return;
    }

    if (line.rfind("+CMT:", 0) == 0) {
        ESP_LOGI(TAG, "URC: %s", line.c_str());
        std::string oa_hex, scts;
        ExtractQuoted(line, 0, oa_hex);
        ExtractQuoted(line, 2, scts);
        pending_msg_.sender    = Ucs2HexToUtf8(oa_hex);
        pending_msg_.timestamp = scts;
        pending_msg_.content.clear();
        pending_cmt_ = true;
        return;
    }

    LineEvent ev;
    ev.len = std::min<int>(line.size(), A7670_LINE_MAX - 1);
    memcpy(ev.data, line.data(), ev.len);
    ev.data[ev.len] = 0;
    xQueueSend(line_queue_, &ev, 0);
}

bool A7670Sms::SendAt(const std::string& cmd, std::string& response,
                     int timeout_ms, const std::string& success_token) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    response.clear();

    LineEvent stale;
    while (xQueueReceive(line_queue_, &stale, 0) == pdTRUE) {}

    std::string out = cmd + "\r\n";
    uart_write_bytes(uart_port_, out.data(), out.size());
    ESP_LOGD(TAG, ">> %s", cmd.c_str());

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            ESP_LOGW(TAG, "timeout waiting for %s, got: %s",
                     success_token.c_str(), response.c_str());
            return false;
        }
        LineEvent ev;
        if (xQueueReceive(line_queue_, &ev, deadline - now) != pdTRUE) {
            return false;
        }
        std::string line(ev.data, ev.len);
        ESP_LOGD(TAG, "<< %s", line.c_str());

        if (line == cmd) continue;  // 回显
        if (line == "OK" && success_token == "OK") return true;
        if (line.rfind("ERROR", 0) == 0 ||
            line.rfind("+CME ERROR", 0) == 0 ||
            line.rfind("+CMS ERROR", 0) == 0) {
            response += line;
            return false;
        }
        if (!response.empty()) response += "\n";
        response += line;
        if (line == success_token) return true;
    }
}

bool A7670Sms::WaitPrompt(int timeout_ms) {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) return false;
        LineEvent ev;
        if (xQueueReceive(line_queue_, &ev, deadline - now) != pdTRUE) return false;
        if (ev.len > 0 && ev.data[0] == '>') return true;
    }
}

bool A7670Sms::SendSms(const std::string& recipient_utf8, const std::string& content_utf8) {
    if (!ready_) return false;

    std::string number_hex  = Utf8ToUcs2Hex(recipient_utf8);
    std::string content_hex = Utf8ToUcs2Hex(content_utf8);
    if (number_hex.empty() || content_hex.empty()) return false;

    std::lock_guard<std::mutex> lk(cmd_mutex_);
    LineEvent stale;
    while (xQueueReceive(line_queue_, &stale, 0) == pdTRUE) {}

    std::string head = "AT+CMGS=\"" + number_hex + "\"\r";
    uart_write_bytes(uart_port_, head.data(), head.size());

    if (!WaitPrompt(5000)) {
        ESP_LOGE(TAG, "no '>' prompt for CMGS");
        return false;
    }

    uart_write_bytes(uart_port_, content_hex.data(), content_hex.size());
    char ctrlz = 0x1A;
    uart_write_bytes(uart_port_, &ctrlz, 1);

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(60000);
    bool got_ref = false;
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) return false;
        LineEvent ev;
        if (xQueueReceive(line_queue_, &ev, deadline - now) != pdTRUE) return false;
        std::string line(ev.data, ev.len);
        if (line.rfind("+CMGS:", 0) == 0) { got_ref = true; continue; }
        if (line == "OK") return got_ref;
        if (line.rfind("ERROR", 0) == 0 ||
            line.rfind("+CMS ERROR", 0) == 0 ||
            line.rfind("+CME ERROR", 0) == 0) {
            ESP_LOGE(TAG, "CMGS failed: %s", line.c_str());
            return false;
        }
    }
}

bool A7670Sms::TakePendingIncoming(SmsMessage& out) {
    if (!has_pending_incoming_) return false;
    out = pending_incoming_;
    has_pending_incoming_ = false;
    return true;
}

// UTF-8 → UCS2 (BMP only, hex 大写)
std::string A7670Sms::Utf8ToUcs2Hex(const std::string& utf8) {
    std::string out;
    out.reserve(utf8.size() * 4);
    static const char hex[] = "0123456789ABCDEF";
    size_t i = 0;
    while (i < utf8.size()) {
        uint32_t cp = 0;
        uint8_t c = static_cast<uint8_t>(utf8[i]);
        int extra = 0;
        if (c < 0x80)             { cp = c;          extra = 0; }
        else if ((c & 0xE0)==0xC0){ cp = c & 0x1F;   extra = 1; }
        else if ((c & 0xF0)==0xE0){ cp = c & 0x0F;   extra = 2; }
        else if ((c & 0xF8)==0xF0){ cp = c & 0x07;   extra = 3; }
        else { return ""; }
        if (i + extra >= utf8.size()) return "";
        for (int k = 0; k < extra; ++k) {
            uint8_t cc = static_cast<uint8_t>(utf8[i + 1 + k]);
            if ((cc & 0xC0) != 0x80) return "";
            cp = (cp << 6) | (cc & 0x3F);
        }
        i += 1 + extra;
        if (cp > 0xFFFF) cp = 0x003F;  // 超出 BMP，替换为 '?'
        out.push_back(hex[(cp >> 12) & 0xF]);
        out.push_back(hex[(cp >>  8) & 0xF]);
        out.push_back(hex[(cp >>  4) & 0xF]);
        out.push_back(hex[(cp >>  0) & 0xF]);
    }
    return out;
}

std::string A7670Sms::Ucs2HexToUtf8(const std::string& hex) {
    std::string out;
    out.reserve(hex.size() / 2);
    if (hex.size() % 4 != 0) return out;
    for (size_t i = 0; i + 3 < hex.size(); i += 4) {
        int n0 = hex_nibble(hex[i]);
        int n1 = hex_nibble(hex[i+1]);
        int n2 = hex_nibble(hex[i+2]);
        int n3 = hex_nibble(hex[i+3]);
        if ((n0|n1|n2|n3) < 0) return "";
        uint32_t cp = (n0 << 12) | (n1 << 8) | (n2 << 4) | n3;
        if (cp < 0x80) {
            out.push_back(static_cast<char>(cp));
        } else if (cp < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
        }
    }
    return out;
}
