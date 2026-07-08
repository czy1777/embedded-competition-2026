#include "a7670_sms_mcp_tool.h"
#include "a7670_sms.h"
#include "mcp_server.h"

#include <cJSON.h>

namespace {

cJSON* SmsToJson(const A7670Sms::SmsMessage& m) {
    cJSON* j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "sender",  m.sender.c_str());
    cJSON_AddStringToObject(j, "content", m.content.c_str());
    cJSON_AddStringToObject(j, "time",    m.timestamp.c_str());
    return j;
}

}  // namespace

void RegisterA7670SmsMcpTools(A7670Sms* sms) {
    if (sms == nullptr) return;
    auto& srv = McpServer::GetInstance();

    srv.AddTool(
        "self.sms.send",
        "通过 4G 模组发送短信。当用户说 \"给某人发短信\"、\"发个信息给\"、\"短信通知\" 等意图时调用。"
        "recipient 为收件人手机号（11 位国内号或 +86 开头国际号，不要带空格或破折号）。"
        "content 为短信正文，支持中文。返回 true 表示已成功提交到运营商网络。",
        PropertyList({
            Property("recipient", kPropertyTypeString),
            Property("content",   kPropertyTypeString),
        }),
        [sms](const PropertyList& props) -> ReturnValue {
            auto r = props["recipient"].value<std::string>();
            auto c = props["content"].value<std::string>();
            return sms->SendSms(r, c);
        });

    srv.AddTool(
        "self.sms.read_pending",
        "读取最近一条主动上报但尚未朗读的新短信。"
        "当用户被告知收到新短信、并要求 \"读一下\"、\"念出来\"、\"是什么内容\" 时调用。"
        "返回 JSON：{sender, content, time}；调用后该短信将被标记为已读。"
        "如果没有待处理短信，返回 {\"empty\":true}，请告诉用户没有新短信。",
        PropertyList(),
        [sms](const PropertyList&) -> ReturnValue {
            A7670Sms::SmsMessage m;
            if (!sms->TakePendingIncoming(m)) {
                return std::string("{\"empty\":true}");
            }
            return SmsToJson(m);
        });
}
