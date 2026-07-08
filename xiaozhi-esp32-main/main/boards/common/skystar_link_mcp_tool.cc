#include "skystar_link_mcp_tool.h"

#ifdef CONFIG_ENABLE_SKYSTAR_LINK

#include <cstdio>
#include <string>

#include <cJSON.h>
#include <esp_log.h>

#include "mcp_server.h"
#include "settings.h"
#include "skystar_link.h"
#include "a7670_sms.h"

namespace {
constexpr const char* kEmergencyNs       = "emergency";
constexpr const char* kKeyContactPhone   = "phone";
constexpr const char* kKeyContactName    = "name";
constexpr const char* TAG                = "SkyStarMcp";

// 构造一条短信正文: "【紧急】用户跌倒。时间 HH:MM:SS, 位置 lat,lng (高德链接)"
// 没 GPS / 没 UTC 时各自降级为 "未知"。注意 A7670 短信会编码到 UCS-2, 中文 OK。
std::string BuildFallSmsBody() {
    char buf[256];
    auto& link = SkyStarLink::GetInstance();

    SkyStarLink::HealthSnapshot snap{};
    uint32_t snap_age = 0;
    bool has_snap = link.GetHealthSnapshot(snap, snap_age);

    double lat = 0, lng = 0;
    uint32_t gps_age = 0;
    bool has_gps = link.GetLatestLocation(lat, lng, gps_age);

    // 时间用 UTC (Smart_Watch 来的); 没有就不写
    char time_part[32];
    if (has_snap && (snap.utc_h | snap.utc_m | snap.utc_s)) {
        std::snprintf(time_part, sizeof(time_part), "%02u:%02u:%02u UTC",
                      snap.utc_h, snap.utc_m, snap.utc_s);
    } else {
        std::snprintf(time_part, sizeof(time_part), "时间未知");
    }

    if (has_gps) {
        std::snprintf(buf, sizeof(buf),
                      "【紧急】用户跌倒。%s, 位置 %.6f,%.6f "
                      "(https://uri.amap.com/marker?position=%.6f,%.6f) 请尽快联系。",
                      time_part, lat, lng, lng, lat);
    } else {
        std::snprintf(buf, sizeof(buf),
                      "【紧急】用户跌倒。%s, GPS 暂未定位。请尽快联系。",
                      time_part);
    }
    return std::string(buf);
}
}  // namespace

void RegisterSkyStarLinkMcpTools(SkyStarLink* link) {
    if (link == nullptr) return;
    auto& srv = McpServer::GetInstance();

    // ---- self.emergency.set_contact ----
    srv.AddTool(
        "self.emergency.set_contact",
        "设置紧急联系人的手机号和昵称, 信息持久化保存。"
        "当用户说\"我妈手机是 138xxxx\"、\"把紧急联系人改成 ...\"等意图时调用。"
        "phone 必须是 11 位国内号或 +86 开头的国际号, 不要带空格或破折号。"
        "name 是可选的称呼 (例如 \"妈妈\"、\"老伴\"), 没说就传空字符串。"
        "返回 true 表示已写入 NVS。",
        PropertyList({
            Property("phone", kPropertyTypeString),
            Property("name",  kPropertyTypeString),
        }),
        [](const PropertyList& props) -> ReturnValue {
            auto phone = props["phone"].value<std::string>();
            auto name  = props["name"].value<std::string>();
            if (phone.empty()) return false;
            Settings s(kEmergencyNs, true);
            s.SetString(kKeyContactPhone, phone);
            s.SetString(kKeyContactName,  name);
            return true;
        });

    // ---- self.emergency.notify_fall ----
    srv.AddTool(
        "self.emergency.notify_fall",
        "用户确认跌倒需要联系紧急联系人后调用本工具, 会通过 4G 短信把"
        "\"用户跌倒\"+\"当前 GPS 位置\"发给已配置的紧急联系人。"
        "用户必须先确认意图 (例如对\"要不要联系您的紧急联系人?\"回答\"要/好/联系吧\") 才能调用。"
        "若用户拒绝, 不要调用。"
        "返回 JSON: {sent:bool, recipient:string, body:string} 或 {error:\"no_contact\"} (未配置联系人时)。",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            Settings s(kEmergencyNs, false);
            std::string phone = s.GetString(kKeyContactPhone, "");
            std::string name  = s.GetString(kKeyContactName,  "");
            if (phone.empty()) {
                cJSON* j = cJSON_CreateObject();
                cJSON_AddStringToObject(j, "error", "no_contact");
                return j;
            }
            std::string body = BuildFallSmsBody();
            bool ok = A7670Sms::GetInstance().SendSms(phone, body);
            ESP_LOGW(TAG, "emergency notify_fall to %s (%s): %s",
                     phone.c_str(), name.c_str(), ok ? "OK" : "FAIL");

            cJSON* j = cJSON_CreateObject();
            cJSON_AddBoolToObject  (j, "sent",      ok);
            cJSON_AddStringToObject(j, "recipient", phone.c_str());
            cJSON_AddStringToObject(j, "name",      name.c_str());
            cJSON_AddStringToObject(j, "body",      body.c_str());
            return j;
        });

    // ---- self.health.get_snapshot ----
    srv.AddTool(
        "self.health.get_snapshot",
        "查询用户当前的健康/环境/位置快照。"
        "当用户问\"我心率多少\"、\"现在多少度\"、\"空气怎么样\"、\"我在哪\"时调用。"
        "返回 JSON: {hr, spo2, body_temp_c, fall_active, pm25, env_temp_c, env_humi_pct, "
        "lat, lng, gps_fix, age_ms, gps_age_ms, watch_connected}。"
        "若 age_ms 大于 5000 说明数据可能不新鲜; 若无任何数据返回 {\"empty\":true}。",
        PropertyList(),
        [link](const PropertyList&) -> ReturnValue {
            SkyStarLink::HealthSnapshot s{};
            uint32_t snap_age = 0;
            bool has_snap = link->GetHealthSnapshot(s, snap_age);

            double lat = 0, lng = 0;
            uint32_t gps_age = 0;
            bool has_gps = link->GetLatestLocation(lat, lng, gps_age);

            if (!has_snap && !has_gps) {
                return std::string("{\"empty\":true}");
            }
            cJSON* j = cJSON_CreateObject();
            if (has_snap) {
                cJSON_AddNumberToObject(j, "hr",            s.hr);
                cJSON_AddNumberToObject(j, "spo2",          s.spo2);
                cJSON_AddNumberToObject(j, "body_temp_c",   s.body_temp_c);
                cJSON_AddBoolToObject  (j, "fall_active",   s.fall_active);
                cJSON_AddNumberToObject(j, "pm25",          s.pm25);
                cJSON_AddNumberToObject(j, "env_temp_c",    s.env_temp_c);
                cJSON_AddNumberToObject(j, "env_humi_pct",  s.env_humi_pct);
                cJSON_AddNumberToObject(j, "age_ms",        snap_age);
                cJSON_AddBoolToObject  (j, "watch_connected", s.watch_connected);
                cJSON_AddNumberToObject(j, "uptime_s",      s.uptime_s);
            }
            if (has_gps) {
                cJSON_AddNumberToObject(j, "lat",        lat);
                cJSON_AddNumberToObject(j, "lng",        lng);
                cJSON_AddBoolToObject  (j, "gps_fix",    true);
                cJSON_AddNumberToObject(j, "gps_age_ms", gps_age);
            } else {
                cJSON_AddBoolToObject  (j, "gps_fix",    false);
            }
            return j;
        });
}

#endif  // CONFIG_ENABLE_SKYSTAR_LINK
