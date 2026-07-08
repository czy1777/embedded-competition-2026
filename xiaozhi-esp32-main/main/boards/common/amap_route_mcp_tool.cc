#include "amap_route_mcp_tool.h"

#ifdef CONFIG_ENABLE_AMAP_ROUTE

#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <memory>
#include <cmath>

#include <esp_log.h>
#include <esp_timer.h>
#include <cJSON.h>

#include "mcp_server.h"
#include "board.h"
#include "http.h"

#ifdef CONFIG_ENABLE_SKYSTAR_LINK
#include "skystar_link.h"
#endif

#define TAG "AmapRoute"

namespace {

// --------------------- URL encode (RFC 3986) ---------------------
std::string UrlEncode(const std::string& s) {
    std::string out;
    out.reserve(s.size() * 3);
    char buf[4];
    for (unsigned char c : s) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            out.push_back((char)c);
        } else {
            std::snprintf(buf, sizeof(buf), "%%%02X", c);
            out.append(buf);
        }
    }
    return out;
}

// --------------------- HTTP GET (复用 keep-alive 连接) ---------------------
bool HttpGet(Http* http, const std::string& url, std::string& body) {
    if (!http) return false;
    if (!http->Open("GET", url)) {
        ESP_LOGE(TAG, "GET open failed: %s", url.c_str());
        return false;
    }
    int status = http->GetStatusCode();
    if (status != 200) {
        ESP_LOGE(TAG, "GET status %d: %s", status, url.c_str());
        http->Close();
        return false;
    }
    body = http->ReadAll();
    http->Close();
    return true;
}

// --------------------- cJSON helpers ---------------------
struct CJsonHolder {
    cJSON* root = nullptr;
    explicit CJsonHolder(cJSON* p) : root(p) {}
    ~CJsonHolder() { if (root) cJSON_Delete(root); }
    CJsonHolder(const CJsonHolder&) = delete;
    CJsonHolder& operator=(const CJsonHolder&) = delete;
};

std::string GetStr(cJSON* obj, const char* key) {
    cJSON* v = cJSON_GetObjectItem(obj, key);
    if (cJSON_IsString(v) && v->valuestring) return v->valuestring;
    return "";
}

double GetNum(cJSON* obj, const char* key, double def = 0.0) {
    cJSON* v = cJSON_GetObjectItem(obj, key);
    if (cJSON_IsNumber(v)) return v->valuedouble;
    if (cJSON_IsString(v) && v->valuestring) return std::atof(v->valuestring);
    return def;
}

// --------------------- LatLng ---------------------
struct LatLng {
    double lng = 0.0;
    double lat = 0.0;
    bool valid = false;
};

// --------------------- 策略参数 ---------------------
// 整次工具调用的总耗时上限(ms)。服务端 MCP 调用硬超时为 10s，本地预算压到 5s
// 即可保证设备永远先于服务端返回，从而避免长耗时 HTTPS 占用 WiFi/CPU 时
// 影响 TTS 音频接收与播放。
constexpr int kRouteBudgetMs = 5000;
// 步行距离上限(km)。超过此距离则不再请求步行规划接口，直接给出友好提示。
constexpr double kMaxWalkingKm = 8.0;

// 大圆距离(球面距离)，单位 km。
double GreatCircleKm(const LatLng& a, const LatLng& b) {
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
    constexpr double kEarthKm = 6371.0;
    double dLat = (b.lat - a.lat) * kDeg2Rad;
    double dLng = (b.lng - a.lng) * kDeg2Rad;
    double sa = std::sin(dLat * 0.5);
    double sl = std::sin(dLng * 0.5);
    double h = sa * sa +
               std::cos(a.lat * kDeg2Rad) * std::cos(b.lat * kDeg2Rad) * sl * sl;
    return 2.0 * kEarthKm * std::asin(std::sqrt(h));
}

// --------------------- AMap geocode ---------------------
// 把地址文本转成经纬度。失败时返回 valid=false。
LatLng AmapGeocode(Http* http, const std::string& address) {
    LatLng result;
    std::string url = "https://restapi.amap.com/v3/geocode/geo?key=";
    url += CONFIG_AMAP_API_KEY;
    url += "&address=" + UrlEncode(address);

    std::string resp;
    if (!HttpGet(http, url, resp)) return result;

    cJSON* json = cJSON_Parse(resp.c_str());
    if (!json) {
        ESP_LOGE(TAG, "geocode parse fail");
        return result;
    }
    CJsonHolder holder(json);
    if (GetStr(json, "status") != "1") {
        ESP_LOGE(TAG, "geocode fail: %s", GetStr(json, "info").c_str());
        return result;
    }
    cJSON* arr = cJSON_GetObjectItem(json, "geocodes");
    if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) == 0) {
        ESP_LOGW(TAG, "geocode no result for %s", address.c_str());
        return result;
    }
    cJSON* first = cJSON_GetArrayItem(arr, 0);
    std::string loc = GetStr(first, "location");  // "lng,lat"
    auto comma = loc.find(',');
    if (comma == std::string::npos) return result;
    result.lng = std::atof(loc.substr(0, comma).c_str());
    result.lat = std::atof(loc.substr(comma + 1).c_str());
    result.valid = true;
    return result;
}

// --------------------- AMap walking direction ---------------------
struct WalkingRoute {
    bool valid = false;
    int distance_m = 0;
    int duration_s = 0;
    std::vector<std::string> instructions;  // 各 step 的 instruction
    std::string error;
};

WalkingRoute AmapWalkingRoute(Http* http, const LatLng& origin, const LatLng& dest) {
    WalkingRoute r;
    char ob[32], db[32];
    std::snprintf(ob, sizeof(ob), "%.6f,%.6f", origin.lng, origin.lat);
    std::snprintf(db, sizeof(db), "%.6f,%.6f", dest.lng, dest.lat);

    std::string url = "https://restapi.amap.com/v3/direction/walking?key=";
    url += CONFIG_AMAP_API_KEY;
    url += "&origin=";
    url += ob;
    url += "&destination=";
    url += db;

    std::string resp;
    if (!HttpGet(http, url, resp)) {
        r.error = "网络请求失败";
        return r;
    }

    cJSON* json = cJSON_Parse(resp.c_str());
    if (!json) {
        r.error = "返回解析失败";
        return r;
    }
    CJsonHolder holder(json);
    if (GetStr(json, "status") != "1") {
        r.error = GetStr(json, "info");
        if (r.error.empty()) r.error = "高德接口错误";
        return r;
    }
    cJSON* route = cJSON_GetObjectItem(json, "route");
    cJSON* paths = cJSON_GetObjectItem(route, "paths");
    if (!cJSON_IsArray(paths) || cJSON_GetArraySize(paths) == 0) {
        r.error = "未找到步行路线";
        return r;
    }
    cJSON* path = cJSON_GetArrayItem(paths, 0);
    r.distance_m = (int)GetNum(path, "distance");
    r.duration_s = (int)GetNum(path, "duration");

    cJSON* steps = cJSON_GetObjectItem(path, "steps");
    if (cJSON_IsArray(steps)) {
        int n = cJSON_GetArraySize(steps);
        for (int i = 0; i < n; ++i) {
            cJSON* s = cJSON_GetArrayItem(steps, i);
            std::string ins = GetStr(s, "instruction");
            if (!ins.empty()) r.instructions.push_back(std::move(ins));
        }
    }
    r.valid = true;
    return r;
}

// --------------------- 把路径渲染成中文文本 ---------------------
std::string FormatRoute(const std::string& origin_desc,
                        const std::string& dest_desc,
                        const WalkingRoute& r) {
    if (!r.valid) {
        std::string s = "路径规划失败";
        if (!r.error.empty()) s += "：" + r.error;
        return s;
    }
    std::string s = "从" + origin_desc + "步行到" + dest_desc;
    s += "，全程约 ";
    if (r.distance_m >= 1000) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.1f 公里",
                      r.distance_m / 1000.0);
        s += buf;
    } else {
        s += std::to_string(r.distance_m) + " 米";
    }
    s += "，预计用时 ";
    int minutes = (r.duration_s + 30) / 60;
    if (minutes <= 0) minutes = 1;
    s += std::to_string(minutes) + " 分钟。";

    // 取前 5 条 step 简要播报，避免太长
    int show = std::min<int>(5, (int)r.instructions.size());
    if (show > 0) {
        s += "主要路线：";
        for (int i = 0; i < show; ++i) {
            s += std::to_string(i + 1) + ". " + r.instructions[i] + "；";
        }
        if ((int)r.instructions.size() > show) {
            s += "之后到达终点。";
        }
    }
    return s;
}

}  // namespace

// --------------------- MCP tool registration ---------------------
void RegisterAmapRouteMcpTools() {
    static const char* kAmapKey = CONFIG_AMAP_API_KEY;
    if (kAmapKey[0] == '\0') {
        ESP_LOGW(TAG, "AMAP_API_KEY 未配置，跳过路径规划工具注册");
        return;
    }

    auto& mcp = McpServer::GetInstance();
    mcp.AddTool(
        "self.map.plan_walking_route",
        "规划步行路线。\n"
        "用途：用户用中文说出想去的地方时调用本工具。\n"
        "参数：\n"
        "  destination (必填)：目的地名称或地址，如\"北京西站\"、\"国贸三期\"。\n"
        "  origin (可选)：起点名称或地址；不填则使用设备当前位置（基于 GPS）。\n"
        "返回：包含距离、用时、关键转向指令的中文文本，AI 应直接朗读给用户。",
        PropertyList({
            Property("destination", kPropertyTypeString),
            Property("origin", kPropertyTypeString, std::string(""))
        }),
        [](const PropertyList& props) -> ReturnValue {
            std::string dest_text = props["destination"].value<std::string>();
            std::string origin_text = props["origin"].value<std::string>();
            if (dest_text.empty()) {
                return std::string("请告诉我您想去哪里。");
            }

            int64_t t0 = esp_timer_get_time();
            auto remaining_ms = [t0]() -> int {
                int64_t used = (esp_timer_get_time() - t0) / 1000;
                int64_t left = (int64_t)kRouteBudgetMs - used;
                return left > 0 ? (int)left : 0;
            };

            // 整个工具调用共享一个 HTTP 连接，复用 TLS 握手
            auto http = Board::GetInstance().GetNetwork()->CreateHttp(0);
            if (!http) {
                return std::string("网络模块未就绪。");
            }
            http->SetHeader("User-Agent", "xiaozhi-esp32");
            http->SetKeepAlive(true);

            LatLng origin;
            std::string origin_desc;
            if (!origin_text.empty()) {
                http->SetTimeout(remaining_ms());
                origin = AmapGeocode(http.get(), origin_text);
                origin_desc = origin_text;
                if (!origin.valid) {
                    if (remaining_ms() == 0) {
                        return std::string("网络繁忙，请稍后再试。");
                    }
                    return std::string("未找到起点：") + origin_text +
                           "，请换一个更具体的说法。";
                }
            } else {
#ifdef CONFIG_ENABLE_SKYSTAR_LINK
                uint32_t age_ms = 0;
                if (SkyStarLink::GetInstance().GetLatestLocation(
                        origin.lat, origin.lng, age_ms)) {
                    origin.valid = true;
                    if (age_ms > 60000) {
                        ESP_LOGW(TAG, "GPS data %u ms old", (unsigned)age_ms);
                    }
                }
#endif
                if (!origin.valid) {
                    std::string home = CONFIG_AMAP_HOME_LOCATION;
                    auto comma = home.find(',');
                    if (comma != std::string::npos) {
                        origin.lng = std::atof(home.substr(0, comma).c_str());
                        origin.lat = std::atof(home.substr(comma + 1).c_str());
                        origin.valid = (origin.lng != 0.0 && origin.lat != 0.0);
                        if (origin.valid) origin_desc = "默认起点";
                    }
                }
                if (!origin.valid) {
                    return std::string("还没收到 GPS 定位，请稍等片刻或在指令中说出起点。");
                }
                if (origin_desc.empty()) origin_desc = "当前位置";
            }

            if (remaining_ms() == 0) {
                return std::string("网络繁忙，请稍后再试。");
            }

            http->SetTimeout(remaining_ms());
            LatLng dest = AmapGeocode(http.get(), dest_text);
            if (!dest.valid) {
                if (remaining_ms() == 0) {
                    return std::string("网络繁忙，请稍后再试。");
                }
                return std::string("未找到目的地：") + dest_text +
                       "，请换一个更具体的说法。";
            }

            // 直线距离短路：太远不再走步行规划，避免长耗时 HTTPS 抖动 TTS。
            double km = GreatCircleKm(origin, dest);
            ESP_LOGI(TAG, "great-circle distance %.2f km", km);
            if (km > kMaxWalkingKm) {
                char buf[160];
                std::snprintf(buf, sizeof(buf),
                              "从%s到%s直线距离约 %.1f 公里，步行不太合适，"
                              "建议改用公交、驾车或骑行。",
                              origin_desc.c_str(), dest_text.c_str(), km);
                return std::string(buf);
            }

            if (remaining_ms() == 0) {
                return std::string("网络繁忙，请稍后再试。");
            }
            http->SetTimeout(remaining_ms());
            WalkingRoute route = AmapWalkingRoute(http.get(), origin, dest);
            int elapsed_ms = (int)((esp_timer_get_time() - t0) / 1000);
            ESP_LOGI(TAG, "plan_walking_route done in %d ms", elapsed_ms);
            if (!route.valid && remaining_ms() == 0) {
                return std::string("网络繁忙，请稍后再试。");
            }
            return FormatRoute(origin_desc, dest_text, route);
        });
    ESP_LOGI(TAG, "AMap walking route MCP tool registered");
}

#endif  // CONFIG_ENABLE_AMAP_ROUTE
