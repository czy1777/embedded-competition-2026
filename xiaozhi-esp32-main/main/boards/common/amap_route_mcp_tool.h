#ifndef AMAP_ROUTE_MCP_TOOL_H
#define AMAP_ROUTE_MCP_TOOL_H

#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_AMAP_ROUTE

// 注册高德步行路径规划 MCP 工具:
//   self.map.plan_walking_route(destination, [origin])
// 起点未指定时使用 Unwired Labs LocationAPI(WiFi 定位) 自动获取。
void RegisterAmapRouteMcpTools();

#endif  // CONFIG_ENABLE_AMAP_ROUTE

#endif  // AMAP_ROUTE_MCP_TOOL_H
