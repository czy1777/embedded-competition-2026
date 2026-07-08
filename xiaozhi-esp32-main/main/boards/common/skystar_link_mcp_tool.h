#ifndef SKYSTAR_LINK_MCP_TOOL_H
#define SKYSTAR_LINK_MCP_TOOL_H

#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_SKYSTAR_LINK

class SkyStarLink;

void RegisterSkyStarLinkMcpTools(SkyStarLink* link);

#endif  // CONFIG_ENABLE_SKYSTAR_LINK

#endif  // SKYSTAR_LINK_MCP_TOOL_H
