# 基于 RT-Thread 的多主控协同助行装置与多模态感知系统

2026 年嵌入式系统设计竞赛（RT-Thread 赛道）参赛作品。

面向养老助行与健康监护场景，将**实时运动控制、多模态感知与边缘—云协同分析**融为一体。系统以两块运行 RT-Thread 标准版的 STM32 主控为核心，配合 ESP32 穿戴端、嘉楠 K230 视觉节点与 Node.js 云端后台，四端通过统一的 **12 字节定长 BLE 帧协议**互联，构建了从"感知—控制—上云—分析—交互"的完整闭环。

## 系统架构

```
   ┌──────────────┐   BLE(12B帧)   ┌──────────────┐   MQTT    ┌──────────────┐
   │  Smart_Watch │──────────────▶│    SPARK     │──────────▶│     web1     │
   │  ESP32 穿戴端 │  健康/跌倒     │  STM32 网关   │  遥测上云  │  Node.js 云端 │
   └──────────────┘               └──────┬───────┘           └──────────────┘
                                          │ BLE 双向透传               │ DeepSeek
   ┌──────────────┐   UART        ┌──────┴───────┐               步态/健康分析
   │     K230     │──────────────▶│   SkyStar    │
   │  视觉关键点   │  下肢6关键点   │  STM32 主控   │──UART──▶ xiaozhi 语音交互
   └──────────────┘               └──────────────┘
                                    电机驱动 / LVGL 触摸屏
```

## 目录结构

| 目录 | 平台 | 职责 |
|------|------|------|
| `SkyStar/` | STM32 + RT-Thread 标准版 | 主控：双闭环步进电机驱动、LVGL 触摸人机界面、K230 视觉接收、全系统 BLE 数据中枢、小智语音上行 |
| `SPARK/` | 星火 1 号 STM32F407 + RT-Thread | 传感网关：温湿度/姿态/GPS/PM2.5 采集，BLE 主机汇聚数据，经 MQTT 上云 |
| `Smart_Watch/` | ESP32 + ESP-IDF | 穿戴端：MPU6050 跌倒检测，MAX30102 心率/血氧，DS18B20 体温，BLE 上报 |
| `xiaozhi-esp32-main/` | ESP32 + ESP-IDF | 小智语音交互模块（基于开源项目二次开发），健康/环境/跌倒信息语音播报 |
| `K230/` | 嘉楠 K230 CanMV | 轻量姿态估计，实时提取下肢 6 个关键点用于步态分析 |
| `web1/` | Node.js | 云端后台：MQTT 订阅遥测、Socket.IO 实时推送、SQLite 持久化、调用大模型生成健康分析报告 |

## 核心特性

- **多主控协同架构**：双 RT-Thread STM32 分担实时控制与传感网关职能，充分利用多线程调度、设备驱动框架、FinSH 命令行与自动初始化机制。
- **惯性 + 视觉双通道感知**：穿戴 IMU 负责即时跌倒急停，K230 视觉负责细粒度步态量化（步频、左右对称性、膝关节活动度、平衡稳定性）。
- **统一通信协议**：四端采用同一套 12 字节定长 BLE 帧（`0xAA` 帧头 + type + 8 字节载荷 + XOR 校验 + `0x55` 帧尾），配合 K230 反向 36 字节长帧。
- **实时控制与安全策略**：电机控速状态机对摇杆/面板/差速三控制源进行仲裁，叠加跌倒急停与坡度自适应。
- **边缘—云协同**：STM32 侧负责实时控制与数据汇聚，云端负责计算密集的步态分析与大模型推理。

## 快速开始

各子模块相对独立，按需构建：

- **SkyStar / SPARK**（RT-Thread）：使用 RT-Thread Studio 导入，或在 [RT-Thread ENV](https://www.rt-thread.org/) 中执行 `scons`。SPARK 的 `rt-thread`/`libraries`/`packages` 由 ENV `pkgs --update` 自动拉取。
- **Smart_Watch / xiaozhi-esp32-main**（ESP-IDF）：`idf.py build flash monitor`。依赖组件由 `idf.py reconfigure` 自动获取。
- **web1**（Node.js）：`npm install`，复制 `.env.example` 为 `.env` 并按需填写 AI 相关配置，然后 `node server.js`。
- **K230**：将 `K230/person_keypoint_detect.py` 及模型拷贝至 K230 CanMV 文件系统运行。

## 配置说明

烧录 SPARK 前需在 `SPARK/applications/cloud_config.h` 中把 `CLOUD_WIFI_SSID` / `CLOUD_WIFI_PASSWORD` 改为你的实际 WiFi（仓库中为占位符）。云端 AI 分析默认为 mock，接入真实模型请在 `web1/.env` 中配置（该文件不入库）。

## 许可证

本仓库以 [MIT License](./LICENSE) 开源。其中 `xiaozhi-esp32-main/` 基于开源项目二次开发，请遵循其原始许可证。
