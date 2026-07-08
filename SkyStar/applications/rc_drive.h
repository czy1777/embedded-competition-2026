/*
 * rc_drive.h
 * 电机控速状态机 — 多源仲裁（nRF24 摇杆 / LVGL 面板）+ 跌倒停止 + 坡度自适应
 *
 * 上层只负责 INIT_APP_EXPORT 自动注册 rc_drive_init, 业务调用通常不需要包含此头文件.
 * LVGL 侧通过 extern 的 volatile 全局与本模块交换数据 (g_ui_speed_x10 / g_ui_go_flag / 只读 g_ctrl_mode / g_enable_output).
 * 调参集中在本文件的宏, 改完重编即生效.
 */
#ifndef APPLICATIONS_RC_DRIVE_H_
#define APPLICATIONS_RC_DRIVE_H_

#include <stdint.h>

/* ---- 控制模式 ---- */
typedef enum {
    MODE_RC         = 0,   /* nRF24 摇杆权威 */
    MODE_NORMAL     = 1,   /* LVGL 面板 g_ui_speed_x10 权威, 需 g_enable_output=1 才实际下发 */
    MODE_TRANSITION = 2,   /* 软件层把 last_cmd 拖到 0, 防跳变 */
    MODE_FALL_HOLD  = 3,   /* 跌倒锁定, 两轮 0, 5s 后转 PAUSED */
} ctrl_mode_t;

/* ---- 调参区 ---- */

/* 摇杆死区 (中位 1500 上下), 防止手抖触发 */
#define RC_DEAD_ZONE                60

/* 电机方向补偿. 两个轮镜像安装时通常有一个物理方向反的,
 * 上电点击 START 观察哪个轮向后转, 把那一侧从 +1 改成 -1 即可.
 * 对 RC / NORMAL / TRANSITION 三种模式都生效. */
#define RC_LEFT_SIGN               (+1)
#define RC_RIGHT_SIGN              (-1)

/* NORMAL 模式最终 RPM 上限 (2205 RPM ≈ 1.5 km/h, 硬件安全空间) */
#define RC_MAX_RPM_DEFAULT          2205

/* RC 摇杆模式内部 RPM 上限, 沿用老年助行器步行速度上限.
 * 传动 i=50, 轮径 180mm; 1470 RPM ≈ 1.0 km/h. */
#define RC_MAX_RPM_JOYSTICK         1470
/* 兼容别名, 老代码里的 RC_MAX_RPM 现仅在 mix_diff 内部使用 */
#define RC_MAX_RPM                  RC_MAX_RPM_JOYSTICK

/* EMM 加速度参数. 手册公式: (256-acc)*50us = 1RPM 加速时间.
 * 150 在 RC_MAX_RPM_JOYSTICK=1470 下起步约 1.5s, 跟手且不冲. */
#define RC_ACC                      150

/* 跌倒制动 acc — 更激进, 目标 ~0.3s 内到零.
 * acc 越大 (越接近 255), 每 RPM 步进时间 (256-acc)*50us 越短.
 * FALL_ACC=230 相对 acc=150 快约 4x. 若手感不到位, 可提到 240~250
 * 或直接改用 Emm_StopNow (0xFE 0x98, 硬停). */
#define FALL_ACC                    230

/* 跌倒事件的 "新鲜" 窗口, 与 ui_main.c fall banner 使用同一预谓词 */
#define FALL_FRESH_MS               5000

/* TRANSITION 每周期把 last_cmd 幅值减去这么多 (RPM) */
#define RAMP_STEP_RPM               150

/* TRANSITION 完成判据: last_cmd 都为 0 连续 N 个周期 */
#define TRANSITION_SETTLE_CYCLES    3

/* 坡度自适应系数 (x100). 100 = 不变, 125 = +25%, 70 = -30% */
#define SLOPE_UPHILL_FACTOR_X100    125
#define SLOPE_DOWNHILL_FACTOR_X100  70
#define SLOPE_FLAT_FACTOR_X100      100
/* 一阶 LPF 位移: (target - cur) >> N. N=2 即 α=1/4, τ ≈ 4 个周期 ≈ 400ms */
#define SLOPE_LPF_SHIFT             2

/* 差速转向 (NORMAL 模式, 通过薄膜按键触发).
 * 内侧轮减速到 base_rpm * TURN_INNER_FACTOR_X100 / 100, 外侧轮保持 base_rpm.
 * 前驱后向轮结构下, 内侧轮变慢自然绕外侧轮弧形转向.
 * 30 = 内轮 30% 速度, 转弯半径较紧. 若太急可提到 50~60. */
#define TURN_INNER_FACTOR_X100      30
/* LPF 位移, 同 SLOPE_LPF_SHIFT. τ ≈ 400ms, 转向过渡平滑, 不猛跳. */
#define TURN_LPF_SHIFT              2

/* 失联判定阈值 (ms). 比 nrf24l01_app.c 的 1500ms 扫频阈值严格;
 * 超过该时长没有新包就把目标速度归零, 让 EMM 按 acc 渐停.
 * 注意: 不调 Emm_StopNow, 急停对老人重心不友好. */
#define RC_LINK_TIMEOUT_MS          300

/* 控制周期 (ms). Emm_SendCommand 内部 mdelay 50ms, 两个电机依次发即 100ms 一轮 */
#define RC_PERIOD_MS                100

/* ---- LVGL 侧共享变量 (rc_drive.c 定义, ui_main.c 引用) ---- */

/* UI 目标速度镜像, 单位 x10 km/h. 范围与 SPEED_MIN/SPEED_MAX 一致 (5..15). */
extern volatile int16_t g_ui_speed_x10;

/* GO 按钮 latch, 0=stop 1=go. UI 按钮点击时翻转 (仅 NORMAL 生效). */
extern volatile uint8_t g_ui_go_flag;

/* 只读: 当前控制模式 (ctrl_mode_t). UI 用来刷 mode 指示标签. */
extern volatile uint8_t g_ctrl_mode;

/* 只读: 输出使能. UI 用来刷 GO 按钮的 START/STOP 标签.
 * FALL_HOLD 结束 / TRANSITION 落到 NORMAL 时状态机会强制清零, 逼用户重按 GO. */
extern volatile uint8_t g_enable_output;

/* ---- 自动注册函数 ---- */
int rc_drive_init(void);

#endif /* APPLICATIONS_RC_DRIVE_H_ */
