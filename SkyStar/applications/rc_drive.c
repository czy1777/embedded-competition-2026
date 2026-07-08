/*
 * rc_drive.c
 * 电机控速状态机 — 多源仲裁 (nRF24 摇杆 / LVGL 面板) + 跌倒停止 + 坡度自适应
 *
 * 状态机四种状态见 rc_drive.h ctrl_mode_t.
 * 单周期处理顺序 (100ms):
 *   1) 读遥测快照 (fall/slope)
 *   2) 更新 linked (nrf_pkt_cnt 增量)
 *   3) fall_fresh 优先: 强制 FALL_HOLD 并快速软刹
 *   4) 非 fall: 根据 linked 边沿触发 RC/NORMAL 之间的 TRANSITION
 *   5) TRANSITION: 软件把 last_cmd 拖到 0, 稳定 3 周期后按 linked 决定去 RC 还是 NORMAL
 *   6) 按 mode 计算 left/right target (RC 用 mix_diff, NORMAL 用 UI 值 + 坡度)
 *   7) split_signed + Emm_VelocityControl 下发
 *
 * 与 LVGL 的耦合走 volatile 全局, 匹配 nrf24l01 的 Remote/nrf_pkt_cnt 惯例.
 */
#include <rtthread.h>
#include <stdlib.h>
#include "rc_drive.h"
#include "Emm_driver.h"
#include "nrf24l01_app.h"
#include "uart_app.h"
#include "JYD-18.h"
#include "key_app.h"

/* main.c 里已 Create 并 Enable 的两个电机 */
extern Emm_Motor_t motor_test;
extern Emm_Motor_t motor_test2;

/* 物理左右映射. 万一上车试出来反了, 改这两个宏 (互换或反向) */
#define RC_LEFT_MOTOR    (&motor_test)
#define RC_RIGHT_MOTOR   (&motor_test2)

/* 摇杆中位 */
#define RC_MID           1500
/* 摇杆相对中位的最大幅度 (1000~2000 -> ±500) */
#define RC_RANGE         500

/* EMM_VelocityControl 的 dir 字段: 0/1 两个值.
 * 实际左右正负哪个是前进, 上车校核时通过反转电机接线或这两个宏微调. */
#define DIR_POS          0
#define DIR_NEG          1

/* ---- LVGL 侧共享 volatile 全局 (定义在这里, extern 在 rc_drive.h) ---- */
volatile int16_t g_ui_speed_x10   = 10;                   /* 默认 1.0 km/h */
volatile uint8_t g_ui_go_flag     = 0;
volatile uint8_t g_ctrl_mode      = (uint8_t)MODE_TRANSITION;
volatile uint8_t g_enable_output  = 0;

/* ---- file-static 状态 ---- */
static uint16_t g_max_rpm_runtime        = RC_MAX_RPM_DEFAULT;
static int      g_last_left_rpm_cmd      = 0;
static int      g_last_right_rpm_cmd     = 0;
static uint8_t  g_transition_zero_cycles = 0;
static int      g_slope_factor_x100      = SLOPE_FLAT_FACTOR_X100;
/* 差速转向 factor (x100), NORMAL 模式下随按键 LPF, 分别乘到左右轮 */
static int      g_left_factor_x100       = 100;
static int      g_right_factor_x100      = 100;

/* 调试注入 */
static rt_tick_t g_fall_inject_ts     = 0;
static rt_tick_t g_slope_inject_expire = 0;
static uint8_t   g_slope_inject_state = 0;

/* ---- 工具函数 ---- */

static inline void split_signed(int rpm_signed, uint8_t *dir, uint16_t *rpm)
{
    if (rpm_signed >= 0) {
        *dir = DIR_POS;
        *rpm = (uint16_t)rpm_signed;
    } else {
        *dir = DIR_NEG;
        *rpm = (uint16_t)(-rpm_signed);
    }
}

/* 把带符号 last_cmd 幅值向 0 拉近, 每次减 step */
static int slew_toward_zero(int cur, int step)
{
    if (cur > 0) {
        cur -= step;
        if (cur < 0) cur = 0;
    } else if (cur < 0) {
        cur += step;
        if (cur > 0) cur = 0;
    }
    return cur;
}

static const char *mode_name(uint8_t m)
{
    switch (m) {
        case MODE_RC:         return "RC";
        case MODE_NORMAL:     return "NORMAL";
        case MODE_TRANSITION: return "TRANSITION";
        case MODE_FALL_HOLD:  return "FALL_HOLD";
        default:              return "?";
    }
}

static void log_mode_change(uint8_t new_mode, const char *reason)
{
    rt_kprintf("[MODE] %s -> %s (%s)\n",
               mode_name(g_ctrl_mode), mode_name(new_mode),
               reason ? reason : "");
}

/* 摇杆值 -> 左右轮含符号 RPM (差速混合). RC 模式内部用, 上限为 JOYSTICK cap. */
static void mix_diff(int16_t pitch, int16_t yaw,
                     int *left_rpm_out, int *right_rpm_out)
{
    int fwd  = (int)pitch - RC_MID;
    int turn = (int)yaw   - RC_MID;

    if (fwd  >  -RC_DEAD_ZONE && fwd  < RC_DEAD_ZONE) fwd  = 0;
    if (turn > -RC_DEAD_ZONE && turn < RC_DEAD_ZONE) turn = 0;

    int left_raw  = fwd + turn;
    int right_raw = fwd - turn;

    int peak = abs(left_raw);
    if (abs(right_raw) > peak) peak = abs(right_raw);

    if (peak == 0) {
        *left_rpm_out  = 0;
        *right_rpm_out = 0;
        return;
    }

    int denom = (peak > RC_RANGE) ? peak : RC_RANGE;
    *left_rpm_out  = left_raw  * RC_MAX_RPM_JOYSTICK / denom;
    *right_rpm_out = right_raw * RC_MAX_RPM_JOYSTICK / denom;
}

/* ---- 主线程 ---- */

static void rc_drive_thread(void *p)
{
    uint32_t   last_pkt_cnt = 0;
    rt_tick_t  last_pkt_tick = rt_tick_get();
    uint8_t    linked = 0;
    uint8_t    prev_linked = 0;
    int        left_target = 0;
    int        right_target = 0;
    uint8_t    l_dir, r_dir;
    uint16_t   l_rpm, r_rpm;
    telemetry_t snap;
    (void)p;

    /* 开机默认进 TRANSITION, 300ms 后按 linked 落地 */
    g_ctrl_mode = (uint8_t)MODE_TRANSITION;
    g_transition_zero_cycles = 0;
    rt_kprintf("[MODE] boot -> TRANSITION\n");
    rt_kprintf("[RC] drive thread started, max=%dRPM (RC=%d) dead=%d period=%dms\n",
               (int)g_max_rpm_runtime, RC_MAX_RPM_JOYSTICK,
               RC_DEAD_ZONE, RC_PERIOD_MS);

    while (1) {
        /* 1) 遥测 */
        jdy18_get_telemetry(&snap);
        rt_tick_t now = rt_tick_get();

        /* 2) linked (无论 mode 都更新, FALL_HOLD 里也保持新鲜) */
        prev_linked = linked;
        if (nrf_pkt_cnt != last_pkt_cnt) {
            last_pkt_cnt = nrf_pkt_cnt;
            last_pkt_tick = now;
            if (!linked) rt_kprintf("[RC] link up\n");
            linked = 1;
        } else if (linked &&
                   (now - last_pkt_tick) > rt_tick_from_millisecond(RC_LINK_TIMEOUT_MS)) {
            linked = 0;
            rt_kprintf("[RC] link lost\n");
        }

        /* 3) 跌倒优先: 来自遥测 或 MSH 注入 */
        rt_bool_t fall_from_snap = snap.fall_active
                                && snap.fall_ts != 0
                                && (now - snap.fall_ts) < rt_tick_from_millisecond(FALL_FRESH_MS);
        rt_bool_t fall_from_inj  = g_fall_inject_ts != 0
                                && (now - g_fall_inject_ts) < rt_tick_from_millisecond(FALL_FRESH_MS);
        rt_bool_t fall_fresh = fall_from_snap || fall_from_inj;

        if (fall_fresh) {
            if (g_ctrl_mode != (uint8_t)MODE_FALL_HOLD) {
                log_mode_change((uint8_t)MODE_FALL_HOLD,
                                fall_from_inj ? "injected" : "fall detected");
                g_ctrl_mode = (uint8_t)MODE_FALL_HOLD;
                /* 边沿一次: 用 FALL_ACC 快速软刹 */
                Emm_VelocityControl(RC_LEFT_MOTOR,  DIR_POS, 0, FALL_ACC, false);
                Emm_VelocityControl(RC_RIGHT_MOTOR, DIR_POS, 0, FALL_ACC, false);
            } else {
                /* 保持: 每周期 heartbeat 0, 用 RC_ACC 避免频繁改 acc */
                Emm_VelocityControl(RC_LEFT_MOTOR,  DIR_POS, 0, RC_ACC, false);
                Emm_VelocityControl(RC_RIGHT_MOTOR, DIR_POS, 0, RC_ACC, false);
            }
            g_last_left_rpm_cmd = 0;
            g_last_right_rpm_cmd = 0;
            rt_thread_mdelay(RC_PERIOD_MS);
            continue;
        }

        /* 3b) FALL_HOLD 到期: 转入 TRANSITION, 强制清 GO 逼用户重按 */
        if (g_ctrl_mode == (uint8_t)MODE_FALL_HOLD) {
            log_mode_change((uint8_t)MODE_TRANSITION, "fall_fresh expired");
            g_ctrl_mode = (uint8_t)MODE_TRANSITION;
            g_enable_output = 0;
            g_ui_go_flag = 0;
            g_transition_zero_cycles = 0;
        }

        /* 4) linked 边沿触发 mode 切换 */
        if (g_ctrl_mode == (uint8_t)MODE_RC && !linked && prev_linked) {
            log_mode_change((uint8_t)MODE_TRANSITION, "link_lost");
            g_ctrl_mode = (uint8_t)MODE_TRANSITION;
            g_transition_zero_cycles = 0;
        } else if (g_ctrl_mode == (uint8_t)MODE_NORMAL && linked && !prev_linked) {
            log_mode_change((uint8_t)MODE_TRANSITION, "link_up");
            g_ctrl_mode = (uint8_t)MODE_TRANSITION;
            g_transition_zero_cycles = 0;
        }

        /* 5) TRANSITION 稳定判定: last_cmd 都为 0 连续 N 个周期 */
        if (g_ctrl_mode == (uint8_t)MODE_TRANSITION) {
            if (g_last_left_rpm_cmd == 0 && g_last_right_rpm_cmd == 0) {
                g_transition_zero_cycles++;
                if (g_transition_zero_cycles >= TRANSITION_SETTLE_CYCLES) {
                    /* 按当前 linked 状态决定去 RC 还是 NORMAL */
                    uint8_t settled_to = linked ? (uint8_t)MODE_RC : (uint8_t)MODE_NORMAL;
                    log_mode_change(settled_to, "settled");
                    g_ctrl_mode = settled_to;
                    if (settled_to == (uint8_t)MODE_NORMAL) {
                        /* 进 NORMAL 时清 GO, 逼用户按 START */
                        g_enable_output = 0;
                        g_ui_go_flag = 0;
                    }
                }
            } else {
                g_transition_zero_cycles = 0;
            }
        }

        /* 6) 按 mode 计算 target */
        if (g_ctrl_mode == (uint8_t)MODE_RC) {
            int16_t pitch = linked ? Remote.pitch : RC_MID;
            int16_t yaw   = linked ? Remote.yaw   : RC_MID;
            mix_diff(pitch, yaw, &left_target, &right_target);
        } else if (g_ctrl_mode == (uint8_t)MODE_NORMAL) {
            /* 6a) 更新坡度 factor (LPF) */
            uint8_t slope_state = snap.slope_state;
            if (g_slope_inject_expire != 0 && now < g_slope_inject_expire) {
                slope_state = g_slope_inject_state;
            }
            int target_slope = SLOPE_FLAT_FACTOR_X100;
            if (slope_state == 1)      target_slope = SLOPE_UPHILL_FACTOR_X100;
            else if (slope_state == 2) target_slope = SLOPE_DOWNHILL_FACTOR_X100;
            g_slope_factor_x100 += (target_slope - g_slope_factor_x100) >> SLOPE_LPF_SHIFT;

            /* 6b) 更新差速转向 factor (LPF).
             * 前驱后向轮结构下, 按左键让左轮减速 -> 绕左侧弧形转弯 */
            uint8_t l_held = g_key_left_held;
            uint8_t r_held = g_key_right_held;
            int target_left  = 100;
            int target_right = 100;
            if (l_held && !r_held) target_left  = TURN_INNER_FACTOR_X100;
            if (r_held && !l_held) target_right = TURN_INNER_FACTOR_X100;
            /* 双键同按视为不转向, 保持直行 */
            g_left_factor_x100  += (target_left  - g_left_factor_x100)  >> TURN_LPF_SHIFT;
            g_right_factor_x100 += (target_right - g_right_factor_x100) >> TURN_LPF_SHIFT;

            /* 6c) GO 门闩: 未启用则输出 0 */
            if (g_ui_go_flag && g_enable_output) {
                int base_rpm = (int)g_ui_speed_x10 * 147;               /* 5..15 -> 735..2205 */
                base_rpm = (base_rpm * g_slope_factor_x100) / 100;
                left_target  = (base_rpm * g_left_factor_x100)  / 100;
                right_target = (base_rpm * g_right_factor_x100) / 100;
                if (left_target  > (int)g_max_rpm_runtime) left_target  = (int)g_max_rpm_runtime;
                if (right_target > (int)g_max_rpm_runtime) right_target = (int)g_max_rpm_runtime;
                if (left_target  < 0) left_target  = 0;
                if (right_target < 0) right_target = 0;
            } else {
                left_target = right_target = 0;
            }
        } else if (g_ctrl_mode == (uint8_t)MODE_TRANSITION) {
            left_target  = slew_toward_zero(g_last_left_rpm_cmd,  RAMP_STEP_RPM);
            right_target = slew_toward_zero(g_last_right_rpm_cmd, RAMP_STEP_RPM);
        } else {
            /* FALL_HOLD 分支上面已经 continue 掉了, 这里保险起见 */
            left_target = right_target = 0;
        }

        /* 7) 下发 (先按 RC_*_SIGN 做方向补偿, 再拆成 dir + abs_rpm) */
        split_signed(left_target  * RC_LEFT_SIGN,  &l_dir, &l_rpm);
        split_signed(right_target * RC_RIGHT_SIGN, &r_dir, &r_rpm);
        Emm_VelocityControl(RC_LEFT_MOTOR,  l_dir, l_rpm, RC_ACC, false);
        Emm_VelocityControl(RC_RIGHT_MOTOR, r_dir, r_rpm, RC_ACC, false);

        /* 8) 缓存 last_cmd (供 TRANSITION 稳定判定和 rc_stat 观察) */
        g_last_left_rpm_cmd  = left_target;
        g_last_right_rpm_cmd = right_target;

        rt_thread_mdelay(RC_PERIOD_MS);
    }
}

int rc_drive_init(void)
{
    g_ui_speed_x10   = 10;
    g_ui_go_flag     = 0;
    g_ctrl_mode      = (uint8_t)MODE_TRANSITION;
    g_enable_output  = 0;

    rt_thread_t t = rt_thread_create("rc_drive",
                                     rc_drive_thread, RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2,
                                     5);
    if (t == RT_NULL) {
        rt_kprintf("[RC] thread create FAIL\n");
        return -RT_ERROR;
    }
    rt_thread_startup(t);
    return RT_EOK;
}
INIT_APP_EXPORT(rc_drive_init);

/* ---- MSH 诊断命令 ---- */

static void rc_stat(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("mode=%s enable=%u ui_speed_x10=%d go=%u\n",
               mode_name(g_ctrl_mode), g_enable_output,
               (int)g_ui_speed_x10, g_ui_go_flag);
    rt_kprintf("last_cmd L=%d R=%d slope_x100=%d turnL=%d turnR=%d\n",
               g_last_left_rpm_cmd, g_last_right_rpm_cmd,
               g_slope_factor_x100, g_left_factor_x100, g_right_factor_x100);
    rt_kprintf("keys L=%u R=%u  max_rpm=%u\n",
               g_key_left_held, g_key_right_held, g_max_rpm_runtime);
}
MSH_CMD_EXPORT(rc_stat, dump rc_drive state);

static void rc_inject_fall(int argc, char **argv)
{
    (void)argc; (void)argv;
    g_fall_inject_ts = rt_tick_get();
    rt_kprintf("[MODE] fall injected, will hold ~%dms\n", FALL_FRESH_MS);
}
MSH_CMD_EXPORT(rc_inject_fall, inject fall event for FALL_FRESH_MS);

static void rc_inject_slope(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("usage: rc_inject_slope <0=flat|1=uphill|2=downhill> [seconds=10]\n");
        return;
    }
    int s = atoi(argv[1]);
    int sec = (argc >= 3) ? atoi(argv[2]) : 10;
    if (s < 0 || s > 2) {
        rt_kprintf("bad slope state (expect 0/1/2)\n");
        return;
    }
    g_slope_inject_state  = (uint8_t)s;
    g_slope_inject_expire = rt_tick_get() + rt_tick_from_millisecond(sec * 1000);
    rt_kprintf("[MODE] slope inject state=%d for %ds\n", s, sec);
}
MSH_CMD_EXPORT(rc_inject_slope, inject slope state for testing);
