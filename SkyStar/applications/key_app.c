/*
 * key_app.c
 * 薄膜按键手势控制 (无屏版, 取代原 LVGL 面板作为唯一人机输入):
 *   短按左   -> 加速 (+0.1 km/h)        短按右 -> 减速 (-0.1 km/h)
 *   长按左   -> 左转 (置 g_key_left_held) 长按右 -> 右转 (置 g_key_right_held)
 *   双键同按 -> 翻转电机 GO/STOP, 并取消当前单键手势, 直到双键都松开
 *
 * 与 rc_drive 的耦合 (全部走 volatile 全局, rc_drive 只在 NORMAL 模式消费):
 *   - 调速   写 g_ui_speed_x10                 (原 ui_main.c 速度盘的活)
 *   - GO/STOP 写 g_enable_output + g_ui_go_flag (原 ui_main.c GO 按钮的活)
 *   - 转向   写 g_key_left_held / g_key_right_held (差速转向, 语义不变)
 * rc_drive 不需要任何改动: 遥控器(nRF)未连通时系统稳定停在 NORMAL 模式, 按键直接生效.
 *
 * 手势判定 (10ms 采样, 30ms 防抖):
 *   - 长按转向: 按住 >= KEY_LONG_PRESS_MS 立即置位, 松手清零 (随按随转).
 *   - 短按调速: 松手瞬间, 且按下时长 < KEY_LONG_PRESS_MS, 且未参与双键手势 -> 触发一次.
 *   - 双键手势优先级最高, 触发后作废两键的进行中单键手势, 避免松手误触发调速/转向.
 *
 * RT-Thread BSP (drv_gpio.c) 启动时已开 GPIOB 时钟, stm32_pin_mode 自带 HAL_GPIO_Init,
 * 无需 CubeMX 预配置, 一次 rt_pin_mode 即可工作.
 * 日志走 rt_kprintf (RT-Thread console = UART1; 工程刻意禁用了 uart1_printf, 勿用).
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <drv_common.h>
#include "key_app.h"
#include "rc_drive.h"    /* g_ui_speed_x10 / g_ui_go_flag / g_enable_output */

/* ---- 调参区 ---- */

#define KEY_LEFT_PIN          GET_PIN(B, 8)
#define KEY_RIGHT_PIN         GET_PIN(B, 9)

#define KEY_PERIOD_MS         10    /* 扫描周期 */
#define KEY_DEBOUNCE_SAMPLES  3     /* 连续 N 次相同电平才认定稳定, N*周期 = 30ms */
#define KEY_LONG_PRESS_MS     400   /* 按住 >= 此时长判长按(转向); 短于此为短按(调速) */

#define KEY_SPEED_STEP        1     /* 每次短按调速步进 (x10 km/h) = 0.1 km/h */
#define KEY_SPEED_MIN         5     /* 0.5 km/h, 与原面板 SPEED_MIN 一致 */
#define KEY_SPEED_MAX         15    /* 1.5 km/h, 与原面板 SPEED_MAX / RC_MAX_RPM 一致 */

/* 按键接 GND + 内部上拉, 按下=低电平 */
#define KEY_LEVEL_PRESSED     0
#define KEY_LEVEL_RELEASED    1

/* ---- 转向输出 (供 rc_drive 读, extern 在 key_app.h) ---- */
volatile uint8_t g_key_left_held  = 0;
volatile uint8_t g_key_right_held = 0;

/* ---- 单键去抖状态 ---- */
typedef struct {
    const char *name;          /* "L" / "R", 仅用于日志 */
    rt_base_t   pin;
    rt_uint8_t  last_sample;    /* 上次原始电平 */
    rt_uint8_t  stable_level;   /* 去抖后稳定电平 */
    rt_uint8_t  same_count;     /* 连续相同采样计数 */
} key_debounce_t;

static key_debounce_t s_left  = { "L", KEY_LEFT_PIN,  KEY_LEVEL_RELEASED, KEY_LEVEL_RELEASED, 0 };
static key_debounce_t s_right = { "R", KEY_RIGHT_PIN, KEY_LEVEL_RELEASED, KEY_LEVEL_RELEASED, 0 };

/* 采样 + 去抖, 返回稳定后是否按下 (1/0) */
static rt_uint8_t debounce_pressed(key_debounce_t *k)
{
    rt_uint8_t sample = (rt_uint8_t)rt_pin_read(k->pin);

    if (sample == k->last_sample) {
        if (k->same_count < KEY_DEBOUNCE_SAMPLES) k->same_count++;
    } else {
        k->same_count  = 1;
        k->last_sample = sample;
    }
    if (k->same_count >= KEY_DEBOUNCE_SAMPLES) {
        k->stable_level = sample;
    }
    return (k->stable_level == KEY_LEVEL_PRESSED) ? 1 : 0;
}

/* ---- 动作 ---- */

static void speed_step(int delta_x10)
{
    int v = (int)g_ui_speed_x10 + delta_x10;
    if (v < KEY_SPEED_MIN) v = KEY_SPEED_MIN;
    if (v > KEY_SPEED_MAX) v = KEY_SPEED_MAX;
    g_ui_speed_x10 = (int16_t)v;
    rt_kprintf("[KEY] speed=%d.%d km/h\n", v / 10, v % 10);
}

static void motor_toggle(void)
{
    uint8_t on = g_enable_output ? 0 : 1;
    g_enable_output = on;
    g_ui_go_flag    = on;    /* 与原 ui_main.c GO 按钮一致: 两个门闩一起翻 */
    rt_kprintf("[KEY] motor %s\n", on ? "GO" : "STOP");
}

/* ---- 手势主线程 ---- */

static void key_thread(void *p)
{
    (void)p;

    rt_tick_t  l_press_tick = 0, r_press_tick = 0;  /* 按下起始 tick, 0 = 已作废 */
    rt_uint8_t l_prev = 0, r_prev = 0;              /* 上一轮稳态按下状态, 用于取边沿 */
    rt_uint8_t l_long = 0, r_long = 0;              /* 长按转向是否已触发 */
    rt_uint8_t chord_consumed = 0;                  /* 双键手势已触发, 等两键都松开 */
    const rt_tick_t long_ticks = rt_tick_from_millisecond(KEY_LONG_PRESS_MS);

    rt_kprintf("[KEY] gesture ctrl started: tap=speed hold=turn both=GO/STOP "
               "(period=%dms debounce=%dms long=%dms)\n",
               KEY_PERIOD_MS, KEY_PERIOD_MS * KEY_DEBOUNCE_SAMPLES, KEY_LONG_PRESS_MS);

    while (1) {
        rt_uint8_t lp  = debounce_pressed(&s_left);
        rt_uint8_t rp  = debounce_pressed(&s_right);
        rt_tick_t  now = rt_tick_get();

        /* --- 双键同按: 优先级最高, 触发一次 GO/STOP, 作废两键单键手势 --- */
        if (lp && rp) {
            if (!chord_consumed) {
                motor_toggle();
                chord_consumed = 1;
                l_long = r_long = 0;
                g_key_left_held = g_key_right_held = 0;  /* 撤销可能已触发的转向 */
                l_press_tick = r_press_tick = 0;         /* 作废, 松手不再算短按 */
            }
            l_prev = lp; r_prev = rp;
            rt_thread_mdelay(KEY_PERIOD_MS);
            continue;
        }

        /* 双键手势后, 封锁单键逻辑, 直到两键都松开 */
        if (chord_consumed) {
            if (!lp && !rp) chord_consumed = 0;
            l_prev = lp; r_prev = rp;
            rt_thread_mdelay(KEY_PERIOD_MS);
            continue;
        }

        /* --- 左键: 短按加速 / 长按左转 --- */
        if (lp && !l_prev) {                    /* 按下沿 */
            l_press_tick = now;
            l_long = 0;
        }
        if (lp && !l_long && l_press_tick != 0 && (now - l_press_tick) >= long_ticks) {
            l_long = 1;                         /* 进入长按 -> 左转 */
            g_key_left_held = 1;
            rt_kprintf("[KEY] L turn ON\n");
        }
        if (!lp && l_prev) {                    /* 松开沿 */
            if (l_long) {
                g_key_left_held = 0;
                l_long = 0;
                rt_kprintf("[KEY] L turn OFF\n");
            } else if (l_press_tick != 0) {
                speed_step(+KEY_SPEED_STEP);    /* 短按 -> 加速 */
            }
            l_press_tick = 0;
        }

        /* --- 右键: 短按减速 / 长按右转 (镜像) --- */
        if (rp && !r_prev) {
            r_press_tick = now;
            r_long = 0;
        }
        if (rp && !r_long && r_press_tick != 0 && (now - r_press_tick) >= long_ticks) {
            r_long = 1;
            g_key_right_held = 1;
            rt_kprintf("[KEY] R turn ON\n");
        }
        if (!rp && r_prev) {
            if (r_long) {
                g_key_right_held = 0;
                r_long = 0;
                rt_kprintf("[KEY] R turn OFF\n");
            } else if (r_press_tick != 0) {
                speed_step(-KEY_SPEED_STEP);    /* 短按 -> 减速 */
            }
            r_press_tick = 0;
        }

        l_prev = lp; r_prev = rp;
        rt_thread_mdelay(KEY_PERIOD_MS);
    }
}

int key_app_init(void)
{
    /* 引脚配置 + 去抖状态初始化 (读一次实际电平, 避免开机抖动误报) */
    key_debounce_t *keys[] = { &s_left, &s_right };
    for (rt_size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        rt_pin_mode(keys[i]->pin, PIN_MODE_INPUT_PULLUP);
        rt_uint8_t lvl = (rt_uint8_t)rt_pin_read(keys[i]->pin);
        keys[i]->last_sample  = lvl;
        keys[i]->stable_level = lvl;
        keys[i]->same_count   = KEY_DEBOUNCE_SAMPLES;
    }

    rt_thread_t t = rt_thread_create("key_scan",
                                     key_thread, RT_NULL,
                                     1024,
                                     RT_THREAD_PRIORITY_MAX / 2 + 1,
                                     5);
    if (t == RT_NULL) {
        rt_kprintf("[KEY] thread create FAIL\n");
        return -RT_ERROR;
    }
    rt_thread_startup(t);
    return RT_EOK;
}
INIT_APP_EXPORT(key_app_init);
