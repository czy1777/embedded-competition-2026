/*
 * key_app.h
 * 把手两侧薄膜按键 (左 PB8 / 右 PB9) -> 差速转向控制源
 *
 * 接法: 按键一端接 GND, 另一端接对应 GPIO, MCU 端内部上拉.
 *       按下 = 低电平.
 *
 * 输出:
 *   g_key_left_held  / g_key_right_held  (volatile uint8_t, 0/1)
 *   稳态电平翻转 (30ms 防抖) 时更新, rc_drive NORMAL 模式据此差速转向.
 *   RC 模式下摇杆自带 yaw 通道, 本按键状态被忽略.
 *
 * 同时保留串口 [KEY] 事件日志, 便于调试.
 */
#ifndef APPLICATIONS_KEY_APP_H_
#define APPLICATIONS_KEY_APP_H_

#include <stdint.h>

/* 稳态按下状态. 只写方: key_app.c; 只读方: rc_drive.c */
extern volatile uint8_t g_key_left_held;
extern volatile uint8_t g_key_right_held;

int key_app_init(void);

#endif /* APPLICATIONS_KEY_APP_H_ */
