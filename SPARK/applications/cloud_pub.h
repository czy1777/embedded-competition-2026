/**
 * @file    cloud_pub.h
 * @brief   5 类遥测的 MQTT 发布函数 (健康/环境/姿态/GPS/心跳)
 *
 *   所有发布函数:
 *     - 内部检查 cloud_is_ready(),未就绪直接返回不发包
 *     - 用 snprintf 拼 JSON (避免 cJSON 的动态分配)
 *     - QoS 0,无持久化;失败仅打印日志,不阻塞调用方
 */

#ifndef APPLICATIONS_CLOUD_PUB_H
#define APPLICATIONS_CLOUD_PUB_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int cloud_pub_health(void);
int cloud_pub_env(void);
int cloud_pub_attitude(void);
int cloud_pub_gps(void);
int cloud_pub_heartbeat(void);
int cloud_pub_kp(void);     /* K230 步态 6 关键点 */

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_CLOUD_PUB_H */
