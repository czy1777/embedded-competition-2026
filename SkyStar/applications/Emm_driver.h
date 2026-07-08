#ifndef APPLICATIONS_EMM_DRIVER_H_
#define APPLICATIONS_EMM_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rtthread.h>
#include <uart_app.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define EMM_UART_TIMEOUT  1000  /* UART通信超时时间 (ms) */
#define EMM_MAX_RESPONSE_SIZE 32 /* 最大响应长度 */
#define EMM_FIXED_END_BYTE 0x6B /* 协议固定结束字节 */

/* SysParams_t 系统参数枚举 */
typedef enum {
    S_VER   = 0,            /* 获取固件版本和对应硬件版本 */
    S_RL    = 1,            /* 获取读取 */
    S_PID   = 2,            /* 获取PID */
    S_VBUS  = 3,            /* 获取母线电压 */
    S_CPHA  = 5,            /* 获取编码器相位 */
    S_ENCL  = 7,            /* 获取编码器校准偏移值 */
    S_TPOS  = 8,            /* 获取目标位置角度 */
    S_VEL   = 9,            /* 获取实时转速 */
    S_CPOS  = 10,           /* 获取实时位置角度 */
    S_PERR  = 11,           /* 获取位置误差角度 */
    S_FLAG  = 13,           /* 获取使能/堵转/上电状态标志位 */
    S_Conf  = 14,           /* 获取驱动信息 */
    S_State = 15,           /* 获取系统状态 */
    S_ORG   = 16,           /* 获取回原成功/失败状态标志位 */
} SysParams_t;

/* 电机接收状态枚举 */
typedef enum {
    EMM_STATE_IDLE = 0,        // 空闲状态
    EMM_STATE_RECEIVING,       // 正在接收数据
    EMM_STATE_DATA_READY,      // 数据包接收完成
    EMM_STATE_ERROR            // 接收错误
} Emm_State_t;

/* 硬件配置结构体 */
typedef struct {
    uart_port_id_t port;          // 串口端口ID
    uint32_t timeout_ms;          // 串口传输超时时间
} Emm_HW_t;

/* 电机运行数据结构体 */
typedef struct {
    uint8_t  addr;                  /* 电机地址 */
    char     version[16];           /* 固件版本 */
    uint8_t  status;                /* 通用状态 */
    uint8_t  error;                 /* 错误代码 */
    uint8_t  ctrl_mode;             /* 控制模式 */
    uint8_t  protection;            /* 保护状态 */
    uint8_t  closed_loop_state;     /* 闭环状态 */
    uint8_t  encoder_state;         /* 编码器校准状态 */
    uint8_t  sync_state;            /* 同步状态 */
    uint8_t  origin_state;          /* 回原状态 */
    uint16_t voltage;               /* 母线电压(V) */
    uint16_t current;               /* 电流(mA) */
    int32_t  encoder;               /* 编码器值 */
    uint8_t  dir;                   /* 方向 (0: CW, 1: CCW) */
    int16_t  speed;                 /* 实时速度(RPM) */
    int32_t  position;              /* 实时位置 */
    int32_t  target_pos;            /* 目标位置 */
    uint16_t target_speed;          /* 目标速度 */
    uint8_t  acceleration;          /* 加速度 */
    uint8_t  subdivision;           /* 细分 */
    uint16_t pwm_duty;              /* PWM占空比 */
    uint32_t timestamp;             /* 数据更新时间戳 */
    uint8_t  data_valid;            /* 数据有效标志 */
} Emm_Data_t;

/* 电机实例结构体 */
typedef struct {
    Emm_HW_t hw;                    // 硬件配置
    uint8_t  address;               // 电机地址
    uint8_t  enable;                // 实例使能标志
    Emm_Data_t data;                // 电机运行数据
    Emm_State_t state;              // 当前接收状态
    uint8_t rx_buffer[EMM_MAX_RESPONSE_SIZE]; // 接收缓冲区
    uint8_t rx_index;               // 接收索引
    SysParams_t last_read_param_req; // 上次请求的系统参数类型
} Emm_Motor_t;

/* 内部解析用响应结构体 */
typedef struct {
    uint8_t  addr;
    uint8_t  func;
    uint8_t  dir;
    int16_t  speed;
    int32_t  position;
    uint8_t  status;
    uint8_t  error;
    int32_t  encoder;
    int16_t  temperature;
    uint16_t voltage;
    uint16_t current;
    int32_t  target_pos;
    uint16_t target_speed;
    uint8_t  acceleration;
    uint8_t  subdivision;
    uint8_t  ctrl_mode;
    uint8_t  protection;
    uint16_t pwm_duty;
    uint8_t  closed_loop_state;
    uint8_t  encoder_state;
    uint8_t  sync_state;
    uint8_t  origin_state;
    char     version[16];
    uint8_t  raw_data[32];
    uint8_t  data_len;
    uint8_t  s_vel_is;
    uint8_t  valid;
} Emm_V5_Response_t;

/* Exported functions prototypes ---------------------------------------------*/

int8_t Emm_Create(Emm_Motor_t* emm, uart_port_id_t port, uint8_t addr, uint32_t timeout_ms);
int8_t Emm_ProcessBuffer(Emm_Motor_t* emm, uint8_t* buffer, uint16_t length);
int8_t Emm_ResetCurrentPositionToZero(Emm_Motor_t* emm);
int8_t Emm_ResetClogProtection(Emm_Motor_t* emm);
int8_t Emm_ReadSysParams(Emm_Motor_t* emm, SysParams_t s);
int8_t Emm_ModifyControlMode(Emm_Motor_t* emm, bool svF, uint8_t ctrl_mode);
int8_t Emm_SetEnableControl(Emm_Motor_t* emm, bool state, bool snF);
int8_t Emm_VelocityControl(Emm_Motor_t* emm, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
int8_t Emm_PositionControl(Emm_Motor_t* emm, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
int8_t Emm_StopNow(Emm_Motor_t* emm, bool snF);
int8_t Emm_SynchronousMotion(Emm_Motor_t* emm);
int8_t Emm_SetOrigin(Emm_Motor_t* emm, bool svF);
int8_t Emm_ModifyOriginParams(Emm_Motor_t* emm, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
int8_t Emm_TriggerOriginReturn(Emm_Motor_t* emm, uint8_t o_mode, bool snF);
int8_t Emm_InterruptOrigin(Emm_Motor_t* emm);
int8_t Emm_GetVersion(Emm_Motor_t* emm, char* version_buf, uint8_t buf_len);
int16_t Emm_GetSpeed(Emm_Motor_t* emm);
int32_t Emm_GetPosition(Emm_Motor_t* emm);
bool Emm_IsEnabled(Emm_Motor_t* emm);
Emm_Data_t* Emm_GetData(Emm_Motor_t* emm);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_EMM_DRIVER_H_ */
