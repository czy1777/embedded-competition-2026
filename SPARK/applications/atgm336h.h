#ifndef APPLICATIONS_ATGM336H_H_
#define APPLICATIONS_ATGM336H_H_

#include <bsp_system.h>

// 定义串口接收缓冲区最大长度
#define USART_REC_LEN      200

// 定义GPS数据缓冲区及相关信息长度
#define GPS_Buffer_Length  80
#define UTCTime_Length     11
#define latitude_Length    11
#define N_S_Length         2
#define longitude_Length   12
#define E_W_Length         2

// 定义保存GPS数据的结构体
typedef struct SaveData
{
    char GPS_Buffer[GPS_Buffer_Length];  // 用于存储接收到的GPS原始数据
    char isGetData;                      // 标志位，指示是否获取到了GPS数据
    char isParseData;                    // 标志位，指示数据是否已被解析
    char UTCTime[UTCTime_Length];        // 用于存储UTC时间
    char latitude[latitude_Length];      // 用于存储纬度数据
    char N_S[N_S_Length];                // 用于存储纬度方向（N/S）
    char longitude[longitude_Length];    // 用于存储经度数据
    char E_W[E_W_Length];                // 用于存储经度方向（E/W）
    char isUsefull;                      // 标志位，指示定位信息是否有效
} _SaveData;

// 定义存储经纬度数据的结构体
typedef struct _LatitudeAndLongitude_s
{
    float latitude;  // 纬度
    float longitude; // 经度
    char N_S;        // 纬度方向（北/南）
    char E_W;        // 经度方向（东/西）
} LatitudeAndLongitude_s;

// 定义串口接收数据缓冲区及相关变量
extern uint16_t point1;
extern _SaveData Save_Data;
extern LatitudeAndLongitude_s g_LatAndLongData;

int atgm336h_app_init(void);  // RT-Thread 应用初始化入口
void atgm336h_init(void);     // 初始化GPS模块
void clrStruct(void);         // 清除结构体数据
void parseGpsBuffer(void);    // 解析GPS数据缓冲区
void printGpsBuffer(void);    // 打印GPS数据
void atgm336h_task(void);     // GPS模块任务函数

#endif /* APPLICATIONS_ATGM336H_H_ */
