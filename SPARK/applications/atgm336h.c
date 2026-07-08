#include <atgm336h.h>

/*============================================================================
 *                             全局变量
 *============================================================================*/

uint16_t point1 = 0; // 用于记录接收到的数据长度
float longitude;     // 用于存储经度
float latitude;      // 用于存储纬度

// 定义保存GPS数据的全局结构体实例
_SaveData Save_Data;
// 定义存储经纬度数据的全局结构体实例
LatitudeAndLongitude_s g_LatAndLongData =
{
    .E_W = 0,
    .N_S = 0,
    .latitude = 0.0,
    .longitude = 0.0
};

// 串口接收缓冲区
static char USART_RX_BUF[USART_REC_LEN];

/*============================================================================
 *                             私有函数
 *============================================================================*/

/**
 * @brief GPS 数据接收回调（由 uart_app 框架调用）
 */
static void gps_rx_callback(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    for (rt_size_t i = 0; i < size; i++)
    {
        char ch = data[i];

        // 检测帧头 '$'
        if (ch == '$')
        {
            point1 = 0;
        }

        // 存储数据
        if (point1 < USART_REC_LEN - 1)
        {
            USART_RX_BUF[point1++] = ch;
        }

        // 检查是否收到换行符（一帧结束）
        if (ch == '\n' && point1 > 6)
        {
            // 检查是否是 RMC 帧 (GPRMC, GNRMC) 或 GGA 帧 (GPGGA, GNGGA)
            // RMC: $xxRMC - 位置 3,4,5 = 'R','M','C'
            // GGA: $xxGGA - 位置 3,4,5 = 'G','G','A'
            bool is_rmc = (USART_RX_BUF[3] == 'R' && USART_RX_BUF[4] == 'M' && USART_RX_BUF[5] == 'C');
            bool is_gga = (USART_RX_BUF[3] == 'G' && USART_RX_BUF[4] == 'G' && USART_RX_BUF[5] == 'A');

            if (is_rmc || is_gga)
            {
                memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);
                memcpy(Save_Data.GPS_Buffer, USART_RX_BUF, (point1 < GPS_Buffer_Length) ? point1 : GPS_Buffer_Length - 1);
                Save_Data.isGetData = true;
                Save_Data.isParseData = false;  // 标记需要重新解析
            }
            // 重置缓冲区准备下一帧
            point1 = 0;
            memset(USART_RX_BUF, 0, USART_REC_LEN);
        }
    }
}

/*============================================================================
 *                             公开函数
 *============================================================================*/

// 初始化GPS模块
void atgm336h_init(void)
{
    clrStruct(); // 清除结构体数据
    // 设置 UART3 接收回调
    uart_set_rx_callback(UART_PORT_3, gps_rx_callback);
}

// 清除结构体数据函数
void clrStruct(void)
{
    Save_Data.isGetData = false;   // 标记未获取到GPS数据
    Save_Data.isParseData = false; // 标记数据未解析
    Save_Data.isUsefull = false;   // 标记定位信息无效
    // 清空各缓冲区
    memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);
    memset(Save_Data.UTCTime, 0, UTCTime_Length);
    memset(Save_Data.latitude, 0, latitude_Length);
    memset(Save_Data.N_S, 0, N_S_Length);
    memset(Save_Data.longitude, 0, longitude_Length);
    memset(Save_Data.E_W, 0, E_W_Length);
}

// 错误日志函数（不再死循环，避免阻塞RTOS）
void errorLog(int num)
{
    uart1_printf("GPS ERROR: %d\r\n", num);
}

// 解析GPS数据缓冲区函数
void parseGpsBuffer(void)
{
    char *subString;      // 指向当前解析位置
    char *subStringNext;  // 指向下个解析位置
    int field = 0;        // 当前字段编号

    uint16_t Number = 0, Integer = 0, Decimal = 0; // 用于经纬度数值转换

    if (Save_Data.isGetData) // 如果已经获取到GPS数据
    {
        Save_Data.isGetData = false;
        // 调试打印已关闭, 如需开启取消下面两行注释
        // uart1_printf("**************\r\n");
        // uart1_printf("%s", Save_Data.GPS_Buffer);

        // 判断是 GGA 还是 RMC 帧
        bool is_gga = (Save_Data.GPS_Buffer[3] == 'G' && Save_Data.GPS_Buffer[4] == 'G' && Save_Data.GPS_Buffer[5] == 'A');
        bool is_rmc = (Save_Data.GPS_Buffer[3] == 'R' && Save_Data.GPS_Buffer[4] == 'M' && Save_Data.GPS_Buffer[5] == 'C');

        // GGA: $xxGGA,time,lat,N/S,lon,E/W,quality,...
        // RMC: $xxRMC,time,status,lat,N/S,lon,E/W,...

        int lat_field = is_gga ? 2 : 3;   // 纬度字段位置
        int ns_field = is_gga ? 3 : 4;    // N/S 字段位置
        int lon_field = is_gga ? 4 : 5;   // 经度字段位置
        int ew_field = is_gga ? 5 : 6;    // E/W 字段位置
        int status_field = is_gga ? 6 : 2; // 状态字段位置 (GGA: quality, RMC: status)
        int max_field = 7;

        subString = Save_Data.GPS_Buffer;

        for (field = 0; field < max_field; field++)
        {
            // 查找下一个逗号
            subStringNext = strstr(subString, ",");
            if (subStringNext == NULL) break;

            if (field > 0)
            {
                int len = subStringNext - subString;
                if (len > 0)
                {
                    if (field == 1) // UTC 时间
                    {
                        memset(Save_Data.UTCTime, 0, UTCTime_Length);
                        memcpy(Save_Data.UTCTime, subString, (len < UTCTime_Length) ? len : UTCTime_Length - 1);
                    }
                    else if (field == lat_field) // 纬度
                    {
                        memset(Save_Data.latitude, 0, latitude_Length);
                        memcpy(Save_Data.latitude, subString, (len < latitude_Length) ? len : latitude_Length - 1);
                    }
                    else if (field == ns_field) // N/S
                    {
                        memset(Save_Data.N_S, 0, N_S_Length);
                        memcpy(Save_Data.N_S, subString, (len < N_S_Length) ? len : N_S_Length - 1);
                    }
                    else if (field == lon_field) // 经度
                    {
                        memset(Save_Data.longitude, 0, longitude_Length);
                        memcpy(Save_Data.longitude, subString, (len < longitude_Length) ? len : longitude_Length - 1);
                    }
                    else if (field == ew_field) // E/W
                    {
                        memset(Save_Data.E_W, 0, E_W_Length);
                        memcpy(Save_Data.E_W, subString, (len < E_W_Length) ? len : E_W_Length - 1);
                    }
                    else if (field == status_field) // 状态
                    {
                        // GGA: quality (1=GPS fix), RMC: status (A=valid)
                        if (is_gga)
                            Save_Data.isUsefull = (subString[0] >= '1' && subString[0] <= '5');
                        else
                            Save_Data.isUsefull = (subString[0] == 'A');
                    }
                }
            }
            subString = subStringNext + 1;
        }

        Save_Data.isParseData = true;

        if (Save_Data.isParseData && Save_Data.isUsefull)
        {
            // 获取纬度方向和经度方向
            g_LatAndLongData.N_S = Save_Data.N_S[0];
            g_LatAndLongData.E_W = Save_Data.E_W[0];

            // 转换纬度数据 (格式: DDMM.MMMM)
            Number = 0; Integer = 0; Decimal = 0;
            for (uint8_t i = 0; i < 9 && Save_Data.latitude[i]; i++)
            {
                if (i < 2)
                {
                    Number *= 10;
                    Number += Save_Data.latitude[i] - '0';
                }
                else if (i < 4)
                {
                    Integer *= 10;
                    Integer += Save_Data.latitude[i] - '0';
                }
                else if (i == 4); // 跳过小数点
                else if (i < 9)
                {
                    Decimal *= 10;
                    Decimal += Save_Data.latitude[i] - '0';
                }
            }
            g_LatAndLongData.latitude = 1.0 * Number + (1.0 * Integer + 1.0 * Decimal / 10000) / 60;

            // 转换经度数据 (格式: DDDMM.MMMM)
            Number = 0; Integer = 0; Decimal = 0;
            for (uint8_t i = 0; i < 10 && Save_Data.longitude[i]; i++)
            {
                if (i < 3)
                {
                    Number *= 10;
                    Number += Save_Data.longitude[i] - '0';
                }
                else if (i < 5)
                {
                    Integer *= 10;
                    Integer += Save_Data.longitude[i] - '0';
                }
                else if (i == 5); // 跳过小数点
                else if (i < 10)
                {
                    Decimal *= 10;
                    Decimal += Save_Data.longitude[i] - '0';
                }
            }
            g_LatAndLongData.longitude = 1.0 * Number + (1.0 * Integer + 1.0 * Decimal / 10000) / 60;

            // 更新全局经纬度变量
            longitude = g_LatAndLongData.longitude;
            latitude = g_LatAndLongData.latitude;

            // 进行南北纬、东西经处理
            if (g_LatAndLongData.E_W == 'W')
                longitude = -longitude;
            if (g_LatAndLongData.N_S == 'S')
                latitude = -latitude;
        }
    }
}

// 打印GPS数据函数
void printGpsBuffer(void)
{
    if (Save_Data.isParseData) // 如果数据已解析
    {
        Save_Data.isParseData = false;

        // 打印UTC时间
        uart1_printf("Save_Data.UTCTime = %s\r\n", Save_Data.UTCTime);

        if (Save_Data.isUsefull) // 如果数据有效
        {
            Save_Data.isUsefull = false;
            // 打印原始经纬度数据
            uart1_printf("Save_Data.latitude = %s\r\n", Save_Data.latitude);
            uart1_printf("Save_Data.N_S = %s\r\n", Save_Data.N_S);
            uart1_printf("Save_Data.longitude = %s\r\n", Save_Data.longitude);
            uart1_printf("Save_Data.E_W = %s\r\n", Save_Data.E_W);

            // 打印转换后的经纬度数据（使用整数模拟小数）
            int lat_int = (int)g_LatAndLongData.latitude;
            int lat_dec = abs((int)(g_LatAndLongData.latitude * 10000) % 10000);
            int lon_int = (int)g_LatAndLongData.longitude;
            int lon_dec = abs((int)(g_LatAndLongData.longitude * 10000) % 10000);

            uart1_printf("latitude: %c,%d.%04d\r\n", g_LatAndLongData.N_S, lat_int, lat_dec);
            uart1_printf("longitude: %c,%d.%04d\r\n", g_LatAndLongData.E_W, lon_int, lon_dec);
        }
        else
        {
            // 提示GPS数据无效
            uart1_printf("GPS DATA is not usefull!\r\n");
        }
    }
}

// GPS模块任务函数
void atgm336h_task(void)
{
    parseGpsBuffer(); // 解析GPS数据
    /* 周期串口打印已关闭, 如需调试取消下行注释 */
    // printGpsBuffer();
}

/*============================================================================
 *                             RT-Thread 应用初始化
 *============================================================================*/

/**
 * @brief GPS 应用初始化
 */
int atgm336h_app_init(void)
{
    atgm336h_init();
    uart1_printf("ATGM336H GPS module initialized (UART3)\r\n");
    return RT_EOK;
}
