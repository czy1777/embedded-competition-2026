/**
 * @file    JYD-18.c
 * @brief   JDY-18 蓝牙 4.2 模块驱动
 *
 * JDY-18 配置为主机透传模式，连接 ESP32-S3 (SmartWatch-S3)，
 * 接收 BLE Notification 透传的健康数据包并解析。
 *
 * 通信方式: UART4, 9600 baud
 * 数据包格式: 12 字节固定长度 (header 0xAA, tail 0x55, XOR checksum)
 */

#include "JYD-18.h"
#include "uart_app.h"

/*============================================================================
 *                             私有定义
 *============================================================================*/

/* 帧解析状态机 */
typedef enum {
    PARSE_WAIT_HEADER = 0,  /* 等待帧头 0xAA */
    PARSE_RECEIVING         /* 正在接收帧数据 */
} parse_state_t;

/*============================================================================
 *                             私有变量
 *============================================================================*/

/* JDY-18 状态 */
static jdy18_state_t jdy18_state = JDY18_STATE_UNINIT;

/* 最新的健康数据 */
static ble_health_packet_t latest_packet;
static rt_bool_t has_new_data = RT_FALSE;
static rt_tick_t last_update_tick = 0;

/* 帧解析缓冲区 */
static rt_uint8_t parse_buf[PACKET_SIZE];
static rt_uint8_t parse_index = 0;
static parse_state_t parse_state = PARSE_WAIT_HEADER;

/* AT 响应标志 */
static volatile rt_bool_t at_mode = RT_FALSE;

/*============================================================================
 *                             私有函数
 *============================================================================*/

/**
 * @brief 计算 XOR 校验和
 */
static rt_uint8_t calculate_checksum(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint8_t checksum = 0;
    for (rt_size_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief 验证并处理一个完整的数据包
 */
static void process_packet(const rt_uint8_t *buf)
{
    const ble_health_packet_t *pkt = (const ble_health_packet_t *)buf;

    /* 验证帧尾 */
    if (pkt->tail != PACKET_TAIL)
    {
        return;
    }

    /* 验证校验和 (XOR 字节 0 到 9) */
    rt_uint8_t expected_checksum = calculate_checksum(buf, 10);
    if (pkt->checksum != expected_checksum)
    {
        return;
    }

    /* 验证数据类型 */
    if (pkt->type != PACKET_TYPE_HEALTH && pkt->type != PACKET_TYPE_FALL_ALERT)
    {
        return;
    }

    /* 校验通过，更新数据 */
    rt_memcpy(&latest_packet, pkt, sizeof(ble_health_packet_t));
    has_new_data = RT_TRUE;
    last_update_tick = rt_tick_get();

    /* 更新连接状态 */
    if (jdy18_state != JDY18_STATE_CONNECTED)
    {
        jdy18_state = JDY18_STATE_CONNECTED;
        rt_kprintf("[JDY18] BLE connected (data received)\n");
    }
}

/**
 * @brief 帧解析状态机 - 逐字节解析
 */
static void parse_byte(rt_uint8_t byte)
{
    switch (parse_state)
    {
    case PARSE_WAIT_HEADER:
        if (byte == PACKET_HEADER)
        {
            parse_buf[0] = byte;
            parse_index = 1;
            parse_state = PARSE_RECEIVING;
        }
        break;

    case PARSE_RECEIVING:
        parse_buf[parse_index++] = byte;
        if (parse_index >= PACKET_SIZE)
        {
            /* 收满 12 字节，验证数据包 */
            process_packet(parse_buf);
            /* 重置解析状态 */
            parse_state = PARSE_WAIT_HEADER;
            parse_index = 0;
        }
        break;
    }
}

/**
 * @brief UART4 接收回调 (由 uart_app 框架调用)
 */
static void jdy18_rx_callback(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    /* AT 模式下，将响应打印到控制台 */
    if (at_mode)
    {
        data[size] = '\0';
        rt_kprintf("[JDY18] AT resp: %s\n", data);
        return;
    }

    /* 透传模式：逐字节送入帧解析状态机 */
    for (rt_size_t i = 0; i < size; i++)
    {
        parse_byte(data[i]);
    }
}

/*============================================================================
 *                             公开函数
 *============================================================================*/

void jdy18_init(void)
{
    /* 注册 UART4 接收回调，替换默认回调 */
    uart_set_rx_callback(JDY18_UART_PORT, jdy18_rx_callback);

    /* 初始化状态 */
    jdy18_state = JDY18_STATE_DISCONNECTED;
    has_new_data = RT_FALSE;
    parse_state = PARSE_WAIT_HEADER;
    parse_index = 0;
    rt_memset(&latest_packet, 0, sizeof(latest_packet));

    rt_kprintf("[JDY18] Initialized on UART4 (9600 baud)\n");
}

void jdy18_send_at(const char *cmd)
{
    rt_size_t cmd_len;
    rt_uint8_t crlf[2] = {'\r', '\n'};

    if (cmd == RT_NULL)
    {
        return;
    }

    cmd_len = rt_strlen(cmd);
    uart_send(JDY18_UART_PORT, (const rt_uint8_t *)cmd, cmd_len);
    uart_send(JDY18_UART_PORT, crlf, 2);
}

void jdy18_configure(void)
{
    rt_kprintf("[JDY18] Starting configuration...\n");
    at_mode = RT_TRUE;

    /* 设为主机模式 */
    jdy18_send_at("AT+ROLE1");
    rt_thread_mdelay(200);

    /* 恢复默认透传 UUID (FFE0/FFE1), 匹配 ESP32-S3 */
    jdy18_send_at("AT+SVRUUIDFFE0");
    rt_thread_mdelay(200);

    jdy18_send_at("AT+CHRUUIDFFE1");
    rt_thread_mdelay(200);

    /* 确保波特率 9600 */
    jdy18_send_at("AT+BAUD4");
    rt_thread_mdelay(200);

    /* 开机唤醒模式 */
    jdy18_send_at("AT+STARTEN0");
    rt_thread_mdelay(200);

    /* 关闭状态输出，避免混入透传数据流 */
    jdy18_send_at("AT+ENLOG0");
    rt_thread_mdelay(200);

    /* 重启模块使配置生效 */
    jdy18_send_at("AT+RESET");
    rt_thread_mdelay(1000);

    at_mode = RT_FALSE;
    rt_kprintf("[JDY18] Configuration done\n");
}

void jdy18_scan_and_bind(void)
{
    rt_kprintf("[JDY18] Scanning for SmartWatch-S3...\n");
    at_mode = RT_TRUE;

    /* 发送扫描指令 */
    jdy18_send_at("AT+INQ");

    /* 等待扫描结果返回 (扫描约需 3-5 秒) */
    rt_thread_mdelay(5000);

    /*
     * 扫描结果格式: +DEV:N=MAC,RSSI,NAME
     * 例如: +DEV:1=AABBCCDDEEFF,-60,SmartWatch-S3
     *
     * 注意: 扫描结果通过 AT 响应回调打印到控制台，
     * 用户需要根据打印信息手动执行绑定指令:
     *   AT+BAND<MAC>   例如 AT+BANDAABBCCDDEEFF
     *
     * 也可以通过 jdy18_send_at("AT+BANDXXXXXXXXXXXX") 程序化绑定。
     */

    at_mode = RT_FALSE;
    rt_kprintf("[JDY18] Scan complete. Check console for results.\n");
    rt_kprintf("[JDY18] To bind, call: jdy18_send_at(\"AT+BAND<MAC>\")\n");
}

rt_bool_t jdy18_get_health_data(ble_health_packet_t *pkt)
{
    if (pkt == RT_NULL || !has_new_data)
    {
        return RT_FALSE;
    }

    rt_memcpy(pkt, &latest_packet, sizeof(ble_health_packet_t));
    has_new_data = RT_FALSE;
    return RT_TRUE;
}

rt_bool_t jdy18_peek_latest(ble_health_packet_t *pkt)
{
    if (pkt == RT_NULL || last_update_tick == 0)
    {
        return RT_FALSE;
    }
    rt_memcpy(pkt, &latest_packet, sizeof(ble_health_packet_t));
    return RT_TRUE;
}

jdy18_state_t jdy18_get_state(void)
{
    return jdy18_state;
}

rt_bool_t jdy18_is_connected(void)
{
    return (jdy18_state == JDY18_STATE_CONNECTED) ? RT_TRUE : RT_FALSE;
}

rt_tick_t jdy18_get_last_update_tick(void)
{
    return last_update_tick;
}

void jdy18_task(void)
{
    ble_health_packet_t pkt;

    /* 检查数据超时 -> 判定断连 (超过 3 秒无数据) */
    if (jdy18_state == JDY18_STATE_CONNECTED)
    {
        rt_tick_t now = rt_tick_get();
        if (now - last_update_tick > rt_tick_from_millisecond(3000) && last_update_tick != 0)
        {
            jdy18_state = JDY18_STATE_DISCONNECTED;
            rt_kprintf("[JDY18] BLE disconnected (data timeout)\n");
        }
    }

    /* 读取并打印最新健康数据 */
    if (jdy18_get_health_data(&pkt))
    {
        if (pkt.type == PACKET_TYPE_HEALTH)
        {
            rt_kprintf("[JDY18] HR=%d SpO2=%d Temp=%d.%02d Fall=%d\n",
                       pkt.heart_rate,
                       pkt.spo2,
                       pkt.temperature / 100,
                       pkt.temperature >= 0 ? (pkt.temperature % 100) : ((-pkt.temperature) % 100),
                       pkt.fall_detected);
        }
        else if (pkt.type == PACKET_TYPE_FALL_ALERT)
        {
            rt_kprintf("[JDY18] *** FALL ALERT ***\n");
        }
    }
}

/*============================================================================
 *                             MSH 命令 (调试用)
 *============================================================================*/

/**
 * @brief MSH: 配置 JDY-18 (一次性)
 */
static void cmd_jdy18_config(void)
{
    jdy18_configure();
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_config, jdy18_config, Configure JDY-18 BLE module);

/**
 * @brief MSH: 扫描从机
 */
static void cmd_jdy18_scan(void)
{
    jdy18_scan_and_bind();
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_scan, jdy18_scan, Scan for BLE peripherals);

/**
 * @brief MSH: 发送 AT 指令
 */
static void cmd_jdy18_at(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: jdy18_at <command>\n");
        rt_kprintf("Example: jdy18_at AT+VERSION\n");
        return;
    }

    at_mode = RT_TRUE;
    jdy18_send_at(argv[1]);
    rt_thread_mdelay(500);
    at_mode = RT_FALSE;
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_at, jdy18_at, Send AT command to JDY-18);

/**
 * @brief MSH: 绑定指定 MAC 地址
 */
static void cmd_jdy18_bind(int argc, char **argv)
{
    char cmd[32];

    if (argc < 2)
    {
        rt_kprintf("Usage: jdy18_bind <MAC>\n");
        rt_kprintf("Example: jdy18_bind AABBCCDDEEFF\n");
        return;
    }

    at_mode = RT_TRUE;

    rt_snprintf(cmd, sizeof(cmd), "AT+BAND%s", argv[1]);
    jdy18_send_at(cmd);
    rt_thread_mdelay(500);

    /* 重启使绑定生效并自动连接 */
    jdy18_send_at("AT+RESET");
    rt_thread_mdelay(1000);

    at_mode = RT_FALSE;
    rt_kprintf("[JDY18] Bound to %s, module resetting...\n", argv[1]);
}
MSH_CMD_EXPORT_ALIAS(cmd_jdy18_bind, jdy18_bind, Bind JDY-18 to MAC address);
