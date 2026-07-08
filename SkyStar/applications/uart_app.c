#include "uart_app.h"

/*============================================================================
 *                             私有类型定义
 *============================================================================*/

/* 串口端口运行时结构体 */
typedef struct {
    const char *name;                           /* 设备名称 */
    rt_device_t device;                         /* 设备句柄 */
    struct rt_ringbuffer ringbuffer;            /* 环形缓冲区 */
    rt_uint8_t rb_pool[UART_BUFFER_SIZE];       /* 缓冲区存储池 */
    rt_uint8_t read_buffer[UART_BUFFER_SIZE];   /* 读取缓冲区 */
    rt_tick_t last_rx_tick;                     /* 上次接收tick */
    uart_rx_callback_t rx_callback;             /* 用户回调 */
    rt_bool_t initialized;                      /* 初始化标志 */
} uart_port_t;

/*============================================================================
 *                             私有变量
 *============================================================================*/

/* 串口端口实例数组 */
static uart_port_t uart_ports[UART_PORT_MAX] = {
    [UART_PORT_1] = { .name = "uart1", .initialized = RT_FALSE },
    [UART_PORT_2] = { .name = "uart2", .initialized = RT_FALSE },
    [UART_PORT_3] = { .name = "uart3", .initialized = RT_FALSE },
    [UART_PORT_4] = { .name = "uart4", .initialized = RT_FALSE },
    [UART_PORT_5] = { .name = "uart5", .initialized = RT_FALSE },
    [UART_PORT_6] = { .name = "uart6", .initialized = RT_FALSE }
};

/*============================================================================
 *                             私有函数
 *============================================================================*/

/**
 * @brief 根据设备指针查找端口ID
 */
static uart_port_id_t find_port_by_device(rt_device_t dev)
{
    for (int i = 0; i < UART_PORT_MAX; i++) {
        if (uart_ports[i].device == dev) {
            return (uart_port_id_t)i;
        }
    }
    return UART_PORT_MAX;
}

/**
 * @brief 通用串口接收回调（中断上下文）
 */
static rt_err_t uart_common_rx_ind(rt_device_t dev, rt_size_t size)
{
    uart_port_id_t port_id;
    uart_port_t *port;
    rt_size_t read_size;
    rt_uint8_t temp_buffer[64];  /* 中断中使用较小的临时缓冲区 */

    /* 查找对应的端口 */
    port_id = find_port_by_device(dev);
    if (port_id >= UART_PORT_MAX) {
        return -RT_ERROR;
    }

    port = &uart_ports[port_id];
    if (!port->initialized) {
        return -RT_ERROR;
    }

    /* 更新接收时间戳 */
    port->last_rx_tick = rt_tick_get();

    /* 从串口设备读取数据 */
    read_size = rt_device_read(dev, 0, temp_buffer, sizeof(temp_buffer));

    if (read_size > 0) {
        /* 将数据写入环形缓冲区 */
        rt_ringbuffer_put(&port->ringbuffer, temp_buffer, read_size);
    }

    return RT_EOK;
}

/**
 * @brief 串口数据处理线程入口
 */
static void uart_thread_entry(void *parameter)
{
    uart_port_id_t port_id = (uart_port_id_t)(rt_ubase_t)parameter;
    uart_port_t *port = &uart_ports[port_id];
    rt_size_t size;
    rt_tick_t current_tick;

    while (1) {
        current_tick = rt_tick_get();

        /* 检查是否超时（数据接收完成） */
        if (current_tick - port->last_rx_tick >= UART_TIMEOUT_TICKS) {
            size = rt_ringbuffer_data_len(&port->ringbuffer);
            if (size > 0) {
                /* 读取所有数据 */
                size = rt_ringbuffer_get(&port->ringbuffer,
                                          port->read_buffer,
                                          UART_BUFFER_SIZE - 1);
                if (size > 0) {
                    port->read_buffer[size] = '\0';  /* 添加字符串结束符 */

                    /* 调用用户回调 */
                    if (port->rx_callback != RT_NULL) {
                        port->rx_callback(port_id, port->read_buffer, size);
                    }

                    /* 清空读取缓冲区 */
                    rt_memset(port->read_buffer, 0, sizeof(port->read_buffer));
                }
            }
        }

        rt_thread_mdelay(10);  /* 10ms 轮询周期 */
    }
}

/**
 * @brief 初始化指定串口端口
 */
static rt_err_t uart_port_init(uart_port_id_t port_id)
{
    rt_err_t result;
    uart_port_t *port;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    char thread_name[RT_NAME_MAX];
    rt_thread_t thread;

    if (port_id >= UART_PORT_MAX) {
        return -RT_EINVAL;
    }

    port = &uart_ports[port_id];

    /* 避免重复初始化 */
    if (port->initialized) {
        return -RT_ERROR;
    }

    /* 初始化环形缓冲区 */
    rt_ringbuffer_init(&port->ringbuffer, port->rb_pool, UART_BUFFER_SIZE);

    /* 查找串口设备 */
    port->device = rt_device_find(port->name);
    if (port->device == RT_NULL) {
        rt_kprintf("Find %s failed!\n", port->name);
        return -RT_ERROR;
    }

    /* 检查设备是否已经被打开（如 uart1 被控制台使用） */
    if (port->device->open_flag != 0) {
        /* 设备已被打开（如 uart1 被控制台使用），直接复用，不关闭重开 */
    } else {
        /* 配置串口参数 - 全部使用 115200 */
        config.baud_rate = BAUD_RATE_115200;
        config.data_bits = DATA_BITS_8;
        config.stop_bits = STOP_BITS_1;
        config.parity    = PARITY_NONE;
        config.bufsz     = UART_BUFFER_SIZE;

        result = rt_device_control(port->device, RT_DEVICE_CTRL_CONFIG, &config);
        if (result != RT_EOK) {
            rt_kprintf("Configure %s failed!\n", port->name);
            return -RT_ERROR;
        }

        /* 打开设备（中断接收+中断发送模式） */
        result = rt_device_open(port->device, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
        if (result != RT_EOK) {
            rt_kprintf("Open %s failed!\n", port->name);
            return -RT_ERROR;
        }
    }

    /* 设置接收回调 */
    rt_device_set_rx_indicate(port->device, uart_common_rx_ind);

    /* 创建数据处理线程 */
    static const int port_nums[] = {1, 2, 3, 4, 5, 6};
    rt_snprintf(thread_name, sizeof(thread_name), "uart%d", port_nums[port_id]);
    thread = rt_thread_create(thread_name,
                               uart_thread_entry,
                               (void *)(rt_ubase_t)port_id,
                               2048,
                               RT_THREAD_PRIORITY_MAX / 2,
                               10);
    if (thread != RT_NULL) {
        rt_thread_startup(thread);
    } else {
        rt_kprintf("Create %s thread failed!\n", thread_name);
        return -RT_ERROR;
    }

    port->initialized = RT_TRUE;
    port->last_rx_tick = rt_tick_get();

    return RT_EOK;
}

/*============================================================================
 *                             公开函数实现
 *============================================================================*/

/**
 * @brief 向指定串口发送数据
 */
rt_size_t uart_send(uart_port_id_t port_id, const rt_uint8_t *data, rt_size_t size)
{
    uart_port_t *port;

    if (port_id >= UART_PORT_MAX || data == RT_NULL || size == 0) {
        return 0;
    }

    port = &uart_ports[port_id];
    if (!port->initialized || port->device == RT_NULL) {
        return 0;
    }

    return rt_device_write(port->device, 0, data, size);
}

/**
 * @brief 通用格式化输出函数
 */
static int uart_printf_internal(uart_port_id_t port_id, const char *format, va_list args)
{
    char buffer[256];
    int len;

    if (port_id >= UART_PORT_MAX) {
        return 0;
    }

    len = rt_vsnprintf(buffer, sizeof(buffer), format, args);
    return (int)uart_send(port_id, (rt_uint8_t *)buffer, len);
}

/**
 * @brief 向 UART1 格式化输出
 */
int uart1_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = uart_printf_internal(UART_PORT_1, format, args);
    va_end(args);

    return len;
}

/**
 * @brief 向 UART2 格式化输出
 */
int uart2_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = uart_printf_internal(UART_PORT_2, format, args);
    va_end(args);

    return len;
}

/**
 * @brief 向 UART3 格式化输出
 */
int uart3_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = uart_printf_internal(UART_PORT_3, format, args);
    va_end(args);

    return len;
}

/**
 * @brief 向 UART4 格式化输出
 */
int uart4_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = uart_printf_internal(UART_PORT_4, format, args);
    va_end(args);

    return len;
}

/**
 * @brief 向 UART5 格式化输出
 */
int uart5_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = uart_printf_internal(UART_PORT_5, format, args);
    va_end(args);

    return len;
}

/**
 * @brief 向 UART6 格式化输出
 */
int uart6_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = uart_printf_internal(UART_PORT_6, format, args);
    va_end(args);

    return len;
}

/**
 * @brief 设置接收回调函数
 */
void uart_set_rx_callback(uart_port_id_t port_id, uart_rx_callback_t callback)
{
    if (port_id < UART_PORT_MAX) {
        uart_ports[port_id].rx_callback = callback;
    }
}

/**
 * @brief 获取接收缓冲区数据长度
 */
rt_size_t uart_get_data_len(uart_port_id_t port_id)
{
    if (port_id >= UART_PORT_MAX || !uart_ports[port_id].initialized) {
        return 0;
    }
    return rt_ringbuffer_data_len(&uart_ports[port_id].ringbuffer);
}

/**
 * @brief 从接收缓冲区读取数据
 */
rt_size_t uart_read_data(uart_port_id_t port_id, rt_uint8_t *buffer, rt_size_t size)
{
    if (port_id >= UART_PORT_MAX || !uart_ports[port_id].initialized ||
        buffer == RT_NULL || size == 0) {
        return 0;
    }
    return rt_ringbuffer_get(&uart_ports[port_id].ringbuffer, buffer, size);
}

/*============================================================================
 *                             默认接收回调
 *============================================================================*/

static void uart1_default_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    uart1_printf("UART1 RX [%d]: %s\r\n", size, data);
}

static void uart2_default_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    uart2_printf("UART2 RX [%d]: %s\r\n", size, data);
}

static void uart3_default_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    uart3_printf("UART3 RX [%d]: %s\r\n", size, data);
}

static void uart4_default_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    uart4_printf("UART4 RX [%d]: %s\r\n", size, data);
}

static void uart5_default_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    uart5_printf("UART5 RX [%d]: %s\r\n", size, data);
}

static void uart6_default_rx_handler(uart_port_id_t port, rt_uint8_t *data, rt_size_t size)
{
    uart6_printf("UART6 RX [%d]: %s\r\n", size, data);
}

/*============================================================================
 *                             应用初始化
 *============================================================================*/

/**
 * @brief UART 应用初始化函数
 */
int uart_app_init(void)
{
    rt_err_t result;

    /* UART1 是 finsh 控制台，不要由应用层接管，否则 msh 收不到键盘输入 */

    /* 初始化 UART2 */
    result = uart_port_init(UART_PORT_2);
    if (result != RT_EOK) {
        rt_kprintf("UART2 init failed!\n");
    }

    /* 初始化 UART3 */
    result = uart_port_init(UART_PORT_3);
    if (result != RT_EOK) {
        rt_kprintf("UART3 init failed!\n");
    }

    /* 初始化 UART4 */
    result = uart_port_init(UART_PORT_4);
    if (result != RT_EOK) {
        rt_kprintf("UART4 init failed!\n");
    }

    /* 初始化 UART5 */
    result = uart_port_init(UART_PORT_5);
    if (result != RT_EOK) {
        rt_kprintf("UART5 init failed!\n");
    }

    /* 初始化 UART6 */
    result = uart_port_init(UART_PORT_6);
    if (result != RT_EOK) {
        rt_kprintf("UART6 init failed!\n");
    }

    /* 设置默认接收回调 (不含 UART1, 留给 finsh) */
    uart_set_rx_callback(UART_PORT_2, uart2_default_rx_handler);
    uart_set_rx_callback(UART_PORT_3, uart3_default_rx_handler);
    uart_set_rx_callback(UART_PORT_4, uart4_default_rx_handler);
    uart_set_rx_callback(UART_PORT_5, uart5_default_rx_handler);
    uart_set_rx_callback(UART_PORT_6, uart6_default_rx_handler);

    return RT_EOK;
}

/* 导出到自动初始化 */
INIT_APP_EXPORT(uart_app_init);
