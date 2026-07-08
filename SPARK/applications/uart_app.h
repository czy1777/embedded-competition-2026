#ifndef APPLICATIONS_UART_APP_H_
#define APPLICATIONS_UART_APP_H_

#include <bsp_system.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *                             宏定义
 *============================================================================*/
#define UART_BUFFER_SIZE        256     /* 环形缓冲区大小 */
#define UART_TIMEOUT_TICKS      50      /* 超时tick数 */

/*============================================================================
 *                             类型定义
 *============================================================================*/

/* 串口端口枚举 */
typedef enum {
    UART_PORT_1 = 0,    /* UART1 */
    UART_PORT_2,        /* UART2 */
    UART_PORT_3,        /* UART3 */
    UART_PORT_4,        /* UART4 */
    UART_PORT_MAX
} uart_port_id_t;

/* 接收数据回调函数类型 */
typedef void (*uart_rx_callback_t)(uart_port_id_t port, rt_uint8_t *data, rt_size_t size);

/*============================================================================
 *                             函数声明
 *============================================================================*/

/**
 * @brief  向 UART1 格式化输出
 * @param  format 格式字符串
 * @param  ...    可变参数
 * @return 输出的字符数
 */
int uart1_printf(const char *format, ...);

/**
 * @brief  向 UART3 格式化输出
 * @param  format 格式字符串
 * @param  ...    可变参数
 * @return 输出的字符数
 */
int uart3_printf(const char *format, ...);

/**
 * @brief  向 UART2 格式化输出
 * @param  format 格式字符串
 * @param  ...    可变参数
 * @return 输出的字符数
 */
int uart2_printf(const char *format, ...);

/**
 * @brief  向 UART4 格式化输出
 * @param  format 格式字符串
 * @param  ...    可变参数
 * @return 输出的字符数
 */
int uart4_printf(const char *format, ...);

/**
 * @brief  向指定串口发送数据
 * @param  port   端口ID (UART_PORT_1 或 UART_PORT_3)
 * @param  data   数据缓冲区
 * @param  size   数据长度
 * @return 实际发送的字节数
 */
rt_size_t uart_send(uart_port_id_t port, const rt_uint8_t *data, rt_size_t size);

/**
 * @brief  设置指定串口的接收回调函数
 * @param  port     端口ID
 * @param  callback 回调函数指针
 */
void uart_set_rx_callback(uart_port_id_t port, uart_rx_callback_t callback);

/**
 * @brief  获取指定串口接收缓冲区数据长度
 * @param  port  端口ID
 * @return 缓冲区中的数据长度
 */
rt_size_t uart_get_data_len(uart_port_id_t port);

/**
 * @brief  从指定串口接收缓冲区读取数据
 * @param  port   端口ID
 * @param  buffer 目标缓冲区
 * @param  size   期望读取的长度
 * @return 实际读取的字节数
 */
rt_size_t uart_read_data(uart_port_id_t port, rt_uint8_t *buffer, rt_size_t size);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_UART_APP_H_ */
