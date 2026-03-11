#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdarg.h>

/* ============================================================
 * UART1 模块
 * PC4 = USART1_TX (TTL串口发送)
 * PC5 = USART1_RX (TTL串口接收)
 * 波特率: 115200, 8N1, 无硬件流控
 * ============================================================ */

/* 发送超时 (ms) */
#define UART1_TX_TIMEOUT_MS     (100U)
/* printf 格式化缓冲区大小 (字节) */
#define UART1_PRINTF_BUF_SIZE   (256U)

/* ============================================================
 * 对外接口
 * ============================================================ */

/* 发送单字节 */
void UART1_SendByte(uint8_t byte);

/* 发送数据缓冲区, len 字节 */
void UART1_SendBuffer(const uint8_t *buf, uint16_t len);

/* 发送 C 字符串 (不含末尾 '\0') */
void UART1_SendString(const char *str);

/* printf 风格格式化发送 */
void UART1_Printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif /* __UART_H */
