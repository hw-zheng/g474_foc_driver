#include "uart.h"
#include <string.h>
#include <stdio.h>

/* huart1 由 main.c 的 MX_USART1_UART_Init() 初始化 */
extern UART_HandleTypeDef huart1;

/* ============================================================
 * UART1_SendByte
 * 阻塞发送 1 字节
 * ============================================================ */
void UART1_SendByte(uint8_t byte)
{
    HAL_UART_Transmit(&huart1, &byte, 1U, UART1_TX_TIMEOUT_MS);
}

/* ============================================================
 * UART1_SendBuffer
 * 阻塞发送 len 字节缓冲区
 * ============================================================ */
void UART1_SendBuffer(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U)) return;
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, UART1_TX_TIMEOUT_MS);
}

/* ============================================================
 * UART1_SendString
 * 阻塞发送 C 字符串
 * ============================================================ */
void UART1_SendString(const char *str)
{
    if (str == NULL) return;
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0U) return;
    HAL_UART_Transmit(&huart1, (uint8_t*)str, len, UART1_TX_TIMEOUT_MS);
}

/* ============================================================
 * UART1_Printf
 * 格式化字符串后通过 UART1 发送
 * 用法: UART1_Printf("speed=%d rpm\r\n", speed);
 * ============================================================ */
void UART1_Printf(const char *fmt, ...)
{
    char buf[UART1_PRINTF_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        /* 若格式化结果超出缓冲区则截断末尾, 保证不越界 */
        uint16_t txLen = (len < (int)sizeof(buf)) ? (uint16_t)len : (uint16_t)(sizeof(buf) - 1U);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, txLen, UART1_TX_TIMEOUT_MS);
    }
}
