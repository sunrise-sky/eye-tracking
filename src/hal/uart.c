/**
 * @file uart.c
 * @brief UART 通信模块实现（针对 STM32F4 USART1）
 */

#include "uart.h"
#include "config.h"
#include <string.h>

/* =====================================================================
 * 私有宏
 * ===================================================================== */
#define FRAME_HEADER_1   GAZE_FRAME_HEADER   /* 0xAA */
#define FRAME_HEADER_2   GAZE_FRAME_TAIL     /* 0x55 */
#define MAX_FRAME_LEN    32U

/* =====================================================================
 * 私有函数声明
 * ===================================================================== */
static void     usart1_send_byte(uint8_t b);
static void     send_frame(uint8_t type, const uint8_t *payload, uint8_t len);
static void     pack_float(float f, uint8_t buf[4]);

/* =====================================================================
 * 公共接口实现
 * ===================================================================== */

bool UART_Init(void)
{
    /*
     * STM32F4 USART1 配置：
     *   - PA9 (TX) / PA10 (RX)，AF7
     *   - 波特率: UART_BAUDRATE (115200)
     *   - 数据位: 8，停止位: 1，无校验
     *   - 使能 TX；RX 使能 + 接收中断（用于命令接收）
     *
     * 以下为伪代码，实际移植替换为 STM32 HAL/LL 调用：
     */
    GPIO_ConfigAFOutput(GPIOA, UART_TX_PIN, GPIO_AF_USART1);
    GPIO_ConfigAFInput (GPIOA, UART_RX_PIN, GPIO_AF_USART1);

    USART1_Configure(UART_BAUDRATE, USART_WORDLEN_8B,
                     USART_STOPBITS_1, USART_PARITY_NONE);
    USART1_EnableTx();
    USART1_EnableRx();
    USART1_EnableRxInterrupt();
    USART1_Enable();

    return true;
}

void UART_SendGaze(float gaze_x, float gaze_y, uint8_t confidence)
{
    uint8_t payload[9];
    pack_float(gaze_x, &payload[0]);
    pack_float(gaze_y, &payload[4]);
    payload[8] = confidence;
    send_frame(UART_FRAME_GAZE, payload, 9U);
}

void UART_SendPupil(float pupil_x, float pupil_y)
{
    uint8_t payload[8];
    pack_float(pupil_x, &payload[0]);
    pack_float(pupil_y, &payload[4]);
    send_frame(UART_FRAME_PUPIL, payload, 8U);
}

void UART_SendStatus(uint8_t status)
{
    send_frame(UART_FRAME_STATUS, &status, 1U);
}

bool UART_RecvByte(uint8_t *byte)
{
    /* 检查 USART1 接收缓冲区是否有数据（非阻塞） */
    if (!USART1_RxDataAvailable()) {
        return false;
    }
    *byte = USART1_ReadByte();
    return true;
}

uint8_t UART_CRC8(const uint8_t *data, uint32_t len)
{
    /* CRC-8，多项式 x^8 + x^2 + x + 1（0x07），初值 0x00 */
    uint8_t crc = 0x00U;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* =====================================================================
 * 私有函数实现
 * ===================================================================== */

/**
 * @brief 发送一帧数据
 *
 * 帧格式：[HDR1] [HDR2] [LEN] [TYPE] [PAYLOAD × LEN-1] [CRC8]
 * LEN 字段表示 TYPE + PAYLOAD 的字节数之和。
 */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[MAX_FRAME_LEN];
    uint8_t idx = 0U;

    frame[idx++] = FRAME_HEADER_1;
    frame[idx++] = FRAME_HEADER_2;
    frame[idx++] = (uint8_t)(len + 1U);  /* LEN = type(1) + payload(len) */
    frame[idx++] = type;

    for (uint8_t i = 0; i < len; i++) {
        frame[idx++] = payload[i];
    }

    /* CRC8 覆盖 LEN + TYPE + PAYLOAD */
    uint8_t crc = UART_CRC8(&frame[2], (uint32_t)(idx - 2U));
    frame[idx++] = crc;

    /* 逐字节发送 */
    for (uint8_t i = 0; i < idx; i++) {
        usart1_send_byte(frame[i]);
    }
}

/** 将 float 按小端序打包为 4 字节 */
static void pack_float(float f, uint8_t buf[4])
{
    uint32_t raw;
    memcpy(&raw, &f, 4);
    buf[0] = (uint8_t)(raw & 0xFFU);
    buf[1] = (uint8_t)((raw >>  8) & 0xFFU);
    buf[2] = (uint8_t)((raw >> 16) & 0xFFU);
    buf[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

/** 等待 USART1 发送缓冲区空后写入一字节 */
static void usart1_send_byte(uint8_t b)
{
    while (!USART1_TxReady()) { /* 等待 */ }
    USART1_WriteByte(b);
}
