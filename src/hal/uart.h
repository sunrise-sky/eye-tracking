/**
 * @file uart.h
 * @brief UART 通信模块接口
 *
 * 负责将眼动追踪结果通过串口发送给上位机（PC / 显示控制器）。
 *
 * 数据帧格式（二进制，小端序）：
 *   [0xAA] [0x55] [LEN:1B] [TYPE:1B] [PAYLOAD:LEN-1B] [CRC8:1B]
 *
 *   TYPE=0x01: 注视点坐标帧
 *     PAYLOAD: float gaze_x (4B) + float gaze_y (4B) + uint8_t conf (1B)
 *     LEN = 10
 *
 *   TYPE=0x02: 瞳孔坐标帧（未标定时）
 *     PAYLOAD: float pupil_x (4B) + float pupil_y (4B)
 *     LEN = 9
 *
 *   TYPE=0x10: 状态帧
 *     PAYLOAD: uint8_t status (1B)
 *     LEN = 2
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

/* 帧类型 */
#define UART_FRAME_GAZE    0x01  /**< 注视点坐标帧（已标定） */
#define UART_FRAME_PUPIL   0x02  /**< 瞳孔像素坐标帧 */
#define UART_FRAME_STATUS  0x10  /**< 系统状态帧 */

/* 状态码 */
#define STATUS_OK          0x00
#define STATUS_NO_PUPIL    0x01
#define STATUS_CALIB_REQ   0x02  /**< 需要标定 */
#define STATUS_CALIB_DONE  0x03

/**
 * @brief 初始化 UART 外设
 * @return true 成功，false 失败
 */
bool UART_Init(void);

/**
 * @brief 发送注视点坐标帧
 * @param gaze_x    注视点 X（归一化 0.0~1.0 或屏幕像素）
 * @param gaze_y    注视点 Y
 * @param confidence 置信度 0~255
 */
void UART_SendGaze(float gaze_x, float gaze_y, uint8_t confidence);

/**
 * @brief 发送瞳孔坐标帧（未标定时使用）
 * @param pupil_x  瞳孔质心 X（图像像素）
 * @param pupil_y  瞳孔质心 Y（图像像素）
 */
void UART_SendPupil(float pupil_x, float pupil_y);

/**
 * @brief 发送系统状态帧
 * @param status  状态码
 */
void UART_SendStatus(uint8_t status);

/**
 * @brief 接收一字节（非阻塞）
 * @param byte 输出接收到的字节
 * @return true 有数据，false 无数据
 */
bool UART_RecvByte(uint8_t *byte);

/**
 * @brief 计算 CRC8（多项式 0x07，初值 0x00）
 * @param data 数据指针
 * @param len  数据长度（字节）
 * @return CRC8 校验值
 */
uint8_t UART_CRC8(const uint8_t *data, uint32_t len);

#endif /* UART_H */
