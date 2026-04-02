/**
 * @file i2c.h
 * @brief I2C / SCCB 接口（用于 OV7670 寄存器读写）
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化 I2C1 外设（400 kHz，Fast Mode）
 * @return true 成功
 */
bool I2C_Init(void);

/**
 * @brief I2C 写操作
 * @param dev_addr 7-bit 设备地址（左移 1 位的写地址）
 * @param data     数据缓冲区
 * @param len      字节数
 * @return true 成功，false NACK 或超时
 */
bool I2C_Write(uint8_t dev_addr, const uint8_t *data, uint32_t len);

/**
 * @brief I2C 读操作
 * @param dev_addr 7-bit 设备地址（左移 1 位的读地址，最低位置 1）
 * @param data     接收缓冲区
 * @param len      期望读取字节数
 * @return true 成功，false NACK 或超时
 */
bool I2C_Read(uint8_t dev_addr, uint8_t *data, uint32_t len);

#endif /* I2C_H */
