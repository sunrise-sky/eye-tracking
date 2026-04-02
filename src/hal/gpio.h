/**
 * @file gpio.h
 * @brief GPIO 操作接口（平台抽象层）
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

/* 引脚编号定义（与 STM32 HAL 保持一致） */
#define GPIO_PIN_4   ((uint16_t)0x0010U)
#define GPIO_PIN_5   ((uint16_t)0x0020U)
#define GPIO_PIN_6   ((uint16_t)0x0040U)
#define GPIO_PIN_7   ((uint16_t)0x0080U)
#define GPIO_PIN_8   ((uint16_t)0x0100U)
#define GPIO_PIN_9   ((uint16_t)0x0200U)
#define GPIO_PIN_10  ((uint16_t)0x0400U)

/* GPIO 端口（伪地址，实际由链接脚本映射到 STM32 外设基地址） */
#define GPIOA  ((GPIO_Port_t *)0x40020000UL)
#define GPIOB  ((GPIO_Port_t *)0x40020400UL)
#define GPIOC  ((GPIO_Port_t *)0x40020800UL)
#define GPIOE  ((GPIO_Port_t *)0x40021000UL)

/* 复用功能编号（STM32F4） */
#define GPIO_AF_USART1  7U
#define GPIO_AF_I2C1    4U
#define GPIO_AF_DCMI    13U

/** GPIO 端口类型（不透明指针占位符） */
typedef struct GPIO_Port_s GPIO_Port_t;

/**
 * @brief 配置引脚为推挽输出
 */
void GPIO_ConfigOutput(GPIO_Port_t *port, uint16_t pin);

/**
 * @brief 配置引脚为复用功能输出
 */
void GPIO_ConfigAFOutput(GPIO_Port_t *port, uint16_t pin, uint8_t af);

/**
 * @brief 配置引脚为复用功能输入
 */
void GPIO_ConfigAFInput(GPIO_Port_t *port, uint16_t pin, uint8_t af);

/**
 * @brief 设置引脚电平
 */
void GPIO_WritePin(GPIO_Port_t *port, uint16_t pin, bool state);

/**
 * @brief 翻转引脚电平
 */
void GPIO_TogglePin(GPIO_Port_t *port, uint16_t pin);

/**
 * @brief 读取引脚电平
 */
bool GPIO_ReadPin(GPIO_Port_t *port, uint16_t pin);

/**
 * @brief 配置 MCO1 输出（用于给 OV7670 提供 XCLK）
 * @param freq_hz 目标频率（Hz），内部根据 PLL 参数选择合适分频
 */
void GPIO_ConfigMCO1(uint32_t freq_hz);

#endif /* GPIO_H */
