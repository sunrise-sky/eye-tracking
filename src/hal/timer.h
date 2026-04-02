/**
 * @file timer.h
 * @brief 定时器与延时函数接口
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/**
 * @brief 初始化 SysTick，产生 1 ms 中断用于系统节拍
 */
void Timer_Init(void);

/**
 * @brief 获取系统启动以来的毫秒计数
 * @return 毫秒时间戳
 */
uint32_t Timer_GetTick(void);

/**
 * @brief 阻塞延时（毫秒）
 * @param ms 延时时间（毫秒）
 */
void Timer_DelayMs(uint32_t ms);

/**
 * @brief 阻塞延时（微秒，基于 DWT 周期计数器）
 * @param us 延时时间（微秒）
 */
void Timer_DelayUs(uint32_t us);

#endif /* TIMER_H */
