/**
 * @file ov7670.h
 * @brief OV7670 摄像头驱动接口
 *
 * 提供 OV7670 CMOS 图像传感器的初始化、配置与图像采集接口。
 * 硬件接口：SCCB（兼容 I2C）配置 + STM32 DCMI 数字摄像头接口采集。
 */

#ifndef OV7670_H
#define OV7670_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* =====================================================================
 * 寄存器地址定义
 * ===================================================================== */
#define OV7670_REG_GAIN       0x00  /**< AGC 增益控制 */
#define OV7670_REG_BLUE       0x01  /**< AWB 蓝色通道增益 */
#define OV7670_REG_RED        0x02  /**< AWB 红色通道增益 */
#define OV7670_REG_VREF       0x03  /**< 垂直帧控制 */
#define OV7670_REG_COM1       0x04  /**< 公共控制 1 */
#define OV7670_REG_COM2       0x09  /**< 公共控制 2 */
#define OV7670_REG_PID        0x0A  /**< 产品 ID 高字节，固定值 0x76 */
#define OV7670_REG_VER        0x0B  /**< 产品 ID 低字节，固定值 0x73 */
#define OV7670_REG_COM7       0x12  /**< 公共控制 7（格式选择） */
#define OV7670_REG_COM8       0x13  /**< 公共控制 8（AGC/AWB/AEC） */
#define OV7670_REG_COM10      0x15  /**< 公共控制 10（同步信号极性） */
#define OV7670_REG_HSTART     0x17  /**< 水平起始点 */
#define OV7670_REG_HSTOP      0x18  /**< 水平终止点 */
#define OV7670_REG_VSTRT      0x19  /**< 垂直起始点 */
#define OV7670_REG_VSTOP      0x1A  /**< 垂直终止点 */
#define OV7670_REG_MVFP       0x1E  /**< 镜像/翻转 */
#define OV7670_REG_AEW        0x24  /**< AGC/AEC 稳定上限 */
#define OV7670_REG_AEB        0x25  /**< AGC/AEC 稳定下限 */
#define OV7670_REG_COM15      0x40  /**< 输出数据范围 */
#define OV7670_REG_COM17      0x42  /**< DSP 颜色条使能 */
#define OV7670_REG_CLKRC      0x11  /**< 时钟控制 */
#define OV7670_REG_TSLB       0x3A  /**< 行缓冲测试选项 */
#define OV7670_REG_COM11      0x3B  /**< 公共控制 11 */
#define OV7670_REG_SCALING_XSC 0x70 /**< 水平缩放系数 */
#define OV7670_REG_SCALING_YSC 0x71 /**< 垂直缩放系数 */
#define OV7670_REG_RESET      0xFF  /**< 软件复位（哨兵寄存器地址） */

/* COM7 格式位 */
#define COM7_FMT_QVGA         0x10  /**< QVGA 320×240 */
#define COM7_FMT_VGA          0x00  /**< VGA  640×480 */
#define COM7_FMT_YUV          0x00  /**< YUV 输出 */
#define COM7_FMT_RGB          0x04  /**< RGB 输出 */
#define COM7_BIT_RESET        0x80  /**< 软件复位 */

/* =====================================================================
 * 数据类型
 * ===================================================================== */

/** 摄像头输出格式 */
typedef enum {
    OV7670_FMT_YUV422 = 0,  /**< YUV422 格式（Y 分量可直接用作灰度） */
    OV7670_FMT_RGB565,       /**< RGB565 格式 */
    OV7670_FMT_GRAY,         /**< 灰度格式（内部转换，仅输出 Y 分量） */
} OV7670_Format_t;

/** 摄像头分辨率 */
typedef enum {
    OV7670_RES_QVGA = 0,    /**< 320×240 */
    OV7670_RES_VGA,          /**< 640×480 */
    OV7670_RES_QQVGA,        /**< 160×120 */
} OV7670_Resolution_t;

/** 摄像头初始化参数 */
typedef struct {
    OV7670_Format_t     format;     /**< 输出格式 */
    OV7670_Resolution_t resolution; /**< 分辨率 */
    uint8_t             brightness; /**< 亮度 0-255 */
    uint8_t             contrast;   /**< 对比度 0-255 */
    bool                mirror;     /**< 水平镜像 */
    bool                flip;       /**< 垂直翻转 */
} OV7670_Config_t;

/** 帧缓冲区描述符 */
typedef struct {
    uint8_t  *data;     /**< 指向帧数据的指针 */
    uint32_t  width;    /**< 帧宽度（像素） */
    uint32_t  height;   /**< 帧高度（像素） */
    uint32_t  size;     /**< 数据字节数 */
    bool      ready;    /**< 帧是否已就绪 */
} OV7670_Frame_t;

/* =====================================================================
 * 函数接口
 * ===================================================================== */

/**
 * @brief 初始化 OV7670 摄像头
 * @param cfg 指向配置参数结构体的指针
 * @return true 成功，false 失败
 */
bool OV7670_Init(const OV7670_Config_t *cfg);

/**
 * @brief 复位摄像头至默认状态
 */
void OV7670_Reset(void);

/**
 * @brief 读取摄像头产品 ID，用于验证通信
 * @param pid  输出参数，高字节 PID（应为 0x76）
 * @param ver  输出参数，低字节 VER（应为 0x73）
 * @return true 读取成功，false 失败
 */
bool OV7670_ReadID(uint8_t *pid, uint8_t *ver);

/**
 * @brief 写摄像头寄存器
 * @param reg  寄存器地址
 * @param val  写入值
 * @return true 成功，false SCCB 通信错误
 */
bool OV7670_WriteReg(uint8_t reg, uint8_t val);

/**
 * @brief 读摄像头寄存器
 * @param reg  寄存器地址
 * @param val  输出参数，读取到的值
 * @return true 成功，false SCCB 通信错误
 */
bool OV7670_ReadReg(uint8_t reg, uint8_t *val);

/**
 * @brief 设置亮度
 * @param brightness 亮度值 0-255
 */
void OV7670_SetBrightness(uint8_t brightness);

/**
 * @brief 设置对比度
 * @param contrast 对比度值 0-255
 */
void OV7670_SetContrast(uint8_t contrast);

/**
 * @brief 启动连续帧采集（DMA 模式）
 * @param frame 指向帧缓冲区描述符的指针
 * @return true 成功，false 失败
 */
bool OV7670_StartCapture(OV7670_Frame_t *frame);

/**
 * @brief 停止帧采集
 */
void OV7670_StopCapture(void);

/**
 * @brief 等待一帧数据就绪（阻塞，带超时）
 * @param frame    帧缓冲区描述符
 * @param timeout_ms 超时时间（毫秒）
 * @return true 成功获取新帧，false 超时
 */
bool OV7670_WaitFrame(OV7670_Frame_t *frame, uint32_t timeout_ms);

/**
 * @brief DCMI 帧完成回调（由中断处理程序调用）
 */
void OV7670_FrameCompleteCallback(void);

#endif /* OV7670_H */
