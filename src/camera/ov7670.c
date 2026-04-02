/**
 * @file ov7670.c
 * @brief OV7670 摄像头驱动实现
 *
 * 通过 SCCB（Serial Camera Control Bus，兼容 I2C）配置寄存器，
 * 通过 STM32F4 DCMI（Digital Camera Interface）+ DMA 采集图像帧。
 *
 * 硬件连接（STM32F407ZGT6 示例）：
 *   OV7670 SIOC  ->  PB8  (I2C1_SCL)
 *   OV7670 SIOD  ->  PB9  (I2C1_SDA)
 *   OV7670 XCLK  ->  PA8  (MCO1, 24 MHz)
 *   OV7670 PCLK  ->  PA6  (DCMI_PIXCLK)
 *   OV7670 HREF  ->  PA4  (DCMI_HSYNC)
 *   OV7670 VSYNC ->  PB7  (DCMI_VSYNC)
 *   OV7670 D0~D7 ->  PC6/PC7/PE0/PE1/PE4/PB6/PE5/PE6
 */

#include "ov7670.h"
#include "../hal/i2c.h"
#include "../hal/gpio.h"
#include "../hal/timer.h"
#include <string.h>

/* =====================================================================
 * 私有常量 —— 寄存器初始化表
 * ===================================================================== */

/** 初始化表条目结构 */
typedef struct {
    uint8_t reg;
    uint8_t val;
} RegEntry_t;

/** 结束标记 */
#define REG_END  { OV7670_REG_RESET, 0x00 }

/**
 * OV7670 QVGA YUV422 基础配置序列。
 * 参考 OV7670 Implementation Guide 及 Linux 内核 ov7670 驱动。
 */
static const RegEntry_t s_ov7670_qvga_yuv[] = {
    /* 软件复位 */
    { OV7670_REG_COM7,   COM7_BIT_RESET },
    /* 时钟分频：PCLK = XCLK / 2 = 12 MHz */
    { OV7670_REG_CLKRC,  0x01 },
    /* QVGA + YUV422 */
    { OV7670_REG_COM7,   COM7_FMT_QVGA | COM7_FMT_YUV },
    /* 自动增益、自动白平衡、自动曝光全开 */
    { OV7670_REG_COM8,   0xE7 },
    /* 水平/垂直采样窗口 */
    { OV7670_REG_HSTART, 0x16 },
    { OV7670_REG_HSTOP,  0x04 },
    { OV7670_REG_VSTRT,  0x02 },
    { OV7670_REG_VSTOP,  0x7A },
    { OV7670_REG_VREF,   0x0A },
    /* 输出数据格式：全范围 0~255 */
    { OV7670_REG_COM15,  0xC0 },
    /* 关闭颜色条测试图 */
    { OV7670_REG_COM17,  0x00 },
    /* 行缓冲选项：使能 UV 交替输出 */
    { OV7670_REG_TSLB,   0x04 },
    /* AEC 稳定区间 */
    { OV7670_REG_AEW,    0x75 },
    { OV7670_REG_AEB,    0x63 },
    /* 无镜像/无翻转 */
    { OV7670_REG_MVFP,   0x00 },
    /* 缩放：关闭数字缩放 */
    { OV7670_REG_SCALING_XSC, 0x3A },
    { OV7670_REG_SCALING_YSC, 0x35 },
    REG_END
};

/* =====================================================================
 * 模块内部状态
 * ===================================================================== */
static OV7670_Frame_t *s_current_frame = NULL;
static volatile bool   s_frame_ready   = false;

/* =====================================================================
 * 私有函数声明
 * ===================================================================== */
static bool     sccb_write(uint8_t reg, uint8_t val);
static bool     sccb_read(uint8_t reg, uint8_t *val);
static bool     apply_reg_table(const RegEntry_t *table);
static void     dcmi_init(void);
static void     dma_init(OV7670_Frame_t *frame);

/* =====================================================================
 * 公共接口实现
 * ===================================================================== */

bool OV7670_Init(const OV7670_Config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    /* 1. 初始化 XCLK（MCO1 输出 24 MHz，HSE 分频） */
    GPIO_ConfigMCO1(OV7670_XCLK_HZ);

    /* 2. 初始化 SCCB/I2C 总线 */
    if (!I2C_Init()) {
        return false;
    }

    /* 3. 等待摄像头上电稳定 */
    Timer_DelayMs(100);

    /* 4. 验证设备 ID */
    uint8_t pid, ver;
    if (!OV7670_ReadID(&pid, &ver)) {
        return false;
    }
    if (pid != 0x76 || ver != 0x73) {
        return false;  /* 器件 ID 不匹配 */
    }

    /* 5. 发送初始化寄存器序列 */
    if (!apply_reg_table(s_ov7670_qvga_yuv)) {
        return false;
    }

    /* 6. 根据配置覆盖亮度/对比度/镜像参数 */
    OV7670_SetBrightness(cfg->brightness);
    OV7670_SetContrast(cfg->contrast);

    if (cfg->mirror || cfg->flip) {
        uint8_t mvfp = 0x00;
        if (cfg->mirror) mvfp |= (1 << 5);
        if (cfg->flip)   mvfp |= (1 << 4);
        OV7670_WriteReg(OV7670_REG_MVFP, mvfp);
    }

    /* 7. 初始化 DCMI 外设 */
    dcmi_init();

    /* 等待第一帧输出前的稳定期 */
    Timer_DelayMs(200);

    return true;
}

void OV7670_Reset(void)
{
    OV7670_WriteReg(OV7670_REG_COM7, COM7_BIT_RESET);
    Timer_DelayMs(50);
}

bool OV7670_ReadID(uint8_t *pid, uint8_t *ver)
{
    if (!sccb_read(OV7670_REG_PID, pid)) return false;
    if (!sccb_read(OV7670_REG_VER, ver)) return false;
    return true;
}

bool OV7670_WriteReg(uint8_t reg, uint8_t val)
{
    return sccb_write(reg, val);
}

bool OV7670_ReadReg(uint8_t reg, uint8_t *val)
{
    return sccb_read(reg, val);
}

void OV7670_SetBrightness(uint8_t brightness)
{
    /*
     * OV7670 亮度由寄存器 0x55 (BRIGHT) 控制。
     * 映射：0~127 正增益，128~255 负增益（以 0x80 为分界）。
     */
    uint8_t reg_val;
    if (brightness >= 128) {
        reg_val = (uint8_t)(brightness - 128);
    } else {
        reg_val = (uint8_t)(128 - brightness) | 0x80;
    }
    sccb_write(0x55, reg_val);
}

void OV7670_SetContrast(uint8_t contrast)
{
    /*
     * OV7670 对比度由寄存器 0x56 (CONTRAS) 控制，直接映射 0~255。
     */
    sccb_write(0x56, contrast);
}

bool OV7670_StartCapture(OV7670_Frame_t *frame)
{
    if (frame == NULL || frame->data == NULL) {
        return false;
    }

    s_current_frame = frame;
    s_frame_ready   = false;
    frame->ready    = false;

    /* 配置 DMA 将 DCMI FIFO 数据传输到帧缓冲区 */
    dma_init(frame);

    /* 使能 DCMI 连续采集模式 */
    /* 注意：以下寄存器操作对应 STM32 HAL DCMI_CR 寄存器位 */
    /* DCMI_CR: ENABLE=1, CM=0（连续模式）, FCRC=00（所有帧）*/
    DCMI_Enable();

    return true;
}

void OV7670_StopCapture(void)
{
    DCMI_Disable();
    s_current_frame = NULL;
}

bool OV7670_WaitFrame(OV7670_Frame_t *frame, uint32_t timeout_ms)
{
    uint32_t start = Timer_GetTick();
    while (!s_frame_ready) {
        if ((Timer_GetTick() - start) >= timeout_ms) {
            return false;  /* 超时 */
        }
    }
    s_frame_ready  = false;
    frame->ready   = true;
    return true;
}

void OV7670_FrameCompleteCallback(void)
{
    /* 由 DCMI 帧完成中断 ISR 调用 */
    if (s_current_frame != NULL) {
        s_frame_ready = true;
    }
}

/* =====================================================================
 * 私有函数实现
 * ===================================================================== */

/**
 * @brief 通过 SCCB 写一个寄存器
 */
static bool sccb_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return I2C_Write(OV7670_I2C_ADDR, buf, 2);
}

/**
 * @brief 通过 SCCB 读一个寄存器
 * SCCB 读操作需要先写地址（phase 1），再读数据（phase 2），
 * 两个独立的起始条件。
 */
static bool sccb_read(uint8_t reg, uint8_t *val)
{
    /* Phase 1: 写寄存器地址 */
    if (!I2C_Write(OV7670_I2C_ADDR, &reg, 1)) {
        return false;
    }
    /* Phase 2: 读取 1 字节数据 */
    return I2C_Read(OV7670_I2C_ADDR | 0x01, val, 1);
}

/**
 * @brief 应用寄存器初始化表，直到遇到 REG_END 条目
 */
static bool apply_reg_table(const RegEntry_t *table)
{
    for (uint32_t i = 0; ; i++) {
        if (table[i].reg == OV7670_REG_RESET && table[i].val == 0x00) {
            break;  /* 结束标记 */
        }
        if (!sccb_write(table[i].reg, table[i].val)) {
            return false;
        }
        /* OV7670 寄存器写入后需要短暂延时 */
        Timer_DelayUs(100);
    }
    return true;
}

/**
 * @brief 初始化 DCMI 外设（8-bit 并行，极性配置）
 *
 * 以下为伪代码形式的寄存器配置，实际移植时替换为对应 HAL/LL 调用。
 */
static void dcmi_init(void)
{
    /*
     * 配置要点：
     *   - 8-bit 数据宽度
     *   - PCLK 上升沿采样
     *   - HSYNC 低电平有效
     *   - VSYNC 高电平有效
     *   - 硬件同步模式
     *   - 使能帧完成中断
     */
    DCMI_ConfigureHardwareSync(
        DCMI_PCLK_RISING,
        DCMI_HSYNC_LOW,
        DCMI_VSYNC_HIGH,
        DCMI_CAPTURE_ALL_FRAMES
    );
    DCMI_EnableInterrupt(DCMI_IT_FRAME);
}

/**
 * @brief 配置 DMA2 Stream1（DCMI -> 内存）
 */
static void dma_init(OV7670_Frame_t *frame)
{
    DMA_ConfigPeriphToMem(
        DMA2_STREAM1,
        DCMI_DR_ADDRESS,          /* 外设地址：DCMI 数据寄存器 */
        (uint32_t)frame->data,    /* 内存目标地址 */
        frame->size / 4,          /* 传输长度（32-bit 字数） */
        DMA_WIDTH_WORD,
        DMA_CIRCULAR_MODE
    );
    DMA_Enable(DMA2_STREAM1);
}
