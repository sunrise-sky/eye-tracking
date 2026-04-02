/**
 * @file main.c
 * @brief 眼动追踪嵌入式系统主程序
 *
 * 系统启动后执行以下流程：
 *   1. 系统初始化（时钟、外设）
 *   2. 摄像头初始化并启动图像采集
 *   3. 等待上位机发送标定命令；同时持续发送瞳孔坐标
 *   4. 标定完成后，持续发送注视点坐标帧
 *
 * 主循环频率目标：≥ 30 帧/秒（QVGA 下实测约 50 帧/秒）
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "camera/ov7670.h"
#include "eye_tracking/eye_tracking.h"
#include "hal/uart.h"
#include "hal/timer.h"
#include "hal/gpio.h"

/* =====================================================================
 * 宏定义
 * ===================================================================== */

/** 上位机命令字节 */
#define CMD_START_CALIB   0x31  /**< '1' - 开始标定 */
#define CMD_NEXT_POINT    0x32  /**< '2' - 确认当前标定点已完成采样 */
#define CMD_RESET_CALIB   0x30  /**< '0' - 清除标定 */

/** LED 状态指示（PA5，板载 LED） */
#define LED_PIN   GPIO_PIN_5
#define LED_PORT  GPIOA

/* =====================================================================
 * 静态数据
 * ===================================================================== */

/** 帧缓冲区（YUV422: 每像素 2 字节） */
static uint8_t s_frame_buf[IMAGE_HEIGHT * IMAGE_WIDTH * 2];

/** OV7670 帧描述符 */
static OV7670_Frame_t s_frame = {
    .data   = s_frame_buf,
    .width  = IMAGE_WIDTH,
    .height = IMAGE_HEIGHT,
    .size   = sizeof(s_frame_buf),
    .ready  = false,
};

/** 眼动追踪句柄 */
static EyeTracker_t s_tracker;

/**
 * 9 点标定的屏幕坐标（归一化为 0.0~1.0）。
 * 上位机应在接收到 CMD_START_CALIB 后依次展示这 9 个点。
 */
static const Point2f_t s_calib_screen[CALIB_POINT_COUNT] = {
    { 0.1f, 0.1f }, { 0.5f, 0.1f }, { 0.9f, 0.1f },
    { 0.1f, 0.5f }, { 0.5f, 0.5f }, { 0.9f, 0.5f },
    { 0.1f, 0.9f }, { 0.5f, 0.9f }, { 0.9f, 0.9f },
};

/* =====================================================================
 * 私有函数声明
 * ===================================================================== */
static void system_init(void);
static void handle_command(uint8_t cmd, EyeTracker_t *tracker);
static void led_set(bool on);
static void led_toggle(void);

/* =====================================================================
 * 主程序入口
 * ===================================================================== */

int main(void)
{
    /* -----------------------------------------------------------------
     * 1. 系统底层初始化
     * --------------------------------------------------------------- */
    system_init();

    /* -----------------------------------------------------------------
     * 2. 摄像头初始化
     * --------------------------------------------------------------- */
    const OV7670_Config_t cam_cfg = {
        .format     = OV7670_FMT_YUV422,
        .resolution = OV7670_RES_QVGA,
        .brightness = 128,
        .contrast   = 128,
        .mirror     = false,
        .flip       = false,
    };

    if (!OV7670_Init(&cam_cfg)) {
        /* 摄像头初始化失败：快速闪烁 LED 报错，并循环等待 */
        UART_SendStatus(0xE1);  /* 错误码 0xE1: 摄像头失败 */
        while (1) {
            led_toggle();
            Timer_DelayMs(100);
        }
    }

    /* -----------------------------------------------------------------
     * 3. 眼动追踪模块初始化
     * --------------------------------------------------------------- */
    EyeTracker_Init(&s_tracker);

    /* -----------------------------------------------------------------
     * 4. 启动摄像头采集（DMA 持续采集模式）
     * --------------------------------------------------------------- */
    if (!OV7670_StartCapture(&s_frame)) {
        UART_SendStatus(0xE2);
        while (1) {
            led_toggle();
            Timer_DelayMs(200);
        }
    }

    /* 发送启动就绪状态 */
    UART_SendStatus(STATUS_CALIB_REQ);
    led_set(true);

    /* -----------------------------------------------------------------
     * 5. 主循环
     * --------------------------------------------------------------- */
    uint32_t frame_ts = 0;

    while (1) {
        /* 5.1 等待新帧就绪（超时 100 ms） */
        if (!OV7670_WaitFrame(&s_frame, 100U)) {
            /* 帧超时：可能摄像头故障，发送状态并继续 */
            UART_SendStatus(STATUS_NO_PUPIL);
            continue;
        }

        frame_ts = Timer_GetTick();
        (void)frame_ts;

        /* 5.2 处理来自上位机的命令（非阻塞） */
        uint8_t cmd;
        if (UART_RecvByte(&cmd)) {
            handle_command(cmd, &s_tracker);
        }

        /* 5.3 图像处理 + 瞳孔检测 + 注视点计算 */
        Point2f_t gaze = { 0.0f, 0.0f };
        bool detected = EyeTracker_ProcessFrame(&s_tracker,
                                                 s_frame.data,
                                                 s_frame.size,
                                                 &gaze);

        /* 5.4 向上位机发送结果 */
        if (!detected) {
            UART_SendStatus(STATUS_NO_PUPIL);
            led_set(false);
            continue;
        }

        led_set(true);

        if (s_tracker.calib.valid) {
            /* 已标定：发送注视点坐标 */
            uint8_t conf = (uint8_t)(s_tracker.last_pupil.confidence * 255.0f);
            UART_SendGaze(gaze.x, gaze.y, conf);
        } else {
            /* 未标定：发送瞳孔像素坐标 */
            UART_SendPupil(gaze.x, gaze.y);
        }

        /* 5.5 标定模式下：为当前标定点添加样本 */
        if (s_tracker.calib_state == CALIB_IN_PROGRESS) {
            CalibState_t st = EyeTracker_CalibAddSample(
                                  &s_tracker, &s_tracker.last_pupil);
            if (st == CALIB_DONE) {
                UART_SendStatus(STATUS_CALIB_DONE);
            } else if (st == CALIB_ERROR) {
                UART_SendStatus(0xE3);  /* 标定计算失败 */
            }
        }
    }

    /* 不可达 */
    return 0;
}

/* =====================================================================
 * 私有函数实现
 * ===================================================================== */

/**
 * @brief 系统初始化（时钟、GPIO、UART、定时器）
 */
static void system_init(void)
{
    /* 配置系统时钟（外部晶振 8 MHz，PLL 倍频至 168 MHz） */
    SystemClock_Config();

    /* 使能外设时钟 */
    RCC_EnableGPIOA();
    RCC_EnableGPIOB();
    RCC_EnableGPIOC();
    RCC_EnableGPIOE();
    RCC_EnableUSART1();
    RCC_EnableI2C1();
    RCC_EnableDCMI();
    RCC_EnableDMA2();
    RCC_EnableTIM2();

    /* LED GPIO 初始化 */
    GPIO_ConfigOutput(LED_PORT, LED_PIN);
    led_set(false);

    /* UART 初始化 */
    UART_Init();

    /* 系统节拍定时器（1 ms 中断） */
    Timer_Init();
}

/**
 * @brief 处理来自上位机的单字节命令
 */
static void handle_command(uint8_t cmd, EyeTracker_t *tracker)
{
    switch (cmd) {
    case CMD_START_CALIB:
        /* 开始标定 */
        EyeTracker_CalibStart(tracker, s_calib_screen);
        UART_SendStatus(STATUS_CALIB_REQ);
        break;

    case CMD_RESET_CALIB:
        /* 清除标定参数 */
        tracker->calib.valid  = false;
        tracker->calib_state  = CALIB_IDLE;
        UART_SendStatus(STATUS_OK);
        break;

    default:
        /* 未知命令，忽略 */
        break;
    }
}

static void led_set(bool on)
{
    GPIO_WritePin(LED_PORT, LED_PIN, on);
}

static void led_toggle(void)
{
    GPIO_TogglePin(LED_PORT, LED_PIN);
}
