/**
 * @file eye_tracking.h
 * @brief 眼动追踪核心算法接口
 *
 * 本模块在图像处理模块的输出基础上完成：
 *   1. 瞳孔检测（从 Blob 列表中选出最可能的瞳孔区域）
 *   2. 瞳孔质心坐标精确化（质心法）
 *   3. 注视点标定（9 点线性映射）
 *   4. 注视点坐标输出
 */

#ifndef EYE_TRACKING_H
#define EYE_TRACKING_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "../image_proc/image_proc.h"

/* =====================================================================
 * 数据类型
 * ===================================================================== */

/** 浮点 2D 坐标点 */
typedef struct {
    float x;
    float y;
} Point2f_t;

/** 整型 2D 坐标点 */
typedef struct {
    int32_t x;
    int32_t y;
} Point2i_t;

/** 瞳孔检测结果 */
typedef struct {
    bool      detected;    /**< 是否检测到瞳孔 */
    Point2f_t center;      /**< 瞳孔质心坐标（像素） */
    uint32_t  area;        /**< 瞳孔面积（像素） */
    float     confidence;  /**< 置信度 [0.0, 1.0] */
} PupilResult_t;

/** 标定模式状态 */
typedef enum {
    CALIB_IDLE = 0,      /**< 未在标定 */
    CALIB_IN_PROGRESS,   /**< 标定进行中 */
    CALIB_DONE,          /**< 标定已完成 */
    CALIB_ERROR,         /**< 标定失败 */
} CalibState_t;

/** 标定参数（仿射变换矩阵：瞳孔坐标 -> 屏幕坐标） */
typedef struct {
    float a[6];           /**< 仿射变换系数 [a0..a5]，
                           *   screen_x = a0*px + a1*py + a2
                           *   screen_y = a3*px + a4*py + a5 */
    bool  valid;          /**< 标定结果是否有效 */
} CalibParams_t;

/** 眼动追踪模块句柄 */
typedef struct {
    CalibParams_t  calib;           /**< 标定参数 */
    CalibState_t   calib_state;     /**< 当前标定状态 */
    uint32_t       calib_point_idx; /**< 当前标定点索引 */
    uint32_t       calib_sample_cnt;/**< 当前标定点已采样帧数 */
    Point2f_t      calib_accum;     /**< 标定采样累加器 */
    Point2f_t      calib_pupil [CALIB_POINT_COUNT]; /**< 各标定点对应瞳孔坐标均值 */
    Point2f_t      calib_screen[CALIB_POINT_COUNT]; /**< 各标定点对应屏幕坐标 */
    PupilResult_t  last_pupil;      /**< 上一帧瞳孔结果 */
    Point2f_t      last_gaze;       /**< 上一帧注视点屏幕坐标 */
    uint32_t       frame_count;     /**< 累计处理帧数 */
} EyeTracker_t;

/* =====================================================================
 * 函数接口
 * ===================================================================== */

/**
 * @brief 初始化眼动追踪模块
 * @param tracker 指向 EyeTracker_t 句柄的指针
 */
void EyeTracker_Init(EyeTracker_t *tracker);

/**
 * @brief 从 Blob 列表中检测瞳孔
 *
 * 选择策略：
 *   1. 过滤圆形度低于阈值的 Blob
 *   2. 若有多个候选，优先选面积最大且最接近上一帧瞳孔位置的 Blob
 *   3. 考虑图像中心权重（眼睛一般在图像中部）
 *
 * @param tracker 眼动追踪句柄
 * @param blobs   Blob 列表（已按面积降序排列）
 * @param result  输出瞳孔检测结果
 */
void EyeTracker_DetectPupil(EyeTracker_t *tracker,
                             const BlobList_t *blobs,
                             PupilResult_t *result);

/**
 * @brief 开始 9 点标定流程
 *
 * 调用此函数后，调用方需依次在屏幕上显示 9 个标定点，
 * 并在用户注视每个点时重复调用 EyeTracker_CalibAddSample()。
 *
 * @param tracker       眼动追踪句柄
 * @param screen_points 9 个标定点的屏幕坐标（像素或归一化坐标）
 */
void EyeTracker_CalibStart(EyeTracker_t *tracker,
                            const Point2f_t screen_points[CALIB_POINT_COUNT]);

/**
 * @brief 为当前标定点添加一帧瞳孔位置采样
 *
 * 连续调用 CALIB_SAMPLE_FRAMES 次后自动进入下一个标定点；
 * 全部完成后自动调用 EyeTracker_CalibCompute()。
 *
 * @param tracker 眼动追踪句柄
 * @param pupil   当前帧检测到的瞳孔坐标
 * @return 当前标定状态
 */
CalibState_t EyeTracker_CalibAddSample(EyeTracker_t *tracker,
                                        const PupilResult_t *pupil);

/**
 * @brief 根据采集的标定数据计算仿射变换矩阵（最小二乘法）
 * @param tracker 眼动追踪句柄
 * @return true 成功，false 矩阵奇异无法求解
 */
bool EyeTracker_CalibCompute(EyeTracker_t *tracker);

/**
 * @brief 将瞳孔坐标映射为注视点屏幕坐标
 *
 * 使用已标定的仿射变换，将瞳孔质心（像素坐标）转换为
 * 屏幕上的注视点坐标。
 *
 * @param tracker   眼动追踪句柄
 * @param pupil_pos 瞳孔质心坐标
 * @param gaze_pos  输出注视点屏幕坐标
 * @return true 成功，false 标定无效
 */
bool EyeTracker_PupilToGaze(const EyeTracker_t *tracker,
                              const Point2f_t *pupil_pos,
                              Point2f_t *gaze_pos);

/**
 * @brief 对注视点做指数移动平均平滑，减少抖动
 *
 * @param current   当前原始注视点
 * @param smoothed  上一平滑后注视点（输入/输出）
 * @param alpha     平滑系数 [0,1]，越小越平滑（典型值 0.3）
 */
void EyeTracker_SmoothGaze(const Point2f_t *current,
                            Point2f_t *smoothed,
                            float alpha);

/**
 * @brief 主处理入口：处理一帧图像，输出注视点
 *
 * 内部完成：灰度提取 -> 预处理 -> 二值化 -> 形态学 -> Blob 分析
 *           -> 瞳孔检测 -> 注视点映射 -> 平滑滤波
 *
 * @param tracker    眼动追踪句柄
 * @param yuv_frame  原始 YUV422 帧数据
 * @param frame_size 帧数据字节数
 * @param gaze_out   输出注视点坐标（若未标定则为瞳孔像素坐标）
 * @return true 瞳孔检测成功，false 本帧未检测到瞳孔
 */
bool EyeTracker_ProcessFrame(EyeTracker_t *tracker,
                              const uint8_t *yuv_frame,
                              uint32_t       frame_size,
                              Point2f_t     *gaze_out);

#endif /* EYE_TRACKING_H */
