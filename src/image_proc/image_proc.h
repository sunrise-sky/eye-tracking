/**
 * @file image_proc.h
 * @brief 图像处理模块接口
 *
 * 提供眼动追踪所需的基本图像处理算法：
 *   - YUV422 灰度提取
 *   - 高斯模糊（降噪）
 *   - 自适应阈值二值化
 *   - 形态学腐蚀/膨胀
 *   - 连通区域标记（Blob 分析）
 */

#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* =====================================================================
 * 数据类型定义
 * ===================================================================== */

/** 灰度图像结构 */
typedef struct {
    uint8_t  data[IMAGE_HEIGHT][IMAGE_WIDTH];  /**< 灰度像素数据 */
    uint32_t width;   /**< 图像宽度 */
    uint32_t height;  /**< 图像高度 */
} GrayImage_t;

/** 二值图像结构（每像素 1 字节，0 = 背景，255 = 前景） */
typedef struct {
    uint8_t  data[IMAGE_HEIGHT][IMAGE_WIDTH];
    uint32_t width;
    uint32_t height;
} BinaryImage_t;

/** 连通区域（Blob）描述符 */
typedef struct {
    uint32_t cx;        /**< 质心 X 坐标（像素） */
    uint32_t cy;        /**< 质心 Y 坐标（像素） */
    uint32_t area;      /**< 面积（像素数） */
    uint32_t x_min;     /**< 包围盒左边界 */
    uint32_t x_max;     /**< 包围盒右边界 */
    uint32_t y_min;     /**< 包围盒上边界 */
    uint32_t y_max;     /**< 包围盒下边界 */
    uint32_t perimeter; /**< 周长估计值（像素数） */
    float    circularity; /**< 圆形度 = 4π·Area / Perimeter² */
} Blob_t;

/** Blob 列表结构 */
typedef struct {
    Blob_t   blobs[MAX_BLOBS];
    uint32_t count;
} BlobList_t;

/* =====================================================================
 * 图像处理接口
 * ===================================================================== */

/**
 * @brief 从 YUV422 原始帧提取 Y（亮度）通道为灰度图像
 *
 * YUV422 格式：[Y0 U0 Y1 V0] 每 4 字节表示 2 个像素，
 * 取每组中的 Y0、Y1 字节即可得到灰度图。
 *
 * @param yuv_data  输入 YUV422 数据指针（大小 = width * height * 2）
 * @param gray      输出灰度图像
 */
void ImgProc_YUV422ToGray(const uint8_t *yuv_data, GrayImage_t *gray);

/**
 * @brief 3×3 高斯模糊（sigma ≈ 1.0）
 *
 * 卷积核：
 *   1  2  1
 *   2  4  2  (归一化后)
 *   1  2  1
 *
 * @param src  输入灰度图像
 * @param dst  输出灰度图像（可与 src 相同，内部使用临时缓冲）
 */
void ImgProc_GaussianBlur3x3(const GrayImage_t *src, GrayImage_t *dst);

/**
 * @brief 全局固定阈值二值化
 *
 * dst(x,y) = 255 if src(x,y) <= threshold else 0
 * （低于阈值为前景，适合深色瞳孔检测）
 *
 * @param src       输入灰度图像
 * @param dst       输出二值图像
 * @param threshold 阈值（0-255）
 */
void ImgProc_Threshold(const GrayImage_t *src, BinaryImage_t *dst,
                        uint8_t threshold);

/**
 * @brief 自适应阈值二值化（局部均值法）
 *
 * 对每个像素计算其邻域内均值，若像素值 <= 均值 - C 则置为前景。
 *
 * @param src       输入灰度图像
 * @param dst       输出二值图像
 * @param block_size 邻域大小（奇数，如 11、15）
 * @param C          偏移常量（>0，用于消除噪声）
 */
void ImgProc_AdaptiveThreshold(const GrayImage_t *src, BinaryImage_t *dst,
                                 uint8_t block_size, int8_t C);

/**
 * @brief 形态学腐蚀（3×3 矩形结构元素）
 * @param src 输入二值图像
 * @param dst 输出二值图像
 */
void ImgProc_Erode3x3(const BinaryImage_t *src, BinaryImage_t *dst);

/**
 * @brief 形态学膨胀（3×3 矩形结构元素）
 * @param src 输入二值图像
 * @param dst 输出二值图像
 */
void ImgProc_Dilate3x3(const BinaryImage_t *src, BinaryImage_t *dst);

/**
 * @brief 开运算（腐蚀 -> 膨胀），用于去除小噪声斑点
 * @param img 原地修改图像
 * @param tmp 临时缓冲区（与 img 大小相同）
 */
void ImgProc_Opening(BinaryImage_t *img, BinaryImage_t *tmp);

/**
 * @brief 连通区域分析（4-连通，基于两遍扫描标记算法）
 *
 * @param bin   输入二值图像
 * @param blobs 输出 Blob 列表，最多 MAX_BLOBS 个
 */
void ImgProc_FindBlobs(const BinaryImage_t *bin, BlobList_t *blobs);

/**
 * @brief 按面积降序排列 Blob 列表
 * @param blobs Blob 列表
 */
void ImgProc_SortBlobsByArea(BlobList_t *blobs);

/**
 * @brief 计算图像中某矩形区域的均值亮度（用于瞳孔反光检测）
 *
 * @param gray   输入灰度图像
 * @param x      起始列
 * @param y      起始行
 * @param w      宽度
 * @param h      高度
 * @return 区域均值灰度值
 */
uint8_t ImgProc_RegionMean(const GrayImage_t *gray,
                            uint32_t x, uint32_t y,
                            uint32_t w, uint32_t h);

#endif /* IMAGE_PROC_H */
