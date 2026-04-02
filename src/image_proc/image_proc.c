/**
 * @file image_proc.c
 * @brief 图像处理模块实现
 *
 * 所有算法均针对嵌入式环境优化：
 *   - 避免动态内存分配
 *   - 使用整数运算代替浮点（圆形度除外）
 *   - 循环展开与局部变量复用以减少 Cache Miss
 */

#include "image_proc.h"
#include <string.h>
#include <stddef.h>

/* =====================================================================
 * 内部宏与辅助函数
 * ===================================================================== */

/** 限幅宏：将值箝位到 [lo, hi] */
#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/** 绝对值（整型） */
#define ABS_INT(x)  ((x) < 0 ? -(x) : (x))

/* 用于连通区域标记的临时标签缓冲（16-bit 足够 IMAGE_SIZE 个像素） */
static uint16_t s_label_map[IMAGE_HEIGHT][IMAGE_WIDTH];

/* Union-Find（并查集）用于合并等价标签 */
#define MAX_LABELS  512

static uint16_t s_parent[MAX_LABELS];

static uint16_t uf_find(uint16_t x)
{
    while (s_parent[x] != x) {
        s_parent[x] = s_parent[s_parent[x]];  /* 路径压缩（两步跳） */
        x = s_parent[x];
    }
    return x;
}

static void uf_union(uint16_t a, uint16_t b)
{
    a = uf_find(a);
    b = uf_find(b);
    if (a != b) {
        s_parent[b] = a;  /* 简单合并（不按秩），足够本应用使用 */
    }
}

/* =====================================================================
 * 公共函数实现
 * ===================================================================== */

void ImgProc_YUV422ToGray(const uint8_t *yuv_data, GrayImage_t *gray)
{
    /*
     * YUV422 存储格式（YUYV）：
     *   byte0=Y0, byte1=U0, byte2=Y1, byte3=V0, ...
     * 每 2 个 Y 字节对应 1 个 U + 1 个 V，即每 4 字节 = 2 像素。
     */
    uint32_t n_pixels = gray->width * gray->height;
    const uint8_t *src = yuv_data;
    uint8_t *dst = &gray->data[0][0];

    for (uint32_t i = 0; i < n_pixels; i += 2) {
        dst[i]     = src[0];  /* Y0 */
        dst[i + 1] = src[2];  /* Y1 */
        src += 4;
    }
}

void ImgProc_GaussianBlur3x3(const GrayImage_t *src, GrayImage_t *dst)
{
    /* 高斯核系数（已扩大 16 倍以使用整数运算）：
     *   1  2  1
     *   2  4  2   归一化因子 = 16
     *   1  2  1
     */
    static uint8_t tmp[IMAGE_HEIGHT][IMAGE_WIDTH];
    uint32_t h = src->height;
    uint32_t w = src->width;

    /* 使用临时缓冲避免原地写入导致的数据依赖 */
    for (uint32_t y = 1; y < h - 1; y++) {
        for (uint32_t x = 1; x < w - 1; x++) {
            uint32_t sum =
                (uint32_t)src->data[y-1][x-1]       +
                (uint32_t)src->data[y-1][x  ] * 2U  +
                (uint32_t)src->data[y-1][x+1]       +
                (uint32_t)src->data[y  ][x-1] * 2U  +
                (uint32_t)src->data[y  ][x  ] * 4U  +
                (uint32_t)src->data[y  ][x+1] * 2U  +
                (uint32_t)src->data[y+1][x-1]       +
                (uint32_t)src->data[y+1][x  ] * 2U  +
                (uint32_t)src->data[y+1][x+1];
            tmp[y][x] = (uint8_t)(sum >> 4);  /* 除以 16 */
        }
    }

    /* 边缘行/列直接复制（不做模糊） */
    for (uint32_t x = 0; x < w; x++) {
        tmp[0][x]     = src->data[0][x];
        tmp[h-1][x]   = src->data[h-1][x];
    }
    for (uint32_t y = 0; y < h; y++) {
        tmp[y][0]     = src->data[y][0];
        tmp[y][w-1]   = src->data[y][w-1];
    }

    memcpy(&dst->data[0][0], &tmp[0][0], h * w);
    dst->width  = w;
    dst->height = h;
}

void ImgProc_Threshold(const GrayImage_t *src, BinaryImage_t *dst,
                        uint8_t threshold)
{
    uint32_t n = src->width * src->height;
    const uint8_t *s = &src->data[0][0];
    uint8_t       *d = &dst->data[0][0];

    for (uint32_t i = 0; i < n; i++) {
        d[i] = (s[i] <= threshold) ? 255U : 0U;
    }
    dst->width  = src->width;
    dst->height = src->height;
}

void ImgProc_AdaptiveThreshold(const GrayImage_t *src, BinaryImage_t *dst,
                                 uint8_t block_size, int8_t C)
{
    uint32_t h   = src->height;
    uint32_t w   = src->width;
    uint32_t half = (uint32_t)block_size / 2U;

    dst->width  = w;
    dst->height = h;

    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            /* 计算邻域边界（饱和到图像范围） */
            uint32_t y0 = (y > half)     ? (y - half)     : 0U;
            uint32_t y1 = (y + half < h) ? (y + half)     : (h - 1U);
            uint32_t x0 = (x > half)     ? (x - half)     : 0U;
            uint32_t x1 = (x + half < w) ? (x + half)     : (w - 1U);

            uint32_t sum   = 0U;
            uint32_t count = (y1 - y0 + 1U) * (x1 - x0 + 1U);

            for (uint32_t ky = y0; ky <= y1; ky++) {
                for (uint32_t kx = x0; kx <= x1; kx++) {
                    sum += src->data[ky][kx];
                }
            }

            int16_t mean     = (int16_t)(sum / count);
            int16_t pixel    = (int16_t)src->data[y][x];
            int16_t thresh   = mean - (int16_t)C;

            dst->data[y][x] = (pixel <= thresh) ? 255U : 0U;
        }
    }
}

void ImgProc_Erode3x3(const BinaryImage_t *src, BinaryImage_t *dst)
{
    uint32_t h = src->height;
    uint32_t w = src->width;

    for (uint32_t y = 1; y < h - 1; y++) {
        for (uint32_t x = 1; x < w - 1; x++) {
            /* 若 3×3 邻域内有任何 0（背景），结果为 0 */
            uint8_t v = 255U;
            v &= src->data[y-1][x-1];
            v &= src->data[y-1][x  ];
            v &= src->data[y-1][x+1];
            v &= src->data[y  ][x-1];
            v &= src->data[y  ][x  ];
            v &= src->data[y  ][x+1];
            v &= src->data[y+1][x-1];
            v &= src->data[y+1][x  ];
            v &= src->data[y+1][x+1];
            dst->data[y][x] = v;
        }
    }

    /* 边界置 0 */
    for (uint32_t x = 0; x < w; x++) {
        dst->data[0][x]   = 0U;
        dst->data[h-1][x] = 0U;
    }
    for (uint32_t y = 0; y < h; y++) {
        dst->data[y][0]   = 0U;
        dst->data[y][w-1] = 0U;
    }

    dst->width  = w;
    dst->height = h;
}

void ImgProc_Dilate3x3(const BinaryImage_t *src, BinaryImage_t *dst)
{
    uint32_t h = src->height;
    uint32_t w = src->width;

    for (uint32_t y = 1; y < h - 1; y++) {
        for (uint32_t x = 1; x < w - 1; x++) {
            /* 若 3×3 邻域内有任何 255（前景），结果为 255 */
            uint8_t v = 0U;
            v |= src->data[y-1][x-1];
            v |= src->data[y-1][x  ];
            v |= src->data[y-1][x+1];
            v |= src->data[y  ][x-1];
            v |= src->data[y  ][x  ];
            v |= src->data[y  ][x+1];
            v |= src->data[y+1][x-1];
            v |= src->data[y+1][x  ];
            v |= src->data[y+1][x+1];
            dst->data[y][x] = v;
        }
    }

    /* 边界复制 */
    for (uint32_t x = 0; x < w; x++) {
        dst->data[0][x]   = src->data[0][x];
        dst->data[h-1][x] = src->data[h-1][x];
    }
    for (uint32_t y = 0; y < h; y++) {
        dst->data[y][0]   = src->data[y][0];
        dst->data[y][w-1] = src->data[y][w-1];
    }

    dst->width  = w;
    dst->height = h;
}

void ImgProc_Opening(BinaryImage_t *img, BinaryImage_t *tmp)
{
    ImgProc_Erode3x3(img, tmp);
    ImgProc_Dilate3x3(tmp, img);
}

void ImgProc_FindBlobs(const BinaryImage_t *bin, BlobList_t *blobs)
{
    uint32_t h = bin->height;
    uint32_t w = bin->width;

    /* 初始化并查集 */
    for (uint16_t i = 0; i < MAX_LABELS; i++) {
        s_parent[i] = i;
    }

    memset(s_label_map, 0, sizeof(s_label_map));
    uint16_t next_label = 1U;

    /* === 第一遍扫描：初始标记并记录等价关系 === */
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            if (bin->data[y][x] == 0U) {
                continue;  /* 背景像素，跳过 */
            }

            uint16_t left  = (x > 0)     ? s_label_map[y][x-1]   : 0U;
            uint16_t above = (y > 0)     ? s_label_map[y-1][x]   : 0U;

            if (left == 0U && above == 0U) {
                /* 新区域 */
                if (next_label < MAX_LABELS) {
                    s_label_map[y][x] = next_label++;
                }
            } else if (left != 0U && above == 0U) {
                s_label_map[y][x] = left;
            } else if (left == 0U && above != 0U) {
                s_label_map[y][x] = above;
            } else {
                /* 两个邻居均有标签，合并等价类 */
                uint16_t min_label = (left < above) ? left : above;
                s_label_map[y][x] = min_label;
                uf_union(left, above);
            }
        }
    }

    /* === 第二遍扫描：统计各等价类的像素统计量 === */
    /* 每个标签对应的统计量（面积、矩） */
    static uint32_t area  [MAX_LABELS];
    static uint32_t sum_x [MAX_LABELS];
    static uint32_t sum_y [MAX_LABELS];
    static uint32_t x_min [MAX_LABELS];
    static uint32_t x_max [MAX_LABELS];
    static uint32_t y_min [MAX_LABELS];
    static uint32_t y_max [MAX_LABELS];
    static uint32_t perim [MAX_LABELS];

    memset(area,  0, next_label * sizeof(uint32_t));
    memset(sum_x, 0, next_label * sizeof(uint32_t));
    memset(sum_y, 0, next_label * sizeof(uint32_t));
    for (uint16_t i = 0; i < next_label; i++) {
        x_min[i] = w; x_max[i] = 0U;
        y_min[i] = h; y_max[i] = 0U;
        perim[i] = 0U;
    }

    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            uint16_t lbl = s_label_map[y][x];
            if (lbl == 0U) continue;

            uint16_t root = uf_find(lbl);
            area [root]++;
            sum_x[root] += x;
            sum_y[root] += y;
            if (x < x_min[root]) x_min[root] = x;
            if (x > x_max[root]) x_max[root] = x;
            if (y < y_min[root]) y_min[root] = y;
            if (y > y_max[root]) y_max[root] = y;

            /* 简易周长估计：与 4-邻域背景像素相邻的前景像素计入周长 */
            bool border = false;
            if (x == 0 || x == w-1 || y == 0 || y == h-1) {
                border = true;
            } else {
                if (bin->data[y-1][x] == 0U) border = true;
                if (bin->data[y+1][x] == 0U) border = true;
                if (bin->data[y][x-1] == 0U) border = true;
                if (bin->data[y][x+1] == 0U) border = true;
            }
            if (border) perim[root]++;
        }
    }

    /* === 组装输出 Blob 列表（过滤面积太小/太大的区域） === */
    blobs->count = 0U;
    for (uint16_t i = 1; i < next_label && blobs->count < MAX_BLOBS; i++) {
        if (uf_find(i) != i) continue;  /* 不是根节点，跳过 */
        if (area[i] < PUPIL_MIN_AREA || area[i] > PUPIL_MAX_AREA) continue;

        Blob_t *b = &blobs->blobs[blobs->count];
        b->area      = area[i];
        b->cx        = sum_x[i] / area[i];
        b->cy        = sum_y[i] / area[i];
        b->x_min     = x_min[i];
        b->x_max     = x_max[i];
        b->y_min     = y_min[i];
        b->y_max     = y_max[i];
        b->perimeter = perim[i];

        /* 圆形度：4π·A / P²，使用近似值 π ≈ 3.14159 */
        if (perim[i] > 0U) {
            float p2 = (float)perim[i] * (float)perim[i];
            b->circularity = 4.0f * 3.14159f * (float)area[i] / p2;
        } else {
            b->circularity = 0.0f;
        }

        blobs->count++;
    }
}

void ImgProc_SortBlobsByArea(BlobList_t *blobs)
{
    /* 简单插入排序（Blob 数量通常很少，性能足够） */
    for (uint32_t i = 1; i < blobs->count; i++) {
        Blob_t key = blobs->blobs[i];
        int32_t j  = (int32_t)i - 1;
        while (j >= 0 && blobs->blobs[j].area < key.area) {
            blobs->blobs[j + 1] = blobs->blobs[j];
            j--;
        }
        blobs->blobs[j + 1] = key;
    }
}

uint8_t ImgProc_RegionMean(const GrayImage_t *gray,
                            uint32_t x, uint32_t y,
                            uint32_t w, uint32_t h)
{
    uint32_t sum   = 0U;
    uint32_t count = 0U;
    uint32_t x_end = (x + w < gray->width)  ? (x + w) : gray->width;
    uint32_t y_end = (y + h < gray->height) ? (y + h) : gray->height;

    for (uint32_t ky = y; ky < y_end; ky++) {
        for (uint32_t kx = x; kx < x_end; kx++) {
            sum += gray->data[ky][kx];
            count++;
        }
    }
    return (count > 0U) ? (uint8_t)(sum / count) : 0U;
}
