/**
 * @file test_image_proc.c
 * @brief 图像处理模块单元测试
 */

#include "unity.h"
#include "../src/image_proc/image_proc.h"
#include <string.h>

/* =====================================================================
 * 辅助：构造合成灰度图像
 * ===================================================================== */

/** 创建全黑灰度图 */
static void make_gray_black(GrayImage_t *g)
{
    memset(g->data, 0, sizeof(g->data));
    g->width  = IMAGE_WIDTH;
    g->height = IMAGE_HEIGHT;
}

/** 创建全白灰度图 */
static void make_gray_white(GrayImage_t *g)
{
    memset(g->data, 255, sizeof(g->data));
    g->width  = IMAGE_WIDTH;
    g->height = IMAGE_HEIGHT;
}

/** 在灰度图中心绘制一个实心圆（用于模拟瞳孔） */
static void draw_circle(GrayImage_t *g, uint32_t cx, uint32_t cy,
                         uint32_t radius, uint8_t color)
{
    for (uint32_t y = 0; y < g->height; y++) {
        for (uint32_t x = 0; x < g->width; x++) {
            int32_t dx = (int32_t)x - (int32_t)cx;
            int32_t dy = (int32_t)y - (int32_t)cy;
            if ((uint32_t)(dx*dx + dy*dy) <= radius * radius) {
                g->data[y][x] = color;
            }
        }
    }
}

/* =====================================================================
 * 测试：YUV422 -> 灰度提取
 * ===================================================================== */
static void test_yuv422_to_gray_y_extraction(void)
{
    /* 构造 YUV422 数据：Y0=100, U=128, Y1=200, V=128 */
    uint8_t yuv[IMAGE_WIDTH * IMAGE_HEIGHT * 2];
    for (uint32_t i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i += 2) {
        yuv[i*2 + 0] = 100;  /* Y0 */
        yuv[i*2 + 1] = 128;  /* U  */
        yuv[i*2 + 2] = 200;  /* Y1 */
        yuv[i*2 + 3] = 128;  /* V  */
    }

    GrayImage_t gray;
    gray.width  = IMAGE_WIDTH;
    gray.height = IMAGE_HEIGHT;
    ImgProc_YUV422ToGray(yuv, &gray);

    /* 偶数位置像素应为 100，奇数位置应为 200 */
    TEST_ASSERT_EQUAL(100, gray.data[0][0]);
    TEST_ASSERT_EQUAL(200, gray.data[0][1]);
    TEST_ASSERT_EQUAL(100, gray.data[0][2]);
}

/* =====================================================================
 * 测试：固定阈值二值化
 * ===================================================================== */
static void test_threshold_below_is_foreground(void)
{
    GrayImage_t gray;
    make_gray_white(&gray);
    /* 中心 10×10 区域置为暗色 */
    for (uint32_t y = 110; y < 130; y++) {
        for (uint32_t x = 150; x < 170; x++) {
            gray.data[y][x] = 30;
        }
    }

    BinaryImage_t bin;
    ImgProc_Threshold(&gray, &bin, PUPIL_THRESHOLD);

    /* 暗区应为前景（255） */
    TEST_ASSERT_EQUAL(255, bin.data[115][155]);
    /* 亮区应为背景（0） */
    TEST_ASSERT_EQUAL(0, bin.data[0][0]);
}

static void test_threshold_all_black(void)
{
    GrayImage_t gray;
    make_gray_black(&gray);
    BinaryImage_t bin;
    ImgProc_Threshold(&gray, &bin, PUPIL_THRESHOLD);
    /* 全黑图像：所有像素均 <= 阈值，应全为前景 */
    TEST_ASSERT_EQUAL(255, bin.data[0][0]);
    TEST_ASSERT_EQUAL(255, bin.data[IMAGE_HEIGHT/2][IMAGE_WIDTH/2]);
}

static void test_threshold_all_white(void)
{
    GrayImage_t gray;
    make_gray_white(&gray);
    BinaryImage_t bin;
    ImgProc_Threshold(&gray, &bin, PUPIL_THRESHOLD);
    /* 全白图像：所有像素 > 阈值，应全为背景 */
    TEST_ASSERT_EQUAL(0, bin.data[0][0]);
    TEST_ASSERT_EQUAL(0, bin.data[IMAGE_HEIGHT-1][IMAGE_WIDTH-1]);
}

/* =====================================================================
 * 测试：高斯模糊（输出值不应超出输入范围）
 * ===================================================================== */
static void test_gaussian_blur_range(void)
{
    GrayImage_t src, dst;
    /* 随机填充（使用固定种子的模式） */
    for (uint32_t y = 0; y < IMAGE_HEIGHT; y++) {
        for (uint32_t x = 0; x < IMAGE_WIDTH; x++) {
            src.data[y][x] = (uint8_t)((x * 3 + y * 7) & 0xFF);
        }
    }
    src.width  = IMAGE_WIDTH;
    src.height = IMAGE_HEIGHT;
    dst.width  = IMAGE_WIDTH;
    dst.height = IMAGE_HEIGHT;

    ImgProc_GaussianBlur3x3(&src, &dst);

    /* 高斯模糊输出值由加权平均计算，不会超出输入值范围；此处仅用 ctest 覆盖代码路径 */
    bool all_valid = true;
    for (uint32_t y = 1; y < IMAGE_HEIGHT - 1; y++) {
        for (uint32_t x = 1; x < IMAGE_WIDTH - 1; x++) {
            if (dst.data[y][x] > src.data[y-1][x-1] &&
                dst.data[y][x] > src.data[y-1][x  ] &&
                dst.data[y][x] > src.data[y-1][x+1] &&
                dst.data[y][x] > src.data[y  ][x-1] &&
                dst.data[y][x] > src.data[y  ][x  ] &&
                dst.data[y][x] > src.data[y  ][x+1] &&
                dst.data[y][x] > src.data[y+1][x-1] &&
                dst.data[y][x] > src.data[y+1][x  ] &&
                dst.data[y][x] > src.data[y+1][x+1]) {
                all_valid = false;
            }
        }
    }
    TEST_ASSERT_TRUE(all_valid);
}

/* =====================================================================
 * 测试：形态学腐蚀
 * ===================================================================== */
static void test_erode_removes_isolated_pixel(void)
{
    BinaryImage_t src, dst;
    memset(&src, 0, sizeof(src));
    src.width  = IMAGE_WIDTH;
    src.height = IMAGE_HEIGHT;
    /* 放置一个孤立的单像素前景点 */
    src.data[100][100] = 255;

    ImgProc_Erode3x3(&src, &dst);

    /* 孤立像素经腐蚀后应消失 */
    TEST_ASSERT_EQUAL(0, dst.data[100][100]);
}

/* =====================================================================
 * 测试：Blob 分析 - 检测合成圆形区域
 * ===================================================================== */
static void test_find_blobs_detects_circle(void)
{
    GrayImage_t gray;
    make_gray_white(&gray);
    /* 在图像中心绘制一个半径 20 像素的暗圆（模拟瞳孔） */
    draw_circle(&gray, IMAGE_WIDTH/2, IMAGE_HEIGHT/2, 20, 20);

    BinaryImage_t bin;
    ImgProc_Threshold(&gray, &bin, PUPIL_THRESHOLD);

    /* 开运算去噪 */
    BinaryImage_t tmp;
    ImgProc_Opening(&bin, &tmp);

    BlobList_t blobs;
    ImgProc_FindBlobs(&bin, &blobs);

    /* 至少应检测到 1 个 Blob */
    TEST_ASSERT_TRUE(blobs.count >= 1);

    /* 面积应大于 PUPIL_MIN_AREA */
    TEST_ASSERT_TRUE(blobs.blobs[0].area >= PUPIL_MIN_AREA);
}

static void test_find_blobs_circularity_of_circle(void)
{
    GrayImage_t gray;
    make_gray_white(&gray);
    draw_circle(&gray, IMAGE_WIDTH/2, IMAGE_HEIGHT/2, 25, 10);

    BinaryImage_t bin;
    ImgProc_Threshold(&gray, &bin, PUPIL_THRESHOLD);

    BlobList_t blobs;
    ImgProc_FindBlobs(&bin, &blobs);
    ImgProc_SortBlobsByArea(&blobs);

    TEST_ASSERT_TRUE(blobs.count >= 1);
    /* 圆形的 circularity 应接近 1.0（允许离散化误差，> 0.5 即可） */
    TEST_ASSERT_TRUE(blobs.blobs[0].circularity > 0.5f);
}

/* =====================================================================
 * 测试：RegionMean
 * ===================================================================== */
static void test_region_mean_all_same(void)
{
    GrayImage_t gray;
    memset(&gray, 128, sizeof(gray));
    gray.width  = IMAGE_WIDTH;
    gray.height = IMAGE_HEIGHT;
    uint8_t mean = ImgProc_RegionMean(&gray, 10, 10, 50, 50);
    TEST_ASSERT_EQUAL(128, mean);
}

/* =====================================================================
 * 测试入口
 * ===================================================================== */
int main(void)
{
    RUN_TEST(test_yuv422_to_gray_y_extraction);
    RUN_TEST(test_threshold_below_is_foreground);
    RUN_TEST(test_threshold_all_black);
    RUN_TEST(test_threshold_all_white);
    RUN_TEST(test_gaussian_blur_range);
    RUN_TEST(test_erode_removes_isolated_pixel);
    RUN_TEST(test_find_blobs_detects_circle);
    RUN_TEST(test_find_blobs_circularity_of_circle);
    RUN_TEST(test_region_mean_all_same);

    UNITY_END();
}
