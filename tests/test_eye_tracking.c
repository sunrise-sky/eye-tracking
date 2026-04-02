/**
 * @file test_eye_tracking.c
 * @brief 眼动追踪算法模块单元测试
 */

#include "unity.h"
#include "../src/eye_tracking/eye_tracking.h"
#include <string.h>
#include <math.h>

/* =====================================================================
 * 辅助：构造合成测试数据
 * ===================================================================== */

/** 构造一个检测成功的瞳孔结果 */
static PupilResult_t make_pupil(float x, float y, float conf)
{
    PupilResult_t p;
    p.detected    = true;
    p.center.x    = x;
    p.center.y    = y;
    p.area        = 1000;
    p.confidence  = conf;
    return p;
}

/* =====================================================================
 * 测试：EyeTracker_Init 初始化
 * ===================================================================== */
static void test_init_clears_state(void)
{
    EyeTracker_t tracker;
    EyeTracker_Init(&tracker);

    TEST_ASSERT_EQUAL(CALIB_IDLE, tracker.calib_state);
    TEST_ASSERT_FALSE(tracker.calib.valid);
    TEST_ASSERT_FALSE(tracker.last_pupil.detected);
    TEST_ASSERT_EQUAL(0, tracker.frame_count);
}

/* =====================================================================
 * 测试：标定流程
 * ===================================================================== */
static void test_calib_full_flow(void)
{
    EyeTracker_t tracker;
    EyeTracker_Init(&tracker);

    /* 9 个标定点屏幕坐标（归一化） */
    const Point2f_t screen[CALIB_POINT_COUNT] = {
        {0.1f,0.1f},{0.5f,0.1f},{0.9f,0.1f},
        {0.1f,0.5f},{0.5f,0.5f},{0.9f,0.5f},
        {0.1f,0.9f},{0.5f,0.9f},{0.9f,0.9f},
    };

    EyeTracker_CalibStart(&tracker, screen);
    TEST_ASSERT_EQUAL(CALIB_IN_PROGRESS, tracker.calib_state);

    /*
     * 模拟：对每个标定点输入 CALIB_SAMPLE_FRAMES 帧的瞳孔坐标。
     * 使用简单的线性关系模拟（屏幕坐标 ≈ 瞳孔坐标 * 3 + 20）。
     */
    const float scale = 3.0f;
    const float bias  = 20.0f;

    CalibState_t state = CALIB_IN_PROGRESS;
    for (uint32_t pt = 0; pt < CALIB_POINT_COUNT && state == CALIB_IN_PROGRESS; pt++) {
        float px = screen[pt].x * scale + bias;
        float py = screen[pt].y * scale + bias;
        PupilResult_t pupil = make_pupil(px, py, 0.9f);

        for (uint32_t f = 0; f < CALIB_SAMPLE_FRAMES; f++) {
            state = EyeTracker_CalibAddSample(&tracker, &pupil);
        }
    }

    TEST_ASSERT_EQUAL(CALIB_DONE, state);
    TEST_ASSERT_TRUE(tracker.calib.valid);
}

/* =====================================================================
 * 测试：仿射变换映射精度
 * ===================================================================== */
static void test_pupil_to_gaze_accuracy(void)
{
    EyeTracker_t tracker;
    EyeTracker_Init(&tracker);

    /*
     * 构造一个已知的仿射变换：
     *   screen_x = 2.0 * pupil_x + 10.0
     *   screen_y = 3.0 * pupil_y + 5.0
     * 直接填充标定系数，跳过采样流程。
     */
    tracker.calib.a[0] = 2.0f;
    tracker.calib.a[1] = 0.0f;
    tracker.calib.a[2] = 10.0f;
    tracker.calib.a[3] = 0.0f;
    tracker.calib.a[4] = 3.0f;
    tracker.calib.a[5] = 5.0f;
    tracker.calib.valid = true;

    Point2f_t pupil = { 50.0f, 60.0f };
    Point2f_t gaze;
    bool ok = EyeTracker_PupilToGaze(&tracker, &pupil, &gaze);

    TEST_ASSERT_TRUE(ok);
    /* gaze_x = 2*50 + 10 = 110 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 110.0f, gaze.x);
    /* gaze_y = 3*60 + 5 = 185 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 185.0f, gaze.y);
}

/* =====================================================================
 * 测试：未标定时 PupilToGaze 返回 false
 * ===================================================================== */
static void test_pupil_to_gaze_uncalibrated(void)
{
    EyeTracker_t tracker;
    EyeTracker_Init(&tracker);

    Point2f_t pupil = { 100.0f, 100.0f };
    Point2f_t gaze;
    bool ok = EyeTracker_PupilToGaze(&tracker, &pupil, &gaze);
    TEST_ASSERT_FALSE(ok);
}

/* =====================================================================
 * 测试：注视点平滑滤波
 * ===================================================================== */
static void test_smooth_gaze_converges(void)
{
    Point2f_t smoothed = { 0.0f, 0.0f };
    Point2f_t target   = { 100.0f, 200.0f };

    /* 迭代 50 次，应收敛到目标附近 */
    for (int i = 0; i < 50; i++) {
        EyeTracker_SmoothGaze(&target, &smoothed, 0.4f);
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, smoothed.x);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, smoothed.y);
}

static void test_smooth_gaze_alpha_one(void)
{
    /* alpha=1.0 时，平滑后立即等于当前值 */
    Point2f_t smoothed = { 50.0f, 50.0f };
    Point2f_t current  = { 99.0f, 77.0f };

    EyeTracker_SmoothGaze(&current, &smoothed, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 99.0f, smoothed.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 77.0f, smoothed.y);
}

static void test_smooth_gaze_alpha_zero(void)
{
    /* alpha=0.0 时，平滑后不变 */
    Point2f_t smoothed = { 50.0f, 50.0f };
    Point2f_t current  = { 99.0f, 77.0f };

    EyeTracker_SmoothGaze(&current, &smoothed, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, smoothed.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, smoothed.y);
}

/* =====================================================================
 * 测试：瞳孔检测 - 空 Blob 列表
 * ===================================================================== */
static void test_detect_pupil_empty_blobs(void)
{
    EyeTracker_t tracker;
    EyeTracker_Init(&tracker);

    BlobList_t blobs;
    blobs.count = 0;

    PupilResult_t result;
    EyeTracker_DetectPupil(&tracker, &blobs, &result);
    TEST_ASSERT_FALSE(result.detected);
}

/* =====================================================================
 * 测试：瞳孔检测 - 过滤圆形度不足的 Blob
 * ===================================================================== */
static void test_detect_pupil_filters_low_circularity(void)
{
    EyeTracker_t tracker;
    EyeTracker_Init(&tracker);

    BlobList_t blobs;
    blobs.count = 1;
    blobs.blobs[0].cx           = IMAGE_WIDTH / 2;
    blobs.blobs[0].cy           = IMAGE_HEIGHT / 2;
    blobs.blobs[0].area         = 1000;
    blobs.blobs[0].circularity  = 0.1f;  /* 远低于 PUPIL_CIRCULARITY_MIN */
    blobs.blobs[0].perimeter    = 200;

    PupilResult_t result;
    EyeTracker_DetectPupil(&tracker, &blobs, &result);
    TEST_ASSERT_FALSE(result.detected);
}

/* =====================================================================
 * 测试：瞳孔检测 - 选择最佳候选
 * ===================================================================== */
static void test_detect_pupil_selects_best_candidate(void)
{
    EyeTracker_t tracker;
    EyeTracker_Init(&tracker);

    BlobList_t blobs;
    blobs.count = 2;

    /* Blob 0：圆形度高，位于图像中心附近 */
    blobs.blobs[0].cx           = IMAGE_WIDTH / 2;
    blobs.blobs[0].cy           = IMAGE_HEIGHT / 2;
    blobs.blobs[0].area         = 1500;
    blobs.blobs[0].circularity  = 0.9f;
    blobs.blobs[0].perimeter    = 150;

    /* Blob 1：圆形度较低，位于角落 */
    blobs.blobs[1].cx           = 10;
    blobs.blobs[1].cy           = 10;
    blobs.blobs[1].area         = 3000;
    blobs.blobs[1].circularity  = 0.6f;
    blobs.blobs[1].perimeter    = 300;

    PupilResult_t result;
    EyeTracker_DetectPupil(&tracker, &blobs, &result);

    TEST_ASSERT_TRUE(result.detected);
    /* 应选中 Blob 0（综合得分更高） */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, (float)(IMAGE_WIDTH/2),  result.center.x);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, (float)(IMAGE_HEIGHT/2), result.center.y);
}

/* =====================================================================
 * 测试入口
 * ===================================================================== */
int main(void)
{
    RUN_TEST(test_init_clears_state);
    RUN_TEST(test_calib_full_flow);
    RUN_TEST(test_pupil_to_gaze_accuracy);
    RUN_TEST(test_pupil_to_gaze_uncalibrated);
    RUN_TEST(test_smooth_gaze_converges);
    RUN_TEST(test_smooth_gaze_alpha_one);
    RUN_TEST(test_smooth_gaze_alpha_zero);
    RUN_TEST(test_detect_pupil_empty_blobs);
    RUN_TEST(test_detect_pupil_filters_low_circularity);
    RUN_TEST(test_detect_pupil_selects_best_candidate);

    UNITY_END();
}
