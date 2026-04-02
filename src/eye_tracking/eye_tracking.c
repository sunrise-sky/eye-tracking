/**
 * @file eye_tracking.c
 * @brief 眼动追踪核心算法实现
 */

#include "eye_tracking.h"
#include <string.h>
#include <math.h>

/* =====================================================================
 * 模块内部静态缓冲区（避免栈溢出）
 * ===================================================================== */
static GrayImage_t   s_gray;
static BinaryImage_t s_binary;
static BinaryImage_t s_morph_tmp;
static BlobList_t    s_blobs;

/* =====================================================================
 * 私有辅助函数声明
 * ===================================================================== */
static float dist2f(const Point2f_t *a, const Point2f_t *b);
static bool  solve_affine_6pt(const Point2f_t src[],
                               const Point2f_t dst[],
                               uint32_t n,
                               float coeff[6]);

/* =====================================================================
 * 公共接口实现
 * ===================================================================== */

void EyeTracker_Init(EyeTracker_t *tracker)
{
    memset(tracker, 0, sizeof(EyeTracker_t));
    tracker->calib_state      = CALIB_IDLE;
    tracker->calib.valid      = false;
    tracker->last_pupil.detected = false;

    s_gray.width   = IMAGE_WIDTH;
    s_gray.height  = IMAGE_HEIGHT;
    s_binary.width = IMAGE_WIDTH;
    s_binary.height= IMAGE_HEIGHT;
}

void EyeTracker_DetectPupil(EyeTracker_t *tracker,
                             const BlobList_t *blobs,
                             PupilResult_t *result)
{
    result->detected   = false;
    result->confidence = 0.0f;

    if (blobs->count == 0U) {
        return;
    }

    float     best_score = -1.0f;
    const Blob_t *best  = NULL;

    /* 图像中心（用于位置权重） */
    float cx_img = (float)IMAGE_WIDTH  * 0.5f;
    float cy_img = (float)IMAGE_HEIGHT * 0.5f;
    float max_dist = sqrtf(cx_img * cx_img + cy_img * cy_img);

    for (uint32_t i = 0; i < blobs->count; i++) {
        const Blob_t *b = &blobs->blobs[i];

        /* 圆形度过滤 */
        if (b->circularity < PUPIL_CIRCULARITY_MIN) {
            continue;
        }

        /*
         * 综合评分：
         *   - 圆形度权重 60%
         *   - 与图像中心距离权重 20%（越靠中心越好）
         *   - 与上一帧瞳孔位置连续性权重 20%（若有上一帧结果）
         */
        float circ_score  = b->circularity;  /* [0, 1+] */

        float dx = (float)b->cx - cx_img;
        float dy = (float)b->cy - cy_img;
        float dist_center = sqrtf(dx*dx + dy*dy);
        float center_score = 1.0f - (dist_center / max_dist);

        float continuity_score = 0.5f;  /* 默认中性 */
        if (tracker->last_pupil.detected) {
            float d = dist2f(&tracker->last_pupil.center,
                             &(Point2f_t){ (float)b->cx, (float)b->cy });
            /* 距离上一帧瞳孔越近越好，超过 50 像素则认为跳变 */
            float norm_d = d / 50.0f;
            continuity_score = (norm_d > 1.0f) ? 0.0f : (1.0f - norm_d);
        }

        float score = 0.6f * circ_score
                    + 0.2f * center_score
                    + 0.2f * continuity_score;

        if (score > best_score) {
            best_score = score;
            best       = b;
        }
    }

    if (best == NULL) {
        return;
    }

    result->detected      = true;
    result->center.x      = (float)best->cx;
    result->center.y      = (float)best->cy;
    result->area          = best->area;
    result->confidence    = (best_score > 1.0f) ? 1.0f : best_score;
}

void EyeTracker_CalibStart(EyeTracker_t *tracker,
                            const Point2f_t screen_points[CALIB_POINT_COUNT])
{
    tracker->calib_state      = CALIB_IN_PROGRESS;
    tracker->calib_point_idx  = 0U;
    tracker->calib_sample_cnt = 0U;
    tracker->calib_accum.x    = 0.0f;
    tracker->calib_accum.y    = 0.0f;
    tracker->calib.valid      = false;

    for (uint32_t i = 0; i < CALIB_POINT_COUNT; i++) {
        tracker->calib_screen[i] = screen_points[i];
        tracker->calib_pupil[i].x = 0.0f;
        tracker->calib_pupil[i].y = 0.0f;
    }
}

CalibState_t EyeTracker_CalibAddSample(EyeTracker_t *tracker,
                                        const PupilResult_t *pupil)
{
    if (tracker->calib_state != CALIB_IN_PROGRESS) {
        return tracker->calib_state;
    }

    if (!pupil->detected) {
        /* 瞳孔丢失时不计入当前点的采样 */
        return CALIB_IN_PROGRESS;
    }

    /* 累加瞳孔坐标 */
    tracker->calib_accum.x += pupil->center.x;
    tracker->calib_accum.y += pupil->center.y;
    tracker->calib_sample_cnt++;

    if (tracker->calib_sample_cnt >= CALIB_SAMPLE_FRAMES) {
        /* 当前标定点采样完成，计算均值 */
        uint32_t idx = tracker->calib_point_idx;
        tracker->calib_pupil[idx].x =
            tracker->calib_accum.x / (float)tracker->calib_sample_cnt;
        tracker->calib_pupil[idx].y =
            tracker->calib_accum.y / (float)tracker->calib_sample_cnt;

        /* 重置为下一个标定点 */
        tracker->calib_point_idx++;
        tracker->calib_sample_cnt = 0U;
        tracker->calib_accum.x   = 0.0f;
        tracker->calib_accum.y   = 0.0f;

        if (tracker->calib_point_idx >= CALIB_POINT_COUNT) {
            /* 全部标定点采集完成，计算变换矩阵 */
            if (EyeTracker_CalibCompute(tracker)) {
                tracker->calib_state = CALIB_DONE;
            } else {
                tracker->calib_state = CALIB_ERROR;
            }
        }
    }

    return tracker->calib_state;
}

bool EyeTracker_CalibCompute(EyeTracker_t *tracker)
{
    /*
     * 使用最小二乘法求解仿射变换矩阵（6 个参数）。
     * 方程组：
     *   screen_x[i] = a0 * px[i] + a1 * py[i] + a2
     *   screen_y[i] = a3 * px[i] + a4 * py[i] + a5
     *
     * CALIB_POINT_COUNT >= 3 时方程组超定，用最小二乘求解。
     */
    bool ok = solve_affine_6pt(tracker->calib_pupil,
                                tracker->calib_screen,
                                CALIB_POINT_COUNT,
                                tracker->calib.a);
    tracker->calib.valid = ok;
    return ok;
}

bool EyeTracker_PupilToGaze(const EyeTracker_t *tracker,
                              const Point2f_t *pupil_pos,
                              Point2f_t *gaze_pos)
{
    if (!tracker->calib.valid) {
        return false;
    }

    const float *a = tracker->calib.a;
    gaze_pos->x = a[0] * pupil_pos->x + a[1] * pupil_pos->y + a[2];
    gaze_pos->y = a[3] * pupil_pos->x + a[4] * pupil_pos->y + a[5];
    return true;
}

void EyeTracker_SmoothGaze(const Point2f_t *current,
                            Point2f_t *smoothed,
                            float alpha)
{
    smoothed->x = alpha * current->x + (1.0f - alpha) * smoothed->x;
    smoothed->y = alpha * current->y + (1.0f - alpha) * smoothed->y;
}

bool EyeTracker_ProcessFrame(EyeTracker_t *tracker,
                              const uint8_t *yuv_frame,
                              uint32_t       frame_size,
                              Point2f_t     *gaze_out)
{
    (void)frame_size;

    tracker->frame_count++;

    /* ---------------------------------------------------------------
     * 步骤 1：YUV422 -> 灰度
     * ------------------------------------------------------------- */
    ImgProc_YUV422ToGray(yuv_frame, &s_gray);

    /* ---------------------------------------------------------------
     * 步骤 2：高斯模糊降噪
     * ------------------------------------------------------------- */
    ImgProc_GaussianBlur3x3(&s_gray, &s_gray);

    /* ---------------------------------------------------------------
     * 步骤 3：固定阈值二值化（检测暗色瞳孔）
     * ------------------------------------------------------------- */
    ImgProc_Threshold(&s_gray, &s_binary, PUPIL_THRESHOLD);

    /* ---------------------------------------------------------------
     * 步骤 4：形态学开运算（去除小噪点，保留瞳孔区域）
     * ------------------------------------------------------------- */
    ImgProc_Opening(&s_binary, &s_morph_tmp);

    /* ---------------------------------------------------------------
     * 步骤 5：连通区域 Blob 分析
     * ------------------------------------------------------------- */
    ImgProc_FindBlobs(&s_binary, &s_blobs);
    ImgProc_SortBlobsByArea(&s_blobs);

    /* ---------------------------------------------------------------
     * 步骤 6：瞳孔检测
     * ------------------------------------------------------------- */
    EyeTracker_DetectPupil(tracker, &s_blobs, &tracker->last_pupil);

    if (!tracker->last_pupil.detected) {
        return false;
    }

    /* ---------------------------------------------------------------
     * 步骤 7：注视点映射（若已标定）
     * ------------------------------------------------------------- */
    Point2f_t raw_gaze;
    if (EyeTracker_PupilToGaze(tracker, &tracker->last_pupil.center,
                                &raw_gaze)) {
        /* 指数平滑滤波，alpha=0.4 */
        EyeTracker_SmoothGaze(&raw_gaze, &tracker->last_gaze, 0.4f);
        *gaze_out = tracker->last_gaze;
    } else {
        /* 尚未标定，直接输出瞳孔像素坐标 */
        *gaze_out = tracker->last_pupil.center;
    }

    return true;
}

/* =====================================================================
 * 私有辅助函数实现
 * ===================================================================== */

static float dist2f(const Point2f_t *a, const Point2f_t *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    return sqrtf(dx*dx + dy*dy);
}

/**
 * @brief 最小二乘法求仿射变换（6 参数）
 *
 * 求解 A·x = b，其中 A 为 (2n × 6) 矩阵，x = [a0..a5]^T，
 * 使用正规方程 (A^T·A)·x = A^T·b，高斯消元法求解 6×6 线性系统。
 *
 * @param src  源点（瞳孔坐标），n 个
 * @param dst  目标点（屏幕坐标），n 个
 * @param n    点对数（n >= 3）
 * @param coeff 输出 6 个仿射系数
 * @return true 求解成功，false 矩阵奇异
 */
static bool solve_affine_6pt(const Point2f_t src[],
                               const Point2f_t dst[],
                               uint32_t n,
                               float coeff[6])
{
    /*
     * 构造正规方程：
     *   对 x 分量：[px py 1  0  0  0] * [a0..a5]^T = sx
     *   对 y 分量：[0  0  0  px py 1] * [a0..a5]^T = sy
     *
     * 正规方程 (A^T A) c = (A^T b) 分解为两个独立的 3×3 系统：
     *   Mx * [a0 a1 a2]^T = bx
     *   My * [a3 a4 a5]^T = by
     */

    /* 累积矩阵元素 */
    double S11 = 0, S12 = 0, S13 = 0, S22 = 0, S23 = 0, S33 = 0;
    double bx1 = 0, bx2 = 0, bx3 = 0;
    double by1 = 0, by2 = 0, by3 = 0;

    for (uint32_t i = 0; i < n; i++) {
        double px = (double)src[i].x;
        double py = (double)src[i].y;
        double sx = (double)dst[i].x;
        double sy = (double)dst[i].y;

        S11 += px * px;
        S12 += px * py;
        S13 += px;
        S22 += py * py;
        S23 += py;
        S33 += 1.0;

        bx1 += px * sx;
        bx2 += py * sx;
        bx3 += sx;

        by1 += px * sy;
        by2 += py * sy;
        by3 += sy;
    }

    /*
     * 用克莱姆法则求解 3×3 系统（对 x 和 y 各一次）：
     *   | S11 S12 S13 | | a0 |   | bx1 |
     *   | S12 S22 S23 | | a1 | = | bx2 |
     *   | S13 S23 S33 | | a2 |   | bx3 |
     */
    double det = S11 * (S22 * S33 - S23 * S23)
               - S12 * (S12 * S33 - S23 * S13)
               + S13 * (S12 * S23 - S22 * S13);

    if (det < 1e-10 && det > -1e-10) {
        return false;  /* 矩阵接近奇异，无法求解 */
    }

    double inv_det = 1.0 / det;

    /* 余子式 */
    double C11 = S22*S33 - S23*S23;
    double C12 = S23*S13 - S12*S33;
    double C13 = S12*S23 - S22*S13;
    double C22 = S11*S33 - S13*S13;
    double C23 = S12*S13 - S11*S23;
    double C33 = S11*S22 - S12*S12;

    coeff[0] = (float)((C11*bx1 + C12*bx2 + C13*bx3) * inv_det);
    coeff[1] = (float)((C12*bx1 + C22*bx2 + C23*bx3) * inv_det);
    coeff[2] = (float)((C13*bx1 + C23*bx2 + C33*bx3) * inv_det);

    coeff[3] = (float)((C11*by1 + C12*by2 + C13*by3) * inv_det);
    coeff[4] = (float)((C12*by1 + C22*by2 + C23*by3) * inv_det);
    coeff[5] = (float)((C13*by1 + C23*by2 + C33*by3) * inv_det);

    return true;
}
