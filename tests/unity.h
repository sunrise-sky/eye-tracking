/**
 * @file unity.h
 * @brief 最小化 Unity 测试框架头（内联实现，无需外部依赖）
 *
 * 仅供本项目单元测试使用。
 */

#ifndef UNITY_H
#define UNITY_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static int unity_failures = 0;
static int unity_tests    = 0;

#define TEST_ASSERT_TRUE(cond) \
    do { \
        unity_tests++; \
        if (!(cond)) { \
            printf("[FAIL] %s:%d  (%s)\n", __FILE__, __LINE__, #cond); \
            unity_failures++; \
        } \
    } while (0)

#define TEST_ASSERT_FALSE(cond)   TEST_ASSERT_TRUE(!(cond))
#define TEST_ASSERT_EQUAL(a, b)   TEST_ASSERT_TRUE((a) == (b))
#define TEST_ASSERT_NOT_NULL(p)   TEST_ASSERT_TRUE((p) != NULL)

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
    TEST_ASSERT_TRUE(fabsf((float)(actual) - (float)(expected)) <= (float)(delta))

#define RUN_TEST(func) \
    do { \
        printf("[TEST] %s\n", #func); \
        func(); \
    } while (0)

#define UNITY_END() \
    do { \
        printf("\n%d tests, %d failures\n", unity_tests, unity_failures); \
        return (unity_failures == 0) ? 0 : 1; \
    } while (0)

#endif /* UNITY_H */
