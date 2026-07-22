#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#include "odometer.h"

static int s_failed;

static void CheckInt32(const char *name, int32_t actual, int32_t expected)
{
    if (actual != expected) {
        printf("  FAIL: %s = %ld, expected %ld\n",
               name,
               (long)actual,
               (long)expected);
        s_failed = 1;
    }
}

static void TestRelativeDistance(void)
{
    Odometer_Info_t info;

    printf("[TEST] odometer relative distance and delta\n");
    Odometer_Init(1000, -500);
    Odometer_Update(1120, -340);
    (void)Odometer_GetInfo(&info);
    CheckInt32("left_mm", info.left_mm, 120);
    CheckInt32("right_mm", info.right_mm, 160);
    CheckInt32("distance_mm", info.distance_mm, 140);
    CheckInt32("delta_distance_mm", info.delta_distance_mm, 140);

    Odometer_Update(1170, -300);
    (void)Odometer_GetInfo(&info);
    CheckInt32("second left delta", info.delta_left_mm, 50);
    CheckInt32("second right delta", info.delta_right_mm, 40);
    CheckInt32("second distance delta", info.delta_distance_mm, 45);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestSoftwareClear(void)
{
    Odometer_Info_t info;

    printf("[TEST] odometer software clear baseline\n");
    Odometer_Clear(1170, -300);
    Odometer_Update(1200, -250);
    (void)Odometer_GetInfo(&info);
    CheckInt32("cleared left_mm", info.left_mm, 30);
    CheckInt32("cleared right_mm", info.right_mm, 50);
    CheckInt32("cleared distance_mm", info.distance_mm, 40);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestCounterWrap(void)
{
    Odometer_Info_t info;
    int32_t origin = INT32_MAX - 10;
    int32_t wrapped = INT32_MIN + 19;

    printf("[TEST] odometer signed counter wrap\n");
    Odometer_Clear(origin, origin);
    Odometer_Update(wrapped, wrapped);
    (void)Odometer_GetInfo(&info);
    CheckInt32("wrapped left_mm", info.left_mm, 30);
    CheckInt32("wrapped right_mm", info.right_mm, 30);
    CheckInt32("wrapped distance_mm", info.distance_mm, 30);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

int main(void)
{
    TestRelativeDistance();
    TestSoftwareClear();
    TestCounterWrap();

    if (Odometer_GetInfo(0) != PROJECT_PARAM) {
        printf("  FAIL: null info was not rejected\n");
        s_failed = 1;
    }

    printf("\nResult: %s\n", s_failed ? "failed" : "3 passed, 0 failed");
    return s_failed ? 1 : 0;
}
