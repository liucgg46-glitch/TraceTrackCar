#include "attitude_estimator.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned int s_passed = 0U;
static unsigned int s_failed = 0U;

#define TEST_CHECK(condition)                                                   \
    do {                                                                        \
        if (!(condition)) {                                                     \
            (void)printf("  FAIL line %d: %s\n", __LINE__, #condition);         \
            return 0;                                                           \
        }                                                                       \
    } while (0)

static Attitude_Input_t Test_MakeLevelSample(uint32_t timestamp_ms)
{
    Attitude_Input_t input;

    (void)memset(&input, 0, sizeof(input));
    input.accel_filtered_g.z = 1.0f;
    input.timestamp_ms = timestamp_ms;
    return input;
}

static void Test_SetHealthyMagSample(Attitude_Input_t *input)
{
    /* 构造校准后约为 [30, 0, 40] uT、模长约 50 uT 的稳定磁场。 */
    input->mag_filtered_uT.x = ATTITUDE_MAG_CAL_OFFSET_X_UT +
                               30.0f / ATTITUDE_MAG_CAL_SCALE_X;
    input->mag_filtered_uT.y = ATTITUDE_MAG_CAL_OFFSET_Y_UT;
    input->mag_filtered_uT.z = ATTITUDE_MAG_CAL_OFFSET_Z_UT +
                               40.0f / ATTITUDE_MAG_CAL_SCALE_Z;
    input->mag_uT = input->mag_filtered_uT;
    input->mag_valid = 1U;
    input->mag_updated = 1U;
}

static int Test_StaticInputConverges(void)
{
    Attitude_Input_t input;
    Attitude_Info_t info;
    uint32_t timestamp_ms = 10U;
    unsigned int sample;

    Attitude_Init();
    input = Test_MakeLevelSample(timestamp_ms);
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);

    for (sample = 0U; sample < 150U; sample++) {
        timestamp_ms += 10U;
        input.timestamp_ms = timestamp_ms;
        TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
    }

    TEST_CHECK(Attitude_GetInfo(&info) == BSP_OK);
    TEST_CHECK(info.valid == 1U);
    TEST_CHECK(info.stationary == 1U);
    TEST_CHECK(info.update_count == 151U);
    TEST_CHECK(fabsf(info.roll_deg) < 0.1f);
    TEST_CHECK(fabsf(info.pitch_deg) < 0.1f);
    TEST_CHECK(fabsf(info.yaw_deg) < 0.1f);
    return 1;
}

static int Test_ConstantYawRotation(void)
{
    Attitude_Input_t input;
    Attitude_Info_t info;
    uint32_t timestamp_ms = 10U;
    unsigned int sample;

    Attitude_Init();
    input = Test_MakeLevelSample(timestamp_ms);
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);

    input.gyro_filtered_dps.z = 90.0f;
    for (sample = 0U; sample < 100U; sample++) {
        timestamp_ms += 10U;
        input.timestamp_ms = timestamp_ms;
        TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
    }

    TEST_CHECK(Attitude_GetInfo(&info) == BSP_OK);
    TEST_CHECK(fabsf(info.yaw_deg - 90.0f) < 1.0f);
    TEST_CHECK(fabsf(info.roll_deg) < 0.5f);
    TEST_CHECK(fabsf(info.pitch_deg) < 0.5f);
    TEST_CHECK(info.stationary == 0U);
    return 1;
}

static int Test_DuplicateTimestampRejected(void)
{
    Attitude_Input_t input;
    Attitude_Info_t before;
    Attitude_Info_t after;

    Attitude_Init();
    input = Test_MakeLevelSample(10U);
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
    TEST_CHECK(Attitude_GetInfo(&before) == BSP_OK);

    input.gyro_filtered_dps.z = 180.0f;
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_BUSY);
    TEST_CHECK(Attitude_GetInfo(&after) == BSP_OK);
    TEST_CHECK(after.update_count == before.update_count);
    TEST_CHECK(after.timestamp_ms == before.timestamp_ms);
    TEST_CHECK(fabsf(after.yaw_deg - before.yaw_deg) < 0.001f);
    return 1;
}

static int Test_InvalidInputAndRecovery(void)
{
    Attitude_Input_t input;
    Attitude_Info_t info;

    Attitude_Init();
    input = Test_MakeLevelSample(10U);
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
    TEST_CHECK(Attitude_IsValid() == 1U);

    TEST_CHECK(Attitude_Update(0, 0U) == BSP_PARAM);
    TEST_CHECK(Attitude_IsValid() == 0U);
    TEST_CHECK(Attitude_GetInfo(&info) == BSP_ERROR);
    TEST_CHECK(info.valid == 0U);

    input.timestamp_ms += 10U;
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
    TEST_CHECK(Attitude_IsValid() == 1U);

    Attitude_Invalidate();
    TEST_CHECK(Attitude_IsValid() == 0U);
    input.timestamp_ms += 10U;
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
    TEST_CHECK(Attitude_IsValid() == 1U);
    return 1;
}

static int Test_MotorActiveGatesMagnetometer(void)
{
    Attitude_Input_t input;
    Attitude_Info_t before_motor;
    Attitude_Info_t during_motor;
    Attitude_Info_t after_motor;
    uint32_t timestamp_ms = 10U;
    unsigned int sample;

    Attitude_Init();
    input = Test_MakeLevelSample(timestamp_ms);
    Test_SetHealthyMagSample(&input);

    for (sample = 0U; sample < 12U; sample++) {
        input.timestamp_ms = timestamp_ms;
        TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
        timestamp_ms += 10U;
    }
    TEST_CHECK(Attitude_GetInfo(&before_motor) == BSP_OK);
    TEST_CHECK(before_motor.mag_available == 1U);
    TEST_CHECK(before_motor.mag_healthy == 1U);
    TEST_CHECK(before_motor.mag_used == 1U);
    TEST_CHECK(before_motor.mag_accept_count >= 2U);

    input.timestamp_ms = timestamp_ms;
    TEST_CHECK(Attitude_Update(&input, 1U) == BSP_OK);
    timestamp_ms += 10U;
    TEST_CHECK(Attitude_GetInfo(&during_motor) == BSP_OK);
    TEST_CHECK(during_motor.mag_available == 1U);
    TEST_CHECK(during_motor.mag_used == 0U);
    TEST_CHECK(during_motor.mag_accept_count == before_motor.mag_accept_count);

    input.timestamp_ms = timestamp_ms;
    TEST_CHECK(Attitude_Update(&input, 0U) == BSP_OK);
    TEST_CHECK(Attitude_GetInfo(&after_motor) == BSP_OK);
    TEST_CHECK(after_motor.mag_healthy == 1U);
    TEST_CHECK(after_motor.mag_used == 1U);
    TEST_CHECK(after_motor.mag_accept_count == before_motor.mag_accept_count + 1U);
    return 1;
}

typedef int (*Test_Function_t)(void);

typedef struct {
    const char *name;
    Test_Function_t function;
} Test_Case_t;

int main(void)
{
    static const Test_Case_t cases[] = {
        { "static input converges", Test_StaticInputConverges },
        { "constant yaw rotation", Test_ConstantYawRotation },
        { "duplicate timestamp rejected", Test_DuplicateTimestampRejected },
        { "invalid input and recovery", Test_InvalidInputAndRecovery },
        { "motor active gates magnetometer", Test_MotorActiveGatesMagnetometer }
    };
    unsigned int index;

    for (index = 0U; index < (unsigned int)(sizeof(cases) / sizeof(cases[0])); index++) {
        (void)printf("[TEST] %s\n", cases[index].name);
        if (cases[index].function() != 0) {
            s_passed++;
            (void)printf("  PASS\n");
        } else {
            s_failed++;
        }
    }

    (void)printf("\nResult: %u passed, %u failed\n", s_passed, s_failed);
    return (s_failed == 0U) ? 0 : 1;
}
