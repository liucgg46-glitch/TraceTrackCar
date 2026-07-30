#include "ball_balance_k210_adapter.h"

#include "bsp_systick.h"
#include "k210_comm.h"

static uint8_t s_initialized;
static uint8_t s_sequence;
static uint8_t s_last_k210_state;
static uint8_t s_last_app_state;
static int16_t s_last_position_mm_x10;
static uint8_t s_last_confidence;
static uint32_t s_last_sample_ms;
static uint32_t s_pushed_count;
static uint32_t s_busy_count;
static uint32_t s_error_count;

static uint32_t BallBalance_K210Adapter_IncrementU32(uint32_t value)
{
    return (value < 0xFFFFFFFFUL) ? (value + 1UL) : value;
}

static uint8_t BallBalance_K210Adapter_MapState(uint8_t k210_state)
{
    switch (k210_state) {
        case K210_BALL_STATE_VALID:
            return BALL_BALANCE_VISION_VALID;

        case K210_BALL_STATE_HOLD:
            return BALL_BALANCE_VISION_HOLD;

        case K210_BALL_STATE_LOST:
        default:
            return BALL_BALANCE_VISION_LOST;
    }
}

void BallBalance_K210Adapter_Init(void)
{
    s_sequence = 0U;
    s_last_k210_state = K210_BALL_STATE_LOST;
    s_last_app_state = BALL_BALANCE_VISION_LOST;
    s_last_position_mm_x10 = 0;
    s_last_confidence = 0U;
    s_last_sample_ms = 0U;
    s_pushed_count = 0U;
    s_busy_count = 0U;
    s_error_count = 0U;
    s_initialized = 1U;
}

void BallBalance_K210Adapter_Update(void)
{
    int16_t position_tenth_mm;
    uint8_t k210_state;
    uint8_t confidence;
    BSP_Status_t status;
    BallBalance_VisionSample_t sample;

    if (s_initialized == 0U) {
        return;
    }

    status = K210_Comm_GetNewBallPosition(
        &position_tenth_mm,
        &k210_state,
        &confidence
    );
    if (status == BSP_BUSY) {
        s_busy_count = BallBalance_K210Adapter_IncrementU32(s_busy_count);
        return;
    }
    if (status != BSP_OK) {
        s_error_count = BallBalance_K210Adapter_IncrementU32(s_error_count);
        return;
    }

    s_sequence++;
    sample.position_mm_x10 = position_tenth_mm;
    sample.state = BallBalance_K210Adapter_MapState(k210_state);
    sample.valid = (sample.state == BALL_BALANCE_VISION_VALID) ? 1U : 0U;
    sample.confidence = confidence;
    sample.sequence = s_sequence;
    sample.timestamp_ms = BSP_GetTickMs();

    BallBalance_App_PushVisionSample(&sample);

    s_last_k210_state = k210_state;
    s_last_app_state = sample.state;
    s_last_position_mm_x10 = position_tenth_mm;
    s_last_confidence = confidence;
    s_last_sample_ms = sample.timestamp_ms;
    s_pushed_count = BallBalance_K210Adapter_IncrementU32(s_pushed_count);
}

BSP_Status_t BallBalance_K210Adapter_GetInfo(
    BallBalance_K210AdapterInfo_t *info
)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    info->initialized = s_initialized;
    info->sequence = s_sequence;
    info->last_k210_state = s_last_k210_state;
    info->last_app_state = s_last_app_state;
    info->last_position_mm_x10 = s_last_position_mm_x10;
    info->last_confidence = s_last_confidence;
    info->last_sample_ms = s_last_sample_ms;
    info->pushed_count = s_pushed_count;
    info->busy_count = s_busy_count;
    info->error_count = s_error_count;
    return BSP_OK;
}
