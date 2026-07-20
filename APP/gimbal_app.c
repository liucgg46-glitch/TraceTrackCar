#include "gimbal_app.h"
#include "drv_servo.h"
#include "drv_laser.h"

/*
 * 四角使用归一化位置表示，不暴露 PWM 脉宽。
 * 下列数值由原实测四角脉宽等价换算，映射后的端点保持不变。
 */
#define SQUARE_TL_HORIZONTAL_PERMILLE      366
#define SQUARE_TL_PITCH_PERMILLE          (-472)

#define SQUARE_TR_HORIZONTAL_PERMILLE      (-416)
#define SQUARE_TR_PITCH_PERMILLE          (-520)

#define SQUARE_BR_HORIZONTAL_PERMILLE      (-436)
#define SQUARE_BR_PITCH_PERMILLE          414

#define SQUARE_BL_HORIZONTAL_PERMILLE      405
#define SQUARE_BL_PITCH_PERMILLE          408

/* 每条边 200 个 20 ms 插值点，保持原来约 4 s/边的速度。 */
#define SQUARE_EDGE_STEPS                  200U

/* 中位到左上角、左上角回中位各使用 100 个插值点。 */
#define SQUARE_POSITION_STEPS              100U

static const Drv_Servo_Position_t s_square_end_points[] = {
    /* 段 0：当前位置/中位到左上，激光关闭。 */
    {SQUARE_TL_HORIZONTAL_PERMILLE, SQUARE_TL_PITCH_PERMILLE},
    /* 段 1：左上到右上。 */
    {SQUARE_TR_HORIZONTAL_PERMILLE, SQUARE_TR_PITCH_PERMILLE},
    /* 段 2：右上到右下。 */
    {SQUARE_BR_HORIZONTAL_PERMILLE, SQUARE_BR_PITCH_PERMILLE},
    /* 段 3：右下到左下。 */
    {SQUARE_BL_HORIZONTAL_PERMILLE, SQUARE_BL_PITCH_PERMILLE},
    /* 段 4：左下到左上，闭合正方形。 */
    {SQUARE_TL_HORIZONTAL_PERMILLE, SQUARE_TL_PITCH_PERMILLE},
    /* 段 5：左上回中位，激光关闭。 */
    {0, 0}
};

#define SQUARE_SEGMENT_COUNT \
    ((uint8_t)(sizeof(s_square_end_points) / \
               sizeof(s_square_end_points[0])))

static GimbalApp_State_t s_state = GIMBAL_APP_IDLE;
static uint8_t s_square_segment;
static uint16_t s_square_step;
static Drv_Servo_Position_t s_segment_start;

static uint16_t GimbalApp_GetSegmentSteps(uint8_t segment)
{
    if ((segment == 0U) || (segment == 5U)) {
        return SQUARE_POSITION_STEPS;
    }
    return SQUARE_EDGE_STEPS;
}

static uint8_t GimbalApp_StartCurrentSegment(void)
{
    Drv_Servo_Info_t info;

    if (Drv_Servo_GetInfo(&info) != BSP_OK) {
        return 0U;
    }

    s_segment_start = info.current;
    s_square_step = 0U;
    return 1U;
}

static void GimbalApp_EnterSafeStop(void)
{
    Drv_Laser_Off();
    Drv_Servo_Center();
    s_state = GIMBAL_APP_STOP;
}

static void GimbalApp_UpdateSquare(void)
{
    const Drv_Servo_Position_t *end;
    Drv_Servo_Position_t output;
    Drv_Laser_Info_t laser_info;
    uint16_t total_steps;
    int32_t horizontal_delta;
    int32_t pitch_delta;

    if ((Drv_Laser_GetInfo(&laser_info) != BSP_OK) ||
        (laser_info.timeout_tripped != 0U)) {
        GimbalApp_EnterSafeStop();
        return;
    }

    if (s_square_segment >= SQUARE_SEGMENT_COUNT) {
        Drv_Laser_Off();
        Drv_Servo_Center();
        s_state = GIMBAL_APP_IDLE;
        return;
    }

    end = &s_square_end_points[s_square_segment];
    total_steps = GimbalApp_GetSegmentSteps(s_square_segment);
    if (s_square_step < total_steps) {
        s_square_step++;
    }

    horizontal_delta =
        (int32_t)end->horizontal_permille -
        (int32_t)s_segment_start.horizontal_permille;
    pitch_delta =
        (int32_t)end->pitch_permille -
        (int32_t)s_segment_start.pitch_permille;

    output.horizontal_permille = (int16_t)(
        (int32_t)s_segment_start.horizontal_permille +
        (horizontal_delta * (int32_t)s_square_step) /
        (int32_t)total_steps
    );
    output.pitch_permille = (int16_t)(
        (int32_t)s_segment_start.pitch_permille +
        (pitch_delta * (int32_t)s_square_step) /
        (int32_t)total_steps
    );

    if (Drv_Servo_SetImmediatePosition(&output) != BSP_OK) {
        GimbalApp_EnterSafeStop();
        return;
    }

    if (s_square_step < total_steps) {
        return;
    }

    if (s_square_segment == 0U) {
        Drv_Laser_On();
    } else if (s_square_segment == 4U) {
        Drv_Laser_Off();
    }

    s_square_segment++;
    if (s_square_segment >= SQUARE_SEGMENT_COUNT) {
        Drv_Laser_Off();
        s_state = GIMBAL_APP_IDLE;
        return;
    }

    if (GimbalApp_StartCurrentSegment() == 0U) {
        GimbalApp_EnterSafeStop();
    }
}

void GimbalApp_Init(void)
{
    s_state = GIMBAL_APP_IDLE;
    s_square_segment = 0U;
    s_square_step = 0U;
    Drv_Laser_Off();
    Drv_Servo_Center();
}

void GimbalApp_StartSquareTest(void)
{
    Drv_Laser_Off();
    Drv_Laser_ClearTimeoutFlag();
    s_square_segment = 0U;
    s_square_step = 0U;

    if (GimbalApp_StartCurrentSegment() == 0U) {
        GimbalApp_EnterSafeStop();
        return;
    }
    s_state = GIMBAL_APP_SQUARE_TEST;
}

void GimbalApp_Stop(void)
{
    GimbalApp_EnterSafeStop();
}

void GimbalApp_Update(void)
{
    switch (s_state) {
        case GIMBAL_APP_IDLE:
        case GIMBAL_APP_STOP:
            break;

        case GIMBAL_APP_SQUARE_TEST:
            GimbalApp_UpdateSquare();
            break;

        default:
            GimbalApp_EnterSafeStop();
            break;
    }
}

void Gimbal_Update(void)
{
    GimbalApp_Update();
}

GimbalApp_State_t GimbalApp_GetState(void)
{
    return s_state;
}

uint8_t GimbalApp_GetSquareSegment(void)
{
    return s_square_segment;
}
