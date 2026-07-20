#include "gimbal_app.h"
#include "drv_servo.h"
#include "drv_laser.h"

/* ==================== 实测四角数据 ==================== */

/* 左上 */
#define SQUARE_TL_HORIZONTAL_US    1340U
#define SQUARE_TL_PITCH_US         1332U

/* 右上 */
#define SQUARE_TR_HORIZONTAL_US    1145U
#define SQUARE_TR_PITCH_US         1320U

/* 右下 */
#define SQUARE_BR_HORIZONTAL_US    1140U
#define SQUARE_BR_PITCH_US         1607U

/* 左下 */
#define SQUARE_BL_HORIZONTAL_US    1350U
#define SQUARE_BL_PITCH_US         1605U

/*
 * 每条正方形边分成200步。
 * GimbalApp_Update()每20ms调用一次：
 *
 * 200 × 20ms = 4000ms
 *
 * 每条边约4秒，正方形一周约16秒。
 */
#define SQUARE_EDGE_STEPS          200U

/*
 * 中心与左上角之间的定位移动分成100步。
 */
#define SQUARE_POSITION_STEPS      100U

typedef struct {
    uint16_t horizontal_us;
    uint16_t pitch_us;
} GimbalPoint_t;

static const GimbalPoint_t s_square_end_points[] = {
    /* 段0：当前位置/中心 -> 左上，激光关闭 */
    {SQUARE_TL_HORIZONTAL_US, SQUARE_TL_PITCH_US},

    /* 段1：左上 -> 右上 */
    {SQUARE_TR_HORIZONTAL_US, SQUARE_TR_PITCH_US},

    /* 段2：右上 -> 右下 */
    {SQUARE_BR_HORIZONTAL_US, SQUARE_BR_PITCH_US},

    /* 段3：右下 -> 左下 */
    {SQUARE_BL_HORIZONTAL_US, SQUARE_BL_PITCH_US},

    /* 段4：左下 -> 左上，闭合正方形 */
    {SQUARE_TL_HORIZONTAL_US, SQUARE_TL_PITCH_US},

    /* 段5：左上 -> 中心，激光关闭 */
    {SERVO_HORIZONTAL_CENTER_US, SERVO_PITCH_CENTER_US}
};

#define SQUARE_SEGMENT_COUNT    \
    ((uint8_t)(sizeof(s_square_end_points) / \
               sizeof(s_square_end_points[0])))

static GimbalApp_State_t s_state = GIMBAL_APP_IDLE;
static uint8_t s_square_segment;
static uint16_t s_square_step;
static GimbalPoint_t s_segment_start;

static uint16_t GimbalApp_GetSegmentSteps(uint8_t segment)
{
    if ((segment == 0U) || (segment == 5U)) {
        return SQUARE_POSITION_STEPS;
    }

    return SQUARE_EDGE_STEPS;
}

static void GimbalApp_StartCurrentSegment(void)
{
    s_segment_start.horizontal_us =
        Drv_Servo_GetHorizontalPulse();

    s_segment_start.pitch_us =
        Drv_Servo_GetPitchPulse();

    s_square_step = 0U;
}

static void GimbalApp_UpdateSquare(void)
{
    const GimbalPoint_t *end;
    uint16_t total_steps;
    int32_t horizontal_delta;
    int32_t pitch_delta;
    int32_t horizontal_output;
    int32_t pitch_output;

    if (s_square_segment >= SQUARE_SEGMENT_COUNT) {
        Drv_Laser_Off();
        Drv_Servo_SetHorizontalPulse(
            SERVO_HORIZONTAL_CENTER_US
        );
        Drv_Servo_SetPitchPulse(
            SERVO_PITCH_CENTER_US
        );

        s_state = GIMBAL_APP_IDLE;
        return;
    }

    end = &s_square_end_points[s_square_segment];
    total_steps = GimbalApp_GetSegmentSteps(
        s_square_segment
    );

    if (s_square_step < total_steps) {
        s_square_step++;
    }

    horizontal_delta =
        (int32_t)end->horizontal_us -
        (int32_t)s_segment_start.horizontal_us;

    pitch_delta =
        (int32_t)end->pitch_us -
        (int32_t)s_segment_start.pitch_us;

    horizontal_output =
        (int32_t)s_segment_start.horizontal_us +
        (horizontal_delta * (int32_t)s_square_step) /
        (int32_t)total_steps;

    pitch_output =
        (int32_t)s_segment_start.pitch_us +
        (pitch_delta * (int32_t)s_square_step) /
        (int32_t)total_steps;

    /*
     * 轨迹插值直接输出脉宽。
     * 每20ms只输出一个新的插值点，不使用阻塞Delay。
     */
    Drv_Servo_SetHorizontalPulse(
        (uint16_t)horizontal_output
    );

    Drv_Servo_SetPitchPulse(
        (uint16_t)pitch_output
    );

    if (s_square_step >= total_steps) {
        /*
         * 段0完成：已到左上角，开始点亮激光。
         */
        if (s_square_segment == 0U) {
            Drv_Laser_On();
        }

        /*
         * 段4完成：正方形已经闭合，关闭激光。
         */
        if (s_square_segment == 4U) {
            Drv_Laser_Off();
        }

        s_square_segment++;

        if (s_square_segment < SQUARE_SEGMENT_COUNT) {
            GimbalApp_StartCurrentSegment();
        } else {
            Drv_Laser_Off();
            s_state = GIMBAL_APP_IDLE;
        }
    }
}

void GimbalApp_Init(void)
{
    s_state = GIMBAL_APP_IDLE;
    s_square_segment = 0U;
    s_square_step = 0U;

    Drv_Laser_Off();

    /*
     * 设置非阻塞回中目标。
     */
    Drv_Servo_Center();
}

void GimbalApp_StartSquareTest(void)
{
    Drv_Laser_Off();

    s_state = GIMBAL_APP_SQUARE_TEST;
    s_square_segment = 0U;
    s_square_step = 0U;

    GimbalApp_StartCurrentSegment();
}

void GimbalApp_StartTrack(void)
{
    Drv_Laser_Off();
    s_state = GIMBAL_APP_TRACK;
}

void GimbalApp_Stop(void)
{
    Drv_Laser_Off();
    Drv_Servo_Center();

    s_state = GIMBAL_APP_STOP;
}

GimbalApp_State_t GimbalApp_GetState(void)
{
    return s_state;
}

uint8_t GimbalApp_GetSquareSegment(void)
{
    return s_square_segment;
}

void GimbalApp_Update(void)
{
    switch (s_state) {
        case GIMBAL_APP_IDLE:
            break;

        case GIMBAL_APP_SQUARE_TEST:
            GimbalApp_UpdateSquare();
            break;

        case GIMBAL_APP_TRACK:
            /*
             * 下一阶段加入K210视觉闭环。
             */
            break;

        case GIMBAL_APP_STOP:
        default:
            break;
    }

    /*
     * 处理Drv_Servo_Center()等非阻塞目标。
     * 正方形状态使用立即脉宽输出，这里不会重复移动。
     */
    Drv_Servo_Update();
}