#include "motion_action.h"
#include "odometer.h"
#include "angle_control.h"
#include "chassis.h"
#include "bsp_common.h"

static Motion_Info_t s_motion;

static int32_t Motion_Abs32(int32_t x)
{
    return (x >= 0) ? x : -x;
}

static float Motion_AbsFloat(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static int16_t Motion_LimitSpeed(int16_t speed)
{
    int16_t sign = (speed >= 0) ? 1 : -1;
    int32_t abs_speed = speed;

    if (abs_speed < 0) abs_speed = -abs_speed;
    if (abs_speed < MOTION_MIN_ABS_SPEED_CPS) abs_speed = MOTION_MIN_ABS_SPEED_CPS;
    if (abs_speed > MOTION_MAX_ABS_SPEED_CPS) abs_speed = MOTION_MAX_ABS_SPEED_CPS;

    return (int16_t)(sign * abs_speed);
}

static void Motion_SetDone(void)
{
    Chassis_Stop();
    s_motion.state = MOTION_DONE;
    s_motion.action = MOTION_ACTION_NONE;
}

static void Motion_SetError(void)
{
    Chassis_Stop();
    s_motion.state = MOTION_ERROR;
    s_motion.action = MOTION_ACTION_NONE;
}

void Motion_Init(void)
{
    s_motion.state = MOTION_IDLE;
    s_motion.action = MOTION_ACTION_NONE;
    s_motion.target_distance_mm = 0;
    s_motion.target_angle_deg = 0;
    s_motion.speed_cps = 0;
    s_motion.current_distance_mm = 0;
    s_motion.current_yaw_deg = 0.0f;
    s_motion.start_time_ms = 0;
    s_motion.timeout_ms = MOTION_DEFAULT_TIMEOUT_MS;
}

BSP_Status_t Motion_GoDistance(int32_t distance_mm, int16_t speed_cps)
{
    if (s_motion.state == MOTION_RUNNING) return BSP_BUSY;
    if (distance_mm == 0) return BSP_PARAM;

    Odometer_Clear();
    AngleControl_Clear();

    s_motion.action = MOTION_ACTION_GO_DISTANCE;
    s_motion.state = MOTION_RUNNING;
    s_motion.target_distance_mm = distance_mm;
    s_motion.target_angle_deg = 0;
    s_motion.speed_cps = Motion_LimitSpeed(speed_cps);
    if (distance_mm < 0 && s_motion.speed_cps > 0) {
        s_motion.speed_cps = (int16_t)(-s_motion.speed_cps);
    } else if (distance_mm > 0 && s_motion.speed_cps < 0) {
        s_motion.speed_cps = (int16_t)(-s_motion.speed_cps);
    }
    s_motion.current_distance_mm = 0;
    s_motion.current_yaw_deg = 0.0f;
    s_motion.start_time_ms = BSP_GET_TICK();
    s_motion.timeout_ms = MOTION_DEFAULT_TIMEOUT_MS;

    return BSP_OK;
}

BSP_Status_t Motion_TurnAngle(int16_t angle_deg, int16_t speed_cps)
{
    if (s_motion.state == MOTION_RUNNING) return BSP_BUSY;
    if (angle_deg == 0) return BSP_PARAM;

    Odometer_Clear();
    AngleControl_Clear();

    s_motion.action = MOTION_ACTION_TURN_ANGLE;
    s_motion.state = MOTION_RUNNING;
    s_motion.target_distance_mm = 0;
    s_motion.target_angle_deg = angle_deg;
    s_motion.speed_cps = Motion_LimitSpeed(speed_cps);
    if (s_motion.speed_cps < 0) {
        s_motion.speed_cps = (int16_t)(-s_motion.speed_cps);
    }
    s_motion.current_distance_mm = 0;
    s_motion.current_yaw_deg = 0.0f;
    s_motion.start_time_ms = BSP_GET_TICK();
    s_motion.timeout_ms = MOTION_DEFAULT_TIMEOUT_MS;

    return BSP_OK;
}

void Motion_Stop(void)
{
    Chassis_Stop();
    s_motion.state = MOTION_IDLE;
    s_motion.action = MOTION_ACTION_NONE;
}

void Motion_Update(void)
{
    int32_t target_abs;
    int32_t dist_abs;
    float err_deg;
    int16_t cmd;

    if (s_motion.state != MOTION_RUNNING) {
        return;
    }

    Odometer_Update();
    AngleControl_Update();

    s_motion.current_distance_mm = Odometer_GetDistanceMm();
    s_motion.current_yaw_deg = AngleControl_GetYawDeg();

    if ((BSP_GET_TICK() - s_motion.start_time_ms) > s_motion.timeout_ms) {
        Motion_SetError();
        return;
    }

    switch (s_motion.action) {
        case MOTION_ACTION_GO_DISTANCE:
            target_abs = Motion_Abs32(s_motion.target_distance_mm);
            dist_abs = Motion_Abs32(s_motion.current_distance_mm);

            if (dist_abs + MOTION_DISTANCE_TOLERANCE_MM >= target_abs) {
                Motion_SetDone();
            } else {
                Chassis_SetSpeed(s_motion.speed_cps, 0);
            }
            break;

        case MOTION_ACTION_TURN_ANGLE:
            err_deg = AngleControl_GetErrorDeg((float)s_motion.target_angle_deg);
            if (Motion_AbsFloat(err_deg) <= MOTION_ANGLE_TOLERANCE_DEG) {
                Motion_SetDone();
            } else {
                /* 约定 turn > 0 左转，turn < 0 右转。 */
                cmd = (err_deg > 0.0f) ? s_motion.speed_cps : (int16_t)(-s_motion.speed_cps);
                Chassis_SetSpeed(0, cmd);
            }
            break;

        default:
            Motion_SetError();
            break;
    }
}

uint8_t Motion_IsBusy(void)
{
    return (s_motion.state == MOTION_RUNNING) ? 1U : 0U;
}

uint8_t Motion_IsDone(void)
{
    return (s_motion.state == MOTION_DONE) ? 1U : 0U;
}

MotionState_t Motion_GetState(void)
{
    return s_motion.state;
}

BSP_Status_t Motion_GetInfo(Motion_Info_t *info)
{
    if (info == 0) return BSP_PARAM;
    *info = s_motion;
    return BSP_OK;
}
