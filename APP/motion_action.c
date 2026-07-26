#include "motion_action.h"
#include "control_config.h"
#include "odometer.h"
#include "odometer_adapter.h"
#include "heading_estimator.h"
#include "chassis.h"
#include "sensor_manager.h"
#include "bsp_common.h"

static Motion_Info_t s_motion;
static uint8_t s_turn_settle_samples;
static uint8_t s_turn_correction_mode;
static uint8_t s_distance_heading_enable;

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
    if (abs_speed < CONTROL_MOTION_MIN_ABS_SPEED_CPS) abs_speed = CONTROL_MOTION_MIN_ABS_SPEED_CPS;
    if (abs_speed > CONTROL_MOTION_MAX_ABS_SPEED_CPS) abs_speed = CONTROL_MOTION_MAX_ABS_SPEED_CPS;

    return (int16_t)(sign * abs_speed);
}

static void Motion_SetDone(void)
{
    (void)Chassis_ReleaseControl(CHASSIS_OWNER_MOTION);
    s_motion.state = MOTION_DONE;
    s_motion.action = MOTION_ACTION_NONE;
    s_turn_settle_samples = 0U;
    s_turn_correction_mode = 0U;
    s_distance_heading_enable = 0U;
}

static void Motion_SetError(void)
{
    (void)Chassis_ReleaseControl(CHASSIS_OWNER_MOTION);
    s_motion.state = MOTION_ERROR;
    s_motion.action = MOTION_ACTION_NONE;
    s_turn_settle_samples = 0U;
    s_turn_correction_mode = 0U;
    s_distance_heading_enable = 0U;
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
    s_motion.timeout_ms = CONTROL_MOTION_DEFAULT_TIMEOUT_MS;
    s_turn_settle_samples = 0U;
    s_turn_correction_mode = 0U;
    s_distance_heading_enable = 0U;
}

BSP_Status_t Motion_GoDistance(int32_t distance_mm, int16_t speed_cps)
{
    if (s_motion.state == MOTION_RUNNING) return BSP_BUSY;
    if (distance_mm == 0) return BSP_PARAM;
    if (Chassis_AcquireControl(CHASSIS_OWNER_MOTION) != BSP_OK) {
        return BSP_BUSY;
    }

    AppOdometer_Clear();
    Heading_Reset();
    s_distance_heading_enable =
        (Sensor_IsImuReadyForMotion() != 0U) ? 1U : 0U;

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
    s_motion.timeout_ms = CONTROL_MOTION_DEFAULT_TIMEOUT_MS;
    s_turn_settle_samples = 0U;
    s_turn_correction_mode = 0U;

    return BSP_OK;
}

BSP_Status_t Motion_TurnAngle(int16_t angle_deg)
{
    if (s_motion.state == MOTION_RUNNING) return BSP_BUSY;
    if (angle_deg == 0) return BSP_PARAM;
    if (Sensor_IsImuReadyForMotion() == 0U) return BSP_ERROR;
    if (Chassis_AcquireControl(CHASSIS_OWNER_MOTION) != BSP_OK) {
        return BSP_BUSY;
    }

    AppOdometer_Clear();
    Heading_Reset();
    s_distance_heading_enable = 0U;

    s_motion.action = MOTION_ACTION_TURN_ANGLE;
    s_motion.state = MOTION_RUNNING;
    s_motion.target_distance_mm = 0;
    s_motion.target_angle_deg = angle_deg;
    s_motion.speed_cps = Motion_LimitSpeed(CONTROL_MOTION_TURN_SPEED_CPS);
    if (s_motion.speed_cps < 0) {
        s_motion.speed_cps = (int16_t)(-s_motion.speed_cps);
    }
    if (s_motion.speed_cps < CONTROL_MOTION_TURN_MIN_SPEED_CPS) {
        s_motion.speed_cps = CONTROL_MOTION_TURN_MIN_SPEED_CPS;
    }
    s_motion.current_distance_mm = 0;
    s_motion.current_yaw_deg = 0.0f;
    s_motion.start_time_ms = BSP_GET_TICK();
    s_motion.timeout_ms = CONTROL_MOTION_DEFAULT_TIMEOUT_MS;
    s_turn_settle_samples = 0U;
    s_turn_correction_mode = 0U;

    return BSP_OK;
}

void Motion_Stop(void)
{
    (void)Chassis_ReleaseControl(CHASSIS_OWNER_MOTION);
    s_motion.state = MOTION_IDLE;
    s_motion.action = MOTION_ACTION_NONE;
    s_turn_settle_samples = 0U;
    s_turn_correction_mode = 0U;
    s_distance_heading_enable = 0U;
}

void Motion_Update(void)
{
    int32_t target_abs;
    int32_t dist_abs;
    float err_deg;
    float cmd_f;
    int16_t cmd;
    int16_t distance_turn_limit;

    if (s_motion.state != MOTION_RUNNING) {
        return;
    }

    AppOdometer_Update();
    Heading_Update();

    s_motion.current_distance_mm = Odometer_GetDistanceMm();
    s_motion.current_yaw_deg = Heading_GetYawDeg();

    if ((BSP_GET_TICK() - s_motion.start_time_ms) > s_motion.timeout_ms) {
        Motion_SetError();
        return;
    }

    switch (s_motion.action) {
        case MOTION_ACTION_GO_DISTANCE:
            target_abs = Motion_Abs32(s_motion.target_distance_mm);
            dist_abs = Motion_Abs32(s_motion.current_distance_mm);

            if (dist_abs + CONTROL_MOTION_DISTANCE_TOLERANCE_MM >= target_abs) {
                Motion_SetDone();
            } else {
                cmd = 0;

                /*
                 * 距离动作以启动时航向为0°。IMU可用时只修正动作期间
                 * 新产生的偏航，不改变Route已经决定的前进方向。
                 */
                if (s_distance_heading_enable != 0U) {
                    err_deg = Heading_GetErrorDeg(0.0f);
                    if (Motion_AbsFloat(err_deg) >
                        CONTROL_MOTION_DISTANCE_HEADING_DEADBAND_DEG) {
                        cmd_f = Motion_AbsFloat(err_deg) *
                                CONTROL_MOTION_DISTANCE_HEADING_KP_CPS_PER_DEG;
                        distance_turn_limit =
                            CONTROL_MOTION_DISTANCE_HEADING_MAX_TURN_CPS;
                        if (Motion_Abs32((int32_t)s_motion.speed_cps) <
                            (int32_t)distance_turn_limit) {
                            distance_turn_limit = (int16_t)Motion_Abs32(
                                (int32_t)s_motion.speed_cps);
                        }
                        if (cmd_f > (float)distance_turn_limit) {
                            cmd_f = (float)distance_turn_limit;
                        }
                        cmd = (err_deg > 0.0f) ?
                              (int16_t)cmd_f : (int16_t)(-cmd_f);
                    }
                }

                if (Chassis_SetSpeed(CHASSIS_OWNER_MOTION,
                                     s_motion.speed_cps,
                                     cmd) != BSP_OK) {
                    Motion_SetError();
                }
            }
            break;

        case MOTION_ACTION_TURN_ANGLE:
            err_deg = Heading_GetErrorDeg((float)s_motion.target_angle_deg);
            if (Motion_AbsFloat(err_deg) <= CONTROL_MOTION_ANGLE_TOLERANCE_DEG) {
                if (s_turn_settle_samples < 0xFFU) {
                    s_turn_settle_samples++;
                }
                if (Chassis_Stop(CHASSIS_OWNER_MOTION) != BSP_OK) {
                    Motion_SetError();
                    return;
                }
                if (s_turn_settle_samples >= CONTROL_MOTION_TURN_SETTLE_SAMPLES) {
                    Motion_SetDone();
                }
            } else {
                s_turn_settle_samples = 0U;
                /* 约定 turn > 0 左转，turn < 0 右转。 */
                if (((s_motion.target_angle_deg > 0) && (err_deg < 0.0f)) ||
                    ((s_motion.target_angle_deg < 0) && (err_deg > 0.0f))) {
                    /* 首次越过目标后锁定低速回正，避免再次高速跨过目标。 */
                    s_turn_correction_mode = 1U;
                }

                if (s_turn_correction_mode != 0U) {
                    cmd_f = (float)CONTROL_MOTION_TURN_CORRECTION_SPEED_CPS;
                } else {
                    cmd_f = Motion_AbsFloat(err_deg) * CONTROL_MOTION_TURN_KP_CPS_PER_DEG;
                    if (cmd_f > (float)s_motion.speed_cps) {
                        cmd_f = (float)s_motion.speed_cps;
                    }
                    if (cmd_f < (float)CONTROL_MOTION_TURN_MIN_SPEED_CPS) {
                        cmd_f = (float)CONTROL_MOTION_TURN_MIN_SPEED_CPS;
                    }
                }
                cmd = (err_deg > 0.0f) ? (int16_t)cmd_f : (int16_t)(-cmd_f);
                if (Chassis_SetSpeed(CHASSIS_OWNER_MOTION, 0, cmd) != BSP_OK) {
                    Motion_SetError();
                }
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
