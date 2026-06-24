#include "chassis.h"
#include "drv_motor.h"
#include "drv_encoder.h"

static PID_t s_pid_fl;
static PID_t s_pid_fr;
static PID_t s_pid_rl;
static PID_t s_pid_rr;

static Chassis_Info_t s_chassis;

static int16_t Chassis_LimitTarget(int32_t x)
{
    if (x > CHASSIS_TARGET_CPS_MAX) return CHASSIS_TARGET_CPS_MAX;
    if (x < -CHASSIS_TARGET_CPS_MAX) return -CHASSIS_TARGET_CPS_MAX;
    return (int16_t)x;
}

static int16_t Chassis_LimitOutput(float x)
{
    if (x > CHASSIS_SPEED_PID_OUT_MAX) return (int16_t)CHASSIS_SPEED_PID_OUT_MAX;
    if (x < -CHASSIS_SPEED_PID_OUT_MAX) return (int16_t)(-CHASSIS_SPEED_PID_OUT_MAX);
    return (int16_t)x;
}

void Chassis_Init(void)
{
    PID_Config_t cfg;

    cfg.kp = CHASSIS_SPEED_PID_KP;
    cfg.ki = CHASSIS_SPEED_PID_KI;
    cfg.kd = CHASSIS_SPEED_PID_KD;
    cfg.out_min = -CHASSIS_SPEED_PID_OUT_MAX;
    cfg.out_max =  CHASSIS_SPEED_PID_OUT_MAX;
    cfg.integral_min = -CHASSIS_SPEED_PID_I_MAX;
    cfg.integral_max =  CHASSIS_SPEED_PID_I_MAX;

    Motor_Init();
    Drv_Encoder_Init();

    PID_Init(&s_pid_fl, &cfg);
    PID_Init(&s_pid_fr, &cfg);
    PID_Init(&s_pid_rl, &cfg);
    PID_Init(&s_pid_rr, &cfg);

    Chassis_Stop();
}

void Chassis_SetSpeed(int16_t linear_speed_cps, int16_t turn_speed_cps)
{
    s_chassis.linear_target_cps = Chassis_LimitTarget(linear_speed_cps);
    s_chassis.turn_target_cps   = Chassis_LimitTarget(turn_speed_cps);

    /* 差速模型：left = linear - turn，right = linear + turn。 */
    s_chassis.left_target_cps  = Chassis_LimitTarget((int32_t)s_chassis.linear_target_cps - s_chassis.turn_target_cps);
    s_chassis.right_target_cps = Chassis_LimitTarget((int32_t)s_chassis.linear_target_cps + s_chassis.turn_target_cps);

    PID_SetTarget(&s_pid_fl, (float)s_chassis.left_target_cps);
    PID_SetTarget(&s_pid_rl, (float)s_chassis.left_target_cps);
    PID_SetTarget(&s_pid_fr, (float)s_chassis.right_target_cps);
    PID_SetTarget(&s_pid_rr, (float)s_chassis.right_target_cps);

    s_chassis.mode = CHASSIS_MODE_SPEED;
}

void Chassis_Stop(void)
{
    s_chassis.mode = CHASSIS_MODE_STOP;
    s_chassis.linear_target_cps = 0;
    s_chassis.turn_target_cps = 0;
    s_chassis.left_target_cps = 0;
    s_chassis.right_target_cps = 0;

    Chassis_ResetPID();
    Motor_StopAll();
}

void Chassis_ResetPID(void)
{
    PID_Reset(&s_pid_fl);
    PID_Reset(&s_pid_fr);
    PID_Reset(&s_pid_rl);
    PID_Reset(&s_pid_rr);
}

void Chassis_SetPIDConfig(const PID_Config_t *cfg)
{
    PID_SetConfig(&s_pid_fl, cfg);
    PID_SetConfig(&s_pid_fr, cfg);
    PID_SetConfig(&s_pid_rl, cfg);
    PID_SetConfig(&s_pid_rr, cfg);
}

Chassis_Mode_t Chassis_GetMode(void)
{
    return s_chassis.mode;
}

void Chassis_Update(void)
{
    if (s_chassis.mode != CHASSIS_MODE_SPEED) {
        Motor_StopAll();
        return;
    }

    s_chassis.fl_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_FL);
    s_chassis.fr_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_FR);
    s_chassis.rl_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_RL);
    s_chassis.rr_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_RR);

    s_chassis.fl_output = Chassis_LimitOutput(PID_Update(&s_pid_fl, (float)s_chassis.fl_feedback_cps));
    s_chassis.fr_output = Chassis_LimitOutput(PID_Update(&s_pid_fr, (float)s_chassis.fr_feedback_cps));
    s_chassis.rl_output = Chassis_LimitOutput(PID_Update(&s_pid_rl, (float)s_chassis.rl_feedback_cps));
    s_chassis.rr_output = Chassis_LimitOutput(PID_Update(&s_pid_rr, (float)s_chassis.rr_feedback_cps));

    Motor_SetAllPermille(s_chassis.fl_output,
                         s_chassis.fr_output,
                         s_chassis.rl_output,
                         s_chassis.rr_output);
}

BSP_Status_t Chassis_GetInfo(Chassis_Info_t *info)
{
    if (info == 0) return BSP_PARAM;
    *info = s_chassis;
    return BSP_OK;
}
