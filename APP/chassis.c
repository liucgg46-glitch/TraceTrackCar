#include "chassis.h"
#include "control_config.h"
#include "drv_motor.h"
#include "drv_encoder.h"

static Chassis_Info_t s_chassis;

static void Chassis_StopOutput(void)
{
    s_chassis.mode = CHASSIS_MODE_STOP;
    s_chassis.linear_target_cps = 0;
    s_chassis.turn_target_cps = 0;
    s_chassis.left_target_cps = 0;
    s_chassis.right_target_cps = 0;
    s_chassis.fl_output = 0;
    s_chassis.fr_output = 0;
    s_chassis.rl_output = 0;
    s_chassis.rr_output = 0;
    Motor_StopAll();
}

static int16_t Chassis_LimitTarget(int32_t x)
{
    if (x > CONTROL_CHASSIS_TARGET_MAX_CPS) return CONTROL_CHASSIS_TARGET_MAX_CPS;
    if (x < -CONTROL_CHASSIS_TARGET_MAX_CPS) return -CONTROL_CHASSIS_TARGET_MAX_CPS;
    return (int16_t)x;
}

static int16_t Chassis_TargetToPwm(int16_t target_cps)
{
    int32_t pwm;

    pwm = ((int32_t)target_cps * CONTROL_CHASSIS_PWM_MAX_PERMILLE) /
          CONTROL_CHASSIS_TARGET_MAX_CPS;
    if (pwm > CONTROL_CHASSIS_PWM_MAX_PERMILLE) {
        return CONTROL_CHASSIS_PWM_MAX_PERMILLE;
    }
    if (pwm < -CONTROL_CHASSIS_PWM_MAX_PERMILLE) {
        return (int16_t)(-CONTROL_CHASSIS_PWM_MAX_PERMILLE);
    }
    return (int16_t)pwm;
}

void Chassis_Init(void)
{
    /* Motor 已由 Driver_Init() 统一初始化，APP 层只初始化控制状态。 */
    s_chassis.owner = CHASSIS_OWNER_NONE;
    Chassis_StopOutput();
}

BSP_Status_t Chassis_AcquireControl(Chassis_ControlOwner_t owner)
{
    if (owner == CHASSIS_OWNER_NONE) {
        return BSP_PARAM;
    }
    if ((s_chassis.owner != CHASSIS_OWNER_NONE) &&
        (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    s_chassis.owner = owner;
    return BSP_OK;
}

BSP_Status_t Chassis_SetSpeed(Chassis_ControlOwner_t owner,
                              int16_t linear_speed_cps,
                              int16_t turn_speed_cps)
{
    if ((owner == CHASSIS_OWNER_NONE) || (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    s_chassis.linear_target_cps = Chassis_LimitTarget(linear_speed_cps);
    s_chassis.turn_target_cps   = Chassis_LimitTarget(turn_speed_cps);

    /* 差速模型：left = linear - turn，right = linear + turn。 */
    s_chassis.left_target_cps  = Chassis_LimitTarget((int32_t)s_chassis.linear_target_cps - s_chassis.turn_target_cps);
    s_chassis.right_target_cps = Chassis_LimitTarget((int32_t)s_chassis.linear_target_cps + s_chassis.turn_target_cps);

    s_chassis.mode = CHASSIS_MODE_SPEED;
    return BSP_OK;
}

BSP_Status_t Chassis_Stop(Chassis_ControlOwner_t owner)
{
    if ((owner == CHASSIS_OWNER_NONE) || (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    Chassis_StopOutput();
    return BSP_OK;
}

BSP_Status_t Chassis_ReleaseControl(Chassis_ControlOwner_t owner)
{
    if ((owner == CHASSIS_OWNER_NONE) || (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    Chassis_StopOutput();
    s_chassis.owner = CHASSIS_OWNER_NONE;
    return BSP_OK;
}

void Chassis_EmergencyStop(void)
{
    Chassis_StopOutput();
    s_chassis.owner = CHASSIS_OWNER_NONE;
}

Chassis_Mode_t Chassis_GetMode(void)
{
    return s_chassis.mode;
}

Chassis_ControlOwner_t Chassis_GetOwner(void)
{
    return s_chassis.owner;
}

void Chassis_Update(void)
{
    if (s_chassis.mode != CHASSIS_MODE_SPEED) {
        Motor_StopAll();
        return;
    }

    s_chassis.fl_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_FL);
    s_chassis.fr_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_FR);
#if (VEHICLE_REAR_DRIVE_ENABLE != 0U)
    s_chassis.rl_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_RL);
    s_chassis.rr_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_RR);
#else
    s_chassis.rl_feedback_cps = 0;
    s_chassis.rr_feedback_cps = 0;
#endif

    s_chassis.fl_output = Chassis_TargetToPwm(s_chassis.left_target_cps);
    s_chassis.fr_output = Chassis_TargetToPwm(s_chassis.right_target_cps);
#if (VEHICLE_REAR_DRIVE_ENABLE != 0U)
    s_chassis.rl_output = Chassis_TargetToPwm(s_chassis.left_target_cps);
    s_chassis.rr_output = Chassis_TargetToPwm(s_chassis.right_target_cps);
#else
    s_chassis.rl_output = 0;
    s_chassis.rr_output = 0;
#endif

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
