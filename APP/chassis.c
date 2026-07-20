#include "chassis.h"
#include "control_config.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "pid.h"

static Chassis_Info_t s_chassis;
static PID_t s_speed_pid[WHEEL_COUNT];
static uint8_t s_speed_pid_initialized;
static uint32_t s_no_feedback_start_ms[WHEEL_COUNT];
static uint32_t s_wrong_direction_start_ms[WHEEL_COUNT];
static uint8_t s_no_feedback_active[WHEEL_COUNT];
static uint8_t s_wrong_direction_active[WHEEL_COUNT];
static uint32_t s_last_command_ms;

static int32_t Chassis_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void Chassis_ResetFaultMonitors(void)
{
    Wheel_Id_t wheel;

    for (wheel = WHEEL_FL; wheel < WHEEL_COUNT;
         wheel = (Wheel_Id_t)(wheel + 1)) {
        s_no_feedback_start_ms[wheel] = 0U;
        s_wrong_direction_start_ms[wheel] = 0U;
        s_no_feedback_active[wheel] = 0U;
        s_wrong_direction_active[wheel] = 0U;
    }
}

static void Chassis_ResetSpeedControllers(void)
{
    Wheel_Id_t wheel;

    if (s_speed_pid_initialized == 0U) {
        return;
    }

    for (wheel = WHEEL_FL; wheel < WHEEL_COUNT;
         wheel = (Wheel_Id_t)(wheel + 1)) {
        PID_Reset(&s_speed_pid[wheel]);
    }
}

static void Chassis_StopOutput(void)
{
    s_chassis.mode = CHASSIS_MODE_STOP;
    s_chassis.linear_target_cps = 0;
    s_chassis.turn_target_cps = 0;
    s_chassis.left_target_cps = 0;
    s_chassis.right_target_cps = 0;
    s_chassis.left_applied_target_cps = 0;
    s_chassis.right_applied_target_cps = 0;
    s_chassis.fl_output = 0;
    s_chassis.fr_output = 0;
    s_chassis.rl_output = 0;
    s_chassis.rr_output = 0;
    s_chassis.fl_feedforward = 0;
    s_chassis.fr_feedforward = 0;
    s_chassis.rl_feedforward = 0;
    s_chassis.rr_feedforward = 0;
    s_chassis.fl_pid_correction = 0;
    s_chassis.fr_pid_correction = 0;
    s_chassis.rl_pid_correction = 0;
    s_chassis.rr_pid_correction = 0;
    Chassis_ResetFaultMonitors();
    Chassis_ResetSpeedControllers();
    Motor_StopAll();
}

static void Chassis_LatchFault(Chassis_Fault_t fault,
                               uint8_t wheel_mask,
                               uint32_t now_ms)
{
    Chassis_StopOutput();
    s_chassis.owner = CHASSIS_OWNER_NONE;
    s_chassis.fault = fault;
    s_chassis.fault_wheel_mask = wheel_mask;
    s_chassis.fault_time_ms = now_ms;
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
          CONTROL_CHASSIS_FEEDFORWARD_FULL_SPEED_CPS;
    if (pwm > CONTROL_CHASSIS_PWM_MAX_PERMILLE) {
        return CONTROL_CHASSIS_PWM_MAX_PERMILLE;
    }
    if (pwm < -CONTROL_CHASSIS_PWM_MAX_PERMILLE) {
        return (int16_t)(-CONTROL_CHASSIS_PWM_MAX_PERMILLE);
    }
    return (int16_t)pwm;
}

static int16_t Chassis_LimitPwmFloat(float pwm)
{
    if (pwm > (float)CONTROL_CHASSIS_PWM_MAX_PERMILLE) {
        return CONTROL_CHASSIS_PWM_MAX_PERMILLE;
    }
    if (pwm < (float)(-CONTROL_CHASSIS_PWM_MAX_PERMILLE)) {
        return (int16_t)(-CONTROL_CHASSIS_PWM_MAX_PERMILLE);
    }
    return (int16_t)pwm;
}

static uint8_t Chassis_TargetDirectionChanged(int16_t old_target,
                                              int16_t new_target)
{
    return ((((old_target > 0) && (new_target <= 0)) ||
             ((old_target < 0) && (new_target >= 0)))) ? 1U : 0U;
}

static int16_t Chassis_SlewTarget(int16_t current, int16_t target)
{
    int32_t delta = (int32_t)target - current;

    if (delta > CONTROL_CHASSIS_TARGET_SLEW_STEP_CPS) {
        return (int16_t)(current + CONTROL_CHASSIS_TARGET_SLEW_STEP_CPS);
    }
    if (delta < -CONTROL_CHASSIS_TARGET_SLEW_STEP_CPS) {
        return (int16_t)(current - CONTROL_CHASSIS_TARGET_SLEW_STEP_CPS);
    }
    return target;
}

static uint8_t Chassis_WheelFaultMask(Wheel_Id_t wheel)
{
    switch (wheel) {
        case WHEEL_FL: return CHASSIS_FAULT_WHEEL_FL;
        case WHEEL_FR: return CHASSIS_FAULT_WHEEL_FR;
        case WHEEL_RL: return CHASSIS_FAULT_WHEEL_RL;
        case WHEEL_RR: return CHASSIS_FAULT_WHEEL_RR;
        default: return 0U;
    }
}

static Chassis_Fault_t Chassis_CheckWheelFault(Wheel_Id_t wheel,
                                                int16_t target_cps,
                                                int32_t feedback_cps,
                                                int16_t output,
                                                uint32_t now_ms)
{
#if ((CONTROL_CHASSIS_SPEED_LOOP_ENABLE != 0U) && \
     (CONTROL_CHASSIS_ENCODER_FAULT_ENABLE != 0U))
    uint8_t wrong_direction;

    if ((Drv_Encoder_IsWheelEnabled(wheel) == 0U) ||
        (Chassis_Abs32(target_cps) < CONTROL_CHASSIS_FAULT_MIN_TARGET_CPS) ||
        (Chassis_Abs32(output) < CONTROL_CHASSIS_FAULT_MIN_OUTPUT_PERMILLE)) {
        s_no_feedback_active[wheel] = 0U;
        s_wrong_direction_active[wheel] = 0U;
        return CHASSIS_FAULT_NONE;
    }

    if (Chassis_Abs32(feedback_cps) <=
        CONTROL_CHASSIS_FAULT_MAX_FEEDBACK_CPS) {
        if (s_no_feedback_active[wheel] == 0U) {
            s_no_feedback_active[wheel] = 1U;
            s_no_feedback_start_ms[wheel] = now_ms;
        } else if ((uint32_t)(now_ms - s_no_feedback_start_ms[wheel]) >=
                   CONTROL_CHASSIS_NO_FEEDBACK_TIMEOUT_MS) {
            return CHASSIS_FAULT_ENCODER_NO_FEEDBACK;
        }
    } else {
        s_no_feedback_active[wheel] = 0U;
    }

    wrong_direction = 0U;
    if ((Chassis_Abs32(feedback_cps) >=
         CONTROL_CHASSIS_DIRECTION_MIN_FEEDBACK_CPS) &&
        (((target_cps > 0) && (output > 0) && (feedback_cps < 0)) ||
         ((target_cps < 0) && (output < 0) && (feedback_cps > 0)))) {
        wrong_direction = 1U;
    }

    if (wrong_direction != 0U) {
        if (s_wrong_direction_active[wheel] == 0U) {
            s_wrong_direction_active[wheel] = 1U;
            s_wrong_direction_start_ms[wheel] = now_ms;
        } else if ((uint32_t)(now_ms - s_wrong_direction_start_ms[wheel]) >=
                   CONTROL_CHASSIS_DIRECTION_TIMEOUT_MS) {
            return CHASSIS_FAULT_ENCODER_DIRECTION;
        }
    } else {
        s_wrong_direction_active[wheel] = 0U;
    }
#else
    (void)wheel;
    (void)target_cps;
    (void)feedback_cps;
    (void)output;
    (void)now_ms;
#endif

    return CHASSIS_FAULT_NONE;
}

static uint8_t Chassis_CheckFaults(uint32_t now_ms)
{
    static const Wheel_Id_t wheels[WHEEL_COUNT] = {
        WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR
    };
    int16_t targets[WHEEL_COUNT];
    int32_t feedbacks[WHEEL_COUNT];
    int16_t outputs[WHEEL_COUNT];
    Chassis_Fault_t fault;
    Chassis_Fault_t first_fault = CHASSIS_FAULT_NONE;
    uint8_t fault_wheel_mask = 0U;
    Wheel_Id_t wheel;

    targets[WHEEL_FL] = s_chassis.left_applied_target_cps;
    targets[WHEEL_FR] = s_chassis.right_applied_target_cps;
    targets[WHEEL_RL] = s_chassis.left_applied_target_cps;
    targets[WHEEL_RR] = s_chassis.right_applied_target_cps;
    feedbacks[WHEEL_FL] = s_chassis.fl_feedback_cps;
    feedbacks[WHEEL_FR] = s_chassis.fr_feedback_cps;
    feedbacks[WHEEL_RL] = s_chassis.rl_feedback_cps;
    feedbacks[WHEEL_RR] = s_chassis.rr_feedback_cps;
    outputs[WHEEL_FL] = s_chassis.fl_output;
    outputs[WHEEL_FR] = s_chassis.fr_output;
    outputs[WHEEL_RL] = s_chassis.rl_output;
    outputs[WHEEL_RR] = s_chassis.rr_output;

    for (wheel = WHEEL_FL; wheel < WHEEL_COUNT;
         wheel = (Wheel_Id_t)(wheel + 1)) {
        fault = Chassis_CheckWheelFault(wheels[wheel],
                                        targets[wheel],
                                        feedbacks[wheel],
                                        outputs[wheel],
                                        now_ms);
        if (fault != CHASSIS_FAULT_NONE) {
            if (first_fault == CHASSIS_FAULT_NONE) {
                first_fault = fault;
            }
            fault_wheel_mask |= Chassis_WheelFaultMask(wheels[wheel]);
        }
    }

    if (first_fault != CHASSIS_FAULT_NONE) {
        Chassis_LatchFault(first_fault, fault_wheel_mask, now_ms);
        return 1U;
    }

    return 0U;
}

static void Chassis_UpdateWheel(Wheel_Id_t wheel,
                                int16_t target_cps,
                                int32_t feedback_cps,
                                int16_t *feedforward,
                                int16_t *pid_correction,
                                int16_t *output)
{
    int16_t ff;

    if ((feedforward == 0) || (pid_correction == 0) || (output == 0)) {
        return;
    }

    if ((Drv_Encoder_IsWheelEnabled(wheel) == 0U) || (target_cps == 0)) {
        PID_Reset(&s_speed_pid[wheel]);
        *feedforward = 0;
        *pid_correction = 0;
        *output = 0;
        return;
    }

    ff = Chassis_TargetToPwm(target_cps);
    *feedforward = ff;

#if (CONTROL_CHASSIS_SPEED_LOOP_ENABLE != 0U)
    PID_SetTarget(&s_speed_pid[wheel], (float)target_cps);
    *output = Chassis_LimitPwmFloat(
        PID_UpdateWithFeedforward(&s_speed_pid[wheel],
                                  (float)feedback_cps,
                                  (float)ff));
    *pid_correction = (int16_t)(*output - ff);
#else
    (void)feedback_cps;
    *pid_correction = 0;
    *output = ff;
#endif
}

void Chassis_Init(void)
{
    Wheel_Id_t wheel;
    PID_Config_t pid_cfg;

    /* Motor 已由 Driver_Init() 统一初始化，APP 层只初始化控制状态。 */
    pid_cfg.kp = CONTROL_CHASSIS_SPEED_KP;
    pid_cfg.ki = CONTROL_CHASSIS_SPEED_KI;
    pid_cfg.kd = CONTROL_CHASSIS_SPEED_KD;
    pid_cfg.out_min = (float)(-CONTROL_CHASSIS_PWM_MAX_PERMILLE);
    pid_cfg.out_max = (float)CONTROL_CHASSIS_PWM_MAX_PERMILLE;
    pid_cfg.integral_min = -CONTROL_CHASSIS_SPEED_INTEGRAL_LIMIT;
    pid_cfg.integral_max = CONTROL_CHASSIS_SPEED_INTEGRAL_LIMIT;

    for (wheel = WHEEL_FL; wheel < WHEEL_COUNT;
         wheel = (Wheel_Id_t)(wheel + 1)) {
        PID_Init(&s_speed_pid[wheel], &pid_cfg);
    }
    s_speed_pid_initialized = 1U;

    s_chassis.owner = CHASSIS_OWNER_NONE;
    s_chassis.speed_loop_enabled = CONTROL_CHASSIS_SPEED_LOOP_ENABLE;
    s_chassis.fault = CHASSIS_FAULT_NONE;
    s_chassis.fault_wheel_mask = 0U;
    s_chassis.fault_time_ms = 0U;
    s_chassis.command_age_ms = 0U;
    s_last_command_ms = 0U;
    Chassis_StopOutput();
}

BSP_Status_t Chassis_AcquireControl(Chassis_ControlOwner_t owner)
{
    if (owner == CHASSIS_OWNER_NONE) {
        return BSP_PARAM;
    }
    if (s_chassis.fault != CHASSIS_FAULT_NONE) {
        return BSP_ERROR;
    }
    if ((s_chassis.owner != CHASSIS_OWNER_NONE) &&
        (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    if (s_chassis.owner == CHASSIS_OWNER_NONE) {
        s_last_command_ms = BSP_GET_TICK();
        s_chassis.command_age_ms = 0U;
    }
    s_chassis.owner = owner;
    return BSP_OK;
}

BSP_Status_t Chassis_SetSpeed(Chassis_ControlOwner_t owner,
                              int16_t linear_speed_cps,
                              int16_t turn_speed_cps)
{
    int16_t new_left_target;
    int16_t new_right_target;

    if (s_chassis.fault != CHASSIS_FAULT_NONE) {
        return BSP_ERROR;
    }
    if ((owner == CHASSIS_OWNER_NONE) || (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    s_chassis.linear_target_cps = Chassis_LimitTarget(linear_speed_cps);
    s_chassis.turn_target_cps = Chassis_LimitTarget(turn_speed_cps);

    /* 差速模型：left = linear - turn，right = linear + turn。 */
    new_left_target = Chassis_LimitTarget(
        (int32_t)s_chassis.linear_target_cps - s_chassis.turn_target_cps);
    new_right_target = Chassis_LimitTarget(
        (int32_t)s_chassis.linear_target_cps + s_chassis.turn_target_cps);

    s_chassis.left_target_cps = new_left_target;
    s_chassis.right_target_cps = new_right_target;

    s_chassis.mode = CHASSIS_MODE_SPEED;
    s_last_command_ms = BSP_GET_TICK();
    s_chassis.command_age_ms = 0U;
    return BSP_OK;
}

BSP_Status_t Chassis_Stop(Chassis_ControlOwner_t owner)
{
    if ((owner == CHASSIS_OWNER_NONE) || (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    Chassis_StopOutput();
    s_last_command_ms = BSP_GET_TICK();
    s_chassis.command_age_ms = 0U;
    return BSP_OK;
}

BSP_Status_t Chassis_ReleaseControl(Chassis_ControlOwner_t owner)
{
    if ((owner == CHASSIS_OWNER_NONE) || (s_chassis.owner != owner)) {
        return BSP_BUSY;
    }

    Chassis_StopOutput();
    s_chassis.owner = CHASSIS_OWNER_NONE;
    s_last_command_ms = 0U;
    s_chassis.command_age_ms = 0U;
    return BSP_OK;
}

void Chassis_EmergencyStop(void)
{
    Chassis_StopOutput();
    s_chassis.owner = CHASSIS_OWNER_NONE;
    s_last_command_ms = 0U;
    s_chassis.command_age_ms = 0U;
}

BSP_Status_t Chassis_ClearFault(void)
{
    Wheel_Id_t wheel;

    if (s_chassis.fault == CHASSIS_FAULT_NONE) {
        return BSP_OK;
    }
    if (s_chassis.mode != CHASSIS_MODE_STOP) {
        return BSP_BUSY;
    }

    for (wheel = WHEEL_FL; wheel < WHEEL_COUNT;
         wheel = (Wheel_Id_t)(wheel + 1)) {
        if ((Drv_Encoder_IsWheelEnabled(wheel) != 0U) &&
            (Chassis_Abs32(Drv_Encoder_GetWheelRawSpeedCps(wheel)) >
             CONTROL_CHASSIS_FAULT_CLEAR_MAX_CPS)) {
            return BSP_BUSY;
        }
    }

    s_chassis.fault = CHASSIS_FAULT_NONE;
    s_chassis.fault_wheel_mask = 0U;
    s_chassis.fault_time_ms = 0U;
    s_last_command_ms = 0U;
    s_chassis.command_age_ms = 0U;
    Chassis_ResetFaultMonitors();
    Chassis_ResetSpeedControllers();
    return BSP_OK;
}

Chassis_Fault_t Chassis_GetFault(void)
{
    return s_chassis.fault;
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
    uint32_t now_ms;
    int16_t new_left_applied;
    int16_t new_right_applied;

    if (s_chassis.mode != CHASSIS_MODE_SPEED) {
        Motor_StopAll();
        return;
    }

    now_ms = BSP_GET_TICK();
    s_chassis.command_age_ms = (uint32_t)(now_ms - s_last_command_ms);
#if (CONTROL_CHASSIS_COMMAND_WATCHDOG_ENABLE != 0U)
    if (s_chassis.command_age_ms >=
        CONTROL_CHASSIS_COMMAND_TIMEOUT_MS) {
        Chassis_LatchFault(CHASSIS_FAULT_COMMAND_TIMEOUT, 0U, now_ms);
        return;
    }
#endif

    new_left_applied = Chassis_SlewTarget(
        s_chassis.left_applied_target_cps,
        s_chassis.left_target_cps);
    new_right_applied = Chassis_SlewTarget(
        s_chassis.right_applied_target_cps,
        s_chassis.right_target_cps);

    if (Chassis_TargetDirectionChanged(s_chassis.left_applied_target_cps,
                                       new_left_applied) != 0U) {
        PID_Reset(&s_speed_pid[WHEEL_FL]);
        PID_Reset(&s_speed_pid[WHEEL_RL]);
    }
    if (Chassis_TargetDirectionChanged(s_chassis.right_applied_target_cps,
                                       new_right_applied) != 0U) {
        PID_Reset(&s_speed_pid[WHEEL_FR]);
        PID_Reset(&s_speed_pid[WHEEL_RR]);
    }
    s_chassis.left_applied_target_cps = new_left_applied;
    s_chassis.right_applied_target_cps = new_right_applied;

    s_chassis.fl_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_FL);
    s_chassis.fr_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_FR);
#if (VEHICLE_REAR_DRIVE_ENABLE != 0U)
    s_chassis.rl_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_RL);
    s_chassis.rr_feedback_cps = Drv_Encoder_GetWheelSpeedCps(WHEEL_RR);
#else
    s_chassis.rl_feedback_cps = 0;
    s_chassis.rr_feedback_cps = 0;
#endif

    Chassis_UpdateWheel(WHEEL_FL,
                        s_chassis.left_applied_target_cps,
                        s_chassis.fl_feedback_cps,
                        &s_chassis.fl_feedforward,
                        &s_chassis.fl_pid_correction,
                        &s_chassis.fl_output);
    Chassis_UpdateWheel(WHEEL_FR,
                        s_chassis.right_applied_target_cps,
                        s_chassis.fr_feedback_cps,
                        &s_chassis.fr_feedforward,
                        &s_chassis.fr_pid_correction,
                        &s_chassis.fr_output);
#if (VEHICLE_REAR_DRIVE_ENABLE != 0U)
    Chassis_UpdateWheel(WHEEL_RL,
                        s_chassis.left_applied_target_cps,
                        s_chassis.rl_feedback_cps,
                        &s_chassis.rl_feedforward,
                        &s_chassis.rl_pid_correction,
                        &s_chassis.rl_output);
    Chassis_UpdateWheel(WHEEL_RR,
                        s_chassis.right_applied_target_cps,
                        s_chassis.rr_feedback_cps,
                        &s_chassis.rr_feedforward,
                        &s_chassis.rr_pid_correction,
                        &s_chassis.rr_output);
#else
    s_chassis.rl_feedforward = 0;
    s_chassis.rr_feedforward = 0;
    s_chassis.rl_pid_correction = 0;
    s_chassis.rr_pid_correction = 0;
    s_chassis.rl_output = 0;
    s_chassis.rr_output = 0;
#endif

    if (Chassis_CheckFaults(now_ms) != 0U) {
        return;
    }

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
