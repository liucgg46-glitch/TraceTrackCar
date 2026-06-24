#include "pid.h"

static float PID_Limit(float x, float min_val, float max_val)
{
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

void PID_Init(PID_t *pid, const PID_Config_t *cfg)
{
    if (pid == 0 || cfg == 0) return;

    pid->cfg = *cfg;
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->first_update = 1U;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0) return;

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->first_update = 1U;
}

void PID_SetConfig(PID_t *pid, const PID_Config_t *cfg)
{
    if (pid == 0 || cfg == 0) return;
    pid->cfg = *cfg;
    pid->integral = PID_Limit(pid->integral, pid->cfg.integral_min, pid->cfg.integral_max);
    pid->output = PID_Limit(pid->output, pid->cfg.out_min, pid->cfg.out_max);
}

void PID_SetTarget(PID_t *pid, float target)
{
    if (pid == 0) return;
    pid->target = target;
}

float PID_Update(PID_t *pid, float feedback)
{
    float p_out;
    float i_out;
    float d_out;

    if (pid == 0) return 0.0f;

    pid->feedback = feedback;
    pid->error = pid->target - pid->feedback;

    if (pid->first_update) {
        pid->derivative = 0.0f;
        pid->first_update = 0U;
    } else {
        pid->derivative = pid->error - pid->last_error;
    }

    pid->integral += pid->error;
    pid->integral = PID_Limit(pid->integral, pid->cfg.integral_min, pid->cfg.integral_max);

    p_out = pid->cfg.kp * pid->error;
    i_out = pid->cfg.ki * pid->integral;
    d_out = pid->cfg.kd * pid->derivative;

    pid->output = p_out + i_out + d_out;
    pid->output = PID_Limit(pid->output, pid->cfg.out_min, pid->cfg.out_max);

    pid->last_error = pid->error;
    return pid->output;
}

float PID_GetOutput(const PID_t *pid)
{
    if (pid == 0) return 0.0f;
    return pid->output;
}

float PID_GetError(const PID_t *pid)
{
    if (pid == 0) return 0.0f;
    return pid->error;
}
