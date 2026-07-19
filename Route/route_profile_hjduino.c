#include "route_profile_hjduino.h"
#include "route_config.h"
#include "motion_action.h"

static HJduinoRoute_State_t s_state;
static uint16_t s_entry_confirm_samples;
static uint32_t s_run_start_ms;
static uint32_t s_transition_count;

static void HJduinoRoute_SetState(HJduinoRoute_State_t state)
{
    if (s_state != state) {
        s_state = state;
        s_transition_count++;
    }
}

static uint8_t HJduinoRoute_IsRightLoopEntry(const LineDetect_Result_t *line)
{
    if (line->type == LINE_TYPE_RIGHT_BRANCH) return 1U;
    if ((line->type == LINE_TYPE_CROSS) &&
        ((line->black_mask & 0xC0U) != 0U)) return 1U;
    return 0U;
}

static Route_ControlMode_t HJduinoRoute_RunLineTrack(
    const LineDetect_Result_t *line, LineTrack_Output_t *out)
{
    LineTrack_Compute(line, out);
    return (out->valid != 0U) ? ROUTE_CONTROL_LINE_TRACK
                              : ROUTE_CONTROL_STOP;
}

void HJduinoRoute_Init(void)
{
    HJduinoRoute_Reset();
}

void HJduinoRoute_Reset(void)
{
    s_state = HJD_ROUTE_NORMAL_BEFORE_LOOP;
    s_entry_confirm_samples = 0U;
    s_run_start_ms = BSP_GET_TICK();
    s_transition_count = 0U;
}

Route_ControlMode_t HJduinoRoute_Update(const LineDetect_Result_t *line,
                                        LineTrack_Output_t *out)
{
    MotionState_t motion_state;
    BSP_Status_t status;

    if ((line == 0) || (out == 0)) return ROUTE_CONTROL_ERROR;

    switch (s_state) {
    case HJD_ROUTE_NORMAL_BEFORE_LOOP:
        if (((uint32_t)(BSP_GET_TICK() - s_run_start_ms) >=
             HJDUINO_ROUTE_START_GUARD_MS) &&
            HJduinoRoute_IsRightLoopEntry(line)) {
            if (s_entry_confirm_samples < 0xFFFFU) {
                s_entry_confirm_samples++;
            }
        } else {
            s_entry_confirm_samples = 0U;
        }

        if (s_entry_confirm_samples <
            HJDUINO_ROUTE_ENTRY_CONFIRM_SAMPLES) {
            return HJduinoRoute_RunLineTrack(line, out);
        }

        status = Motion_TurnAngle(HJDUINO_ROUTE_ENTRY_TURN_ANGLE_DEG);
        if (status != BSP_OK) {
            HJduinoRoute_SetState(HJD_ROUTE_ERROR);
            return ROUTE_CONTROL_ERROR;
        }

        HJduinoRoute_SetState(HJD_ROUTE_ENTER_LOOP_RIGHT);
        return ROUTE_CONTROL_MOTION;

    case HJD_ROUTE_ENTER_LOOP_RIGHT:
        motion_state = Motion_GetState();
        if (motion_state == MOTION_RUNNING) return ROUTE_CONTROL_MOTION;

        if (motion_state == MOTION_DONE) {
            /* Do not reuse a line command saved before the angle action. */
            LineTrack_Reset();
            HJduinoRoute_SetState(HJD_ROUTE_IN_LOOP);
            return HJduinoRoute_RunLineTrack(line, out);
        }

        HJduinoRoute_SetState(HJD_ROUTE_ERROR);
        return ROUTE_CONTROL_ERROR;

    case HJD_ROUTE_IN_LOOP:
        return HJduinoRoute_RunLineTrack(line, out);

    case HJD_ROUTE_ERROR:
    default:
        HJduinoRoute_SetState(HJD_ROUTE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }
}

BSP_Status_t HJduinoRoute_GetInfo(HJduinoRoute_Info_t *info)
{
    if (info == 0) return BSP_PARAM;
    info->state = s_state;
    info->entry_confirm_samples = s_entry_confirm_samples;
    info->running_ms = (uint32_t)(BSP_GET_TICK() - s_run_start_ms);
    info->transition_count = s_transition_count;
    return BSP_OK;
}
