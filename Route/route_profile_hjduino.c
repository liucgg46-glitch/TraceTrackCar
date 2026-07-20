#include "route_profile_hjduino.h"
#include "route_config.h"

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
                                        const Route_ActionFeedback_t *feedback,
                                        LineTrack_Output_t *out,
                                        Route_ActionRequest_t *request)
{
    if ((line == 0) || (feedback == 0) ||
        (out == 0) || (request == 0)) {
        return ROUTE_CONTROL_ERROR;
    }

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

        request->type = ROUTE_ACTION_TURN_ANGLE;
        request->angle_deg = HJDUINO_ROUTE_ENTRY_TURN_ANGLE_DEG;
        HJduinoRoute_SetState(HJD_ROUTE_ENTER_LOOP_RIGHT);
        return ROUTE_CONTROL_MOTION;

    case HJD_ROUTE_ENTER_LOOP_RIGHT:
        if (feedback->state == ROUTE_ACTION_STATE_RUNNING) {
            return ROUTE_CONTROL_MOTION;
        }

        if (feedback->state == ROUTE_ACTION_STATE_DONE) {
            /* 动作完成后清除旧循迹命令，再使用当前灰度结果重新计算。 */
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
