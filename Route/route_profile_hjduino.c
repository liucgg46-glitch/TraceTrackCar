#include "route_profile_hjduino.h"
#include "route_config.h"
#include "line_track.h"
#include "motion_action.h"

/* 当前 HJduino 路线状态。 */
static HJduinoRoute_State_t s_hjd_state;

/* 入口特征连续出现的样本数量，用于抗单帧误判。 */
static uint16_t s_hjd_entry_confirm_samples;

/* 最近一次 Reset 时的系统毫秒时间，用于启动保护和运行时间统计。 */
static uint32_t s_hjd_run_start_ms;

/* 状态切换次数，用于串口调试判断状态机是否反复误触发。 */
static uint32_t s_hjd_transition_count;

/* 初始化函数统一调用 Reset，保证所有内部状态从确定值开始。 */
void HJduinoRoute_Init(void)
{
    HJduinoRoute_Reset();
}

/*
 * 恢复路线初始状态：
 *   - 处于入环前普通循迹；
 *   - 清空入口确认次数；
 *   - 重新记录起始时间；
 *   - 清空状态切换次数。
 */
void HJduinoRoute_Reset(void)
{
    s_hjd_state = HJD_ROUTE_NORMAL_BEFORE_LOOP;
    s_hjd_entry_confirm_samples = 0U;
    s_hjd_run_start_ms = BSP_GET_TICK();
    s_hjd_transition_count = 0U;
}

/*
 * 判断当前一帧灰度结果是否符合“环形入口”特征。
 *
 * 当前判定规则：
 *   1. line_detect 已直接识别为右分支，立即认为像入口；
 *   2. 或者识别为十字，同时最右侧两路 bit6/bit7 中至少一路为黑。
 *
 * 单独出现 CROSS、FULL_BLACK 或最右侧有黑线，都不会独立触发入口。
 */
static uint8_t HJduinoRoute_IsLoopEntry(const LineDetect_Result_t *line)
{
//	右分支：可以认为是入口
//交叉 + 右侧有线：才认为是入口
//单纯 CROSS / FULL_BLACK 不直接认为入口
//单纯 0xC0 也不直接认为入口
    if (line == 0) {
        return 0U;
    }

    if (line->type == LINE_TYPE_RIGHT_BRANCH) {
        return 1U;
    }

    if ((line->type == LINE_TYPE_CROSS) &&
        ((line->black_mask & 0xC0U) != 0U)) {
        return 1U;
    }

    return 0U;
}

/*
 * HJduino 路线状态机的周期更新函数。
 *
 * line：当前灰度线路识别结果。
 * out ：只有当返回 ROUTE_CONTROL_LINE_TRACK 时才是本周期循迹速度输出。
 *
 * 返回值非常重要：
 *   ROUTE_CONTROL_LINE_TRACK：上层应使用 out 控制底盘；
 *   ROUTE_CONTROL_MOTION：上层不得用 out 覆盖底盘，底盘由 Motion_Update() 控制；
 *   ROUTE_CONTROL_ERROR：上层应停止底盘并报告错误。
 */
Route_ControlMode_t HJduinoRoute_Update(const LineDetect_Result_t *line,
                                        LineTrack_Output_t *out)
{
    BSP_Status_t ret;
    MotionState_t motion_state;

    /* 参数非法时不继续运行状态机。 */
    if ((line == 0) || (out == 0)) {
        return ROUTE_CONTROL_ERROR;
    }

    /*
     * 每个周期先清空 out，避免在 MOTION/ERROR 状态下残留上一帧循迹命令。
     * 当本周期需要普通循迹时，后面的 LineTrack_Compute() 会重新写入这些字段。
     */
    out->linear_cps = 0;
    out->turn_cps = 0;
    out->valid = 0U;

    switch (s_hjd_state) {
    case HJD_ROUTE_NORMAL_BEFORE_LOOP:
        /*
         * 入环之前执行正常循迹。
         * 启动保护时间和连续多帧确认用于防止单帧噪声误启动右转动作。
         */
        /*
         * 第一部分：入口确认计数。
         * 只有“运行时间已超过保护期”且“本帧像入口”时才累加；
         * 任何一帧不满足条件都会把连续确认次数清零。
         */
        if (((uint32_t)(BSP_GET_TICK() - s_hjd_run_start_ms) >=
             HJDUINO_ROUTE_START_GUARD_MS) &&
            HJduinoRoute_IsLoopEntry(line)) {
            /* 饱和计数，防止 uint16_t 加到 65535 后回绕为 0。 */
            if (s_hjd_entry_confirm_samples < 0xFFFFU) {
                s_hjd_entry_confirm_samples++;
            }
        } else {
            s_hjd_entry_confirm_samples = 0U;
        }

        /*
         * 第二部分：入口确认达到阈值后，启动一次非阻塞定角转弯。
         * Motion_TurnAngle() 只负责启动动作，不会在这里等待转完。
         */
        if (s_hjd_entry_confirm_samples >=
            HJDUINO_ROUTE_ENTRY_CONFIRM_SAMPLES) {
            ret = Motion_TurnAngle(HJDUINO_ROUTE_ENTRY_TURN_ANGLE_DEG,
                                   HJDUINO_ROUTE_ENTRY_TURN_SPEED_CPS);
            if (ret == BSP_OK) {
                /* 动作成功启动，进入“等待右转完成”状态。 */
                s_hjd_state = HJD_ROUTE_ENTER_LOOP_RIGHT;
                s_hjd_transition_count++;
                return ROUTE_CONTROL_MOTION;
            }

            /* 动作未能启动（例如参数错误或 motion 忙），路线进入错误态。 */
            s_hjd_state = HJD_ROUTE_ERROR;
            s_hjd_transition_count++;
            return ROUTE_CONTROL_ERROR;
        }

        /* 尚未确认入口时，继续使用普通循迹算法。 */
        LineTrack_Compute(line, out);
        return ROUTE_CONTROL_LINE_TRACK;

    case HJD_ROUTE_ENTER_LOOP_RIGHT:
        /*
         * 定角动作执行期间，motion_action 独占底盘。
         * 路线层只能检查动作状态，不能再次写循迹命令覆盖它。
         */
        /* Motion owns the chassis until the relative IMU yaw reaches the
         * configured entry angle. Route must not overwrite its command.
         */
        motion_state = Motion_GetState();

        if (motion_state == MOTION_DONE) {
            /* 右转完成，认为已经进入环内，立即重新计算一帧循迹输出。 */
            s_hjd_state = HJD_ROUTE_IN_LOOP;
            s_hjd_transition_count++;
            LineTrack_Compute(line, out);
            return ROUTE_CONTROL_LINE_TRACK;
        }

        if (motion_state == MOTION_RUNNING) {
            /* 动作仍在执行，上层应继续让 Motion_Update() 控制底盘。 */
            return ROUTE_CONTROL_MOTION;
        }

        /* IDLE、ERROR 或其他非预期状态都按路线错误处理。 */
        s_hjd_state = HJD_ROUTE_ERROR;
        s_hjd_transition_count++;
        return ROUTE_CONTROL_ERROR;

    case HJD_ROUTE_IN_LOOP:
        /*
         * 进入环形区域后继续普通循迹。
         * 当前代码尚未实现出口识别，因此状态会一直停留在 IN_LOOP，
         * 直到外部调用 Reset 或发生错误。
         */
        /* Continue normal line following in the loop. Exit detection must be
         * added from measured track features instead of guessed timing.
         */
        LineTrack_Compute(line, out);
        return ROUTE_CONTROL_LINE_TRACK;

    case HJD_ROUTE_ERROR:
        /* 错误态保持锁定，等待外部调用 Reset。 */
        return ROUTE_CONTROL_ERROR;

    default:
        /* 状态变量出现非法值时，转入错误态，避免继续输出不可预测命令。 */
        s_hjd_state = HJD_ROUTE_ERROR;
        s_hjd_transition_count++;
        return ROUTE_CONTROL_ERROR;
    }
}

/*
 * 把当前内部状态复制给调用者，用于串口日志、OLED或上位机监控。
 */
BSP_Status_t HJduinoRoute_GetInfo(HJduinoRoute_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    info->state = s_hjd_state;
    info->entry_confirm_samples = s_hjd_entry_confirm_samples;
    info->running_ms = (uint32_t)(BSP_GET_TICK() - s_hjd_run_start_ms);
    info->transition_count = s_hjd_transition_count;
    return BSP_OK;
}
