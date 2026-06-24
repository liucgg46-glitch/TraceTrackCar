#include "nb_wait.h"

/**
 * @brief  非阻塞延时结构体初始化
 * @param  wait: 指向 NB_Wait_t 结构体指针
 * @retval 无
 */
void NB_Wait_Init(NB_Wait_t *wait)
{
    // 空指针保护，防止传入非法地址导致程序崩溃
    if (wait == 0) {
        return;
    }

    // 设置状态为空闲
    wait->state = NB_WAIT_IDLE;
    // 起始时间清零
    wait->start_time_ms = 0U;
    // 延时时长清零
    wait->duration_ms = 0U;
}

/**
 * @brief  启动一段非阻塞延时
 * @param  wait: 结构体指针
 * @param  duration_ms: 需要延时的总毫秒数
 * @retval 无
 */
void NB_Wait_Start(NB_Wait_t *wait, uint32_t duration_ms)
{
    // 空指针保护
    if (wait == 0) {
        return;
    }

    // 记录当前系统毫秒滴答，作为延时起点
    wait->start_time_ms = BSP_GetTickMs();
    // 赋值需要等待的总时长
    wait->duration_ms = duration_ms;
    // 切换状态：延时正在运行
    wait->state = NB_WAIT_RUNNING;
}

/**
 * @brief  定时刷新判断是否超时（必须周期性调用，如放在主循环）
 * @param  wait: 结构体指针
 * @retval 无
 */
void NB_Wait_Update(NB_Wait_t *wait)
{
    // 空指针保护
    if (wait == 0) {
        return;
    }

    // 只有处于运行状态才需要判断超时，空闲/完成状态直接退出
    if (wait->state != NB_WAIT_RUNNING) {
        return;
    }

    // 当前时间 - 起始时间 = 已经过去的时间
    // 如果已耗时间 >= 设置总延时，判定超时
    if ((uint32_t)(BSP_GetTickMs() - wait->start_time_ms) >= wait->duration_ms) {
        // 状态切换为延时完成
        wait->state = NB_WAIT_DONE;
    }
}

/**
 * @brief  强制终止延时，重置为空闲状态
 * @param  wait: 结构体指针
 * @retval 无
 */
void NB_Wait_Stop(NB_Wait_t *wait)
{
    // 空指针保护
    if (wait == 0) {
        return;
    }

    wait->state = NB_WAIT_IDLE;
    wait->start_time_ms = 0U;
    wait->duration_ms = 0U;
}

/**
 * @brief  查询延时是否已经完成
 * @param  wait: 结构体指针
 * @retval 1:延时完成  0:未完成/指针异常
 * @note 内部自动调用一次Update刷新状态，无需外部手动更新
 */
uint8_t NB_Wait_IsDone(NB_Wait_t *wait)
{
    // 空指针保护
    if (wait == 0) {
        return 0U;
    }

    // 先刷新一次超时判断
    NB_Wait_Update(wait);
    // 状态为完成返回1，否则返回0
    return (wait->state == NB_WAIT_DONE) ? 1U : 0U;
}

/**
 * @brief  查询延时是否正在运行中
 * @param  wait: 结构体指针
 * @retval 1:正在运行  0:空闲/已结束/指针异常
 */
uint8_t NB_Wait_IsRunning(NB_Wait_t *wait)
{
    // 空指针保护
    if (wait == 0) {
        return 0U;
    }

    return (wait->state == NB_WAIT_RUNNING) ? 1U : 0U;
}
