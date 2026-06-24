#include "scheduler.h"
#include "app_task_config.h"

/*
 * 这里真正生成 Task_t task_list[]。
 * 修改任务表请去 app_task_config.h，不要改本文件。
 */
// 调用配置头文件中的宏，自动定义、初始化全局任务数组 task_list
APP_SCHEDULER_TASK_LIST_DEFINE();

/**
 * @brief  调度器初始化
 * @retval 无
 * @note 上电调用一次，统一初始化所有任务上次运行时间戳
 */
void Scheduler_Init(void)
{
    uint8_t i;
    // 获取当前系统毫秒时间戳
    uint32_t now = BSP_GetTickMs();

    // 遍历全部任务
    for (i = 0U; i < TASK_NUM; i++) {
        /*
         * task_list[i].last_run_ms 在配置表中一般填 0。
         * 这里统一初始化为当前时间，使系统启动后按周期稳定运行。
         * 如果希望某个任务上电立即运行，可以把它放进对应模块 Init 中做一次初始化。
         */
        // 将每个任务上次执行时间统一赋值为开机当前时刻
        // 避免上电瞬间大量任务同时扎堆触发执行，时序更平稳
        task_list[i].last_run_ms = now;
    }
}

/**
 * @brief  调度器主轮询函数，必须放在 while(1) 死循环里反复调用
 * @retval 无
 */
void Scheduler_Run(void)
{
    uint8_t i;
    // 获取本轮进入调度器的系统毫秒时间
    uint32_t now = BSP_GetTickMs();

    // 逐个遍历所有注册的任务
    for (i = 0U; i < TASK_NUM; i++) {
        // 条件1：任务函数指针为空 / 条件2：周期设置为0 → 判定为无效任务，跳过不处理
        if (task_list[i].task_func == 0 || task_list[i].period_ms == 0U) {
            continue;
        }

        // 当前时间 - 上次执行时间 ≥ 设定周期 → 满足执行条件
        // 无符号uint32_t减法天然适配定时器溢出，不用额外处理溢出逻辑
        if ((uint32_t)(now - task_list[i].last_run_ms) >= task_list[i].period_ms) {
            // 更新本次执行的时间戳，作为下一轮判断起点
            task_list[i].last_run_ms = now;
            // 执行该任务函数
            task_list[i].task_func();
        }
    }
}

/**
 * @brief  获取当前总任务数量
 * @retval 注册的任务总个数 TASK_NUM
 */
uint8_t Scheduler_GetTaskCount(void)
{
    return TASK_NUM;
}

/**
 * @brief  手动重置指定任务的计时起点
 * @param  index：任务在任务数组中的下标
 * @retval 无
 * @usage 外部需要临时重启某个任务计时时调用
 */
void Scheduler_ResetTaskTime(uint8_t index)
{
    // 下标越界保护，防止访问数组越界崩溃
    if (index >= TASK_NUM) {
        return;
    }
    // 将该任务上次运行时间刷新为当前时刻，重新开始倒计时
    task_list[index].last_run_ms = BSP_GetTickMs();
}
