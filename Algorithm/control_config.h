#ifndef __CONTROL_CONFIG_H
#define __CONTROL_CONFIG_H

/*
 * 车辆运动控制参数。
 *
 * 调整底盘速度、循迹或动作库参数时只修改本文件。下面各组参数属于
 * 不同控制环，输入和输出单位不同，因此必须分别调节。
 */

/* 底盘PWM前馈和车轮速度PI闭环。设为0可退回原固定比例开环输出。 */
#ifndef CONTROL_CHASSIS_SPEED_LOOP_ENABLE
#define CONTROL_CHASSIS_SPEED_LOOP_ENABLE          1U
#endif
#define CONTROL_CHASSIS_PWM_MAX_PERMILLE          800     /* 最大输出 ±800‰（80%占空比） */
#define CONTROL_CHASSIS_TARGET_MAX_CPS            5000    /* 上层左右轮目标的安全限幅 */
#define CONTROL_CHASSIS_FEEDFORWARD_FULL_SPEED_CPS 5000   /* 前馈标定：该速度对应最大PWM */

#if ((CONTROL_CHASSIS_SPEED_LOOP_ENABLE != 0U) && \
     (CONTROL_CHASSIS_SPEED_LOOP_ENABLE != 1U))
#error "CONTROL_CHASSIS_SPEED_LOOP_ENABLE must be 0U or 1U"
#endif
#if (CONTROL_CHASSIS_FEEDFORWARD_FULL_SPEED_CPS <= 0)
#error "CONTROL_CHASSIS_FEEDFORWARD_FULL_SPEED_CPS must be positive"
#endif
#if ((CONTROL_CHASSIS_PWM_MAX_PERMILLE <= 0) || \
     (CONTROL_CHASSIS_PWM_MAX_PERMILLE > 1000) || \
     (CONTROL_CHASSIS_TARGET_MAX_CPS <= 0))
#error "chassis output limits are invalid"
#endif

/*
 * 速度环每10 ms更新一次。输出 = 固定比例PWM前馈 + PI修正。
 * KI按“每次调用”累计，修改Chassis_Update周期后必须重新调节。
 */
#define CONTROL_CHASSIS_SPEED_KP                   0.08f
#define CONTROL_CHASSIS_SPEED_KI                   0.002f
#define CONTROL_CHASSIS_SPEED_KD                   0.0f
#define CONTROL_CHASSIS_SPEED_INTEGRAL_LIMIT       20000.0f

/* 每10 ms最多改变的左右轮控制目标，限制命令阶跃带来的机械冲击。 */
#define CONTROL_CHASSIS_TARGET_SLEW_STEP_CPS       200

/*
 * 编码器安全保护仅在速度闭环开启时生效。
 * 高于监控目标和输出后，反馈过低或方向持续相反会锁存故障并停车。
 */
#ifndef CONTROL_CHASSIS_ENCODER_FAULT_ENABLE
#define CONTROL_CHASSIS_ENCODER_FAULT_ENABLE       1U
#endif
#define CONTROL_CHASSIS_FAULT_MIN_TARGET_CPS       800
#define CONTROL_CHASSIS_FAULT_MIN_OUTPUT_PERMILLE  160
#define CONTROL_CHASSIS_FAULT_MAX_FEEDBACK_CPS     200
#define CONTROL_CHASSIS_NO_FEEDBACK_TIMEOUT_MS     500U
#define CONTROL_CHASSIS_DIRECTION_MIN_FEEDBACK_CPS 300
#define CONTROL_CHASSIS_DIRECTION_TIMEOUT_MS       500U
#define CONTROL_CHASSIS_FAULT_CLEAR_MAX_CPS        100

/* 上层速度命令租约；命令源停止刷新而底盘任务仍运行时锁存故障并停车。 */
#ifndef CONTROL_CHASSIS_COMMAND_WATCHDOG_ENABLE
#define CONTROL_CHASSIS_COMMAND_WATCHDOG_ENABLE    1U
#endif
#define CONTROL_CHASSIS_COMMAND_TIMEOUT_MS         200U

#if (CONTROL_CHASSIS_TARGET_SLEW_STEP_CPS <= 0)
#error "CONTROL_CHASSIS_TARGET_SLEW_STEP_CPS must be positive"
#endif
#if ((CONTROL_CHASSIS_ENCODER_FAULT_ENABLE != 0U) && \
     (CONTROL_CHASSIS_ENCODER_FAULT_ENABLE != 1U))
#error "CONTROL_CHASSIS_ENCODER_FAULT_ENABLE must be 0U or 1U"
#endif
#if ((CONTROL_CHASSIS_COMMAND_WATCHDOG_ENABLE != 0U) && \
     (CONTROL_CHASSIS_COMMAND_WATCHDOG_ENABLE != 1U))
#error "CONTROL_CHASSIS_COMMAND_WATCHDOG_ENABLE must be 0U or 1U"
#endif
#if (CONTROL_CHASSIS_COMMAND_TIMEOUT_MS == 0U)
#error "CONTROL_CHASSIS_COMMAND_TIMEOUT_MS must be positive"
#endif
#if ((CONTROL_CHASSIS_FAULT_MIN_TARGET_CPS <= 0) || \
     (CONTROL_CHASSIS_FAULT_MIN_OUTPUT_PERMILLE <= 0) || \
     (CONTROL_CHASSIS_FAULT_MAX_FEEDBACK_CPS < 0) || \
     (CONTROL_CHASSIS_DIRECTION_MIN_FEEDBACK_CPS <= 0) || \
     (CONTROL_CHASSIS_FAULT_CLEAR_MAX_CPS < 0) || \
     (CONTROL_CHASSIS_NO_FEEDBACK_TIMEOUT_MS == 0U) || \
     (CONTROL_CHASSIS_DIRECTION_TIMEOUT_MS == 0U))
#error "chassis encoder fault thresholds must be valid"
#endif
#if ((CONTROL_CHASSIS_FAULT_MIN_TARGET_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS) || \
     (CONTROL_CHASSIS_FAULT_MIN_OUTPUT_PERMILLE > CONTROL_CHASSIS_PWM_MAX_PERMILLE))
#error "chassis encoder fault monitor threshold exceeds chassis limit"
#endif

/*
 * 基础灰度循迹参数。
 *
 * 正常状态使用P/PD循迹和按误差减速；丢线后先消抖确认，再沿最后线路
 * 方向开始分阶段扫描。扫描方向交替，持续时间和转向量逐步增加。
 */
#define CONTROL_LINE_DIRECTION_REVERSE            1U      /* 传感器旋转180度后设1，统一反转循迹左右方向 */

#if ((CONTROL_LINE_DIRECTION_REVERSE != 0U) && \
     (CONTROL_LINE_DIRECTION_REVERSE != 1U))
#error "CONTROL_LINE_DIRECTION_REVERSE must be 0U or 1U"
#endif

#define CONTROL_LINE_BASE_SPEED_CPS               2000    /* 中线正常循迹速度 */
#define CONTROL_LINE_CROSS_SPEED_CPS              1500    /* 十字/全黑区域低速直行 */
#define CONTROL_LINE_MIN_TRACK_SPEED_CPS          1500    /* 大偏差时最低直行速度 */
#define CONTROL_LINE_TURN_MAX_CPS                 800     /* 最大转向量；实际还会限制到不让内侧轮反转 */

#define CONTROL_LINE_KP                           0.35f    /* 比例增大：转弯更积极；过大易摆动 */
#define CONTROL_LINE_KD                           0.04f    /* 微分增大：回正更快、抑制过冲；过大易抖动 */
#define CONTROL_LINE_ERROR_DEADBAND               80      /* 误差死区：±80 内按 0 处理 */
#define CONTROL_LINE_SPEED_FULL_ERROR             200     /* |误差|≤200 保持全速 */
#define CONTROL_LINE_SPEED_MIN_ERROR              1500    /* |误差|≥1500 降到最低速度 */

#define CONTROL_LINE_LOST_CONFIRM_SAMPLES         2U      /* 连续丢线达到该帧数才进入搜索 */
#define CONTROL_LINE_REACQUIRE_CONFIRM_SAMPLES    3U      /* 搜索时连续看到线达到该帧数才恢复循迹 */
#define CONTROL_LINE_SEARCH_TURN_CPS              1000    /* 第一阶段原地找线转向量 */
#define CONTROL_LINE_SEARCH_TURN_STEP_CPS         200     /* 每阶段增加的找线转向量 */
#define CONTROL_LINE_SEARCH_TURN_MAX_CPS          1800    /* 找线转向量上限 */
#define CONTROL_LINE_SEARCH_INITIAL_PHASE_MS      300U    /* 第一阶段沿最后线路方向扫描时间 */
#define CONTROL_LINE_SEARCH_PHASE_STEP_MS         200U    /* 每阶段增加的扫描时间 */
#define CONTROL_LINE_SEARCH_PHASE_MAX_MS          900U    /* 单阶段扫描时间上限 */
#define CONTROL_LINE_SEARCH_TIMEOUT_MS            10000U  /* 从首次丢线开始计时，超时后停车 */

/*
 * 编译期检查循迹参数之间的基本安全关系，避免错误配置进入实车测试。
 * KP、KD为浮点参数，仍需通过诊断页面和路测确认。
 */
#if ((CONTROL_LINE_BASE_SPEED_CPS < 0) || \
     (CONTROL_LINE_BASE_SPEED_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS) || \
     (CONTROL_LINE_CROSS_SPEED_CPS < 0) || \
     (CONTROL_LINE_CROSS_SPEED_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS) || \
     (CONTROL_LINE_MIN_TRACK_SPEED_CPS < 0) || \
     (CONTROL_LINE_MIN_TRACK_SPEED_CPS > CONTROL_LINE_BASE_SPEED_CPS) || \
     (CONTROL_LINE_TURN_MAX_CPS < 0) || \
     (CONTROL_LINE_TURN_MAX_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS))
#error "line speed parameters are invalid"
#endif

#if ((CONTROL_LINE_ERROR_DEADBAND < 0) || \
     (CONTROL_LINE_SPEED_FULL_ERROR < 0) || \
     (CONTROL_LINE_SPEED_MIN_ERROR <= CONTROL_LINE_SPEED_FULL_ERROR))
#error "line error thresholds are invalid"
#endif

#if ((CONTROL_LINE_LOST_CONFIRM_SAMPLES == 0U) || \
     (CONTROL_LINE_REACQUIRE_CONFIRM_SAMPLES == 0U) || \
     (CONTROL_LINE_SEARCH_TURN_CPS <= 0) || \
     (CONTROL_LINE_SEARCH_TURN_STEP_CPS < 0) || \
     (CONTROL_LINE_SEARCH_TURN_MAX_CPS < CONTROL_LINE_SEARCH_TURN_CPS) || \
     (CONTROL_LINE_SEARCH_TURN_MAX_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS) || \
     (CONTROL_LINE_SEARCH_INITIAL_PHASE_MS == 0U) || \
     (CONTROL_LINE_SEARCH_PHASE_STEP_MS == 0U) || \
     (CONTROL_LINE_SEARCH_PHASE_MAX_MS < CONTROL_LINE_SEARCH_INITIAL_PHASE_MS) || \
     (CONTROL_LINE_SEARCH_TIMEOUT_MS <= CONTROL_LINE_SEARCH_INITIAL_PHASE_MS))
#error "line loss recovery parameters are invalid"
#endif

/* 动作库统一参数（待实测）。 */
#define CONTROL_MOTION_DISTANCE_TOLERANCE_MM      8       /* 直行距离容差：差8mm内算完成 */

/*
 * 距离动作的IMU航向保持。
 * 只抵消动作期间产生的偏航，不负责灰度搜线或改变Route目标方向。
 */
#define CONTROL_MOTION_DISTANCE_HEADING_ENABLE          1U
#define CONTROL_MOTION_DISTANCE_HEADING_DEADBAND_DEG    1.0f
#define CONTROL_MOTION_DISTANCE_HEADING_KP_CPS_PER_DEG  20.0f
#define CONTROL_MOTION_DISTANCE_HEADING_MAX_TURN_CPS    600

#define CONTROL_MOTION_ANGLE_TOLERANCE_DEG        3.0f    /* 转角容差：误差≤3°进入稳定确认 */
#define CONTROL_MOTION_DEFAULT_TIMEOUT_MS         8000U   /* 单动作超时：超8秒报错 */
#define CONTROL_MOTION_MIN_ABS_SPEED_CPS          150     /* 直行最低速度：防止推不动 */
#define CONTROL_MOTION_MAX_ABS_SPEED_CPS          4000    /* 直行最高速度 */
#define CONTROL_MOTION_TURN_SPEED_CPS             3000    /* 转弯默认速度参数 */
#define CONTROL_MOTION_TURN_MIN_SPEED_CPS         1000    /* 转弯最低速度：≤125°转弯全程跑此值 */
#define CONTROL_MOTION_TURN_CORRECTION_SPEED_CPS  500     /* 转弯末段修正速度 */
#define CONTROL_MOTION_TURN_KP_CPS_PER_DEG        25.0f   /* 角度误差→转向速度的比例 */
#define CONTROL_MOTION_TURN_SETTLE_SAMPLES        10U     /* 连续稳定N次(每次10ms)算完成 */

#if ((CONTROL_MOTION_DISTANCE_HEADING_ENABLE != 0U) && \
     (CONTROL_MOTION_DISTANCE_HEADING_ENABLE != 1U))
#error "CONTROL_MOTION_DISTANCE_HEADING_ENABLE must be 0U or 1U"
#endif

#if ((CONTROL_MOTION_DISTANCE_HEADING_MAX_TURN_CPS <= 0) || \
     (CONTROL_MOTION_DISTANCE_HEADING_MAX_TURN_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS))
#error "distance heading turn limit is invalid"
#endif

#endif /* __CONTROL_CONFIG_H */
