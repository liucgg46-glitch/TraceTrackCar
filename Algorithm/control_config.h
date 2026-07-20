#ifndef __CONTROL_CONFIG_H
#define __CONTROL_CONFIG_H

/*
 * 车辆运动控制参数。
 *
 * 调整底盘速度、循迹或动作库参数时只修改本文件。下面各组参数属于
 * 不同控制环，输入和输出单位不同，因此必须分别调节。
 */

/* 底盘开环 PWM：上层目标按固定比例直接换算为占空比，不使用速度反馈修正。 */
#define CONTROL_CHASSIS_PWM_MAX_PERMILLE          800     /* 最大输出 ±800‰（80%占空比） */
#define CONTROL_CHASSIS_TARGET_MAX_CPS            5000    /* 此目标值对应最大 PWM 输出 */

/*
 * 基础灰度循迹参数。
 *
 * 当前版本只使用：P/PD 循迹、按误差减速、固定方向丢线搜索。
 * 不再使用边缘模式、宽线模式、输出斜坡和多阶段反向扫描。
 */
#define CONTROL_LINE_DIRECTION_REVERSE            1U      /* 传感器旋转180度后设1，统一反转循迹左右方向 */

#if ((CONTROL_LINE_DIRECTION_REVERSE != 0U) && \
     (CONTROL_LINE_DIRECTION_REVERSE != 1U))
#error "CONTROL_LINE_DIRECTION_REVERSE must be 0U or 1U"
#endif

#define CONTROL_LINE_BASE_SPEED_CPS               3000    /* 中线正常循迹速度 */
#define CONTROL_LINE_CROSS_SPEED_CPS              2500    /* 十字/全黑区域低速直行 */
#define CONTROL_LINE_MIN_TRACK_SPEED_CPS          2500    /* 大偏差时最低直行速度 */
#define CONTROL_LINE_TURN_MAX_CPS                 400    /* 最大转向量；实际还会限制到不让内侧轮反转 */

#define CONTROL_LINE_KP                           0.3f   /* 比例增大：转弯更积极；过大易摆动 */
#define CONTROL_LINE_KD                           0.2f   /* 微分增大：回正更快、抑制过冲；过大易抖动 */
#define CONTROL_LINE_ERROR_DEADBAND               80      /* 误差死区：±80 内按 0 处理 */
#define CONTROL_LINE_SPEED_FULL_ERROR             200     /* |误差|≤500 保持全速 */
#define CONTROL_LINE_SPEED_MIN_ERROR              1500    /* |误差|≥2500 降到最低速度 */

#define CONTROL_LINE_SEARCH_TURN_CPS              1000    /* 丢线后原地找线的转向量 */
#define CONTROL_LINE_SEARCH_TIMEOUT_MS            2500U   /* 连续找线超时后输出无效并停车 */

/*
 * 编译期只检查整数参数之间的安全关系，避免错误配置进入实车测试。
 * KP、KD 为浮点参数，仍需通过诊断页面和路测确认。
 */
#if ((CONTROL_CHASSIS_PWM_MAX_PERMILLE <= 0) || \
     (CONTROL_CHASSIS_PWM_MAX_PERMILLE > 1000))
#error "CONTROL_CHASSIS_PWM_MAX_PERMILLE must be in 1..1000"
#endif

#if (CONTROL_CHASSIS_TARGET_MAX_CPS <= 0)
#error "CONTROL_CHASSIS_TARGET_MAX_CPS must be positive"
#endif

#if ((CONTROL_LINE_BASE_SPEED_CPS < 0) || \
     (CONTROL_LINE_BASE_SPEED_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS))
#error "CONTROL_LINE_BASE_SPEED_CPS is out of range"
#endif

#if ((CONTROL_LINE_CROSS_SPEED_CPS < 0) || \
     (CONTROL_LINE_CROSS_SPEED_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS))
#error "CONTROL_LINE_CROSS_SPEED_CPS is out of range"
#endif

#if ((CONTROL_LINE_MIN_TRACK_SPEED_CPS < 0) || \
     (CONTROL_LINE_MIN_TRACK_SPEED_CPS > CONTROL_LINE_BASE_SPEED_CPS))
#error "CONTROL_LINE_MIN_TRACK_SPEED_CPS must be in 0..BASE_SPEED"
#endif

#if ((CONTROL_LINE_TURN_MAX_CPS < 0) || \
     (CONTROL_LINE_TURN_MAX_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS))
#error "CONTROL_LINE_TURN_MAX_CPS is out of range"
#endif

#if ((CONTROL_LINE_ERROR_DEADBAND < 0) || \
     (CONTROL_LINE_SPEED_FULL_ERROR < 0) || \
     (CONTROL_LINE_SPEED_MIN_ERROR <= CONTROL_LINE_SPEED_FULL_ERROR))
#error "Line error thresholds are invalid"
#endif

#if ((CONTROL_LINE_SEARCH_TURN_CPS <= 0) || \
     (CONTROL_LINE_SEARCH_TURN_CPS > CONTROL_CHASSIS_TARGET_MAX_CPS) || \
     (CONTROL_LINE_SEARCH_TIMEOUT_MS == 0U))
#error "Line search parameters are invalid"
#endif

/* 动作库统一参数（待实测）。 */
#define CONTROL_MOTION_DISTANCE_TOLERANCE_MM      8       /* 直行距离容差：差8mm内算完成 */
#define CONTROL_MOTION_ANGLE_TOLERANCE_DEG        3.0f    /* 转角容差：误差≤3°进入稳定确认 */
#define CONTROL_MOTION_DEFAULT_TIMEOUT_MS         8000U   /* 单动作超时：超8秒报错 */
#define CONTROL_MOTION_MIN_ABS_SPEED_CPS          150     /* 直行最低速度：防止推不动 */
#define CONTROL_MOTION_MAX_ABS_SPEED_CPS          4000    /* 直行最高速度 */
#define CONTROL_MOTION_TURN_SPEED_CPS             3000    /* 转弯默认速度参数 */
#define CONTROL_MOTION_TURN_MIN_SPEED_CPS         1000    /* 转弯最低速度：≤125°转弯全程跑此值 */
#define CONTROL_MOTION_TURN_CORRECTION_SPEED_CPS  500     /* 转弯末段修正速度 */
#define CONTROL_MOTION_TURN_KP_CPS_PER_DEG        25.0f   /* 角度误差→转向速度的比例 */
#define CONTROL_MOTION_TURN_SETTLE_SAMPLES        10U     /* 连续稳定N次(每次10ms)算完成 */

#endif /* __CONTROL_CONFIG_H */
