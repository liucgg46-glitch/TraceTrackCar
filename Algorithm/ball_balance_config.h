#ifndef __BALL_BALANCE_CONFIG_H
#define __BALL_BALANCE_CONFIG_H

/*
 * 钢球平衡控制集中配置。
 * 角度单位为deg，位置单位为mm，速度单位为mm/s，加速度单位为mm/s^2。
 */

/* 当前实测水平角；首次上板仍需用舵机标定档复核。 */
#define BALL_BALANCE_LEVEL_ANGLE_DEG                    93.0f    /* 摆杆处于水平状态时的舵机角度 */
#define BALL_BALANCE_LEVEL_ANGLE_X10                    930U     /* 水平舵机角度，单位为0.1° */

/*
 * 舵机角大于局部平衡角时钢球向负方向加速。
 * 该增益描述舵机角经过连杆后对摆杆倾角的等效作用，不能直接使用
 * 理想滚动小球在同角度斜面上的理论增益。
 */
#define BALL_BALANCE_SERVO_TO_ACCEL_SIGN                (-1.0f)  /* 舵机角度变化与钢球加速度的方向关系 */
#define BALL_BALANCE_B0_MM_S2_PER_DEG                   15.0f    /* 每1°动态角对应的钢球加速度模型增益 */

#define BALL_BALANCE_CONTROL_PERIOD_MS                  10U      /* 平衡控制任务周期，单位ms */
#define BALL_BALANCE_CONTROL_PERIOD_S                   0.010f   /* 平衡控制任务周期，单位s */

#define BALL_BALANCE_VALID_TIMEOUT_MS                   600U     /* 连续无VALID位置后判定数据超时的时间 */
#define BALL_BALANCE_REACQUIRE_VALID_COUNT              2U       /* 超时后恢复控制所需的连续VALID帧数 */
#define BALL_BALANCE_MIN_CONFIDENCE                     60U      /* K210位置数据允许参与控制的最低置信度 */
#define BALL_BALANCE_POSITION_ABS_MAX_MM_X10            1200     /* 钢球位置允许的最大绝对值，单位0.1mm */

#define BALL_BALANCE_NATURAL_FREQ_RAD_S                 1.8f     /* 闭环自然频率，越大响应越快、控制越强 */
#define BALL_BALANCE_DAMPING_RATIO                      1.15f    /* 闭环阻尼比，越大制动越强、振荡越小 */

/*
 * 控制输入滤波只作用于反馈计算，不修改K210原始位置和状态估计器内部状态。
 * 参考加速度降权用于避免加速/制动切换直接造成舵机大角度反向。
 */
#define BALL_BALANCE_VELOCITY_FILTER_TIME_S             0.050f   /* 钢球速度反馈的低通滤波时间常数 */
#define BALL_BALANCE_VELOCITY_DEADBAND_MM_S             3.0f     /* 进入速度死区的阈值，降低零速附近噪声 */
#define BALL_BALANCE_VELOCITY_DEADBAND_EXIT_MM_S        6.0f     /* 退出速度死区的阈值，避免零速状态反复切换 */
#define BALL_BALANCE_DISTURBANCE_FILTER_TIME_S          0.200f   /* 等效扰动估计值的低通滤波时间常数 */
#define BALL_BALANCE_DISTURBANCE_COMPENSATION_GAIN      0.0f     /* 扰动估计参与控制的比例，0表示关闭 */
#define BALL_BALANCE_REFERENCE_ACCEL_FEEDFORWARD_GAIN   0.0f     /* 参考加速度前馈比例，0表示关闭 */
#define BALL_BALANCE_DYNAMIC_FILTER_TIME_S              0.060f   /* 动态舵机角的低通滤波时间常数 */
#define BALL_BALANCE_DYNAMIC_HARD_LIMIT_DEG             30.0f    /* 正常状态反馈允许输出的最大动态角 */

 /* 舵机客观存在的全行程边界；滚球业务角度另有独立限幅。 */
#define BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG             0.0f     /* 舵机允许输出的绝对最小角度 */
#define BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG             180.0f   /* 舵机允许输出的绝对最大角度 */

/*
 * 滚球机构的独立机械安全角。当前仅有Driver全行程标定资料，因此暂时引用该范围；
 * 若实物齿条端点小于舵机全行程，必须在上板标定后收窄，具体安全端点待确认。
 */
#define BALL_BALANCE_SERVO_SAFE_MIN_DEG                 BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG
#define BALL_BALANCE_SERVO_SAFE_MAX_DEG                 BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG

/*
 * 到点保持在满足条件后记录当前稳定舵机角，并冻结该角度直到退出条件成立。
 */
#define BALL_BALANCE_TARGET_LOCK_ENTER_ERROR_MM         3.0f     /* 进入到点状态允许的最大位置误差 */
#define BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM          6.0f     /* 超过该位置误差后退出到点状态 */
#define BALL_BALANCE_TARGET_LOCK_SPEED_MM_S             6.0f     /* 进入到点状态允许的最大钢球速度 */
#define BALL_BALANCE_TARGET_LOCK_TIME_MS                250U     /* 满足到点条件所需的连续保持时间 */

/*
 * 舵机命令采用速度、加速度受限的二阶轨迹，不再使用每周期固定角度硬限幅。
 * 接近请求角时，TRACK_TIME自动降低速度以减少顿挫。
 */
#define BALL_BALANCE_SERVO_MAX_SPEED_DEG_S              120.0f   /* 舵机命令允许的最大角速度 */
#define BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2             2500.0f  /* 舵机命令允许的最大角加速度 */
#define BALL_BALANCE_SERVO_TRACK_TIME_S                 0.030f   /* 舵机接近目标角时的减速跟踪时间 */
#define BALL_BALANCE_SERVO_COMMAND_DEADBAND_DEG         0.4f     /* 小于该差值时保持上一目标角，抑制PWM微抖 */

#define BALL_REFERENCE_MAX_SPEED_MM_S                   80.0f    /* 参考位置轨迹允许的最大速度 */
#define BALL_REFERENCE_MAX_ACCEL_MM_S2                  160.0f   /* 参考位置轨迹允许的最大加速度 */
#define BALL_REFERENCE_MAX_JERK_MM_S3                   600.0f   /* 参考位置轨迹允许的最大加加速度 */
#define BALL_REFERENCE_BRAKE_DISTANCE_FACTOR            1.08f    /* 制动距离放大系数，越大越早开始减速 */
#define BALL_REFERENCE_SNAP_POSITION_MM                 0.20f    /* 小于该位置误差时允许吸附到目标位置 */
#define BALL_REFERENCE_SNAP_SPEED_MM_S                  0.50f    /* 小于该速度时允许吸附到目标状态 */
#define BALL_REFERENCE_SNAP_ACCEL_MM_S2                 2.0f     /* 小于该加速度时允许吸附到目标状态 */
#define BALL_REFERENCE_FINAL_APPROACH_POSITION_MM       4.0f     /* 距离目标多近时进入最终慢速接近阶段 */
#define BALL_REFERENCE_FINAL_APPROACH_SPEED_MM_S        5.0f     /* 最终慢速接近阶段允许的参考速度 */

/*
 * 脱困状态机参数。脱困角没有固定角度上限，RAMP只受最终舵机机械安全角约束。
 * 单位：误差和位移mm、速度mm/s、时间ms、角度deg。
 */
#define BALL_BALANCE_BREAKAWAY_MIN_ERROR_MM             5.0f     /* 误差超过该值才允许判定卡住 */
#define BALL_BALANCE_BREAKAWAY_STUCK_SPEED_MM_S         3.0f     /* 低于该速度才允许判定卡住或再次停住 */
#define BALL_BALANCE_BREAKAWAY_IDLE_DWELL_MS            300U     /* IDLE满足卡住条件后进入RAMP的等待时间 */
#define BALL_BALANCE_BREAKAWAY_RESTART_DWELL_MS         300U     /* DECAY再次停住后恢复RAMP的等待时间 */
#define BALL_BALANCE_BREAKAWAY_GROWTH_DEG_S             10.0f    /* RAMP脱困角连续增长速度 */
#define BALL_BALANCE_BREAKAWAY_START_PROGRESS_MM        1.5f     /* 从RAMP起点累计前进达到该位移后确认启动 */
#define BALL_BALANCE_BREAKAWAY_FORWARD_STEP_MIN_MM      0.1f     /* 单帧向目标运动的最小有效位移 */
#define BALL_BALANCE_BREAKAWAY_FORWARD_FRAME_COUNT      2U       /* 确认启动所需的连续向目标运动帧数 */
#define BALL_BALANCE_BREAKAWAY_DECAY_RATE_DEG_S         12.0f    /* DECAY正常撤销脱困角的最大速度 */
#define BALL_BALANCE_BREAKAWAY_CLEAR_RATE_DEG_S         30.0f    /* 异常、越过目标或目标变化时的快速平滑清零速度 */
#define BALL_BALANCE_BREAKAWAY_DECAY_SPEED_SCALE_MM_S   20.0f    /* 速度越接近该值，DECAY目标角下降越快 */
#define BALL_BALANCE_BREAKAWAY_DECAY_PROGRESS_WEIGHT    0.5f     /* DECAY目标角中运动进度相对剩余误差的权重 */
#define BALL_BALANCE_BREAKAWAY_COOLDOWN_MS              250U     /* 脱困角清零后的冷却时间 */
#define BALL_BALANCE_BREAKAWAY_VALID_AGE_MS             120U     /* 允许启动和推进脱困的VALID数据最大年龄 */
#define BALL_BALANCE_TARGET_CHANGE_EPSILON_MM           0.05f    /* 判定目标位置发生变化的最小差值 */

/* 三状态增广卡尔曼滤波器参数。协方差单位与对应状态平方保持一致。 */
#define BALL_ESTIMATOR_Q_POSITION                       0.02f    /* 位置状态的过程噪声大小 */
#define BALL_ESTIMATOR_Q_VELOCITY                       0.80f    /* 速度状态的过程噪声大小 */
#define BALL_ESTIMATOR_Q_DISTURBANCE                    12.0f    /* 扰动状态的过程噪声大小 */
#define BALL_ESTIMATOR_R_POSITION                       16.0f    /* K210位置测量噪声大小 */
#define BALL_ESTIMATOR_INITIAL_P_POSITION               25.0f    /* 初始化时位置估计的不确定度 */
#define BALL_ESTIMATOR_INITIAL_P_VELOCITY               400.0f   /* 初始化时速度估计的不确定度 */
#define BALL_ESTIMATOR_INITIAL_P_DISTURBANCE            2500.0f  /* 初始化时扰动估计的不确定度 */
#define BALL_ESTIMATOR_INNOVATION_LIMIT_MM              35.0f    /* 单帧位置创新允许的最大绝对值 */
#define BALL_ESTIMATOR_EDGE_RESET_POSITION_MM           105.0f   /* 钢球接近水管边缘时允许触发重置的位置 */
#define BALL_ESTIMATOR_REJECT_RESET_COUNT               2U       /* 连续拒绝多少次测量后重置估计器 */

#define BALL_EQUILIBRIUM_MAP_POINT_COUNT                7U       /* 局部平衡角查找表的标定点数量 */
#define BALL_EQUILIBRIUM_POS_0_MM                       (-120.0f) /* 局部平衡角表第0个位置点 */
#define BALL_EQUILIBRIUM_POS_1_MM                       (-80.0f)  /* 局部平衡角表第1个位置点 */
#define BALL_EQUILIBRIUM_POS_2_MM                       (-50.0f)  /* 局部平衡角表第2个位置点 */
#define BALL_EQUILIBRIUM_POS_3_MM                       0.0f      /* 局部平衡角表中心位置点 */
#define BALL_EQUILIBRIUM_POS_4_MM                       50.0f     /* 局部平衡角表第4个位置点 */
#define BALL_EQUILIBRIUM_POS_5_MM                       80.0f     /* 局部平衡角表第5个位置点 */
#define BALL_EQUILIBRIUM_POS_6_MM                       120.0f    /* 局部平衡角表第6个位置点 */

/* 底盘线性加速度前馈当前未接通，以下参数供接入去重力缓存后统一使用。 */
#define BALL_VEHICLE_IMU_FORWARD_AXIS                   0U       /* 底盘IMU中沿水管方向使用的加速度轴 */
#define BALL_VEHICLE_IMU_FORWARD_SIGN                   1.0f     /* IMU加速度方向与钢球坐标方向的符号关系 */
#define BALL_VEHICLE_IMU_LOWPASS_ALPHA                  0.20f    /* 底盘加速度前馈的低通滤波系数 */
#define BALL_VEHICLE_IMU_TIMEOUT_MS                     100U     /* IMU数据超过该时间未更新则判定失效 */

/* 手动联调任务按100ms周期输出流式日志；未注册测试任务时不会产生串口输出。 */
#define BALL_BALANCE_DEBUG_STREAM_ENABLE                1U       /* 是否开启钢球控制CSV调试输出 */
#define BALL_BALANCE_DEBUG_STREAM_PERIOD_MS             100U     /* 钢球控制调试数据的输出周期 */

static __inline float BallBalance_Model_DynamicAngleToAccelMmS2(
    float dynamic_angle_deg
)
{
    return BALL_BALANCE_SERVO_TO_ACCEL_SIGN *
           BALL_BALANCE_B0_MM_S2_PER_DEG *
           dynamic_angle_deg;
}

static __inline float BallBalance_Model_AccelToDynamicAngleDeg(
    float acceleration_mm_s2
)
{
    return acceleration_mm_s2 /
           (BALL_BALANCE_SERVO_TO_ACCEL_SIGN *
            BALL_BALANCE_B0_MM_S2_PER_DEG);
}

#endif /* __BALL_BALANCE_CONFIG_H */
