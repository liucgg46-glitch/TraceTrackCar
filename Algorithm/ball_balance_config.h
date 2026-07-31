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

#define BALL_BALANCE_NATURAL_FREQ_RAD_S                 2.5f     /* 闭环自然频率，越大响应越快、控制越强 */
#define BALL_BALANCE_DAMPING_RATIO                      1.15f    /* 闭环阻尼比，越大制动越强、振荡越小 */

/*
 * 控制输入滤波只作用于反馈计算，不修改K210原始位置和状态估计器内部状态。
 * 参考加速度降权用于避免加速/制动切换直接造成舵机大角度反向。
 */
#define BALL_BALANCE_VELOCITY_FILTER_TIME_S             0.050f   /* 钢球速度反馈的低通滤波时间常数 */
#define BALL_BALANCE_VELOCITY_DEADBAND_MM_S             3.0f     /* 小于该速度时按静止处理，降低速度噪声 */
#define BALL_BALANCE_DISTURBANCE_FILTER_TIME_S          0.200f   /* 等效扰动估计值的低通滤波时间常数 */
#define BALL_BALANCE_DISTURBANCE_COMPENSATION_GAIN      0.0f     /* 扰动估计参与控制的比例，0表示关闭 */
#define BALL_BALANCE_REFERENCE_ACCEL_FEEDFORWARD_GAIN   0.0f     /* 参考加速度前馈比例，0表示关闭 */
#define BALL_BALANCE_DYNAMIC_FILTER_TIME_S              0.060f   /* 动态舵机角的低通滤波时间常数 */
#define BALL_BALANCE_DYNAMIC_HARD_LIMIT_DEG             40.0f     /* 正常状态反馈允许输出的最大动态角 */

 /* 舵机客观存在的全行程边界；滚球业务角度另有独立限幅。 */
#define BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG             0.0f     /* 舵机允许输出的绝对最小角度 */
#define BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG             180.0f   /* 舵机允许输出的绝对最大角度 */

/*
 * 到点标志只用于settled/target_locked状态，不冻结瞬时舵机角。
 */
#define BALL_BALANCE_TARGET_LOCK_ENTER_ERROR_MM         3.0f     /* 进入到点状态允许的最大位置误差 */
#define BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM          6.0f     /* 超过该位置误差后退出到点状态 */
#define BALL_BALANCE_TARGET_LOCK_SPEED_MM_S             6.0f     /* 进入到点状态允许的最大钢球速度 */
#define BALL_BALANCE_TARGET_LOCK_TIME_MS                250U     /* 满足到点条件所需的连续保持时间 */

/*
 * 舵机命令采用速度、加速度受限的二阶轨迹，不再使用每周期固定角度硬限幅。
 * 接近请求角时，TRACK_TIME自动降低速度以减少顿挫。
 */
#define BALL_BALANCE_SERVO_MAX_SPEED_DEG_S              180.0f   /* 舵机命令允许的最大角速度 */
#define BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2             4000.0f  /* 舵机命令允许的最大角加速度 */
#define BALL_BALANCE_SERVO_TRACK_TIME_S                 0.015f   /* 舵机接近目标角时的减速跟踪时间 */

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
 * 有界脱困补偿。单位：误差mm、速度mm/s、时间ms、角度deg。
 * 只在参考误差较大且VALID新鲜时缓慢增加，用于克服静摩擦。
 */
#define BALL_BALANCE_BREAKAWAY_MIN_ERROR_MM             8.0f     /* 误差大于该值时才允许启动脱困补偿 */
#define BALL_BALANCE_BREAKAWAY_CLEAR_ERROR_MM           3.0f     /* 误差小于该值时开始撤销脱困补偿 */
#define BALL_BALANCE_BREAKAWAY_DWELL_MS                 50.0      /* 满足卡住条件多久后启动脱困补偿 */
#define BALL_BALANCE_BREAKAWAY_START_DEG                10.0f     /* 脱困补偿启动时立即加入的初始角度 */
#define BALL_BALANCE_BREAKAWAY_MAX_DEG                  20.0f     /* 脱困补偿允许达到的最大附加角度 */
#define BALL_BALANCE_BREAKAWAY_GROWTH_DEG_S             4.0f     /* 脱困补偿角度的增长速度 */
#define BALL_BALANCE_BREAKAWAY_RELEASE_DEG_S            2.0f    /* 钢球开始运动后脱困角的撤销速度 */
#define BALL_BALANCE_BREAKAWAY_STUCK_SPEED_MM_S         6.0f    /* 低于该速度时才可能判定钢球卡住 */
#define BALL_BALANCE_BREAKAWAY_RELEASE_SPEED_MM_S       15.0f     /* 超过该速度时开始撤销脱困补偿 */
#define BALL_BALANCE_BREAKAWAY_PROGRESS_MM              0.5f     /* 单次VALID测量确认有效前进的最小距离 */
#define BALL_BALANCE_BREAKAWAY_PROGRESS_COUNT           2U       /* 连续多少次有效前进后确认已经脱困 */
#define BALL_BALANCE_BREAKAWAY_VALID_AGE_MS             60U      /* 允许增加脱困角的VALID数据最大年龄 */

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
