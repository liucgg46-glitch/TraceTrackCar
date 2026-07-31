#ifndef __BALL_BALANCE_CONFIG_H
#define __BALL_BALANCE_CONFIG_H

/*
 * 钢球平衡集中配置。
 * 角度单位为deg，位置单位为mm，速度单位为mm/s，加速度单位为mm/s^2。
 */

/* 当前实测水平角；首次上板仍需用舵机标定框架复核。 */
#define BALL_BALANCE_LEVEL_ANGLE_DEG                    93.0f    /* 摆杆处于水平状态时的舵机角度 */
#define BALL_BALANCE_LEVEL_ANGLE_X10                    930U     /* 水平舵机角度，单位为0.1度 */

/*
 * 舵机角大于局部平衡角时钢球向负方向加速。
 * B0仅供卡尔曼预测模型使用，新串级控制器不再把加速度模型用于控制律计算。
 */
#define BALL_BALANCE_SERVO_TO_ACCEL_SIGN                (-1.0f)  /* 舵机角度变化与钢球加速度的方向关系 */
#define BALL_BALANCE_B0_MM_S2_PER_DEG                   15.0f    /* 每度动态角对应的钢球加速度模型增益 */

#define BALL_BALANCE_CONTROL_PERIOD_MS                  10U      /* 平衡控制任务周期，单位ms */
#define BALL_BALANCE_CONTROL_PERIOD_S                   0.010f   /* 平衡控制任务周期，单位s */

#define BALL_BALANCE_VALID_TIMEOUT_MS                   600U     /* 连续无VALID位置后判定数据超时的时间 */
#define BALL_BALANCE_REACQUIRE_VALID_COUNT              2U       /* 超时后恢复控制所需的连续VALID帧数 */
#define BALL_BALANCE_MIN_CONFIDENCE                     60U      /* K210位置数据允许参与控制的最低置信度 */
#define BALL_BALANCE_POSITION_ABS_MAX_MM_X10            1200     /* 钢球位置允许的最大绝对值，单位0.1mm */

/*
 * 外环制动距离速度曲线 + 内环速度PI参数。首次上板建议：
 * 1. 先将BALL_BALANCE_VELOCITY_KI设为0，只调VELOCITY_KP，使实际速度能够跟随目标速度且不过度振荡。
 * 2. 再逐渐增加VELOCITY_KI，直到能够自然克服静摩擦。
 * 3. 再调整BRAKE_ACCEL和TARGET_VELOCITY_MAX，决定整体运行速度和接近目标时的减速距离。
 * 4. 最后调整HOLD阈值。
 *
 * BRAKE_ACCEL增大：更晚减速、接近目标速度更高，但更容易过冲。
 * BRAKE_ACCEL减小：更早减速、更稳定。
 * TARGET_VELOCITY_MAX控制远距离最高速度。
 * TARGET_ACCEL_MAX控制目标速度变化快慢。
 */
//#define BALL_BALANCE_POSITION_KP                        1.95f     /* 保留给兼容和对比调参，当前制动距离外环不使用 */
#define BALL_BALANCE_BRAKE_ACCEL_MM_S2                  80.0f     /* 制动距离速度曲线使用的等效减速度 */
#define BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S           80.0f    /* 外环目标速度绝对限幅 */
#define BALL_BALANCE_TARGET_ACCEL_MAX_MM_S2             800.0f   /* 外环目标速度最大变化率 */
#define BALL_BALANCE_POSITION_DEADBAND_MM               3.0f     /* 位置误差不超过该值时目标速度置零 */
#define BALL_BALANCE_VELOCITY_KP                        0.60f    /* 速度PI比例角增益 */
#define BALL_BALANCE_VELOCITY_KI                        1.2f    /* 速度PI积分角增益 */
#define BALL_BALANCE_VELOCITY_FILTER_TIME_S             0.050f   /* 钢球速度反馈的低通滤波时间常数 */

/*
 * 舵机客观存在的全行程边界。滚球业务层最终也只使用该绝对安全范围。
 */
#define BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG             0.0f     /* 舵机允许输出的绝对最小角度 */
#define BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG             180.0f   /* 舵机允许输出的绝对最大角度 */
#define BALL_BALANCE_SERVO_SAFE_MIN_DEG                 BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG
#define BALL_BALANCE_SERVO_SAFE_MAX_DEG                 BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG

/*
 * 到点保持在满足条件后记录当前实际舵机命令角，并冻结该角度直到退出条件成立。
 */
#define BALL_BALANCE_TARGET_LOCK_ENTER_ERROR_MM         8.0f     /* 进入到点状态允许的最大位置误差 */
#define BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM          10.0f     /* 超过该位置误差后退出到点状态 */
#define BALL_BALANCE_TARGET_LOCK_SPEED_MM_S             15.0f     /* 进入到点状态允许的最大钢球速度和目标速度 */
#define BALL_BALANCE_TARGET_LOCK_SERVO_SPEED_DEG_S      10.0f     /* 进入到点状态允许的最大舵机命令速度 */
#define BALL_BALANCE_TARGET_LOCK_TIME_MS                50U     /* 满足到点条件所需的连续保持时间 */
#define BALL_BALANCE_TARGET_CHANGE_EPSILON_MM           0.05f    /* 判定目标位置发生变化的最小差值 */

/*
 * 舵机命令采用速度、加速度受限的二阶轨迹，不再使用运动过程0.4度目标死区。
 */
#define BALL_BALANCE_SERVO_MAX_SPEED_DEG_S              180.0f   /* 舵机命令允许的最大角速度 */
#define BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2             4000.0f  /* 舵机命令允许的最大角加速度 */
#define BALL_BALANCE_SERVO_TRACK_TIME_S                 0.020f   /* 舵机接近目标角时的减速跟踪时间 */

#define BALL_REFERENCE_MAX_SPEED_MM_S                   80.0f    /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_MAX_ACCEL_MM_S2                  160.0f   /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_MAX_JERK_MM_S3                   600.0f   /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_BRAKE_DISTANCE_FACTOR            1.08f    /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_SNAP_POSITION_MM                 0.20f    /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_SNAP_SPEED_MM_S                  0.50f    /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_SNAP_ACCEL_MM_S2                 2.0f     /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_FINAL_APPROACH_POSITION_MM       4.0f     /* 参考轨迹保留编译参数，当前控制链路不使用 */
#define BALL_REFERENCE_FINAL_APPROACH_SPEED_MM_S        5.0f     /* 参考轨迹保留编译参数，当前控制链路不使用 */

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
