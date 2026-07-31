#ifndef __BALL_BALANCE_CONFIG_H
#define __BALL_BALANCE_CONFIG_H

/*
 * 钢球平衡控制集中配置。
 * 角度单位为deg，位置单位为mm，速度单位为mm/s，加速度单位为mm/s^2。
 */

/* 当前实测水平角；首次上板仍需用舵机标定档复核。 */
#define BALL_BALANCE_LEVEL_ANGLE_DEG                    93.0f
#define BALL_BALANCE_LEVEL_ANGLE_X10                    930U

/*
 * 舵机角大于局部平衡角时钢球向负方向加速。
 * 该增益描述舵机角经过连杆后对摆杆倾角的等效作用，不能直接使用
 * 理想滚动小球在同角度斜面上的理论增益。
 */
#define BALL_BALANCE_SERVO_TO_ACCEL_SIGN                (-1.0f)
#define BALL_BALANCE_B0_MM_S2_PER_DEG                   25.0f

#define BALL_BALANCE_CONTROL_PERIOD_MS                  10U
#define BALL_BALANCE_CONTROL_PERIOD_S                   0.010f

#define BALL_BALANCE_VALID_TIMEOUT_MS                   600U
#define BALL_BALANCE_REACQUIRE_VALID_COUNT              2U
#define BALL_BALANCE_MIN_CONFIDENCE                     60U
#define BALL_BALANCE_POSITION_ABS_MAX_MM_X10            1200

#define BALL_BALANCE_NATURAL_FREQ_RAD_S                 2.3f
#define BALL_BALANCE_DAMPING_RATIO                      1.15f

/*
 * 控制输入滤波只作用于反馈计算，不修改K210原始位置和状态估计器内部状态。
 * 参考加速度降权用于避免加速/制动切换直接造成舵机大角度反向。
 */
#define BALL_BALANCE_VELOCITY_FILTER_TIME_S             0.100f
#define BALL_BALANCE_VELOCITY_DEADBAND_MM_S             6.0f
#define BALL_BALANCE_DISTURBANCE_FILTER_TIME_S          0.200f
#define BALL_BALANCE_REFERENCE_ACCEL_FEEDFORWARD_GAIN   0.15f
#define BALL_BALANCE_DYNAMIC_FILTER_TIME_S              0.060f
#define BALL_BALANCE_DYNAMIC_SOFT_LIMIT_DEG             16.0f

/* 只保留舵机客观存在的全行程边界，不再设置滚球业务角度上限。 */
#define BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG             0.0f
#define BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG             180.0f

/*
 * 到点后锁住当前舵机角；只有位置重新越过退出滞环才恢复控制。
 * 进入阈值小于退出阈值，用于隔离视觉测量噪声。
 */
#define BALL_BALANCE_TARGET_LOCK_ENTER_ERROR_MM         3.0f
#define BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM          6.0f
#define BALL_BALANCE_TARGET_LOCK_SPEED_MM_S             6.0f
#define BALL_BALANCE_TARGET_LOCK_TIME_MS                250U

/*
 * 舵机命令采用速度、加速度受限的二阶轨迹，不再使用每周期固定角度硬限幅。
 * 大误差时允许高速追踪；接近请求角时，TRACK_TIME自动降低速度以减少顿挫。
 */
#define BALL_BALANCE_SERVO_MAX_SPEED_DEG_S              180.0f
#define BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2             3600.0f
#define BALL_BALANCE_SERVO_TRACK_TIME_S                 0.050f

#define BALL_REFERENCE_MAX_SPEED_MM_S                   160.0f
#define BALL_REFERENCE_MAX_ACCEL_MM_S2                  600.0f
#define BALL_REFERENCE_MAX_JERK_MM_S3                   3000.0f
#define BALL_REFERENCE_BRAKE_DISTANCE_FACTOR            1.08f
#define BALL_REFERENCE_SNAP_POSITION_MM                 0.20f
#define BALL_REFERENCE_SNAP_SPEED_MM_S                  0.50f
#define BALL_REFERENCE_SNAP_ACCEL_MM_S2                 2.0f
#define BALL_REFERENCE_FINAL_APPROACH_POSITION_MM       4.0f
#define BALL_REFERENCE_FINAL_APPROACH_SPEED_MM_S        5.0f

/*
 * 连续脱困爬坡：目标外且球未移动时持续增大，检测到运动后缓慢释放。
 * 不设置补偿角上限，最终只受舵机0°～180°物理行程约束。
 */
#define BALL_BALANCE_BREAKAWAY_DWELL_MS                 120U
#define BALL_BALANCE_BREAKAWAY_START_DEG                2.0f
#define BALL_BALANCE_BREAKAWAY_GROWTH_DEG_S             18.0f
#define BALL_BALANCE_BREAKAWAY_RELEASE_DEG_S            12.0f
#define BALL_BALANCE_BREAKAWAY_MOVING_SPEED_MM_S        6.0f
#define BALL_BALANCE_BREAKAWAY_PROGRESS_MM              2.0f
#define BALL_BALANCE_BREAKAWAY_PROGRESS_COUNT           2U

/* 三状态增广卡尔曼滤波器参数。协方差单位与对应状态平方保持一致。 */
#define BALL_ESTIMATOR_Q_POSITION                       0.02f
#define BALL_ESTIMATOR_Q_VELOCITY                       0.80f
#define BALL_ESTIMATOR_Q_DISTURBANCE                    12.0f
#define BALL_ESTIMATOR_R_POSITION                       16.0f
#define BALL_ESTIMATOR_INITIAL_P_POSITION               25.0f
#define BALL_ESTIMATOR_INITIAL_P_VELOCITY               400.0f
#define BALL_ESTIMATOR_INITIAL_P_DISTURBANCE            2500.0f
#define BALL_ESTIMATOR_INNOVATION_LIMIT_MM              35.0f
#define BALL_ESTIMATOR_EDGE_RESET_POSITION_MM           105.0f
#define BALL_ESTIMATOR_REJECT_RESET_COUNT               2U

#define BALL_EQUILIBRIUM_MAP_POINT_COUNT                7U
#define BALL_EQUILIBRIUM_POS_0_MM                       (-120.0f)
#define BALL_EQUILIBRIUM_POS_1_MM                       (-80.0f)
#define BALL_EQUILIBRIUM_POS_2_MM                       (-50.0f)
#define BALL_EQUILIBRIUM_POS_3_MM                       0.0f
#define BALL_EQUILIBRIUM_POS_4_MM                       50.0f
#define BALL_EQUILIBRIUM_POS_5_MM                       80.0f
#define BALL_EQUILIBRIUM_POS_6_MM                       120.0f

/* 底盘线性加速度前馈当前未接通，以下参数供接入去重力缓存后统一使用。 */
#define BALL_VEHICLE_IMU_FORWARD_AXIS                   0U
#define BALL_VEHICLE_IMU_FORWARD_SIGN                   1.0f
#define BALL_VEHICLE_IMU_LOWPASS_ALPHA                  0.20f
#define BALL_VEHICLE_IMU_TIMEOUT_MS                     100U

/* 手动联调任务按100ms周期输出流式日志；未注册测试任务时不会产生串口输出。 */
#define BALL_BALANCE_DEBUG_STREAM_ENABLE                1U
#define BALL_BALANCE_DEBUG_STREAM_PERIOD_MS             100U

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
