#ifndef __BALL_BALANCE_CONFIG_H
#define __BALL_BALANCE_CONFIG_H

/*
 * 钢球平衡控制集中配置。
 * 角度单位为deg，位置单位为mm，速度单位为mm/s，加速度单位为mm/s^2。
 */

/* 当前实测水平角；首次上板仍需用舵机标定档复核。 */
#define BALL_BALANCE_LEVEL_ANGLE_DEG                    90.0f
#define BALL_BALANCE_LEVEL_ANGLE_X10                    900U

/* 舵机角大于局部平衡角时钢球向负方向加速。 */
#define BALL_BALANCE_SERVO_TO_ACCEL_SIGN                (-1.0f)
#define BALL_BALANCE_B0_MM_S2_PER_DEG                   100.0f

#define BALL_BALANCE_CONTROL_PERIOD_MS                  10U
#define BALL_BALANCE_CONTROL_PERIOD_S                   0.010f

#define BALL_BALANCE_VALID_TIMEOUT_MS                   200U
#define BALL_BALANCE_REACQUIRE_VALID_COUNT              3U
#define BALL_BALANCE_MIN_CONFIDENCE                     60U
#define BALL_BALANCE_POSITION_ABS_MAX_MM_X10            1200

#define BALL_BALANCE_NATURAL_FREQ_RAD_S                 1.8f
#define BALL_BALANCE_DAMPING_RATIO                      1.0f

#define BALL_BALANCE_DYNAMIC_ANGLE_LIMIT_DEG            2.5f
#define BALL_BALANCE_ANGLE_SLEW_DEG_PER_UPDATE          0.3f
#define BALL_BALANCE_ABS_SAFE_MIN_DEG                   80.0f
#define BALL_BALANCE_ABS_SAFE_MAX_DEG                   180.0f

#define BALL_REFERENCE_MAX_SPEED_MM_S                   80.0f
#define BALL_REFERENCE_MAX_ACCEL_MM_S2                  160.0f
#define BALL_REFERENCE_MAX_JERK_MM_S3                   600.0f
#define BALL_REFERENCE_SNAP_POSITION_MM                 0.20f
#define BALL_REFERENCE_SNAP_SPEED_MM_S                  0.50f
#define BALL_REFERENCE_SNAP_ACCEL_MM_S2                 2.0f

#define BALL_BALANCE_SETTLE_ERROR_MM                    6.0f
#define BALL_BALANCE_SETTLE_SPEED_MM_S                  10.0f
#define BALL_BALANCE_SETTLE_TIME_MS                     250U

#define BALL_BALANCE_STUCK_ERROR_MM                     8.0f
#define BALL_BALANCE_STUCK_SPEED_MM_S                   8.0f
#define BALL_BALANCE_STUCK_TIME_MS                      300U
#define BALL_BALANCE_STICTION_START_DEG                 0.3f
#define BALL_BALANCE_STICTION_MAX_DEG                   1.0f
#define BALL_BALANCE_STICTION_STEP_DEG                  0.05f
#define BALL_BALANCE_STICTION_STEP_PERIOD_MS            100U
#define BALL_BALANCE_STICTION_RELEASE_SPEED_MM_S        15.0f

/* 三状态增广卡尔曼滤波器参数。协方差单位与对应状态平方保持一致。 */
#define BALL_ESTIMATOR_Q_POSITION                       0.02f
#define BALL_ESTIMATOR_Q_VELOCITY                       0.80f
#define BALL_ESTIMATOR_Q_DISTURBANCE                    12.0f
#define BALL_ESTIMATOR_R_POSITION                       16.0f
#define BALL_ESTIMATOR_INITIAL_P_POSITION               25.0f
#define BALL_ESTIMATOR_INITIAL_P_VELOCITY               400.0f
#define BALL_ESTIMATOR_INITIAL_P_DISTURBANCE            2500.0f
#define BALL_ESTIMATOR_INNOVATION_LIMIT_MM              35.0f

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

/* 正式比赛默认关闭流式日志；专项测试按100ms周期输出。 */
#if defined(PROJECT_TEST_TASKS_ENABLE) && \
    (PROJECT_TEST_TASKS_ENABLE != 0U)
#define BALL_BALANCE_DEBUG_STREAM_ENABLE                1U
#else
#define BALL_BALANCE_DEBUG_STREAM_ENABLE                0U
#endif
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
