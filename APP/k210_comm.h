#ifndef __K210_COMM_H
#define __K210_COMM_H

#include <stdint.h>
#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * K210与STM32通信协议
 * ============================================================================
 *
 * 文件编码：
 *   UTF-8
 *
 * STM32接口：
 *   USART2_TX：PA2
 *   USART2_RX：PA3
 *
 * 当前接线：
 *   STM32 PA2（TX） -> K210 RX
 *   STM32 PA3（RX） <- K210 IO6（TX）
 *   STM32 GND       -- K210 GND
 *
 * 串口参数：
 *   波特率：115200
 *   数据位：8位
 *   校验位：无
 *   停止位：1位
 *
 * 固定帧格式：
 *
 *   [0] 0xAA
 *   [1] 0x55
 *   [2] CMD
 *   [3] DATA1
 *   [4] DATA2
 *   [5] DATA3
 *   [6] CHECKSUM
 *
 * CHECKSUM：
 *   前6个字节累加和的低8位。
 *
 * 设计原则：
 *   1. 保留旧版数字、目标点和激光点协议；
 *   2. 保留现有多数字快照协议；
 *   3. 新增通用视觉快照协议；
 *   4. 新旧协议可以同时存在，逐步迁移；
 *   5. STM32控制层不直接读取串口原始字节。
 */

/*
 * ============================================================================
 * 固定帧参数
 * ============================================================================
 */

#define K210_FRAME_HEAD1                  0xAAU
#define K210_FRAME_HEAD2                  0x55U
#define K210_FRAME_SIZE                   7U

/*
 * ============================================================================
 * K210发送给STM32：旧版单结果命令
 * ============================================================================
 *
 * 这些命令暂时保留，用于兼容已经完成的程序。
 */

/* 单数字识别结果 */
#define K210_CMD_DIGIT_RESULT             0x01U

/* 普通目标中心点 */
#define K210_CMD_TARGET_POINT             0x02U

/* 激光点中心坐标 */
#define K210_CMD_LASER_POINT              0x03U

/* 普通目标有效状态 */
#define K210_CMD_TARGET_STATE             0x04U

/* K210心跳 */
#define K210_CMD_HEARTBEAT                0x05U

/*
 * ============================================================================
 * K210发送给STM32：多数字快照命令
 * ============================================================================
 *
 * 一次完整的多数字结果由以下帧组成：
 *
 *   DIGIT_SNAPSHOT_BEGIN
 *   DIGIT_SNAPSHOT_ITEM × N
 *   DIGIT_SNAPSHOT_END
 */

#define K210_CMD_DIGIT_SNAPSHOT_BEGIN     0x10U
#define K210_CMD_DIGIT_SNAPSHOT_ITEM      0x11U
#define K210_CMD_DIGIT_SNAPSHOT_END       0x12U

/*
 * ============================================================================
 * K210发送给STM32：通用视觉快照命令
 * ============================================================================
 *
 * 通用视觉结果可以用于：
 *
 *   颜色目标
 *   数字目标
 *   矩形或黑框
 *   圆形目标
 *   激光点
 *   线段或路径
 *   十字或路口
 *
 * 一次完整结果建议按照以下顺序发送：
 *
 *   VISION_BEGIN
 *
 *   对每个目标依次发送：
 *       VISION_TARGET_INFO
 *       VISION_TARGET_X
 *       VISION_TARGET_Y
 *       VISION_TARGET_SIZE
 *       VISION_TARGET_AREA
 *
 *   VISION_END
 *
 * 第一阶段可以暂时只发送：
 *
 *   VISION_BEGIN
 *   VISION_TARGET_INFO
 *   VISION_TARGET_X
 *   VISION_TARGET_Y
 *   VISION_END
 */

#define K210_CMD_VISION_BEGIN             0x20U
#define K210_CMD_VISION_TARGET_INFO       0x21U
#define K210_CMD_VISION_TARGET_X          0x22U
#define K210_CMD_VISION_TARGET_Y          0x23U
#define K210_CMD_VISION_TARGET_SIZE       0x24U
#define K210_CMD_VISION_TARGET_AREA       0x25U
#define K210_CMD_VISION_END               0x26U


/*
 * ============================================================================
 * K210发送给STM32：道路视觉状态命令
 * ============================================================================
 *
 * ROAD_LINE_STATE：
 *   DATA1：红线中心X压缩值，0～255，对应QVGA横坐标0～319；
 *   DATA2：有符号循迹误差加128；
 *   DATA3：
 *       bit0：循迹结果有效；
 *       bit1：使用NEAR区域；
 *       bit2：使用MID后备区域；
 *       bit3：当前处于十字路口；
 *       bit4：当前处于停车区。
 *
 * ROAD_EVENT_STATE：
 *   DATA1：累计十字路口数量；
 *   DATA2：累计停车区数量；
 *   DATA3：K210_RoadEvent_t。
 */

#define K210_CMD_ROAD_LINE_STATE          0x30U
#define K210_CMD_ROAD_EVENT_STATE         0x31U

#define K210_CMD_SET_ROAD_PROFILE         0x40U

/*
 * ============================================================================
 * STM32发送给K210：控制命令
 * ============================================================================
 */

#define K210_CMD_START_DETECT             0x81U
#define K210_CMD_STOP_DETECT              0x82U
#define K210_CMD_SET_MODE                 0x83U

/*
 * ============================================================================
 * 数量限制
 * ============================================================================
 */

/*
 * STM32端一次最多缓存8个数字。
 *
 * 这是通信缓存容量，不代表画面中必须存在8个数字。
 */
#define K210_MAX_DIGITS                   8U

/*
 * STM32端一次最多缓存8个通用视觉目标。
 */
#define K210_MAX_VISION_TARGETS           8U

/*
 * ============================================================================
 * 旧版目标状态
 * ============================================================================
 */

#define K210_TARGET_LOST                  0U
#define K210_TARGET_VALID                 1U

/*
 * ============================================================================
 * 多数字识别结果状态
 * ============================================================================
 */

#define K210_RESULT_EMPTY                 0U
#define K210_RESULT_NORMAL                1U
#define K210_RESULT_AMBIGUOUS             2U
#define K210_RESULT_OVERFLOW              3U

/*
 * ============================================================================
 * 通用视觉模式
 * ============================================================================
 *
 * mode表示K210当前正在运行哪一种识别算法。
 */

typedef enum {
    K210_VISION_MODE_NONE = 0U,

    /* 颜色或色块识别 */
    K210_VISION_MODE_COLOR = 1U,

    /* 数字识别 */
    K210_VISION_MODE_DIGIT = 2U,

    /* 矩形或黑框识别 */
    K210_VISION_MODE_RECT = 3U,

    /* 圆形识别 */
    K210_VISION_MODE_CIRCLE = 4U,

    /* 红色激光点识别 */
    K210_VISION_MODE_LASER = 5U,

    /* 路线、黑线或直线识别 */
    K210_VISION_MODE_LINE = 6U,

    /* 十字、路口或交叉点识别 */
    K210_VISION_MODE_CROSS = 7U,

    /* 现场临时扩展模式 */
    K210_VISION_MODE_CUSTOM = 8U
} K210_VisionMode_t;


/*
 * ============================================================================
 * 道路循迹来源
 * ============================================================================
 */

typedef enum {
    K210_LINE_SOURCE_NONE = 0U,
    K210_LINE_SOURCE_NEAR = 1U,
    K210_LINE_SOURCE_MID = 2U
} K210_LineSource_t;

/*
 * ============================================================================
 * 道路事件
 * ============================================================================
 */

typedef enum {
    K210_ROAD_EVENT_NONE = 0U,
    K210_ROAD_EVENT_CROSS_ENTER = 1U,
    K210_ROAD_EVENT_CROSS_LEAVE = 2U,
    K210_ROAD_EVENT_STOP_ENTER = 3U,
    K210_ROAD_EVENT_STOP_LEAVE = 4U
} K210_RoadEvent_t;

typedef enum {
    K210_ROAD_PROFILE_CURRENT = 0U,
    K210_ROAD_PROFILE_OLD = 1U,
    K210_ROAD_PROFILE_BRIGHT = 2U,
    K210_ROAD_PROFILE_DARK = 3U
} K210_RoadProfile_t;

/*
 * ============================================================================
 * 通用视觉目标类别
 * ============================================================================
 *
 * class_id表示当前目标属于哪一种类型。
 *
 * 例如：
 *
 *   mode     = K210_VISION_MODE_DIGIT
 *   class_id = K210_TARGET_CLASS_DIGIT
 *   value    = 5
 *
 * 表示当前运行数字识别算法，并识别到了数字5。
 */

typedef enum {
    K210_TARGET_CLASS_NONE = 0U,

    /* 普通颜色色块 */
    K210_TARGET_CLASS_COLOR_BLOB = 1U,

    /* 数字目标 */
    K210_TARGET_CLASS_DIGIT = 2U,

    /* 矩形或黑框 */
    K210_TARGET_CLASS_RECT = 3U,

    /* 圆形 */
    K210_TARGET_CLASS_CIRCLE = 4U,

    /* 激光点 */
    K210_TARGET_CLASS_LASER = 5U,

    /* 线段或路径 */
    K210_TARGET_CLASS_LINE = 6U,

    /* 十字或路口 */
    K210_TARGET_CLASS_CROSS = 7U,

    /* 停车区域 */
    K210_TARGET_CLASS_PARKING = 8U,

    /* 钢球或球形目标 */
    K210_TARGET_CLASS_BALL = 9U,

    /* 管口、圆孔或投放口 */
    K210_TARGET_CLASS_PORT = 10U,

    /* 现场临时扩展目标 */
    K210_TARGET_CLASS_CUSTOM = 15U
} K210_TargetClass_t;

/*
 * ============================================================================
 * 通用颜色编号
 * ============================================================================
 *
 * 当class_id为K210_TARGET_CLASS_COLOR_BLOB时，
 * value可以使用以下颜色编号。
 */

typedef enum {
    K210_COLOR_NONE = 0U,
    K210_COLOR_RED = 1U,
    K210_COLOR_BLUE = 2U,
    K210_COLOR_GREEN = 3U,
    K210_COLOR_BLACK = 4U,
    K210_COLOR_WHITE = 5U,
    K210_COLOR_YELLOW = 6U,
    K210_COLOR_ORANGE = 7U,
    K210_COLOR_PURPLE = 8U
} K210_ColorId_t;

/*
 * ============================================================================
 * 通用视觉结果状态
 * ============================================================================
 */

typedef enum {
    /*
     * 正常识别到目标。
     *
     * target_count应大于0。
     */
    K210_VISION_RESULT_OK = 0U,

    /*
     * 当前没有检测到目标。
     *
     * target_count应等于0。
     */
    K210_VISION_RESULT_NO_TARGET = 1U,

    /*
     * 结果不稳定或存在歧义。
     */
    K210_VISION_RESULT_AMBIGUOUS = 2U,

    /*
     * 目标数量超过通信缓存容量。
     */
    K210_VISION_RESULT_OVERFLOW = 3U,

    /*
     * K210算法或模型运行错误。
     */
    K210_VISION_RESULT_ERROR = 4U
} K210_VisionResultStatus_t;

/*
 * ============================================================================
 * 多数字识别结构
 * ============================================================================
 */

/*
 * 一个数字的识别结果。
 *
 * digit：
 *   数字类别，当前有效范围为1～8。
 *
 * confidence：
 *   置信度，范围为0～100。
 *
 * center_x：
 *   数字框中心点在QVGA画面中的横坐标，
 *   范围为0～319。
 */
typedef struct {
    uint8_t digit;
    uint8_t confidence;
    uint16_t center_x;
} K210_DigitItem_t;

/*
 * 一次完整的多数字识别快照。
 *
 * sequence：
 *   快照序号。
 *
 * status：
 *   K210_RESULT_EMPTY、
 *   K210_RESULT_NORMAL、
 *   K210_RESULT_AMBIGUOUS或
 *   K210_RESULT_OVERFLOW。
 *
 * count：
 *   本次快照包含的有效数字数量。
 *
 * items：
 *   按画面横坐标从左到右排列。
 */
typedef struct {
    uint8_t sequence;
    uint8_t status;
    uint8_t count;

    K210_DigitItem_t items[K210_MAX_DIGITS];
} K210_DigitSnapshot_t;

/*
 * ============================================================================
 * 通用视觉结果结构
 * ============================================================================
 */

/*
 * 单个通用视觉目标。
 *
 * class_id：
 *   目标类别，例如数字、矩形、圆形或激光点。
 *
 * value：
 *   目标附加值。
 *
 *   数字模式：
 *       value为数字1～8。
 *
 *   颜色模式：
 *       value为K210_ColorId_t。
 *
 *   其他模式：
 *       没有附加值时设为0。
 *
 * confidence：
 *   识别置信度，范围为0～100。
 *
 * center_x、center_y：
 *   目标中心坐标。
 *
 *   QVGA模式下：
 *       center_x范围为0～319；
 *       center_y范围为0～239。
 *
 * width、height：
 *   目标外接框宽度和高度。
 *
 * area：
 *   目标面积。
 *
 *   对色块通常使用blob.pixels()；
 *   对矩形也可以使用width × height；
 *   对激光点可以使用光斑像素数。
 */
typedef struct {
    uint8_t class_id;
    uint8_t value;
    uint8_t confidence;
    uint8_t reserved;

    uint16_t center_x;
    uint16_t center_y;

    uint16_t width;
    uint16_t height;

    uint32_t area;
} K210_VisionTarget_t;

/*
 * 一次完整的通用视觉识别结果。
 *
 * sequence：
 *   快照序号。
 *
 * mode：
 *   当前K210视觉工作模式。
 *
 * status：
 *   K210_VisionResultStatus_t。
 *
 * target_count：
 *   当前有效目标数量。
 *
 * targets：
 *   目标数组。
 *
 *   建议K210发送前先按横坐标从左到右排序。
 */
typedef struct {
    uint8_t sequence;
    uint8_t mode;
    uint8_t status;
    uint8_t target_count;

    K210_VisionTarget_t targets[K210_MAX_VISION_TARGETS];
} K210_VisionResult_t;


/*
 * ============================================================================
 * 道路视觉状态
 * ============================================================================
 */

typedef struct {
    /* 解码后的QVGA横坐标，范围0～319 */
    uint16_t line_x;

    /* K210发送的循迹误差，通常为line_x - 160 */
    int16_t line_error;

    /* 0：无效；1：有效 */
    uint8_t line_valid;

    /* K210_LineSource_t */
    uint8_t line_source;

    /* 0：不在十字路口；1：处于十字路口 */
    uint8_t cross_valid;

    /* 0：不在停车区；1：处于停车区 */
    uint8_t stop_valid;

    /* 累计十字路口数量 */
    uint8_t cross_count;

    /* 累计停车区数量 */
    uint8_t stop_count;

    /* 最近一次道路事件，K210_RoadEvent_t */
    uint8_t road_event;

    /* 收到新的道路循迹状态 */
    uint8_t new_line_state;

    /* 收到新的非NONE道路事件 */
    uint8_t new_road_event;
} K210_RoadState_t;

/*
 * ============================================================================
 * K210通信状态和诊断信息
 * ============================================================================
 */

typedef struct {
    /*
     * K210在线状态：
     *
     *   0：离线
     *   1：在线
     */
    uint8_t online;

    /*
     * 当前通用视觉工作模式。
     *
     * 只有接入通用视觉协议后才会更新。
     */
    uint8_t vision_mode;

    /*
     * 是否收到新的通用视觉结果。
     *
     *   0：没有新结果
     *   1：有新结果
     */
    uint8_t new_vision_result;

    /*
     * 保留字段，用于结构体对齐和后续扩展。
     */
    uint8_t reserved;

    /*
     * ------------------------------------------------------------------------
     * 兼容旧版单数字协议
     * ------------------------------------------------------------------------
     */

    uint8_t digit;
    uint8_t digit_valid;
    uint8_t digit_confidence;
    uint8_t new_digit;

    /*
     * ------------------------------------------------------------------------
     * 兼容旧版普通目标中心点协议
     * ------------------------------------------------------------------------
     */

    uint16_t target_x;
    uint16_t target_y;
    uint8_t target_valid;
    uint8_t new_target;

    /*
     * ------------------------------------------------------------------------
     * 兼容旧版激光点协议
     * ------------------------------------------------------------------------
     */

    uint16_t laser_x;
    uint16_t laser_y;
    uint8_t laser_valid;
    uint8_t new_laser;


    /*
     * ------------------------------------------------------------------------
     * 道路视觉状态
     * ------------------------------------------------------------------------
     */

    K210_RoadState_t road;

    /* 道路参数档位发送状态 */
    uint8_t selected_road_profile;
    uint8_t road_profile_tx_remaining;
    uint8_t road_profile_last_tx_ok;
    uint8_t road_profile_reserved;
    uint32_t road_profile_tx_count;
    uint32_t road_profile_tx_busy_count;

    /*
     * ------------------------------------------------------------------------
     * 通信统计
     * ------------------------------------------------------------------------
     */

    /* 校验正确的固定帧总数 */
    uint32_t valid_frame_count;

    /* 校验和错误次数 */
    uint32_t checksum_error_count;

    /* 命令、参数或字段格式错误次数 */
    uint32_t format_error_count;

    /*
     * 旧版多数字快照错误次数。
     *
     * 包括：
     *   BEGIN或END不匹配；
     *   ITEM顺序错误；
     *   数量错误；
     *   序号错误。
     */
    uint32_t snapshot_error_count;

    /* 成功提交的旧版多数字快照数量 */
    uint32_t snapshot_count;

    /* 旧版多数字快照覆盖次数 */
    uint32_t snapshot_overwrite_count;

    /*
     * 通用视觉快照错误次数。
     */
    uint32_t vision_error_count;

    /*
     * 成功提交的通用视觉快照数量。
     */
    uint32_t vision_result_count;

    /*
     * 通用视觉快照覆盖次数。
     *
     * 当上一份新结果未被应用层读取时，
     * 新结果覆盖旧结果。
     */
    uint32_t vision_overwrite_count;

    /*
     * 通信超时次数。
     *
     * 每次从在线变为离线时加1。
     */
    uint32_t timeout_count;

    /*
     * 最后一次收到校验正确帧的系统时间。
     *
     * 单位：ms。
     */
    uint32_t last_rx_ms;
} K210_Comm_Info_t;

/*
 * ============================================================================
 * 初始化与周期任务
 * ============================================================================
 */

/*
 * 初始化K210协议层。
 *
 * USART2硬件由BSP_InitAll()统一初始化。
 *
 * 本函数负责：
 *   1. 清空协议状态；
 *   2. 清空帧接收缓存；
 *   3. 清空数字快照缓存；
 *   4. 清空通用视觉快照缓存；
 *   5. 清空USART2软件接收缓存。
 */
void K210_Comm_Init(void);

/*
 * K210通信周期任务。
 *
 * 建议在APP/app_task_config.h中以5ms周期注册：
 *
 *   { K210_Comm_Update, 5U, 0U },
 */
void K210_Comm_Update(void);

/*
 * ============================================================================
 * 通信状态读取接口
 * ============================================================================
 */

/*
 * 获取当前完整的K210通信状态。
 *
 * 返回值：
 *   BSP_OK：
 *       读取成功。
 *
 *   BSP_PARAM：
 *       info为空指针。
 */
BSP_Status_t K210_Comm_GetInfo(K210_Comm_Info_t *info);

/*
 * ============================================================================
 * 旧版单数字读取接口
 * ============================================================================
 */

/*
 * 获取一个新的旧版单数字结果。
 *
 * 返回值：
 *   BSP_OK：
 *       读取到新结果。
 *
 *   BSP_BUSY：
 *       当前没有新结果。
 *
 *   BSP_PARAM：
 *       参数中存在空指针。
 */
BSP_Status_t K210_Comm_GetNewDigit(uint8_t *digit,
                                   uint8_t *valid,
                                   uint8_t *confidence);

/*
 * ============================================================================
 * 多数字快照读取接口
 * ============================================================================
 */

/*
 * 读取当前有效数字。
 *
 * digits至少需要提供K210_MAX_DIGITS个uint8_t空间。
 *
 * 识别结果按照画面中的左右顺序写入digits。
 *
 * 本接口不会消耗K210_Comm_GetNewSnapshot()使用的新快照标志，
 * 因此可以同时用于业务控制和通信日志。
 *
 * 返回值：
 *   1～K210_MAX_DIGITS：
 *       当前有效数字个数。
 *
 *   0：
 *       K210离线、当前没有正常数字结果，
 *       或传入参数无效。
 */
uint8_t K210_Comm_ReadDigits(
    uint8_t digits[K210_MAX_DIGITS]
);

/*
 * 获取一份新的完整数字快照。
 *
 * 返回值：
 *   BSP_OK：
 *       成功读取新快照。
 *
 *   BSP_BUSY：
 *       当前没有新快照。
 *
 *   BSP_PARAM：
 *       snapshot为空指针。
 */
BSP_Status_t K210_Comm_GetNewSnapshot(
    K210_DigitSnapshot_t *snapshot
);

/*
 * ============================================================================
 * 旧版目标坐标读取接口
 * ============================================================================
 */

/*
 * 获取新的普通目标中心坐标。
 */
BSP_Status_t K210_Comm_GetNewTarget(uint16_t *x,
                                    uint16_t *y,
                                    uint8_t *valid);

/*
 * 获取新的激光点中心坐标。
 */
BSP_Status_t K210_Comm_GetNewLaser(uint16_t *x,
                                   uint16_t *y,
                                   uint8_t *valid);

/*
 * ============================================================================
 * 通用视觉结果读取接口
 * ============================================================================
 */

/*
 * 获取一份新的完整通用视觉结果。
 *
 * 读取成功后会清除“新结果”标志。
 *
 * 返回值：
 *   BSP_OK：
 *       成功读取新结果。
 *
 *   BSP_BUSY：
 *       当前没有新结果。
 *
 *   BSP_PARAM：
 *       result为空指针。
 */
BSP_Status_t K210_Comm_GetNewVisionResult(
    K210_VisionResult_t *result
);

/*
 * 读取最近一次已经完整提交的通用视觉结果。
 *
 * 本接口不会清除“新结果”标志，
 * 可以供小车控制任务周期读取。
 *
 * 返回值：
 *   BSP_OK：
 *       成功读取最近结果。
 *
 *   BSP_BUSY：
 *       尚未收到任何完整通用视觉结果。
 *
 *   BSP_PARAM：
 *       result为空指针。
 */
BSP_Status_t K210_Comm_ReadLatestVisionResult(
    K210_VisionResult_t *result
);

/*
 * 获取最近一次通用视觉结果中的主目标。
 *
 * 默认将targets[0]作为主目标。
 *
 * K210发送多个目标前，建议按照以下规则排序：
 *
 *   数字模式：
 *       按center_x从左到右排序。
 *
 *   跟踪模式：
 *       将主要跟踪目标放在targets[0]。
 *
 * 返回值：
 *   BSP_OK：
 *       成功读取主目标。
 *
 *   BSP_BUSY：
 *       当前没有有效目标。
 *
 *   BSP_PARAM：
 *       target为空指针。
 */
BSP_Status_t K210_Comm_ReadPrimaryTarget(
    K210_VisionTarget_t *target
);


/*
 * ============================================================================
 * 道路视觉状态读取接口
 * ============================================================================
 */

/*
 * 读取最近一次道路视觉状态，不清除新数据标志。
 */
BSP_Status_t K210_Comm_ReadRoadState(
    K210_RoadState_t *road
);

/*
 * 获取一份新的道路循迹状态。
 *
 * 读取成功后清除new_line_state。
 */
BSP_Status_t K210_Comm_GetNewRoadLineState(
    K210_RoadState_t *road
);

/*
 * 获取一个新的道路事件。
 *
 * 读取成功后清除new_road_event。
 */
BSP_Status_t K210_Comm_GetNewRoadEvent(
    uint8_t *cross_count,
    uint8_t *stop_count,
    uint8_t *road_event
);

/*
 * ============================================================================
 * STM32向K210发送控制帧
 * ============================================================================
 */

/*
 * STM32通过USART2向K210发送固定7字节帧。
 */
BSP_Status_t K210_Comm_SendFrame(uint8_t command,
                                 uint8_t data1,
                                 uint8_t data2,
                                 uint8_t data3);

/*
 * 请求K210开始识别。
 *
 * mode使用K210_VisionMode_t。
 */
BSP_Status_t K210_Comm_StartDetect(uint8_t mode);

/*
 * 请求K210停止识别。
 */
BSP_Status_t K210_Comm_StopDetect(void);

/*
 * 请求K210切换识别模式。
 *
 * mode使用K210_VisionMode_t。
 */
BSP_Status_t K210_Comm_SetMode(uint8_t mode);
BSP_Status_t K210_Comm_SendRoadProfile(uint8_t profile_id);
BSP_Status_t K210_Comm_SelectRoadProfile(uint8_t profile_id);
void K210_Comm_RestartRoadProfileSync(void);

#ifdef __cplusplus
}
#endif

#endif /* __K210_COMM_H */