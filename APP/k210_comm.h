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
 * STM32接口：
 *   USART2_TX：PA2
 *   USART2_RX：PA3
 *
 * 当前第一阶段接线：
 *   K210 IO6（TX） -> STM32 PA3（RX）
 *   K210 GND       -> STM32 GND
 *
 * 串口参数：
 *   波特率：115200
 *   数据位：8位
 *   校验位：无
 *   停止位：1位
 *
 * 固定帧格式：
 *   [0] 0xAA
 *   [1] 0x55
 *   [2] CMD
 *   [3] DATA1
 *   [4] DATA2
 *   [5] DATA3
 *   [6] CHECKSUM
 *
 * CHECKSUM为前6个字节累加和的低8位。
 */

/* 固定帧参数 */
#define K210_FRAME_HEAD1              0xAAU
#define K210_FRAME_HEAD2              0x55U
#define K210_FRAME_SIZE               7U

/*
 * K210发送给STM32的旧版单结果命令。
 * 当前多数字识别主要使用后面的快照命令。
 */
#define K210_CMD_DIGIT_RESULT         0x01U
#define K210_CMD_TARGET_POINT         0x02U
#define K210_CMD_LASER_POINT          0x03U
#define K210_CMD_TARGET_STATE         0x04U
#define K210_CMD_HEARTBEAT            0x05U

/*
 * K210发送给STM32的可变数量数字快照命令。
 *
 * 一次完整结果由以下帧组成：
 *   1个BEGIN帧
 *   0个或多个ITEM帧
 *   1个END帧
 */
#define K210_CMD_DIGIT_SNAPSHOT_BEGIN 0x10U
#define K210_CMD_DIGIT_SNAPSHOT_ITEM  0x11U
#define K210_CMD_DIGIT_SNAPSHOT_END   0x12U

/* 一次数字识别快照的状态 */
#define K210_RESULT_EMPTY             0U
#define K210_RESULT_NORMAL            1U
#define K210_RESULT_AMBIGUOUS         2U
#define K210_RESULT_OVERFLOW          3U

/*
 * STM32端一次最多缓存8个数字。
 * 这只是通信缓存容量，不表示实际画面必须存在8个数字。
 */
#define K210_MAX_DIGITS               8U

/* STM32发送给K210的控制命令，第一阶段暂时不用 */
#define K210_CMD_START_DETECT         0x81U
#define K210_CMD_STOP_DETECT          0x82U
#define K210_CMD_SET_MODE             0x83U

/* 目标状态 */
#define K210_TARGET_LOST              0U
#define K210_TARGET_VALID             1U

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
 *   数字框中心点在QVGA画面中的横坐标，范围为0～319。
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
 *   快照序号，用于判断BEGIN和END是否属于同一次结果。
 *
 * status：
 *   K210_RESULT_EMPTY、NORMAL、AMBIGUOUS或OVERFLOW。
 *
 * count：
 *   本次快照包含的有效数字数量。
 *
 * items：
 *   按画面横坐标从左到右排列的数字结果。
 */
typedef struct {
    uint8_t sequence;
    uint8_t status;
    uint8_t count;
    K210_DigitItem_t items[K210_MAX_DIGITS];
} K210_DigitSnapshot_t;

/* K210通信状态和诊断信息 */
typedef struct {
    /*
     * 在线状态：
     *   0：离线
     *   1：在线
     */
    uint8_t online;

    /* 兼容旧版单数字协议的结果 */
    uint8_t digit;
    uint8_t digit_valid;
    uint8_t digit_confidence;
    uint8_t new_digit;

    /* 兼容目标中心点协议的结果 */
    uint16_t target_x;
    uint16_t target_y;
    uint8_t target_valid;
    uint8_t new_target;

    /* 兼容激光点协议的结果 */
    uint16_t laser_x;
    uint16_t laser_y;
    uint8_t laser_valid;
    uint8_t new_laser;

    /* 通信统计信息 */
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t format_error_count;
    uint32_t snapshot_error_count;
    uint32_t snapshot_count;
    uint32_t snapshot_overwrite_count;
    uint32_t last_rx_ms;
} K210_Comm_Info_t;

/*
 * 初始化K210协议层。
 *
 * USART2硬件由BSP_InitAll()统一初始化，
 * 本函数只初始化协议状态机和数据缓存。
 */
void K210_Comm_Init(void);

/*
 * K210通信周期任务。
 *
 * 建议在APP/app_task_config.h中以5ms周期注册：
 *   { K210_Comm_Update, 5U, 0U },
 */
void K210_Comm_Update(void);

/* 获取当前完整的通信状态 */
BSP_Status_t K210_Comm_GetInfo(K210_Comm_Info_t *info);

/*
 * 获取一个新的旧版单数字结果。
 *
 * 返回值：
 *   BSP_OK：读取到新结果
 *   BSP_BUSY：没有新结果
 *   BSP_PARAM：传入空指针
 */
BSP_Status_t K210_Comm_GetNewDigit(uint8_t *digit,
                                   uint8_t *valid,
                                   uint8_t *confidence);

/*
 * 获取一个已经完整接收并提交的多数字快照。
 *
 * 返回值：
 *   BSP_OK：成功读取到一个新快照
 *   BSP_BUSY：当前没有新快照
 *   BSP_PARAM：传入空指针
 */
BSP_Status_t K210_Comm_GetNewSnapshot(
    K210_DigitSnapshot_t *snapshot
);

/* 获取新的目标中心坐标 */
BSP_Status_t K210_Comm_GetNewTarget(uint16_t *x,
                                    uint16_t *y,
                                    uint8_t *valid);

/* 获取新的激光点坐标 */
BSP_Status_t K210_Comm_GetNewLaser(uint16_t *x,
                                   uint16_t *y,
                                   uint8_t *valid);

/* STM32通过USART2向K210发送一个固定7字节帧 */
BSP_Status_t K210_Comm_SendFrame(uint8_t command,
                                 uint8_t data1,
                                 uint8_t data2,
                                 uint8_t data3);

/* 请求K210开始识别，第一阶段暂时不用 */
BSP_Status_t K210_Comm_StartDetect(uint8_t mode);

/* 请求K210停止识别，第一阶段暂时不用 */
BSP_Status_t K210_Comm_StopDetect(void);

/* 请求K210切换识别模式，第一阶段暂时不用 */
BSP_Status_t K210_Comm_SetMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __K210_COMM_H */