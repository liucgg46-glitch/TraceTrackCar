#ifndef __K210_COMM_H
#define __K210_COMM_H

#include <stdint.h>
#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * K210 <-> STM32 通用通信协议
 * ============================================================================
 *
 * STM32物理接口：
 *   USART2_TX：PA2
 *   USART2_RX：PA3
 *
 * 接线：
 *   K210 TX  -> STM32 PA3
 *   K210 RX  <- STM32 PA2
 *   K210 GND -> STM32 GND
 *
 * 串口参数：
 *   115200
 *   8位数据位
 *   无校验
 *   1位停止位
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
 * CHECKSUM：
 *   前6个字节累加和的低8位。
 */

/* 固定帧定义 */
#define K210_FRAME_HEAD1             0xAAU
#define K210_FRAME_HEAD2             0x55U
#define K210_FRAME_SIZE              7U

/*
 * K210 -> STM32命令。
 */
#define K210_CMD_DIGIT_RESULT        0x01U
#define K210_CMD_TARGET_POINT        0x02U
#define K210_CMD_LASER_POINT         0x03U
#define K210_CMD_TARGET_STATE        0x04U
#define K210_CMD_HEARTBEAT           0x05U

/*
 * STM32 -> K210命令。
 */
#define K210_CMD_START_DETECT        0x81U
#define K210_CMD_STOP_DETECT         0x82U
#define K210_CMD_SET_MODE            0x83U

/* 目标状态 */
#define K210_TARGET_LOST             0U
#define K210_TARGET_VALID            1U

/*
 * K210通信状态和数据缓存。
 */
typedef struct {
    /*
     * 在线状态：
     *   0：离线；
     *   1：在线。
     */
    uint8_t online;

    /*
     * 数字识别结果。
     */
    uint8_t digit;
    uint8_t digit_valid;
    uint8_t digit_confidence;
    uint8_t new_digit;

    /*
     * 目标中心坐标。
     */
    uint16_t target_x;
    uint16_t target_y;
    uint8_t target_valid;
    uint8_t new_target;

    /*
     * 激光点坐标。
     */
    uint16_t laser_x;
    uint16_t laser_y;
    uint8_t laser_valid;
    uint8_t new_laser;

    /*
     * 通信统计信息。
     */
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t format_error_count;
    uint32_t last_rx_ms;
} K210_Comm_Info_t;

/*
 * 初始化K210协议层。
 *
 * USART2底层已经由BSP_InitAll()统一初始化，
 * 本函数不会重复初始化USART2硬件。
 */
void K210_Comm_Init(void);

/*
 * 周期接收任务。
 *
 * 建议任务周期：
 *   5ms
 *
 * 任务表：
 *   { K210_Comm_Update, 5U, 0U },
 */
void K210_Comm_Update(void);

/*
 * 获取完整通信状态快照。
 */
BSP_Status_t K210_Comm_GetInfo(K210_Comm_Info_t *info);

/*
 * 读取新的数字识别结果。
 *
 * 返回值：
 *   BSP_OK：
 *     存在一条新的数字识别结果；
 *
 *   BSP_BUSY：
 *     当前没有新的数字结果；
 *
 *   BSP_PARAM：
 *     输入指针为空。
 *
 * 注意：
 *   valid=0表示K210发送了一条“识别无效”结果，
 *   但通信本身仍然正常，所以函数仍返回BSP_OK。
 */
BSP_Status_t K210_Comm_GetNewDigit(uint8_t *digit,
                                   uint8_t *valid,
                                   uint8_t *confidence);

/*
 * 读取新的目标中心坐标。
 */
BSP_Status_t K210_Comm_GetNewTarget(uint16_t *x,
                                    uint16_t *y,
                                    uint8_t *valid);

/*
 * 读取新的激光点坐标。
 */
BSP_Status_t K210_Comm_GetNewLaser(uint16_t *x,
                                   uint16_t *y,
                                   uint8_t *valid);

/*
 * STM32向K210发送一个通用数据帧。
 */
BSP_Status_t K210_Comm_SendFrame(uint8_t command,
                                 uint8_t data1,
                                 uint8_t data2,
                                 uint8_t data3);

/*
 * 请求K210开始识别。
 */
BSP_Status_t K210_Comm_StartDetect(uint8_t mode);

/*
 * 请求K210停止识别。
 */
BSP_Status_t K210_Comm_StopDetect(void);

/*
 * 请求K210切换视觉模式。
 */
BSP_Status_t K210_Comm_SetMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __K210_COMM_H */