#ifndef __TEST_K210_COMM_H
#define __TEST_K210_COMM_H

#include "k210_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * K210专项通信测试公共接口。
 * 单数字、多数字、道路、视觉和配置档位测试均集中在test_k210_comm.c。
 */
extern volatile K210_Comm_Info_t g_k210_digit_debug_info;
extern volatile K210_Comm_Info_t g_k210_road_debug_info;
extern volatile uint8_t g_k210_profile_test_selected;
extern volatile uint8_t g_k210_profile_test_remaining;
extern volatile uint32_t g_k210_profile_test_tx_count;
extern volatile uint32_t g_k210_profile_test_busy_count;

void Test_K210_DigitCommUpdate(void);
void Test_K210_RoadCommUpdate(void);
void Test_K210_VisionCommUpdate(void);
void Test_K210_RoadProfileUpdate(void);
void Test_K210_SingleDigitCommUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_K210_COMM_H */
