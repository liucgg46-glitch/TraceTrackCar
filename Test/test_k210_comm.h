#ifndef __TEST_K210_ROAD_COMM_H
#define __TEST_K210_ROAD_COMM_H

#include "k210_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 文件编码：
 *   UTF-8
 */

extern volatile K210_Comm_Info_t
    g_k210_road_debug_info;

void Test_K210_RoadCommUpdate(void);
void Test_K210_RoadProfileUpdate(void);
void Test_K210_SingleDigitCommUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_K210_ROAD_COMM_H */
