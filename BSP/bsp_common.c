#include "bsp_common.h"
#include "project_critical.h"

uint32_t Project_EnterCritical(void)
{
    return BSP_EnterCritical();
}

void Project_ExitCritical(uint32_t state)
{
    BSP_ExitCritical(state);
}
