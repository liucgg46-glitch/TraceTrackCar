#include "project_critical.h"

uint32_t Project_EnterCritical(void)
{
    return 0U;
}

void Project_ExitCritical(uint32_t state)
{
    (void)state;
}
