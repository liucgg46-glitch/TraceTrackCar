#include "route_profile_basic.h"
#include "line_track.h"

void BasicRoute_Init(void)
{
    BasicRoute_Reset();
}

void BasicRoute_Reset(void)
{
}

void BasicRoute_Update(const LineDetect_Result_t *line, LineTrack_Output_t *out)
{
    LineTrack_Compute(line, out);
}