#ifndef __APP_DIAGNOSTICS_H
#define __APP_DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* 正式固件运行指示灯任务，建议以 10 ms 周期调用。 */
void AppDiagnostics_HeartbeatUpdate(void);

/* Medicine 状态机运行日志，建议以 200 ms 周期调用。 */
void AppDiagnostics_TaskFSMLogUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DIAGNOSTICS_H */
