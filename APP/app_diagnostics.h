#ifndef __APP_DIAGNOSTICS_H
#define __APP_DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* 正式固件运行指示灯任务，建议以10 ms周期调度。 */
void AppDiagnostics_HeartbeatUpdate(void);

/* H题总任务状态机串口日志，建议以200 ms周期调度。 */
void AppDiagnostics_TaskFSMLogUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DIAGNOSTICS_H */
