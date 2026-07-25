# B题基础巡线说明

## 1. 完成目标

本版本完成2026年通信系电赛模拟竞赛B题基本要求第（1）项：

- KEY1启动；
- 沿黑色引导线完成全程巡线；
- 平稳通过赛道前半段虚线；
- 对赛道右侧三角尖头执行专门的向右回折处理；
- 在终点八路全黑停止线停车；
- 停车后蜂鸣器鸣响600 ms；
- KEY4可随时人工停止。

## 2. 修改文件

| 文件 | 作用 |
| --- | --- |
| `Route/route_profile_b_basic.c/.h` | 新增B题基础巡线赛道状态机 |
| `Route/route_config.h` | 新增并选择`ROUTE_PROFILE_B_BASIC` |
| `Route/route_profile_select.c` | 把新赛道接入统一路线接口 |
| `APP/line_follow_app.c` | 区分到达和异常停车；到达后非阻塞鸣笛 |
| `DroneProject.uvprojx` | 把新路线源码加入Keil工程 |
| `Makefile` | 把新路线源码加入GCC构建清单 |

没有修改灰度通道方向、逐通道阈值、普通巡线PD参数、基础巡线速度、电机映射和引脚。

## 3. 虚线与三角尖头逻辑

题图中，三角尖头之前存在一段虚线。虚线间隙和尖头都可能让八路灰度短暂检测不到黑线，因此不能把“出现丢线”直接等同于“到达尖头”。

新路线使用两阶段判定：

```text
普通巡线
→ 连续确认中间探头稳定压线
→ 检测到连续丢线
→ 进入短距离直行探测
    ├─ 很快重新见线：判为虚线间隙，恢复普通巡线
    └─ 探测超时仍丢线：判为三角尖头
→ 停止前进并固定向右原地转向
→ 中间探头连续重新压住线路
→ 复位LineTrack旧误差
→ 恢复普通巡线
```

短距离探测使用`B_ROUTE_GAP_PROBE_CPS`，只在虚线间隙和尖头入口短暂使用。其余普通路段仍全部调用现有`LineTrack_Compute()`，速度保持仓库当前的2500 cps配置。

只有同时满足“已离开起点、达到最小里程、中线稳定后直接丢线”才进入探测状态。普通弯道、S弯和宽线仍交给原来的巡线算法。

## 4. 终点逻辑

终点判定只有在三角尖头已经通过后才启用，因此起点黑块、前半程虚线和其他宽线不会触发完成停车。

通过尖头后还要满足：

1. 至少继续行驶`B_ROUTE_FINISH_MIN_TRAVEL_AFTER_TIP_MM`；
2. 连续恢复正常中线；
3. 八路连续检测全黑达到`B_ROUTE_FINISH_BLACK_CONFIRM_SAMPLES`。

看到全黑第一帧时，路线输出立即改为零，确认成功后释放底盘控制权并鸣笛。蜂鸣器由现有驱动控制，PG7低电平有效；代码不直接操作GPIO电平。

## 5. 当前参数

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `B_ROUTE_TIP_IGNORE_MS` | 800 ms | 启动后的尖头识别屏蔽时间 |
| `B_ROUTE_TIP_MIN_TRAVEL_MM` | 200 mm | 虚线/尖头特殊识别最小行驶距离 |
| `B_ROUTE_TIP_LOST_CONFIRM_SAMPLES` | 2 | 进入直行探测前的连续丢线帧数 |
| `B_ROUTE_GAP_PROBE_CPS` | 1800 cps | 跨虚线和探测尖头时的短时直行速度 |
| `B_ROUTE_GAP_PROBE_MS` | 160 ms | 虚线重新见线等待时间 |
| `B_ROUTE_GAP_PROBE_MAX_MM` | 60 mm | 直行探测最大里程 |
| `B_ROUTE_GAP_REACQUIRE_CONFIRM_SAMPLES` | 2 | 虚线后重新见线确认帧数 |
| `B_ROUTE_TIP_TURN_CPS` | 1600 cps | 尖头原地右转速度 |
| `B_ROUTE_TIP_TURN_MIN_MS` | 220 ms | 允许重新捕获前的最短转向时间 |
| `B_ROUTE_TIP_TURN_TIMEOUT_MS` | 2200 ms | 尖头找线超时 |
| `B_ROUTE_FINISH_MIN_TRAVEL_AFTER_TIP_MM` | 250 mm | 通过尖头后终点识别屏蔽距离 |
| `B_ROUTE_FINISH_BLACK_CONFIRM_SAMPLES` | 3 | 终点全黑确认帧数 |
| `LINE_FOLLOW_ARRIVAL_BUZZER_MS` | 600 ms | 完成后的鸣笛时间 |

普通路段速度仍来自`Algorithm/control_config.h`：

```c
#define CONTROL_LINE_BASE_SPEED_CPS       2500
#define CONTROL_LINE_CROSS_SPEED_CPS      2000
#define CONTROL_LINE_MIN_TRACK_SPEED_CPS  2000
```

## 6. 上板测试顺序

1. 把车轮悬空，Rebuild确认0 Error；
2. 上电后确认蜂鸣器默认关闭；
3. 把车放在起点黑线上，按KEY1；
4. 观察前半段虚线：小车应短暂保持直行，重新见线后继续，不得在虚线处右转；
5. 观察普通弯道和S弯，速度与原基础巡线保持一致；
6. 到达三角尖头时，车辆先短暂向前探测，确认持续丢线后停止前进并向右原地转向；
7. 重新压线后继续巡线；
8. 终点八路全黑后应立即停车，并鸣笛约600 ms；
9. 任何阶段按KEY4都应停车，且不会触发完成鸣笛。

LCD/OLED路线测试页中：

- `P:2`表示B题基础路线；
- `PS:0`正常巡线，前往尖头；
- `PS:1`虚线/尖头直行探测；
- `PS:2`尖头向右转向；
- `PS:3`通过尖头后前往终点；
- `PS:4`终点全黑确认；
- `PS:5`已到达；
- `PS:6`路线错误。

## 7. 首次实车只允许优先调整的参数

若小车在虚线处误判尖头，应先增大`B_ROUTE_GAP_PROBE_MS`或`B_ROUTE_GAP_PROBE_MAX_MM`，一次只调整一个参数。

若跨越虚线时偏离过大，应适当降低`B_ROUTE_GAP_PROBE_CPS`，不要修改普通巡线PID方向。

若尖头处前冲距离过大，应减小`B_ROUTE_GAP_PROBE_MS`或`B_ROUTE_GAP_PROBE_MAX_MM`；但减小后必须重新确认虚线不会误判。

若尖头尚未转到新线路就恢复，应增大`B_ROUTE_TIP_TURN_MIN_MS`。

若尖头转向过慢或超时，应适当增大`B_ROUTE_TIP_TURN_CPS`，但不得超过底盘目标速度安全限幅。

若停止线偶发漏检，应先检查八路全黑是否真实成立和灰度阈值；不要直接取消连续确认。
