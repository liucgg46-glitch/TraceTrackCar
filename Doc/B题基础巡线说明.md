# B题基础巡线正式程序说明

## 1. 分层边界

本版本严格区分整车任务、赛道巡线和通用算法：

| 层级 | 文件 | 责任 |
| --- | --- | --- |
| 整车任务层 | `APP/task_profile_b_basic.c/.h` | KEY1启动、KEY4停止、运行监控、故障停车、到达鸣笛 |
| 任务选择层 | `APP/task_profile_config.h`、`APP/task_profile_select.c` | 选择并调用B题正式整车任务状态机 |
| 赛道层 | `Route/route_profile_b_basic.c/.h` | 直角、虚线、三角尖头、终点停止线等赛道事件 |
| 通用循迹层 | `Algorithm/line_track.c` | 普通黑线PD循迹、按误差减速和通用丢线保护 |
| 底盘层 | `APP/chassis.c` | 左右轮目标分配和速度闭环 |
| 测试层 | `Test` | 仅专项测试；正式任务表不注册任何`Test_*`函数 |

`Route`中的枚举只表示当前处于哪个赛道路段，不负责整车开始、完成、鸣笛和故障流程。整车生命周期全部放在`APP/task_profile_b_basic.c`。

## 2. 当前编译选择

### 整车任务状态机

`APP/task_profile_config.h`：

```c
#define TASK_PROFILE_B_BASIC  2U
#define TASK_PROFILE_SELECT   TASK_PROFILE_B_BASIC
```

### 赛道方案

`Route/route_config.h`：

```c
#define ROUTE_PROFILE_B_BASIC  2U
#define ROUTE_PROFILE_SELECT   ROUTE_PROFILE_B_BASIC
```

选择层会进行编译期配对检查。B题整车任务状态机只能与B题基础巡线赛道同时使用。

## 3. 正式整车任务状态

`APP/task_profile_b_basic.c`包含以下整车状态：

```text
WAIT_START
    KEY1
      ↓
RUNNING
      ↓ Route上报arrived
ARRIVAL_BUZZER
      ↓ 600 ms
COMPLETE

任意运行故障 → FAULT
任意状态按KEY4 → 安全停车并回到WAIT_START
```

具体责任：

- 上电后先确认KEY1和KEY4稳定松开，防止上电误启动；
- KEY1调用`LineFollow_Start()`启动当前B题赛道；
- 运行期间检查Route到达标志、Route错误和底盘故障；
- 到达后由APP层保持停车并调用蜂鸣器驱动鸣响600 ms；
- KEY4在所有状态下优先安全停车；
- 故障状态不自动重新启动，按KEY4复位后才允许再次运行。

蜂鸣器底层为低电平有效，但任务状态机只调用`Drv_Buzzer_On()`和`Drv_Buzzer_Off()`，不直接操作PG7电平。

## 4. 正式任务表

`APP/app_task_config.h`只注册基础要求第（1）项需要的任务：

```c
{ AppDiagnostics_HeartbeatUpdate, 10U, 0U },
{ AppTask_BSP_Background, 1U, 0U },
{ Key_Update, 10U, 0U },
{ Sensor_Update, 1U, 0U },
{ Encoder_Update, 10U, 0U },
{ TaskProfile_Update, 10U, 0U },
{ LineTrack_Update, 10U, 0U },
{ Motion_Update, 10U, 0U },
{ Chassis_Update, 10U, 0U },
```

没有注册：

- `Test_RouteCmd_Update`；
- `Test_RouteLog`；
- `Test_TaskFSM_Log`；
- K210解析；
- 称重配送业务；
- LCD/OLED测试刷新。

`Motion_Update`只用于直角的按角度粗转，属于B题基础赛道必要任务；三角尖头仍由Route根据灰度重新压线结果完成，不使用固定角度。

## 5. 赛道巡线逻辑

当前灰度输入默认来自Yahboom 8路串口灰度模块。驱动输出给上层的数值已按实测极性归一化：白底为高模拟量，黑线为低模拟量，循迹算法继续按`raw < threshold`判断压线。

普通路段继续使用原参数：

```c
#define CONTROL_LINE_BASE_SPEED_CPS       2500
#define CONTROL_LINE_CROSS_SPEED_CPS      2000
#define CONTROL_LINE_MIN_TRACK_SPEED_CPS  2000
```

### 5.1 直角

虚线之前的直角由Route层单独处理：

1. 识别`LEFT_BRANCH`或`RIGHT_BRANCH`；
2. 放宽侧边图案：左侧或右侧至少两路压线即可作为明确入口；
3. 车身带偏角时，允许先记录最外侧弱特征；
4. 弱特征后快速丢线，仍按记录方向进入直角转弯，不进入虚线探测；
5. Route提交`ROUTE_ACTION_TURN_ANGLE`，Motion按`B_ROUTE_CORNER_TURN_ANGLE_DEG`完成角度粗转；
6. 粗转结束后，以`B_ROUTE_CORNER_REACQUIRE_TURN_CPS`沿同方向低速补转；
7. 中间探头连续重新压线后复位普通PD并继续巡线。

`B_ROUTE_CORNER_TURN_ANGLE_DEG`默认设置为70°，故意小于几何90°，为灰度补转保留余量。实车直角转得不足或过多时优先只调整这个宏。

### 5.2 虚线与三角尖头

直角处理优先于虚线判断。进入虚线后：

```text
稳定中线后丢线
→ 低速直行探测
    ├─ 短时间重新见线：虚线间隙
    └─ 持续丢线：三角尖头
→ 尖头固定向右回折
→ 重新压中线后继续巡线
```

首次确认虚线间隙后会关闭普通直角入口识别，避免三角斜边被误认为第三个直角。

三角尖头的具体控制量为`linear_cps=0`、`turn_cps=-B_ROUTE_TIP_TURN_CPS`，即原地向右转。转动至少`B_ROUTE_TIP_TURN_MIN_MS`后才允许检测中线；连续`B_ROUTE_TIP_REACQUIRE_CONFIRM_SAMPLES`帧重新压住中线后恢复普通巡线，超过`B_ROUTE_TIP_TURN_TIMEOUT_MS`仍未找到线则进入路线错误。

### 5.3 终点

只有通过三角尖头并继续行驶规定距离后才允许识别终点。八路连续全黑达到确认帧数后，Route上报`arrived`并停止输出；APP任务状态机随后执行最终停车和鸣笛。

## 6. 操作

1. 上电后等待按键松开约100 ms；
2. 把小车放在起点；
3. 按KEY1启动；
4. 任意阶段按KEY4停止并回到等待状态；
5. 到达终点后停车，蜂鸣器鸣响约600 ms；
6. 完成状态再次按KEY1可以重新运行。
