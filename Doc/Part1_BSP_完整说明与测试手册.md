# STM32F407 电赛小车 Part1 BSP 基础包：完整说明与测试手册

> 版本：Part1 BSP v1.1  
> 适用平台：STM32F407 / STM32F4 标准外设库 StdPeriph  
> 设计目标：全程非阻塞、可调参、便于 A/B/C 多人协作、换 F407 标准库项目时尽量只改 `.h` 配置区。

---

## 0. 这份文档给谁看

这份文档同时给三类人看：

1. **你自己**：隔一段时间回来能快速想起来每个文件怎么用。
2. **队友 A/B/C**：知道自己该实现哪个接口，不乱改别人模块。
3. **后续 AI**：看到这份文档后，可以直接理解工程分层、接口边界和继续写 Part2/Part3 代码。

本包是电赛小车代码框架的 **Part1：BSP 基础层**。它不写比赛任务，不写具体电机控制算法，不写循迹逻辑，只负责把 MCU 底层外设封装成可复用接口。

---

## 1. 总体设计原则

### 1.1 分层原则

本工程按下面边界划分：

```text
BSP 层：只管 MCU 外设本身
例如 GPIO、PWM、Encoder、ADC、UART、I2C、SPI、EXTI、SysTick

Driver 层：只管某个硬件模块怎么用
例如 drv_motor、drv_gray、drv_imu、drv_oled、drv_k210

Algorithm 层：只管算法
例如 PID、滤波、线检测、里程计、角度控制

App 层：只管任务调度、底盘控制、状态机、调参菜单
例如 scheduler、chassis、motion_action、task_fsm、debug_menu
```

**重要边界：**

```text
BSP 层只管“总线怎么收发 / 外设怎么读写”；
设备层再管“某个模块寄存器怎么读写 / 片选怎么拉 / 数据怎么解释”。
```

举例：

```text
bsp_i2c.c 只提供 I2C_MasterWriteRead()
mpu9250.c 才负责 WHO_AM_I、PWR_MGMT_1、ACCEL_XOUT_H 这些寄存器

bsp_spi.c 只提供 SPI_Transfer()
pmw3901.c 才负责 PMW3901 的 CS、寄存器地址、初始化序列

bsp_adc.c 只返回 ADC 原始值
drv_gray.c 才负责灰度阈值、黑白判断、线误差
```

---

### 1.2 非阻塞原则

全工程禁止在主循环、任务函数、回调函数里写：

```c
HAL_Delay(1000);
delay_ms(1000);
while (uart_rx_done == 0);
while (motion_done == 0);
while (key_pressed == 0);
for (volatile int i = 0; i < 1000000; i++);
```

允许在底层驱动的**调试/初始化阻塞 API**中短时间等待，例如 I2C 扫描、SPI 阻塞读写，但这些函数不能放进 10ms 速度环、循迹、任务状态机里反复调用。

主流程一律使用：

```text
BSP_GetTickMs() + Scheduler_Run() + Module_Update()
```

---

### 1.3 可移植原则

本包目标是：

```text
换 STM32F407 标准库项目时，主要只改 .h 配置区，不改 .c 逻辑。
```

例如：

```text
换 PWM 引脚：改 bsp_pwm.h
换编码器 TIM：改 bsp_encoder.h
换 UART 波特率和 DMA：改 bsp_uart.h
换 I2C 引脚：改 bsp_i2c.h
换 SPI 模式：改 bsp_spi.h
换按键引脚：改 bsp_key.h
```

`.c` 文件只有发现 bug 或扩展通用功能时才改。

---

## 2. 目录结构

```text
e_car_f407_part1_bsp_v1_1
│
├── BSP
│   ├── bsp_common.h
│   ├── bsp_systick.c / bsp_systick.h
│   ├── bsp_gpio.c    / bsp_gpio.h
│   ├── bsp_pwm.c     / bsp_pwm.h
│   ├── bsp_encoder.c / bsp_encoder.h
│   ├── bsp_adc.c     / bsp_adc.h
│   ├── bsp_key.c     / bsp_key.h
│   ├── bsp_exti.c    / bsp_exti.h
│   ├── bsp_uart.c    / bsp_uart.h
│   ├── bsp_i2c.c     / bsp_i2c.h
│   ├── bsp_spi.c     / bsp_spi.h
│   └── bsp_all.c     / bsp_all.h
│
├── App
│   ├── scheduler.c / scheduler.h
│   ├── app_task_config.h
│   ├── app_task_port.c / app_task_port.h
│   └── nb_wait.c / nb_wait.h
│
├── Examples
│   └── main_part1_bsp_test.c
│
└── Docs
    ├── Part1_BSP_使用说明.md
    ├── Part1_任务调度与对接说明.md
    └── Part1_BSP_完整说明与测试手册.md
```

---

## 3. Keil 工程导入方法

### 3.1 复制文件

把下面两个文件夹复制到你的 Keil 工程中：

```text
BSP
App
```

然后把所有 `.c` 文件加入 Keil 工程编译：

```text
BSP/bsp_systick.c
BSP/bsp_gpio.c
BSP/bsp_pwm.c
BSP/bsp_encoder.c
BSP/bsp_adc.c
BSP/bsp_key.c
BSP/bsp_exti.c
BSP/bsp_uart.c
BSP/bsp_i2c.c
BSP/bsp_spi.c
BSP/bsp_all.c

App/scheduler.c
App/app_task_port.c
App/nb_wait.c
```

### 3.2 Include Path

Keil 里添加 include 路径：

```text
BSP
App
```

### 3.3 标准库文件要求

本包是 STM32F4 标准外设库版本，不是 HAL 版本。工程中需要加入常用 StdPeriph 文件，例如：

```text
stm32f4xx_rcc.c
stm32f4xx_gpio.c
stm32f4xx_tim.c
stm32f4xx_adc.c
stm32f4xx_dma.c
stm32f4xx_usart.c
stm32f4xx_i2c.c
stm32f4xx_spi.c
stm32f4xx_exti.c
stm32f4xx_syscfg.c
misc.c
```

如果编译报 `undefined reference` 或 Keil 提示某个函数找不到，优先检查标准库 `.c` 是否没加。

---

## 4. main.c 推荐写法

Part1 阶段推荐 main.c 保持干净：

```c
#include "stm32f4xx.h"
#include "bsp_all.h"
#include "scheduler.h"

int main(void)
{
    SystemInit();

    if (BSP_InitAll(SystemCoreClock) != BSP_OK) {
        while (1) {
            /* 初始化失败：可以后续加错误灯闪烁 */
        }
    }

    Scheduler_Init();

    while (1) {
        Scheduler_Run();
    }
}
```

`main.c` 不写：

```text
PID
循迹判断
任务状态机细节
传感器寄存器读写
delay
while 等待事件
```

---

## 5. 任务调度与多人对接口

### 5.1 静态任务表位置

任务表在：

```text
App/app_task_config.h
```

当前默认任务表：

```c
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                      \
Task_t task_list[] = {                                                        \
    { AppTask_BSP_Background,  1U,   0U },  /* B：BSP 后台维护 */             \
    { Encoder_Update,         10U,   0U },  /* B：编码器速度更新 */           \
    { Sensor_Update,          10U,   0U },  /* C：传感器统一更新 */           \
    { Chassis_Update,         10U,   0U },  /* A：底盘速度闭环 */             \
    { LineTrack_Update,       10U,   0U },  /* A：循迹控制 */                 \
    { TaskFSM_Update,         10U,   0U },  /* A：任务状态机 */               \
    { DebugMenu_Update,       20U,   0U },  /* C：按键菜单/调参 */            \
    { OLED_Update,           100U,   0U },  /* C：OLED 刷新 */                \
    { Log_Update,            200U,   0U },  /* C：串口日志 */                 \
};                                                                            \
const uint8_t TASK_NUM = (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))
```

### 5.2 为什么用 task_list[]

多人协作时，最怕每个人自己往 main.c 里加任务、改周期、改函数名，最后对不上。静态任务表的好处是：

```text
1. 所有周期一眼能看见；
2. A/B/C 对接口固定；
3. 后续 AI 看到 app_task_config.h 就知道任务关系；
4. main.c 不用反复改；
5. 比赛现场只改任务表和参数，不乱动底层。
```

### 5.3 A/B/C 对接口

接口声明在：

```text
App/app_task_port.h
```

```c
void AppTask_BSP_Background(void);  /* B：UART/I2C/SPI 后台维护，1ms */
void Encoder_Update(void);          /* B：编码器增量/速度更新，10ms */
void Sensor_Update(void);           /* C：灰度/IMU/测距/视觉统一更新，10ms */
void Chassis_Update(void);          /* A：底盘速度闭环，10ms */
void LineTrack_Update(void);        /* A：循迹控制，10ms */
void TaskFSM_Update(void);          /* A：总任务状态机，10ms */
void DebugMenu_Update(void);        /* C：按键/OLED 调参菜单，20ms */
void OLED_Update(void);             /* C：OLED 页面刷新，100ms */
void Log_Update(void);              /* C：串口日志输出，200ms */
```

默认弱函数在：

```text
App/app_task_port.c
```

后面 Part2/Part3 写新模块时，不需要改 `scheduler.c`，只要在自己的 `.c` 文件里实现同名函数即可覆盖弱函数。

例如 Part2 的 `chassis.c` 里写：

```c
void Chassis_Update(void)
{
    /* 10ms 速度闭环 */
}
```

---

## 6. 通用状态码和公共工具：bsp_common.h

### 6.1 文件作用

`bsp_common.h` 是所有 BSP 模块的公共基础，提供：

```text
BSP_Status_t 状态码
BSP_WEAK 弱函数宏
BSP_GET_TICK() 时间接口
临界区保护函数
GPIO 时钟自动使能函数
DMA 标志位/中断标志位宏
```

### 6.2 状态码

```c
typedef enum {
    BSP_OK = 0,
    BSP_ERROR,
    BSP_BUSY,
    BSP_TIMEOUT,
    BSP_PARAM
} BSP_Status_t;
```

含义：

| 状态 | 含义 |
|---|---|
| `BSP_OK` | 成功 |
| `BSP_ERROR` | 外设错误 / DMA 错误 / 总线错误 |
| `BSP_BUSY` | 正忙，本次没有执行 |
| `BSP_TIMEOUT` | 超时 |
| `BSP_PARAM` | 参数错误，例如空指针、通道号非法、长度为 0 |

### 6.3 注意事项

`BSP_GET_TICK()` 默认调用 `GetTick()`，而 `GetTick()` 由 `bsp_systick.c` 提供。不要在不同文件里重复实现 `GetTick()`。

---

## 7. 时间基准：bsp_systick.c / bsp_systick.h

### 7.1 文件作用

统一全工程毫秒 tick，替代 `delay`。

常用 API：

```c
BSP_Status_t BSP_SysTick_Init(uint32_t system_core_clock_hz);
void         BSP_SysTick_Inc(void);
uint32_t     BSP_GetTickMs(void);
uint32_t     GetTick(void);
uint8_t      BSP_TimeElapsed(uint32_t *last_time_ms, uint32_t period_ms);
uint8_t      BSP_IsTimeout(uint32_t start_time_ms, uint32_t timeout_ms);
```

### 7.2 配置点

在 `bsp_systick.h`：

```c
#define BSP_SYSTICK_USE_DEFAULT_HANDLER   1
#define BSP_SYSTICK_HZ                    1000UL
```

默认提供 `SysTick_Handler()`。如果你工程的 `stm32f4xx_it.c` 已经有 `SysTick_Handler`，必须删除其中一个，否则会重复定义。

### 7.3 使用示例

每 500ms 翻转 LED：

```c
#include "bsp_systick.h"
#include "bsp_gpio.h"

void Test_SysTick_LED(void)
{
    static uint32_t last = 0;

    if (BSP_TimeElapsed(&last, 500U)) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}
```

把它加入任务表：

```c
{ Test_SysTick_LED, 10U, 0U },
```

### 7.4 测试方法

1. 确认 `BSP_GPIO_CH1` 接 LED。
2. 调用 `BSP_InitAll(SystemCoreClock)`。
3. 调度器运行。

### 7.5 正常现象

LED 大约 0.5 秒翻转一次，1 秒一个完整亮灭周期。说明 SysTick 和调度器正常。

---

## 8. GPIO：bsp_gpio.c / bsp_gpio.h

### 8.1 文件作用

只负责 GPIO 输入/输出，不负责具体业务。适合：

```text
LED
蜂鸣器
电机方向脚
SPI 设备 CS
继电器控制脚
普通输入检测
```

### 8.2 配置点

在 `bsp_gpio.h` 修改：

```c
#define BSP_GPIO_CH1_ENABLE          1
#define BSP_GPIO_CH1_PORT            GPIOC
#define BSP_GPIO_CH1_PIN             GPIO_Pin_13
#define BSP_GPIO_CH1_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH1_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH1_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH1_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH1_INIT_LEVEL      1U
```

`ENABLE=1` 的通道会出现在枚举中。业务代码不要写 `GPIOC`、`GPIO_Pin_13`，而是写：

```c
BSP_GPIO_Write(BSP_GPIO_CH1, 0);
BSP_GPIO_Toggle(BSP_GPIO_CH1);
```

### 8.3 常用 API

```c
void       BSP_GPIO_Init(BSP_GPIO_Id_t id);
void       BSP_GPIO_InitAll(void);
void       BSP_GPIO_Write(BSP_GPIO_Id_t id, uint8_t level);
void       BSP_GPIO_Toggle(BSP_GPIO_Id_t id);
uint8_t    BSP_GPIO_Read(BSP_GPIO_Id_t id);
GPIO_TypeDef *BSP_GPIO_GetPort(BSP_GPIO_Id_t id);
uint16_t   BSP_GPIO_GetPin(BSP_GPIO_Id_t id);
```

### 8.4 测试函数

```c
void Test_GPIO_Toggle(void)
{
    static uint32_t last = 0;

    if (BSP_TimeElapsed(&last, 200U)) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}
```

### 8.5 测试方法

1. 把 CH1 配成 LED 输出。
2. 加入任务表：

```c
{ Test_GPIO_Toggle, 10U, 0U },
```

### 8.6 正常现象

LED 每 200ms 翻转一次。若不亮，检查：

```text
LED 是否低电平点亮；
BSP_GPIO_CH1_INIT_LEVEL 是否相反；
GPIO 端口和引脚是否和板子一致；
是否重复占用了该引脚。
```

---

## 9. PWM：bsp_pwm.c / bsp_pwm.h

### 9.1 文件作用

只负责 TIM PWM 输出，不管电机方向和速度闭环。

适合：

```text
直流电机 PWM
舵机 PWM
蜂鸣器 PWM
风扇/灯光调光
```

电机方向脚请用 `bsp_gpio`，电机逻辑放到后续 `drv_motor.c`。

### 9.2 默认配置

默认模板是：

```text
TIM3_CH1~CH4
PC6~PC9
PSC = 84-1
ARR = 1000-1
占空比单位：permille，0~1000‰
```

大致 PWM 频率：

```text
84MHz / 84 / 1000 = 1kHz
```

### 9.3 常用 API

```c
void       BSP_PWM_Init(BSP_PWM_Id_t id);
void       BSP_PWM_InitAll(void);
BSP_Status_t BSP_PWM_SetCompare(BSP_PWM_Id_t id, uint16_t compare);
BSP_Status_t BSP_PWM_SetDutyPermille(BSP_PWM_Id_t id, uint16_t permille);
uint16_t   BSP_PWM_GetPeriod(BSP_PWM_Id_t id);
void       BSP_PWM_Start(BSP_PWM_Id_t id);
void       BSP_PWM_Stop(BSP_PWM_Id_t id);
```

### 9.4 测试函数：占空比循环变化

```c
#include "bsp_pwm.h"
#include "bsp_systick.h"

void Test_PWM_Ramp(void)
{
    static uint32_t last = 0;
    static uint16_t duty = 0;
    static int8_t dir = 1;

    if (!BSP_TimeElapsed(&last, 20U)) {
        return;
    }

    BSP_PWM_SetDutyPermille(BSP_PWM_CH1, duty);

    if (dir > 0) {
        duty += 10;
        if (duty >= 1000U) {
            duty = 1000U;
            dir = -1;
        }
    } else {
        if (duty >= 10U) duty -= 10U;
        else {
            duty = 0U;
            dir = 1;
        }
    }
}
```

### 9.5 测试方法

1. 用示波器或逻辑分析仪测 `BSP_PWM_CH1` 引脚。
2. 或接电机驱动的 PWM 输入，电机不要悬空乱转，建议先不接轮子。
3. 加入任务表：

```c
{ Test_PWM_Ramp, 10U, 0U },
```

### 9.6 正常现象

示波器能看到 1kHz 左右 PWM，占空比从 0% 慢慢到 100%，再回到 0%。

如果电机驱动测试，电机速度应该逐渐变快再变慢。若完全不动，检查 TIM、GPIO AF、驱动使能脚、PWM 频率和电机供电。

---

## 10. 编码器：bsp_encoder.c / bsp_encoder.h

### 10.1 文件作用

只负责定时器编码器模式计数、周期测速，不负责轮径换算和底盘运动学。

后续分层应为：

```text
bsp_encoder：读 TIM 编码器计数

drv_encoder：换算左右轮速度、方向统一、脉冲转 mm

odometer：里程和角度估算

chassis：速度闭环和差速分配
```

### 10.2 默认配置

默认两路：

```text
BSP_ENCODER_CH1：TIM4，PB6/PB7
BSP_ENCODER_CH2：TIM5，PA0/PA1
更新周期建议：10ms
```

如果小车前进时某一侧速度为负，优先改：

```c
#define BSP_ENCODER_CHx_REVERSE 1
```

不要在上层到处取反。

### 10.3 常用 API

```c
void       BSP_Encoder_Init(BSP_Encoder_Id_t id);
void       BSP_Encoder_InitAll(void);
void       BSP_Encoder_Update(BSP_Encoder_Id_t id);
void       BSP_Encoder_UpdateAll(void);
int16_t    BSP_Encoder_GetDelta(BSP_Encoder_Id_t id);
int32_t    BSP_Encoder_GetSpeedCps(BSP_Encoder_Id_t id);
int32_t    BSP_Encoder_GetTotal(BSP_Encoder_Id_t id);
void       BSP_Encoder_ClearTotal(BSP_Encoder_Id_t id);
void       BSP_Encoder_ClearAllTotal(void);
BSP_Status_t BSP_Encoder_GetInfo(BSP_Encoder_Id_t id, BSP_Encoder_Info_t *info);
```

其中：

```text
Delta：最近一次 Update 的增量脉冲数
SpeedCps：counts per second，单位是 脉冲/秒
Total：累计脉冲数
```

### 10.4 测试函数：串口打印编码器

```c
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include <stdio.h>

void Test_Encoder_Log(void)
{
    char buf[96];
    int n;

    n = sprintf(buf,
                "ENC L: d=%d cps=%ld total=%ld | R: d=%d cps=%ld total=%ld\r\n",
                BSP_Encoder_GetDelta(BSP_ENCODER_CH1),
                BSP_Encoder_GetSpeedCps(BSP_ENCODER_CH1),
                BSP_Encoder_GetTotal(BSP_ENCODER_CH1),
                BSP_Encoder_GetDelta(BSP_ENCODER_CH2),
                BSP_Encoder_GetSpeedCps(BSP_ENCODER_CH2),
                BSP_Encoder_GetTotal(BSP_ENCODER_CH2));

    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}
```

任务表建议：

```c
{ Encoder_Update,      10U, 0U },
{ Test_Encoder_Log,   200U, 0U },
```

### 10.5 测试方法

1. 用手转左轮，观察左编码器数值变化。
2. 用手转右轮，观察右编码器数值变化。
3. 小车向前推，观察左右速度是否都为正。

### 10.6 正常现象

```text
静止：delta≈0，cps≈0
手转左轮：左侧数值变化，右侧基本不变
手转右轮：右侧数值变化，左侧基本不变
小车向前：左右 cps 均为正
小车后退：左右 cps 均为负
```

如果方向反了，改 `BSP_ENCODER_CHx_REVERSE`。

---

## 11. ADC：bsp_adc.c / bsp_adc.h

### 11.1 文件作用

只负责 ADC 原始值采样，不负责灰度阈值、电池电压换算。

后续分层：

```text
bsp_adc：返回 0~4095 原始值

drv_gray：灰度阈值、黑白判断

drv_battery：电压换算、低电压判断
```

### 11.2 默认配置

默认：

```text
ADC1 + DMA Circular
DMA2_Stream4 Channel0
开启 CH1~CH4：PC0~PC3
CH5~CH8 默认关闭
```

如果你用 8 路灰度，把 CH5~CH8 的 `ENABLE` 改成 1，并核对引脚。

### 11.3 常用 API

```c
void       BSP_ADC_Init(void);
uint16_t   BSP_ADC_GetRaw(BSP_ADC_Ch_t ch);
BSP_Status_t BSP_ADC_GetRawArray(uint16_t *out_buf, uint8_t max_count, uint8_t *out_count);
void       BSP_ADC_Start(void);
void       BSP_ADC_Stop(void);
```

`BSP_ADC_GetRaw()` 是非阻塞读取最近一次 DMA 采样值。

### 11.4 测试函数：串口打印 ADC

```c
#include "bsp_adc.h"
#include "bsp_uart.h"
#include <stdio.h>

void Test_ADC_Log(void)
{
    uint16_t adc[8];
    uint8_t count = 0;
    char buf[128];
    int n;

    if (BSP_ADC_GetRawArray(adc, 8, &count) != BSP_OK) {
        return;
    }

    n = sprintf(buf, "ADC:");
    for (uint8_t i = 0; i < count; i++) {
        n += sprintf(&buf[n], " %u", adc[i]);
    }
    n += sprintf(&buf[n], "\r\n");

    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}
```

任务表建议：

```c
{ Test_ADC_Log, 200U, 0U },
```

### 11.5 测试方法

1. ADC 引脚接电位器或灰度模块模拟输出。
2. 转动电位器，或让灰度传感器对准黑/白区域。
3. 看串口输出。

### 11.6 正常现象

```text
0V 附近：ADC 接近 0
3.3V 附近：ADC 接近 4095
灰度模块对黑/白：数值有明显变化
```

如果数值一直 0 或 4095，检查引脚模式、模块供电、ADC 通道号和 DMA 是否冲突。

---

## 12. 按键：bsp_key.c / bsp_key.h

### 12.1 文件作用

非阻塞按键扫描和消抖，不负责菜单逻辑、不负责启停小车。

后续分层：

```text
bsp_key：按键是否按下、是否刚刚按下/释放

debug_menu：切换参数、增减参数、保存参数

task_fsm：启动/急停状态切换
```

### 12.2 默认配置

默认 4 个按键：

```text
KEY1 PE4
KEY2 PE3
KEY3 PE2
KEY4 PE1
上拉输入，低电平有效
消抖 20ms
```

### 12.3 常用 API

```c
void          BSP_Key_Init(BSP_Key_Id_t id);
void          BSP_Key_InitAll(void);
void          BSP_Key_Update(BSP_Key_Id_t id);
void          BSP_Key_UpdateAll(void);
uint8_t       BSP_Key_IsPressed(BSP_Key_Id_t id);
uint8_t       BSP_Key_WasPressed(BSP_Key_Id_t id);
uint8_t       BSP_Key_WasReleased(BSP_Key_Id_t id);
BSP_KeyEvent_t BSP_Key_GetEvent(BSP_Key_Id_t id);
```

`BSP_Key_WasPressed()` 是“按下沿事件”，读一次后清除。适合菜单切换。

### 12.4 测试函数：按键控制 LED

```c
#include "bsp_key.h"
#include "bsp_gpio.h"

void Test_Key_LED(void)
{
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}
```

任务表建议：

```c
{ Sensor_Update, 10U, 0U },      /* 默认弱函数里会调用 BSP_Key_UpdateAll() */
{ Test_Key_LED,  10U, 0U },
```

### 12.5 测试方法

1. 按下 KEY1。
2. 松开 KEY1。
3. 重复几次。

### 12.6 正常现象

每按下一次，LED 翻转一次。长按不会疯狂翻转。若按一次触发多次，增大 `BSP_KEY_DEBOUNCE_MS`。

---

## 13. EXTI：bsp_exti.c / bsp_exti.h

### 13.1 文件作用

只负责外部中断入口和回调分发。适合：

```text
限位开关
超声波 echo 边沿
测速脉冲
必须实时响应的外部信号
```

普通按键优先用 `bsp_key`，不要滥用 EXTI。

### 13.2 配置点

默认 EXTI 通道都是关闭的：

```c
#define BSP_EXTI_CH1_ENABLE 0
#define BSP_EXTI_CH2_ENABLE 0
#define BSP_EXTI_CH3_ENABLE 0
```

启用时改为 1，并核对：

```text
PORT
PIN
EXTI_PortSource
EXTI_PinSource
EXTI_Line
IRQn
Trigger
上下拉
```

### 13.3 常用 API

```c
void       BSP_EXTI_Init(BSP_EXTI_Id_t id);
void       BSP_EXTI_InitAll(void);
BSP_Status_t BSP_EXTI_AttachCallback(BSP_EXTI_Id_t id, BSP_EXTI_Callback_t cb, void *ctx);
void       BSP_EXTI_DispatchIRQ(uint32_t exti_line);
```

### 13.4 测试函数：中断计数

```c
#include "bsp_exti.h"
#include "bsp_uart.h"
#include <stdio.h>

static volatile uint32_t g_exti_count = 0;

static void Test_EXTI_Callback(void *ctx)
{
    (void)ctx;
    g_exti_count++;   /* 中断里只做计数，不 printf */
}

void Test_EXTI_Init(void)
{
    BSP_EXTI_AttachCallback(BSP_EXTI_CH1, Test_EXTI_Callback, 0);
}

void Test_EXTI_Log(void)
{
    char buf[64];
    int n = sprintf(buf, "EXTI count=%lu\r\n", g_exti_count);
    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}
```

### 13.5 测试方法

1. 在 `bsp_exti.h` 里启用 CH1。
2. `BSP_InitAll()` 后调用 `Test_EXTI_Init()`。
3. 给 EXTI 引脚输入上升沿/下降沿。
4. 每 200ms 打印 `Test_EXTI_Log()`。

### 13.6 正常现象

每触发一次边沿，计数增加。若无变化，检查 SYSCFG 标准库文件是否加入、EXTI 线是否匹配、NVIC 是否冲突。

---

## 14. UART：bsp_uart.c / bsp_uart.h

### 14.1 文件作用

通用 UART/USART 非阻塞驱动。适合：

```text
调试串口
无线串口
OpenMV / K210 串口协议
上位机调参
日志输出
```

### 14.2 驱动特点

```text
RX：DMA Circular + IDLE 中断 + 软件环形缓冲
TX：软件环形缓冲 + DMA 分段发送
支持多串口
配置集中在 bsp_uart.h
```

### 14.3 默认配置

默认启用：

```text
UART_PORT1：USART1，PA9/PA10，115200，DMA2 Stream2/7
UART_PORT2：USART2，PD5/PD6，115200，DMA1 Stream5/6
```

备用 PORT3~PORT6 默认关闭。

### 14.4 常用 API

```c
void BSP_UART_Init(UART_Port_t port);
void BSP_UART_InitAll(void);
uint16_t BSP_UART_Write(UART_Port_t port, const uint8_t *data, uint16_t len);
BSP_Status_t BSP_UART_WriteFrame(UART_Port_t port, const uint8_t *data, uint16_t len);
BSP_Status_t BSP_UART_SendData_NonBlocking(UART_Port_t port, const uint8_t *data, uint16_t len);
uint16_t BSP_UART_Read(UART_Port_t port, uint8_t *buf, uint16_t len);
uint8_t BSP_UART_GetChar(UART_Port_t port, uint8_t *ch);
uint16_t BSP_UART_Available(UART_Port_t port);
uint8_t BSP_UART_IsTxBusy(UART_Port_t port);
uint16_t BSP_UART_TxFree(UART_Port_t port);
void BSP_UART_FlushRx(UART_Port_t port);
void BSP_UART_FlushTx(UART_Port_t port);
BSP_Status_t BSP_UART_GetStats(UART_Port_t port, UART_Stats_t *stats);
void BSP_UART_ClearStats(UART_Port_t port);
void BSP_UART_Task(UART_Port_t port);
void BSP_UART_TaskAll(void);
```

### 14.5 `Write` 和 `WriteFrame` 区别

```text
BSP_UART_Write：流式写入，空间不够可能只写一部分，适合日志。
BSP_UART_WriteFrame：整包写入，空间不够则完全不写，适合协议帧。
```

视觉协议、MAVLink、自定义帧必须优先用 `BSP_UART_WriteFrame()`。

### 14.6 测试函数：串口回显

```c
#include "bsp_uart.h"

void Test_UART_Echo(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        BSP_UART_WriteFrame(UART_PORT1, &ch, 1U);
    }
}
```

任务表建议：

```c
{ Test_UART_Echo, 5U, 0U },
```

### 14.7 测试方法

1. USB-TTL 接 USART1：PA9 TX、PA10 RX、GND。
2. 串口助手设置 115200，8N1。
3. 发送任意字符。

### 14.8 正常现象

发送什么，串口助手回显什么。若收不到：

```text
检查 TX/RX 是否交叉；
检查 GND 是否共地；
检查波特率；
检查 stm32f4xx_it.c 是否有重复 USART1_IRQHandler；
检查 DMA Stream 是否和其他外设冲突。
```

### 14.9 测试函数：查看丢包统计

```c
#include "bsp_uart.h"
#include <stdio.h>

void Test_UART_Stats(void)
{
    UART_Stats_t st;
    char buf[96];
    int n;

    if (BSP_UART_GetStats(UART_PORT1, &st) != BSP_OK) {
        return;
    }

    n = sprintf(buf, "UART rx=%u tx=%u ov=%u drop=%u\r\n",
                st.rx_count, st.tx_count, st.rx_overflow, st.tx_drop);
    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}
```

正常情况下 `rx_overflow` 和 `tx_drop` 不应持续增加。持续增加说明读取太慢、日志太多或缓冲区太小。

---

## 15. I2C：bsp_i2c.c / bsp_i2c.h

### 15.1 文件作用

通用 I2C 总线驱动。BSP 层只负责总线事务，不解释寄存器含义。

适合：

```text
OLED
IMU
ToF
EEPROM
其他 I2C 传感器
```

### 15.2 默认配置

默认启用：

```text
I2C_BUS1：I2C1，PB8/PB9，100kHz
DMA1 Stream0 / Stream7
```

BUS2/BUS3 默认关闭。

### 15.3 地址约定

所有 API 的 `dev_addr` 都传 **7-bit 地址**。

例如：

```text
MPU6050 / MPU9250 常见地址：0x68
OLED 常见地址：0x3C
不要传 0xD0 / 0x78 这种左移后的 8-bit 地址
```

### 15.4 纯总线 API

新写设备驱动优先用这些：

```c
BSP_Status_t BSP_I2C_MasterWrite(I2C_Bus_t bus, uint8_t dev_addr, const uint8_t *data, uint16_t len);
BSP_Status_t BSP_I2C_MasterRead(I2C_Bus_t bus, uint8_t dev_addr, uint8_t *buf, uint16_t len);
BSP_Status_t BSP_I2C_MasterWriteRead(I2C_Bus_t bus, uint8_t dev_addr,
                                     const uint8_t *tx_data, uint16_t tx_len,
                                     uint8_t *rx_buf, uint16_t rx_len);
```

### 15.5 寄存器便捷 API

兼容旧工程：

```c
BSP_Status_t BSP_I2C_WriteByte(I2C_Bus_t bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
BSP_Status_t BSP_I2C_WriteBytes(I2C_Bus_t bus, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);
BSP_Status_t BSP_I2C_ReadByte(I2C_Bus_t bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
BSP_Status_t BSP_I2C_ReadBytes(I2C_Bus_t bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len);
```

建议：正式设备层里可以自己拼寄存器地址，调用纯总线 API。

### 15.6 异步 DMA API

```c
BSP_Status_t BSP_I2C_MasterWrite_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                           const uint8_t *tx_data, uint16_t tx_len,
                                           I2C_Callback_t callback);
BSP_Status_t BSP_I2C_MasterRead_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                          uint8_t *rx_buf, uint16_t rx_len,
                                          I2C_Callback_t callback);
BSP_Status_t BSP_I2C_MasterWriteRead_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                               const uint8_t *tx_data, uint16_t tx_len,
                                               uint8_t *rx_buf, uint16_t rx_len,
                                               I2C_Callback_t callback);
```

callback 结果：

```text
0  成功
-1 I2C/DMA 错误
-2 软件超时
```

### 15.7 测试函数：I2C 扫描

```c
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include <stdio.h>

void Test_I2C_Scan(void)
{
    uint8_t addr[16];
    uint8_t found = 0;
    char buf[128];
    int n;

    if (BSP_I2C_ScanBus(I2C_BUS1, addr, 16, &found) != BSP_OK) {
        BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"I2C scan error\r\n", 16);
        return;
    }

    n = sprintf(buf, "I2C found %u:", found);
    for (uint8_t i = 0; i < found; i++) {
        n += sprintf(&buf[n], " 0x%02X", addr[i]);
    }
    n += sprintf(&buf[n], "\r\n");

    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}
```

任务表建议只临时加一次，或 1 秒扫一次，**不要在比赛运行时反复扫描**。

### 15.8 正常现象

接 OLED 常见看到：

```text
I2C found 1: 0x3C
```

接 MPU6050/MPU9250 常见看到：

```text
I2C found 1: 0x68
```

若 found=0，检查：

```text
SCL/SDA 是否接反；
是否有上拉电阻；
模块供电是否 3.3V/5V 兼容；
地址是否被模块 AD0 引脚改变；
I2C 引脚 AF 是否正确；
DMA/IRQ 是否冲突。
```

### 15.9 测试函数：读 MPU WHO_AM_I

```c
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include <stdio.h>

void Test_I2C_ReadWhoAmI(void)
{
    uint8_t reg = 0x75;
    uint8_t id = 0;
    char buf[64];
    int n;

    if (BSP_I2C_MasterWriteRead(I2C_BUS1, 0x68, &reg, 1, &id, 1) == BSP_OK) {
        n = sprintf(buf, "WHO_AM_I=0x%02X\r\n", id);
    } else {
        n = sprintf(buf, "WHO_AM_I read failed\r\n");
    }

    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}
```

正常现象：不同 IMU ID 不完全一样，但应该不是一直 `0x00` 或 `0xFF`。

---

## 16. SPI：bsp_spi.c / bsp_spi.h

### 16.1 文件作用

通用 SPI 总线驱动。BSP SPI 不管理具体设备寄存器，也不固定某个 CS。

适合：

```text
PMW3901 光流
SPI Flash
SPI OLED
SPI IMU
其他 SPI 模块
```

### 16.2 默认配置

默认启用：

```text
SPI_BUS1：SPI1
PA5 SCK
PA6 MISO
PA7 MOSI
Mode3：CPOL=High，CPHA=2Edge
预分频 /128
DMA2 Stream0 / Stream3
```

注意：默认 Mode3 是照顾 PMW3901 模板。换成 SPI Flash 或其他模块时，可能要改成 Mode0。

### 16.3 CS 片选怎么处理

`bsp_spi.c` 不管理 CS。CS 应注册成 GPIO，例如：

```text
BSP_GPIO_CH2 = 某个 SPI 设备 CS
```

设备层这样写：

```c
#define DEV_CS_LOW()   BSP_GPIO_Write(BSP_GPIO_CH2, 0)
#define DEV_CS_HIGH()  BSP_GPIO_Write(BSP_GPIO_CH2, 1)
```

不要在 `bsp_spi.c` 里写死 PMW3901_CS 或 FLASH_CS。

### 16.4 常用 API

```c
void BSP_SPI_Init(SPI_Bus_t bus);
void BSP_SPI_InitAll(void);
uint8_t BSP_SPI_TransferByte(SPI_Bus_t bus, uint8_t tx, uint8_t *rx);
BSP_Status_t BSP_SPI_Transfer(SPI_Bus_t bus,
                              const uint8_t *tx_buf,
                              uint8_t *rx_buf,
                              uint16_t len,
                              uint8_t dummy_tx);
uint8_t BSP_SPI_ReadWriteByte(SPI_Bus_t bus, uint8_t data);
BSP_Status_t BSP_SPI_TransferAsync_DMA(SPI_Bus_t bus,
                                        uint8_t *tx_buf,
                                        uint8_t *rx_buf,
                                        uint16_t len,
                                        SPI_Callback_t cb,
                                        void *ctx);
uint8_t BSP_SPI_IsBusy(SPI_Bus_t bus);
void BSP_SPI_Task(SPI_Bus_t bus);
void BSP_SPI_TaskAll(void);
```

### 16.5 测试函数：读 W25Qxx Flash ID

假设：

```text
SPI Flash CS 接 BSP_GPIO_CH2
SPI 模式应配置为 Mode0：CPOL Low，CPHA 1Edge
```

测试代码：

```c
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include <stdio.h>

#define FLASH_CS_LOW()   BSP_GPIO_Write(BSP_GPIO_CH2, 0)
#define FLASH_CS_HIGH()  BSP_GPIO_Write(BSP_GPIO_CH2, 1)

void Test_SPI_FlashID(void)
{
    uint8_t tx[4] = {0x9F, 0xFF, 0xFF, 0xFF};
    uint8_t rx[4] = {0};
    char buf[80];
    int n;

    FLASH_CS_LOW();
    BSP_SPI_Transfer(SPI_BUS1, tx, rx, 4, 0xFF);
    FLASH_CS_HIGH();

    n = sprintf(buf, "Flash ID: %02X %02X %02X\r\n", rx[1], rx[2], rx[3]);
    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}
```

### 16.6 正常现象

W25Q 系列常见返回类似：

```text
EF 40 17
EF 40 18
```

如果全是 `FF`：

```text
CS 没拉低；
MISO 没接好；
SPI 模式错；
模块没供电；
```

如果全是 `00`：

```text
MISO 被拉低；
接线错误；
SPI 时钟/模式不对。
```

### 16.7 测试函数：SPI DMA 完成标志

```c
static volatile uint8_t spi_done = 0;
static volatile BSP_Status_t spi_status = BSP_BUSY;
static uint8_t spi_tx[16];
static uint8_t spi_rx[16];

static void Test_SPI_DmaCallback(SPI_Bus_t bus, void *ctx, BSP_Status_t status)
{
    (void)bus;
    (void)ctx;
    spi_status = status;
    spi_done = 1;
}

void Test_SPI_DMA_Start(void)
{
    spi_done = 0;
    spi_status = BSP_BUSY;

    for (uint8_t i = 0; i < 16; i++) {
        spi_tx[i] = i;
    }

    BSP_SPI_TransferAsync_DMA(SPI_BUS1, spi_tx, spi_rx, 16, Test_SPI_DmaCallback, 0);
}
```

callback 在中断里，只能置标志，不能 `printf`。

---

## 17. 一键初始化：bsp_all.c / bsp_all.h

### 17.1 文件作用

统一初始化 Part1 BSP：

```c
BSP_Status_t BSP_InitAll(uint32_t system_core_clock_hz);
void         BSP_TaskAll(void);
```

当前 `BSP_InitAll()` 会依次初始化：

```text
SysTick
GPIO
PWM
Encoder
ADC
Key
EXTI
UART
I2C
SPI
```

当前 `BSP_TaskAll()` 会周期维护：

```text
UART_TaskAll
I2C_TaskAll
SPI_TaskAll
```

### 17.2 注意事项

如果某个模块引脚和你的工程冲突，可以不用 `BSP_InitAll()`，改成手动初始化需要的模块。

例如只测 UART 和 GPIO：

```c
BSP_SysTick_Init(SystemCoreClock);
BSP_GPIO_InitAll();
BSP_UART_InitAll();
```

---

## 18. 调度器：scheduler.c / scheduler.h

### 18.1 文件作用

按周期调用 `task_list[]` 里的任务，不做任何业务逻辑。

常用 API：

```c
void     Scheduler_Init(void);
void     Scheduler_Run(void);
uint8_t  Scheduler_GetTaskCount(void);
void     Scheduler_ResetTaskTime(uint8_t index);
```

### 18.2 使用方法

main.c 中：

```c
Scheduler_Init();

while (1) {
    Scheduler_Run();
}
```

### 18.3 调度器测试函数

```c
#include "bsp_gpio.h"

void Test_Scheduler_500ms(void)
{
    BSP_GPIO_Toggle(BSP_GPIO_CH1);
}
```

在 `app_task_config.h` 加：

```c
{ Test_Scheduler_500ms, 500U, 0U },
```

正常现象：LED 每 500ms 翻转一次。

### 18.4 注意事项

每个任务函数内部必须很快返回。不要在 `Encoder_Update`、`Chassis_Update`、`TaskFSM_Update` 里调用长时间阻塞函数。

---

## 19. 非阻塞等待：nb_wait.c / nb_wait.h

### 19.1 文件作用

用于替代 delay。它本身不会卡住 CPU，只记录开始时间和等待时长。

常用 API：

```c
void     NB_Wait_Init(NB_Wait_t *wait);
void     NB_Wait_Start(NB_Wait_t *wait, uint32_t duration_ms);
void     NB_Wait_Update(NB_Wait_t *wait);
void     NB_Wait_Stop(NB_Wait_t *wait);
uint8_t  NB_Wait_IsDone(NB_Wait_t *wait);
uint8_t  NB_Wait_IsRunning(NB_Wait_t *wait);
```

### 19.2 测试函数：按键触发 500ms 后亮灯

```c
#include "nb_wait.h"
#include "bsp_key.h"
#include "bsp_gpio.h"

static NB_Wait_t wait_500ms;

void Test_NBWait_Init(void)
{
    NB_Wait_Init(&wait_500ms);
}

void Test_NBWait_Update(void)
{
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        BSP_GPIO_Write(BSP_GPIO_CH1, 0);
        NB_Wait_Start(&wait_500ms, 500U);
    }

    if (NB_Wait_IsDone(&wait_500ms)) {
        BSP_GPIO_Write(BSP_GPIO_CH1, 1);
        NB_Wait_Stop(&wait_500ms);
    }
}
```

### 19.3 测试方法

1. 初始化后调用 `Test_NBWait_Init()`。
2. 把 `Test_NBWait_Update` 加入任务表，周期 10ms。
3. 按 KEY1。

### 19.4 正常现象

按键后 LED 状态改变，约 500ms 后恢复。等待期间串口、编码器、其他任务仍能运行。

---

## 20. app_task_port：多人对接文件

### 20.1 文件作用

`app_task_port.c/h` 固定 A/B/C 对接函数名。

默认弱函数可以让 Part1 独立编译；后面你们写自己的模块时，直接实现同名函数覆盖。

### 20.2 当前默认弱函数行为

```text
AppTask_BSP_Background：调用 BSP_TaskAll()
Encoder_Update：调用 BSP_Encoder_UpdateAll()
Sensor_Update：调用 BSP_Key_UpdateAll()
Chassis_Update：空
LineTrack_Update：空
TaskFSM_Update：空
DebugMenu_Update：空
OLED_Update：空
Log_Update：空
```

### 20.3 后续 Part2 对接例子

B 写 `drv_encoder.c`：

```c
void Encoder_Update(void)
{
    BSP_Encoder_UpdateAll();
    /* 后续计算 mm/s、保存左右轮速度 */
}
```

A 写 `chassis.c`：

```c
void Chassis_Update(void)
{
    /* 10ms 速度 PID */
}
```

C 写 `sensor_manager.c`：

```c
void Sensor_Update(void)
{
    BSP_Key_UpdateAll();
    /* 灰度、IMU、测距、视觉数据统一更新 */
}
```

### 20.4 对接规则

```text
1. 不改函数名。
2. 不改 task_list 里别人负责的周期，除非三人确认。
3. Update 内不阻塞。
4. 跨模块数据用结构体和 Get 函数，不直接访问别人的 static 变量。
5. 新模块必须有 Init 和 Update。
```

---

## 21. 推荐全模块测试任务表

调试 Part1 时，可以临时把任务表改成：

```c
Task_t task_list[] = {
    { AppTask_BSP_Background,  1U,   0U },
    { Encoder_Update,         10U,   0U },
    { Sensor_Update,          10U,   0U },

    { Test_GPIO_Toggle,      200U,   0U },
    { Test_UART_Echo,          5U,   0U },
    { Test_ADC_Log,          200U,   0U },
    { Test_Encoder_Log,      200U,   0U },
    { Test_Key_LED,           10U,   0U },
};
```

但不要长期把所有日志都开着，串口日志太多会影响调试判断。

---

## 22. Part1 总体验收表

| 模块 | 测试方法 | 正常结果 |
|---|---|---|
| SysTick | LED 500ms 翻转 | 时间稳定，没有卡顿 |
| Scheduler | task_list 周期任务 | 不同周期任务都能运行 |
| GPIO | 翻转 LED / 控制 CS | 电平能正常改变 |
| PWM | 示波器测 PWM / 电机测试 | 频率正确，占空比可变 |
| Encoder | 手转轮子 / 推车 | 对应通道计数，前进为正 |
| ADC | 电位器 / 灰度模块 | 数值随电压变化 |
| Key | 按键翻转 LED | 按一次触发一次，无明显抖动 |
| EXTI | 边沿触发计数 | 每次边沿计数增加 |
| UART | 串口回显 | 发什么回什么 |
| I2C | 扫描 OLED/IMU | 能扫到 0x3C/0x68 等地址 |
| SPI | 读 Flash ID / 读模块 ID | 返回非 0x00/0xFF 的合理 ID |
| NB_Wait | 按键触发 500ms 等待 | 等待期间系统不卡死 |

---

## 23. 常见问题排查

### 23.1 编译报重复定义 IRQHandler

原因：`bsp_uart.c`、`bsp_i2c.c`、`bsp_spi.c`、`bsp_systick.c` 已经生成了一些中断函数，而 `stm32f4xx_it.c` 里也有同名空函数。

处理：删除 `stm32f4xx_it.c` 里的同名空函数。

常见重复：

```text
SysTick_Handler
USART1_IRQHandler
USART2_IRQHandler
DMAx_Streamx_IRQHandler
I2C1_EV_IRQHandler
I2C1_ER_IRQHandler
```

### 23.2 UART 只能发不能收

检查：

```text
TX/RX 是否交叉；
GND 是否共地；
波特率是否一致；
RX DMA Stream 是否配置正确；
USART IRQ 是否重复定义或没有进中断；
```

### 23.3 I2C 扫不到设备

检查：

```text
SCL/SDA 是否接反；
是否有上拉；
模块地址是否正确；
模块供电是否正确；
I2C GPIO 是否复用到正确 AF；
总线是否被某个设备拉低；
```

### 23.4 SPI 读 ID 全是 FF

检查：

```text
CS 是否真的拉低；
MISO 是否接好；
SPI Mode 是否正确；
模块是否供电；
SCK 是否有波形；
```

### 23.5 编码器方向反了

不要在上层到处取负号，直接改：

```c
#define BSP_ENCODER_CHx_REVERSE 1
```

### 23.6 速度环或循迹卡顿

检查任务里是否调用了阻塞 API，例如：

```text
I2C 扫描
长时间 printf
while 等待串口
SPI 阻塞读大块数据
Flash 写入
```

---

## 24. 后续 Part2 开发建议

Part1 完成后，下一步写：

```text
Driver/drv_motor.c/h
Driver/drv_encoder.c/h
Algorithm/pid.c/h
App/chassis.c/h
```

必须对接当前任务表：

```text
Encoder_Update：由 drv_encoder.c 覆盖，周期 10ms
Chassis_Update：由 chassis.c 覆盖，周期 10ms
```

Part2 的目标：

```text
Motor_SetPWM(300, 300) 小车前进
Motor_SetPWM(-300, -300) 小车后退
Motor_SetPWM(300, -300) 原地转向
编码器前进为正
速度 PID 稳定跟随目标速度
Chassis_SetSpeed(linear, turn) 可用
```

---

## 25. AI 继续开发时必须遵守的约定

后续 AI 接着写代码时，必须遵守：

```text
1. 不能在 BSP 层写具体业务逻辑。
2. 新增硬件资源优先在 .h 配置区加宏，不在 .c 写死。
3. 所有周期性模块必须有 Init 和 Update。
4. Update 函数必须非阻塞。
5. 不随便改已有函数名，尤其是 app_task_port.h 里的对接口。
6. 跨模块数据用结构体和 Get 函数输出。
7. 中断回调只置标志，不 printf，不 delay。
8. 协议帧发送用 BSP_UART_WriteFrame，不用可能半包的流式写入。
9. I2C/SPI 的阻塞 API 只用于初始化、扫描、调试，核心控制环优先异步或短事务。
10. 每写一个模块，都要同步写 Markdown 使用说明和最小测试函数。
```

---

## 26. 最小完整测试 main.c

```c
#include "stm32f4xx.h"
#include "bsp_all.h"
#include "scheduler.h"

int main(void)
{
    SystemInit();

    if (BSP_InitAll(SystemCoreClock) != BSP_OK) {
        while (1) {
        }
    }

    Scheduler_Init();

    while (1) {
        Scheduler_Run();
    }
}
```

如果这个 main 能跑起来，并且你按上面模块逐个测试通过，说明 Part1 BSP 基础包可以进入 Part2 电机/编码器/PID/底盘闭环开发。

