# K210钢球识别与UART说明

## 1. SD卡启动链

K210正式启动链如下：

```text
/sd/main.py
    -> /sd/gangqiu_shibie/app.py
    -> /sd/gangqiu_shibie/model/ball.kmodel
```

K210上电后等待约1.5秒，随后自动运行钢球位置检测。调试时可在CanMV IDE
连接后点击“停止”，再运行其他测试脚本。

## 2. 视觉程序功能

- 摄像头：OV2640，RGB565，QVGA 320×240；
- 模型输入：完整画面缩放至224×168，再上下补边到224×224；
- 模型：YOLOv2，`ball.kmodel`；
- 启动时先检测绿色摆杆20个有效样本，再加载KPU；
- 钢球有效控制置信度：0.65；
- 短暂漏检最多保持2帧；
- 位置使用三帧中值滤波；
- UART最多每50ms发送一次，实际更新率受YOLO推理帧率限制。

## 3. 七点位置标定

```text
camera_x: 21, 61, 121, 154, 188, 249, 281
位置/cm : -12, -9, -3, 0, +3, +9, +12
```

两标定点之间使用分段线性插值。相机、镜头或摆杆固定位置发生明显变化后，
应重新采集标定点。

## 4. UART接线

```text
K210 IO8 TX  -> STM32 PA3 / USART2_RX
K210 GND     <-> STM32 GND
STM32 PA9 TX -> USB-TTL RX（调试输出）
```

串口参数：115200，8N1，无校验，无流控。当前钢球检测只要求K210向STM32
单向发送；需要STM32反向控制时，再连接STM32 PA2到K210所选RX引脚。

## 5. 固定帧格式

每帧固定7字节：

```text
[0] 0xAA
[1] 0x55
[2] CMD
[3] DATA1
[4] DATA2
[5] DATA3
[6] CHECKSUM
```

```text
CHECKSUM = (byte0 + byte1 + ... + byte5) & 0xFF
```

## 6. 钢球位置命令0x32

```text
CMD = 0x32
DATA1 = position_tenth_mm高字节
DATA2 = position_tenth_mm低字节
DATA3 bit7..6 = state
DATA3 bit5..0 = confidence_6bit
```

`position_tenth_mm`为大端、有符号16位二进制补码，单位0.1mm：

```text
-12.00cm -> -1200
-6.00cm  -> -600
0.00cm   -> 0
+6.00cm  -> 600
+12.00cm -> 1200
```

方向定义：左侧为负，右侧为正，O点为0。

状态：

```text
0 = LOST
1 = HOLD
2 = VALID
3 = 保留，收到时按格式错误处理
```

置信度转换：

```text
K210:  round(confidence_float * 63)
STM32: round(confidence_6bit * 100 / 63)
```

LOST帧位置和置信度均发送0。HOLD帧发送最近一次有效位置；置信度保留当前
低置信度结果，纯漏检时为0。

中心位置、VALID、约95%的示例：

```text
AA 55 32 00 00 BC ED
```

其中`0xBC`的高两位为`10`，表示VALID；低六位`0x3C=60`。

## 7. STM32接入关系

K210原始UART字节只由`K210_Comm_Update()`解析。滚球闭环模式中，
`BallBalance_K210Adapter_Update()`是正式控制路径中
`K210_Comm_GetNewBallPosition()`的唯一消费者，
它把0x32帧转换为`BallBalance_VisionSample_t`并调用
`BallBalance_App_PushVisionSample()`。

`Test_K210_BallCommUpdate()`只在纯通信测试档位使用；模型辨识、状态反馈测试
和正式比赛模式都由适配层消费新帧，不能同时注册纯通信任务，否则会提前清除
新钢球帧。

适配层不改变0.1 mm单位，也不反转方向。APP只有在状态为VALID、置信度不低于
`BALL_BALANCE_MIN_CONFIDENCE`且位置位于±120.0 mm内时才执行卡尔曼测量更新。
HOLD和LOST只保留诊断状态，不会把旧位置或0 mm重复送入估计器。

## 8. 正常调试输出

纯通信测试中可能看到：

```text
BALL position=-4.40cm state=VALID confidence=86
BALL position=+0.10cm state=HOLD confidence=56
BALL position=+0.00cm state=LOST confidence=0
K210 STATUS online=1 frames=... check_err=0 format_err=0
```

只要`online=1`、`frames`持续增加且校验和格式错误计数保持0，链路即为正常。

## 9. SD卡重要文件

```text
/sd/main.py
/sd/main_auto.py
/sd/gangqiu_shibie/app.py
/sd/gangqiu_shibie/model/ball.kmodel
/sd/config/ball_config.py
/sd/docs/README_SD_CARD.md
/sd/docs/UART_PROTOCOL.md
```

数据集、模型备份和历史程序可保留在SD卡中，部署正式钢球识别不需要删除它们。
