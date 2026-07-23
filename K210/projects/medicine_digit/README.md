# K210送药小车数字识别

本目录保存送药小车当前使用的K210数字识别程序、模型、数据采集工具和补强数据。识别、可变数量数字快照发送、STM32接收解析以及送药路线决策均已完成联调；整车已经完成基本全程实车测试，能够基本走完全程。更换场地、相机安装位置、模型、光照或车辆参数后仍需重新验证。

## 1. 正式运行入口

仓库中的部署文件与SD卡路径对应如下：

| 仓库文件 | SD卡位置 | 作用 |
| --- | --- | --- |
| `K210/main.py` | `/sd/main.py` | K210上电自动启动入口 |
| `K210/projects/medicine_digit/model/digit_detect_stable_293508.py` | `/sd/projects/medicine_digit/model/digit_detect_stable_293508.py` | 当前正式识别程序 |
| `K210/projects/medicine_digit/model/model-293508.kmodel` | `/sd/projects/medicine_digit/model/model-293508.kmodel` | 当前目标检测模型 |

`K210/main.py`上电等待1500 ms后执行`model/digit_detect_stable_293508.py`。脱离CanMV IDE运行时，必须保持上述SD卡目录和文件名不变。

目录外层还保留一份内容相同的`digit_detect_stable_293508.py`，用于查看和备份；正式部署和修改应以`model/`目录中的启动版本为准，修改后应检查两个副本是否仍然一致。

## 2. 当前模型绑定参数

以下参数必须与`model-293508.kmodel`保持一致，不得按数字自然顺序擅自重排：

```python
MODEL_PATH = "/sd/projects/medicine_digit/model/model-293508.kmodel"
LABELS = ("2", "6", "1", "5", "4", "7", "3", "8")
ANCHORS = (
    1.19, 1.47,
    2.00, 2.69,
    1.34, 1.81,
    0.94, 0.94,
    1.03, 1.09,
)
```

当前基准运行参数：

```python
INFERENCE_MODE = "single"
CONF_THRESHOLD = 0.58
NMS_THRESHOLD = 0.30
MIN_BOX_AREA = 120
MAX_BOX_AREA = 30000
MIN_ASPECT_RATIO = 0.20
MAX_ASPECT_RATIO = 2.00
ROI_LEFT = 0
ROI_TOP = 0
ROI_RIGHT = 320
ROI_BOTTOM = 240
TRACK_HISTORY = 5
TRACK_CONFIRM_HITS = 2
TRACK_DELETE_MISSES = 3
OUTPUT_INTERVAL_MS = 200
VFLIP = False
HORIZONTAL_MIRROR = False
```

详细的检测、去重、双窗口、跟踪和调参方法见`Doc/K210数字识别程序说明与调参指南.md`。

## 3. STM32通信

当前采用单向连接：

```text
K210 IO6 / UART1_TX → STM32 PA3 / USART2_RX
K210 GND            → STM32 GND
```

两端串口参数均为115200、8-N-1、无硬件流控。K210每200 ms发送一份稳定数字快照，底层固定帧为：

```text
AA 55 CMD DATA1 DATA2 DATA3 CHECKSUM
```

一份可变数量结果由以下命令组成：

```text
0x10  SNAPSHOT_BEGIN
0x11  SNAPSHOT_ITEM，可重复0～8次
0x12  SNAPSHOT_END
```

校验和为前6字节累加和的低8位。识别结果在K210侧按横坐标从左到右排序，STM32不需要再次排序；超过8个结果时发送`OVERFLOW`状态，不静默裁剪。

STM32对应接口位于：

```text
APP/k210_comm.c
APP/k210_comm.h
```

正式业务依次由`K210_Comm_Update()`解析快照、`TaskFSM_Update()`完成多帧确认和任务决策、Medicine Route执行去程与返程路线。完整协议、日志和验收方法见`Doc/K210数字识别与STM32通信说明.md`。

## 4. 数据采集工具

当前目录保留以下采集工具：

| 文件 | 作用 |
| --- | --- |
| `capture_digit_dataset.py` | 按场景和数字采集数据 |
| `capture_digit_dataset_before_lcd.py` | LCD接入前的历史采集版本 |
| `cleanup_capture.py` | 清理指定场景、指定数字的一批图片 |
| `digit_manager.py` | 统一执行`status`、`capture`或`clear` |
| `run_capture.py` | 采集启动入口 |
| `reinforce_267/` | 指定数字和场景的补强采集脚本 |

采集数据保存在SD卡的：

```text
/sd/digit_capture/handheld/<digit>/
/sd/digit_capture/ground/<digit>/
/sd/digit_capture/background/
```

`digit_manager.py`支持：

```python
ACTION = "status"   # 查看数量
ACTION = "capture"  # 采集
ACTION = "clear"    # 倒计时后清除当前指定批次
SCENE = "handheld"  # handheld、ground或background
DIGIT = 1           # 非background场景使用1～8
```

执行清除前必须再次核对`SCENE`和`DIGIT`。补充或重新训练模型时，数据集划分应按拍摄序列进行，不能把同一连续序列随机拆到训练集和验证集。

## 5. 部署与测试

1. 将仓库中的`K210/main.py`复制到SD卡根目录。
2. 将正式识别脚本和模型复制到`/sd/projects/medicine_digit/model/`。
3. 检查K210 IO6、STM32 PA3及两板共地。
4. 上电后等待约3～5秒完成摄像头和模型初始化。
5. 在K210屏幕确认数字、顺序和检测框正确。
6. 在STM32日志中确认`online=1`，数字、置信度和横坐标与K210显示一致。
7. 依次测试无数字、单数字、多个数字、目标进入和离开画面，以及断开和恢复K210 TX。
8. 正式整车测试时确认目标数字经过多帧锁定，路口视觉决策与去程、卸药等待、返程状态一致。

当前代码已经完成基础通信测试和基本全程实车联调。该结论适用于现有模型、接线、场地和车辆参数；调整模型绑定参数、相机方向、串口协议或路线参数后必须重新测试。
