# 电赛小车参数标定对照表

## 1. 参数作用

| 参数名 | 作用 | 建议处理方式 |
|---|---|---|
| `DRV_ENCODER_WHEEL_DIAMETER_MM` | 轮子等效直径，主要影响直走距离换算 | 距离不准时优先调这个 |
| `DRV_ENCODER_COUNTS_PER_REV` | 轮子转一圈时编码器累计计数 | 用手转轮一圈实测平均值，测准后一般不再乱改 |
| `ANGLE_CONTROL_WHEEL_BASE_MM` | 左右轮中心距/等效轮距，主要影响原地转角估算 | 转角不准时调这个 |

---

## 2. 直走距离与转角误差调整表

| 现象 | 优先改哪个参数 | 怎么改 |
|---|---|---|
| 直走不够，例如目标 `500 mm`，实际 `450 mm` | `DRV_ENCODER_WHEEL_DIAMETER_MM` | 调小 |
| 直走过头，例如目标 `500 mm`，实际 `550 mm` | `DRV_ENCODER_WHEEL_DIAMETER_MM` | 调大 |
| 左/右转不够，例如目标 `90°`，实际 `75°` | `ANGLE_CONTROL_WHEEL_BASE_MM` | 调大 |
| 左/右转过头，例如目标 `90°`，实际 `110°` | `ANGLE_CONTROL_WHEEL_BASE_MM` | 调小 |
| 轮子转一圈 `total` 不是参数值 | `DRV_ENCODER_COUNTS_PER_REV` | 改成实测平均值 |

---

## 3. 直走距离标定公式

如果命令小车走 `目标距离`，实际测得走了 `实际距离`，优先修正轮径：

```text
新轮径 = 旧轮径 × 实际距离 / 目标距离
```

例如：

```text
旧轮径 = 60.0 mm
目标距离 = 500 mm
实际距离 = 450 mm

新轮径 = 60.0 × 450 / 500 = 54.0 mm
```

代码改为：

```c
#define DRV_ENCODER_WHEEL_DIAMETER_MM 54.0f
```

---

## 4. 转角标定公式

如果命令小车转 `目标角度`，实际测得转了 `实际角度`，优先修正等效轮距：

```text
新轮距 = 旧轮距 × 目标角度 / 实际角度
```

例如：

```text
旧轮距 = 160.0 mm
目标角度 = 90°
实际角度 = 75°

新轮距 = 160.0 × 90 / 75 = 192.0 mm
```

代码改为：

```c
#define ANGLE_CONTROL_WHEEL_BASE_MM 192.0f
```

---

## 5. 推荐标定顺序

1. 先测编码器一圈计数，确定：

```c
#define DRV_ENCODER_COUNTS_PER_REV 1450.0f
```

2. 再标定直走距离，只改：

```c
#define DRV_ENCODER_WHEEL_DIAMETER_MM xxx.xf
```

3. 直走基本准了以后，再标定原地转角，只改：

```c
#define ANGLE_CONTROL_WHEEL_BASE_MM xxx.xf
```

不要同时乱改三个参数，否则很难判断误差来自哪里。
