# 姿态算法主机单元测试

本目录直接编译项目中的`Algorithm/attitude_estimator.c`，不复制算法实现。`stubs/bsp_common.h`只提供算法所需的状态码和临界区桩，因此测试不依赖STM32芯片头文件、Keil工程或实物硬件。

## 运行方法

在项目根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Test\host\run_attitude_tests.ps1
```

脚本自动定位Visual Studio C++ x64工具链，在系统临时目录`TraceTrackCarHostTests`中生成测试程序，不向Keil或Makefile固件源列表添加主机测试文件。缺少Visual Studio C++构建工具时，脚本会直接报错退出。

## 当前覆盖场景

1. 静止水平输入：姿态有效、静止状态建立、三个欧拉角保持稳定。
2. 匀速旋转：模拟Z轴90°/s旋转1秒，Yaw结果接近90°。
3. 重复时间戳：返回`BSP_BUSY`，不增加更新次数且不重复积分。
4. 无效输入：空指针和显式失效都会清除有效标志，后续新样本可以恢复。
5. 电机活动门控：`motor_active=1U`时停止使用磁力计，恢复为`0U`后重新参与修正。

测试使用`/W4 /WX`编译，任何主机编译警告或断言失败都会返回非零退出码。修改姿态接口、融合参数、时间戳处理、失效逻辑或磁力计门控后，必须重新运行本测试。
