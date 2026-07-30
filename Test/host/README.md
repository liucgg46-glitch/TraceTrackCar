# 算法主机单元测试

本目录直接编译项目中的`Algorithm/attitude_estimator.c`和`Algorithm/odometer.c`，不复制算法实现。通用状态码来自`Common/project_status.h`，`stubs/project_critical.c`只提供主机临界区空操作实现，因此测试不依赖STM32芯片头文件、Keil工程或实物硬件。

## 测试入口文档一致性检查

修改`Test/test.h`中的公共测试函数、删除测试函数或调整测试任务文档后，在项目根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Test\host\check_test_docs.ps1
```

脚本会核对公共`Test_*`入口是否有真实实现、是否已写入`Doc/测试任务完整手册.md`，并检查`Doc`中任务表示例是否仍使用有效名称和正确的宏续行格式。脚本还会检查任务表与测试开关一致：任务表包含`Test_*`时必须为`1U`，正式任务表时必须为`0U`。

## 运行方法

在项目根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Test\host\run_attitude_tests.ps1
```

脚本自动定位Visual Studio C++ x64工具链，在系统临时目录`TraceTrackCarHostTests`中生成测试程序，不向Keil或Makefile固件源列表添加主机测试文件。缺少Visual Studio C++构建工具时，脚本会直接报错退出。

## 当前覆盖场景

1. 静止水平输入：姿态有效、静止状态建立、三个欧拉角保持稳定。
2. 匀速旋转：模拟Z轴90°/s旋转1秒，Yaw结果接近90°。
3. 重复时间戳：返回`PROJECT_BUSY`，不增加更新次数且不重复积分。
4. 无效输入：空指针和显式失效都会清除有效标志，后续新样本可以恢复。
5. 电机活动门控：`motor_active=1U`时停止使用磁力计，恢复为`0U`后重新参与修正。
6. 里程相对距离：验证左右累计里程、平均里程和周期增量。
7. 软件清零：硬件累计值不清零时，新的软件基准仍从0开始计程。
8. 计数回绕：验证32位有符号累计值跨越边界后的相对距离。

测试使用`/W4 /WX`编译，任何主机编译警告或断言失败都会返回非零退出码。修改姿态接口、融合参数、时间戳处理、失效逻辑、磁力计门控或里程清零/增量逻辑后，必须重新运行本测试。
