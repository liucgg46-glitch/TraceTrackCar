# 项目文档同步规则

- 新增模块或功能处于开发、联调或测试阶段时，不要在每次代码迭代后立即修改文档，也不要把尚未验证的设计写入文档。
- 同一模块或功能测试通过、结果稳定后，再一次性同步更新 `Doc/说明文档.md`；如果涉及参数、引脚、测试任务、显示使用方法或注意事项，还应同步更新 `Doc` 中对应的同名 Markdown 文档。
- 如果任务结束时新功能尚未测试通过，只报告尚待验证和文档尚未同步，不要为了形式上的同步写入阶段性结论；用户明确要求提前记录的内容除外。
- 文档只记录稳定、必要的功能说明、参数、使用方法和注意事项，不记录临时调试进度、当前正在进行的阶段或阶段性结论。
- 在项目全部功能完成前，不在文档中反复维护“当前默认任务表”或“当前注册了哪些任务”；只更新受本次代码修改直接影响的测试任务示例。最终任务注册情况由用户在项目收尾时统一整理，除非用户明确要求提前同步。
- 文档中的代码文件名、函数名、宏名和变量名必须与当前源码完全一致，不得擅自改名。
- `Test/test.h`是嵌入式专项测试公共入口的唯一依据；新增、删除或改名`Test_*`入口后，必须同步`Doc/测试任务注册函数使用方法.md`并运行`Test/host/check_test_docs.ps1`。`Test/test.c`中的`static`辅助函数不得写入任务表。
- Markdown 文档中的任务列表代码示例必须使用宏续行格式：`Task_t task_list[] = {` 行和每个 `{ TaskFunction, period, 0U },` 任务项行的末尾都必须添加 `\`，结束行 `};` 不添加；任务项带注释时必须使用 `/* 中文说明 */`，禁止使用会吞并下一续行的 `//` 注释，并将 `\` 放在块注释之后且保持为该行最后一个非空白字符。
- 无法由源码或现有资料确认的硬件信息必须明确标注为“待确认”，不得猜测。
- 新模块或功能测试通过并准备结束该功能任务前，应复核相关文档与最终通过测试的源码一致，并一次性完成文档更新。

# 代码注释规则

- 新增或修改的代码注释统一使用中文；函数名、变量名、宏名、协议字段和必须保持原样的日志文本除外。
- 注释应说明用途、约束或原因，避免记录临时操作过程和无长期价值的调试进度。

# 分层与封装规则

## 目录职责与允许依赖

- `Common`只放与芯片和业务无关的公共契约，例如`Project_Status_t`和临界区抽象声明；禁止包含STM32、BSP、Driver、APP、Route或Test头文件。
- `BSP`只负责STM32外设、板级资源和`Common`抽象在目标板上的实现；不得依赖Driver、Algorithm、Route、APP或Test。
- `Driver`负责具体器件协议和器件状态机，可以依赖BSP；不得依赖Algorithm、Route、APP或Test。
- `Algorithm`只负责可由纯数据驱动的计算，可以依赖`Common`、本层头文件和标准库；不得直接包含或调用BSP、Driver、Route、APP或Test。
- `Route`负责赛道状态与控制意图，可以依赖`Common`和Algorithm；不得直接读取BSP节拍、具体传感器、Driver、APP、Motion或底盘。
- `APP`负责业务编排、控制权仲裁和上下层适配；Driver数据转换为Algorithm输入、硬件状态转换为Route输入的代码应放在APP。
- `Test`可以依赖被测各层，但测试声明、测试任务、桩实现和主机测试文件只能放在`Test`目录，禁止把`Test_*`声明放回BSP、Driver、Algorithm、Route或正式APP公共头文件。
- `user/main.c`只负责按`BSP_InitAll → Driver_Init → App_Init → Scheduler_Init`顺序启动并运行调度器，不承载器件协议、算法或比赛业务。

## 已解决问题形成的强制边界

- `Algorithm/attitude_estimator.*`只接收`Attitude_Input_t`和`motor_active`，不得重新读取IMU或电机Driver；采样转换与电机活动判断归`APP/sensor_manager.*`。
- `Algorithm/odometer.*`只接收左右累计毫米值并维护软件清零基准，不得读取或清零编码器Driver；硬件读取归`APP/odometer_adapter.*`。
- `Algorithm/line_track.*`和Route所需时间必须由APP以`now_ms`传入；禁止在Algorithm或Route中调用`BSP_GET_TICK()`、`BSP_GetTickMs()`或包含`bsp_systick.h`。
- Algorithm和Route公共接口统一使用`Common/project_status.h`中的`Project_Status_t`与`PROJECT_*`；禁止为了状态码包含硬件相关的`bsp_common.h`。
- Algorithm需要临界区时只调用`Common/project_critical.h`；目标板实现放在BSP，主机桩放在`Test/host/stubs`，不得在算法中直接使用STM32中断指令。
- I2C/SPI共享总线只由`BSP_InitAll()`初始化一次；Driver只能初始化器件状态并发起传输，不得再次调用`BSP_I2C_Init*()`或`BSP_SPI_Init*()`。
- Route只输出`Route_ControlMode_t`、`Route_ActionRequest_t`等控制意图；底盘控制权、Motion启动和Driver访问只能由APP处理。

## 新文件归属与构建同步

- 纯数据类型、通用状态码和可移植抽象接口放`Common`；对应STM32实现放`BSP`，主机替代实现放`Test/host/stubs`。
- 单纯把Driver数据送入Algorithm的薄适配器放`APP`，不要为了少一个文件把硬件读取重新塞进算法。
- 赛道规则放`Route`，整场比赛任务状态机放`APP`，器件读写放`Driver`，寄存器和引脚外设操作放`BSP`。
- 新增或移动参与固件的`.c`文件时，必须同时更新`DroneProject.uvprojx`与`Makefile`；主机测试源文件只进入`Test/host`脚本，不加入固件源列表。
- 正式固件保持`PROJECT_TEST_TASKS_ENABLE=0U`，`APP/app_task_config.h`不得包含`test.h`或注册`Test_*`任务；专项测试通过后立即恢复正式任务表再做最终构建。
# STM32与MSPM0上层接口同步规则

- 两个平台的Common、Algorithm、Route、APP和Test公共API、结构体字段及业务宏应保持同名同义。
- 上层串口代码只使用UART_PORT_K210、UART_PORT_E220和DEBUG_UART_PORT，不直接使用具体USART/UART编号、GPIO、DMA或中断宏。
- 芯片寄存器、引脚复用、DMA和中断差异只能放在BSP、Config和启动文件中，禁止在上层伪造另一平台的底层宏。
- 从另一平台复制APP或Test文件前，先确认对应公共头文件已同步；不得通过修改上层变量名规避接口差异。
