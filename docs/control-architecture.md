# 控制架构

控制流程按运行角色分为两条并列链路。A 主机（含单机配置）执行第 1～4 层；B～G 从机执行第 5～7 层。各节点的最终归一化指令都在控制/分配完成后交给同一 PWM 转换和输出接口。

| 层 | 文件 | 输入 → 输出 |
| --- | --- | --- |
| 1 指令生成 | `src/control/command_generation.cpp` | 遥控、参数及构型目标 → `ControlTargets` |
| 从机命令接收 | `src/control/control_modes.cpp` | 接收命令并发布全局模式、本机目标和上级贡献 |
| 3 主机控制 | `src/control/master_controller.cpp` | 主控制目标与状态 → 各控制器全局结果 |
| 4 主机 mixer/分配 | `src/allocation/master_mixer.cpp` | 各控制器全局结果 → 本机归一化指令及各从机指令-1 |
| 5 从机控制 | `src/control/slave_controller.cpp` | 接收的本机目标与本机状态 → `pitchPidControlOutput` / `pitchIndiControlOutput` |
| 6 从机 mixer/分配 | `src/allocation/slave_mixer.cpp` | 本机控制器结果 → 本机指令-2，含俯仰到副翼交联及阻尼 |
| 7 指令融合 | `src/allocation/command_fusion.cpp` | 从指令-1 + 从指令-2 → 从机最终归一化指令 |

`src/control/control_pipeline.cpp` 只编排上述角色分支，`control_modes.cpp` 准备模式切换状态；实际模式和俯仰算法分派位于主/从控制入口，`control_types.h` 定义层间数据。`src/main.cpp` 仍只编排周期任务。

主机目标使用 `MasterControlTargets`（当前复用 `ControlTargets` 的字段），从机目标使用 `SlaveControlTargets`。从机数组固定为六个槽位，`slaves[0～5]` 对应 B～G；A 单独存储，实际从机数量为 `aircraftCount() - 1`。全局从机目标、主分配结果和独立发送命令均使用这一索引，原通信左侧 B/D/F、右侧 C/E/G 的槽位保持不变。

```text
A 主机：输入 → 模式准备 → 主从目标生成 → 主控制 → 主分配
                                               ├→ 主机最终指令 → PWM → 输出
                                               └→ 从目标 + 从指令-1 → 原机间协议

B～G：原机间协议 → 从目标 → 从控制 → 从分配 → 从指令-2
                 └→ 从指令-1 ──────────────→ 融合 → PWM → 输出
```

## 目标归类和算法兼容

当前固定归类保留整体滚转、偏航、油门和构型控制在主机；A 机俯仰属于主控制的一部分，B～G 俯仰属于从控制。主机目标由 `prepareControlMode()` 的主机分支写入全局 `targets`；从机目标由主分配之后的 `generateSlaveCommand()` 写入 `slaveTargets[]`，不再使用路由函数或包装结构。当前协议传输俯仰角目标、模式及定点作动器贡献；将油门改为本地闭环时，需要同时定义新的本机目标字段和相应通信适配，不能只改一个角色开关。

`generateSlaveCommand()` 在主分配之后初始化本周期从机俯仰目标，再叠加当前滚转和构型输出对应的策略修正。它同时整理从机作动器贡献并组装发送命令，不使用上一周期的输出。

滚转 PID、俯仰 PID、偏航 PID、构型控制、俯仰 INDI、手动直通和滚转阻尼分别封装为独立类（`src/control/*_controller.h/.cpp`）。积分、指令滤波和 INDI 延迟队列由对应对象保存，`controller_instances.cpp` 持有每块飞控的一套实例，旧 `control_state.h/.cpp` 已删除，模式状态统一由 `control_modes.h/.cpp` 持有；六路构型测量由 `configurationControlOutput` 发布，刹车偏移由 mixer 根据全局 `brakeEnabled` 直接计算。主机运行整体滚转、偏航、构型及本机俯仰；从机只运行本机俯仰。入口按参数只运行选中的俯仰算法，INDI 模式不再计算未使用的俯仰 PID。PID 的输出沿用原归一化控制尺度，没有把原 PID 增益重解释为加速度增益；INDI 的运动目标明确为角加速度（deg/s²），其增量求解、限幅、滤波、PWM 标定和舵机估计均在 INDI 的 update 阶段运行。

`prepareControlMode()` 在各机读取本地 `pitch_controller` 参数并更新全局 `activePitchController`；仅主机读取遥控通道更新模式、策略、构型系数和刹车。积分门控、PID/INDI 切换重置和模式显示均在 `runMasterController()` / `runSlaveController()` 内按当前模式准备；INDI 使用当前全局俯仰目标重置。构型积分仍只门控，不因模式切换清零。

## 归一化与融合

`ActuatorCommand` 表示逻辑控制偏移，1 对应 1000 μs，不含机械中位、安装反向或 trim。保留原油门尺度，不把该字段重新定义为物理推力，也不在架构迁移中增加归一化幅值限制。

主分配保留原混控系数及副翼 0.4 缩放、刹车、手动升降舵交联；主机本机俯仰分配、升降舵到副翼补偿和滚转阻尼也在主分配内完成。从分配生成本机俯仰指令及对应副翼补偿，不在最终输出处追加补偿。

从指令-1 的副翼、油门、方向舵为主机给定值；手动时升降舵为完整手动指令，增稳时升降舵只含主机前馈。从指令-2 的副翼为本机交联/阻尼增量；增稳时升降舵为完整本机闭环输出，手动时为零。融合只相加这两种已明确约定的贡献，不执行控制律、交联或 PWM 映射。

`SlaveCommand` 通过 `mode` 携带控制模式，通过 `masterContribution.elevator` 保存当前模式的升降舵贡献。主分配按模式生成该字段；机间直接发送 int16 定点逻辑输出。从机读取命令对象中的结构体，增稳模式融合本机俯仰闭环，手动模式使用上级完整贡献。命令帧为 40 字节，所有节点须使用同版协议。

INDI 原实现使用实测物理 PWM 标定；分配返回时扣除本机中位/trim 并还原控制方向，最终转换时恢复一次。主机前馈只在从机融合时加入一次；副翼补偿使用本机俯仰贡献，继续不把主机升降舵前馈计入交联。

手动从机的副翼交联改用本周期上级手动升降舵指令，避免原实现使用上一次增稳的残留俯仰输出。主分配每周期清零旧前馈缓存，避免切换混控策略后沿用上一个策略的前馈。

## 模式和俯仰参数

仅支持手动和增稳两种模式（通信编码为 0 手动、1 增稳）。积分由本机 SD 整数参数 `integralControlEnabled` 独立控制：0 关闭并清除历史，1 开启，默认 1，修改后下一周期生效。增稳模式由本机 SD 整数参数 `pitch_controller` 选择俯仰算法：0 为 PID，1 为 INDI，默认 1。各机独立配置，修改成功后下一控制周期使用，并通过现有参数保存机制持久化；不增加机间算法选择字段。

主机参考仍由遥控通道 5 选择模式。从机通过命令的 `mode` 准备当前控制模式。手动不运行闭环。

保留原有 `radioInputValid()` 及其地面站、解锁与校准调用，不用它在控制链中额外切换模式。命令链保留原有首帧判断、有效帧缓存、校验及转发行为，不增加命令超时、停止通信、自动失效恢复或失效输出覆盖。原传感器状态链的新鲜度判断及本机锁定/非有限输出保护保持不变。

## 输出与验证

`convertControlCommandsToPWM(commands)` 将本机最终归一化指令一次性转换为物理 PWM，并把各从机指令适配到原微秒偏移缓存。已量化的线上偏移恢复为最近整数，避免归一化往返额外截断。`prepareActuatorCommands()` 只打包通信缓存，`applyAndTransmitActuatorCommands()` 保留硬件输出、锁定及软件限幅。

普通和调试环境使用相同源码布局。架构迁移时的临时本机 C++ 回归测试覆盖 1/3/4/5/7 机、两种俯仰算法、手动与两种增稳、三种混控策略、非零 trim、两个安装方向、融合和模式切换。原公式结果通过对比，本机 PWM 对比容差为 1 μs。新增的命令超时与失效处理已按要求撤除，不属于当前实现。测试使用模拟硬件接口，没有替代板上方向、时序和飞行验证；测试脚本与产物按项目要求删除。

本次控制器类拆分的临时本机回归比较了改动前后 3500 个主机周期及 200 个从机周期，覆盖滚转/俯仰/偏航 PID、构型控制、INDI 目标与 PWM、舵机估计和阻尼；另检查了实例历史隔离、手动直通、算法分派与参数切换。测试使用模拟传感器，临时脚本和产物在测试后删除。

A～G 原始分配结果直接写入全局 `ControlCycleOutput commands`，主机结果位于 `local`，B～G 位于 `slaves[0～5]`。不再使用第二份原始分配数组；`generateSlaveCommand()` 原位完成从机输出缩放及命令组装。

主机周期顺序为 `prepareControlMode()` → `runMasterController()` → `mixMasterCommands()` → `generateSlaveCommand()`。主从机均先调用一次 `prepareControlMode()`：主机读取遥控输入，从机在该函数内接收上级命令并发布模式、目标和贡献；随后运行本机控制器、分配及融合。

控制周期结果统一保存在全局 `ControlCycleOutput commands`，`runControlCycle()` 每周期更新该对象。执行器转换消费纯作动器结果，机间打包读取独立的 `slaveCommands[]`。最终五路物理 PWM 由 `ActuatorOutput[5]` 保存，索引 0～4 为副翼1、副翼2、升降舵、油门、方向舵；上电安全输出、锁定和限幅路径均更新该数组。执行器日志仅记录这些最终值（含 trim），删除从机逻辑输出列及 INDI 中间 PWM 指令列。

`ControlTargets` 仅包含控制目标字段，模式、混控策略和刹车开关不在目标结构体内。控制器直接读取可修改的全局 `targets`、`slaveTargets[]`；从机的上级贡献独立保存在 `slaveCommand` 中，接收时同时发布 `currentFlightMode` 和本机俯仰参考。通信目录的收发、配置和转发函数本次未修改。

`mixMasterCommands()`、`mixSlaveCommands()`、`fuseSlaveCommands()` 均无参数、无返回值，直接更新全局分配结果。主 mixer 将原始分配写入 `commands` 后原位更新本机最终结果；`generateSlaveCommand()` 先使用从机原始副翼值计算手动升降舵交联，再原位执行副翼缩放和刹车叠加，最后组装 `slaveCommands[]`。

姿态及油门目标统一保存在 `targets`，不再维护独立的目标全局变量或手动专用字段。主机手动模式的 roll/pitch/yaw 为 ±1，手动控制器直接使用 `targets.roll/pitch/yaw / 2`；增稳模式为原角度/角速度尺度。分配、从机俯仰目标与度制日志通过目标换算函数读取 `targets`，保留原缩放和俯仰 trim；从机接收的 pitch 始终为度制。

俯仰控制直接使用 `targets.pitch`，已删除中央机俯仰修正缓存及 mixer 到下一周期俯仰参考的交联；PID 与 INDI 均直接使用当前目标，俯仰目标日志直接记录当前目标。

`mixLegacyMaster()` 直接更新 `commands.local/slaves[]`，没有原始分配数组参数或返回值。`commands` 每周期先清零，避免未使用槽位或切换策略后残留旧贡献。

`control/control_output.h/.cpp` 分别持有全局 `rollControlOutput`、`pitchPidControlOutput` / `pitchIndiControlOutput`、`yawControlOutput`、`throttleControlOutput`、`configurationControlOutput`，不再使用汇总输出结构。俯仰 PID 和 INDI 分别持有独立结果，INDI 结果包含其日志诊断；滚转结果包含输出、滤波量和等效滚转；构型结果包含六路输出、误差、角度和角速度。每周期重置结果，各控制器对象仍保留积分和滤波历史。主从控制入口读取全局 `previousFlightMode`、`previousPitchController` 检测切换，不再写入历史变量。主从机均在统一模式准备入口保存旧模式和旧俯仰算法，再分别读取遥控或接收命令，并计算切换状态。

滚转阻尼在主机 `runMasterController()` 和从机 `mixSlaveCommands()` 中调用 `controlControllers().rollDamping.update()` 计算，每周期执行一次，结果存入 `rollDampingControlOutput.dampingCommand`，使用浮点归一化指令（1 对应 1000 us 的逻辑控制偏移）。原 125 倍微秒换算合并为 0.125 倍归一化换算，不再进行整数截断；主从 mixer 直接叠加该结果，旧阻尼标量与入口已删除。俯仰 PID 输出为 `pitchPidControlOutput.pidOutput`，INDI 运动结果与诊断为 `pitchIndiControlOutput`，不再有共同的俯仰输出包装结构。`activePitchController` 由模式准备阶段更新，手动分配直接使用手动俯仰输出；主从控制器仅在增稳模式按选择运行 PID 或 INDI。

从机分配每周期重建 `commands.local` 的本机贡献，再由无参 `fuseSlaveCommands()` 把 `slaveCommand.masterContribution` 的五路值各叠加一次。手动升降舵使用上级完整贡献，增稳升降舵使用本机闭环加上级前馈，保持原有尺度与交联公式。

`ConfigurationController::update()` 统一持有构型输出的运行时写入：直接刷新六路实测角、角速度和控制量，主机增稳模式下计算误差及闭环，其他周期只更新测量并清零命令、保留最近闭环误差及积分历史。已删除单独测量刷新函数、控制器结果缓存和外部结果复制；主从控制入口各调用一次。构型结果由构型控制器自己管理，不使用外部清零函数。

三轴姿态 PID 直接读取传感器层发布的 `ang_vel_filtered[3]`，已删除反馈包装对象及相关文件。普通反馈低通在每次 IMU 测量发布后统一推进一次，手动与增稳模式都持续更新；滤波采样率保持 500 Hz，内置 MPU6050 路径保持 40 Hz 截止，其他原路径保持直通。原始 `ang_vel`、角加速度估计器及 INDI 专用滤波不改变。

各控制器 `update()` 无返回值，直接发布本控制器全局结果。PID 与 INDI 通过 `enabled` 区分本周期是否运行；未启用时仅清零当前输出，不推进内部积分或滤波历史。滚转滤波量和等效滚转角在滚转 `update()` 内发布，保持原手动模式的历史读数。手动直通由 `ManualController::update()` 仅生成独立的 `manualControlOutput`，不覆盖 PID、INDI 或油门控制器结果。mixer 按当前模式选择手动结果或增稳结果，从机目标生成也使用相同的滚转来源。角色入口不再构造结果或复制 getter。INDI 原分配阶段的增量求解、延迟队列和舵机估计整合至 `update()`，仍每个有效 INDI 控制周期只执行一次；输出包含整数物理 PWM，mixer 仅还原为逻辑控制量。日志继续读取同一全局诊断字段。

模式准备与从机命令接收已合并为 `prepareControlMode()` 的主从两个分支。上级命令全局 `slaveCommand` 由模式模块持有，旧接收入口和独立路由文件已删除；通信目录内部协议和收发函数未修改。
