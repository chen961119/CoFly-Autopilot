# 控制架构

## 主从流程

A 主机（含单机配置）负责整体滚转、偏航角速度、油门、构型及本机俯仰；B～G 从机负责本机俯仰及本机副翼交联/阻尼。`control_pipeline.cpp` 每周期清空全局 `ControlCycleOutput commands`，调用一次 `prepareControlMode()`，再按角色运行：

```text
A：模式/目标准备 → runMasterController() → mixMasterCommands()
                                      → generateSlaveCommand()
    ├→ commands.local → PWM → 本机输出
    └→ slaveCommands[] → 机间命令

B～G：接收模式/目标/上级贡献 → runSlaveController() → mixSlaveCommands()
                                               → fuseSlaveCommands()
                                               → PWM → 本机输出
```

| 阶段 | 文件 | 输入 → 输出 |
| --- | --- | --- |
| 模式与主机目标 | `src/control/control_modes.cpp` | 遥控或接收命令 → 模式、`targets`、上级贡献 |
| 主机控制 | `src/control/master_controller.cpp` | 目标与测量 → 各控制器输出 |
| 主机分配 | `src/allocation/master_mixer.cpp` | 手动目标或增稳控制器输出 → 本机最终指令、从机原始分配 |
| 从机目标与发送命令 | `src/control/command_generation.cpp` | 当周期主分配 → `slaveTargets[]`、`slaveCommands[]` |
| 从机控制 | `src/control/slave_controller.cpp` | 本机俯仰目标与测量 → PID 或 INDI 输出 |
| 从机分配 | `src/allocation/slave_mixer.cpp` | 本机俯仰或手动贡献 → 本机交联/阻尼与闭环贡献 |
| 从机融合 | `src/allocation/command_fusion.cpp` | 本机贡献 + 上级贡献 → 最终逻辑指令 |

控制器按类封装，`controller_instances.cpp` 持有每块飞控的一套实例；积分、滤波历史和 INDI 延迟队列归对应对象。模式由 `control_modes.h/.cpp` 管理，输出由 `control_output.h/.cpp` 发布，层间类型定义在 `control_types.h`。

## 目标与结果接口

| 对象 | 含义与尺度 |
| --- | --- |
| `targets` / `ControlTargets` | 所有模式下 roll/pitch 为 rad、yawRate 为 rad/s、throttle 为 0～1 |
| `manual_input[0..3]` | 主机遥控直通输入，顺序为 roll/pitch/throttle/yaw；roll/pitch/yaw 为 -1～1，throttle 为 0～1 |
| `targets.configuration[6]` | 构型角目标，deg，顺序 AB、AC、BD、CE、DF、EG |
| `slaveTargets[0..5]` | B～G 的俯仰角目标，始终为 deg；A 单独存储 |
| `commands.local` / `commands.slaves[0..5]` | 本机和 B～G 的 `ActuatorCommand`，五路逻辑控制偏移 |
| `slaveCommands[]` / `slaveCommand` | 待发送/接收的模式、俯仰目标及 `masterContribution` |
| `ActuatorOutput[0..4]` | 最终物理 PWM，顺序为副翼1、副翼2、升降舵、油门、方向舵 |

`ControlTargets` 只保存目标；模式、策略和刹车开关独立存储。主机分配完成后，`generateSlaveCommand()` 才初始化当周期从机俯仰目标、叠加滚转/构型策略修正，并原位整理从机分配和发送命令。手动交联先读取未缩放的从机副翼，再执行副翼缩放与刹车叠加，不使用上一周期分配。

俯仰直接读取 `targets.pitch`，没有 mixer 反馈到下一周期的中央机俯仰修正缓存。偏航为机体 Z 轴角速度闭环：`YawRatePidController` 接收 rad/s 目标和 rad 滚转/俯仰目标，协调转弯项为 `g/V_cruise*tan(rollTarget)*cos(pitchTarget)`，叠加后与 `ang_vel_filtered[2]` 比较。FF、P、I、D 各自乘增益后直接相加，不再整体乘 0.573；I 项更新为 `I_prev + Ki*error*dt`，再限幅至 `±i_limit_yaw_rate`。发布 `yawRateControlOutput.pidOutput` 以及 `proportionalTerm`、`integralTerm`、`derivativeTerm`、`feedforwardTerm`，后四项之和等于总输出，未启用时五项均为 0，全部进入 SD 日志。内部增益使用 `Kp_yaw_rate` 等名称，持久化键保留旧名称。实装偏航使用油门差动，rudder 字段继续保留在通用五路接口中。

## 模式、策略和状态

增稳模式下，A 主机默认使用空速低通与 PI 调节基础油门，油门杆映射目标空速；手动模式直通。原始 `airspeed` 保留，反馈有效性、参数、最低杆处理和回退行为见 [简单空速控制](airspeed-control.md)。从机继续使用主机分配的油门。

仅有手动（通信编码 0）和增稳（编码 1）两种模式。主机 CH5 大于 0 时为手动，其余位置为增稳；从机使用接收的 `mode`。本机 SD 参数 `pitch_controller` 选择增稳俯仰算法：0 PID、1 INDI，默认 1；各机独立配置，下一周期生效，不通过机间帧同步算法。

滚转、俯仰、构型和空速积分由本机 SD 整数参数 `integralControlEnabled` 控制，默认 1；0 时关闭并清历史。偏航积分独立，不受此开关控制；`Ki_yaw_rate=0` 时清除偏航 I 项。模式或俯仰算法变化触发相应 PID 状态准备和 INDI 重置，INDI 以当前俯仰目标作为差分起点；构型积分不因模式切换统一清零。偏航 `prepare()` 清除切换前积分并重建微分起点，随后 `update()` 在切换当周期即从零开始累加 `Ki*error*dt` 并限幅；首次启用、切换或重新启用的首个周期 D 项为 0。手动不启用俯仰闭环，INDI 模式不计算未使用的俯仰 PID。

主机当前固定 `mixerStrategyCommand=1`、`configurationControlScale=1`。代码中的策略 2/3 不等于已接入遥控切换。CH7 选择构型角 10°/0°/−10°，左右目标交替取反；CH8 小于等于 0.2 时启用刹车。构型测量使用相邻 IMU 滚转差，MT6701 只用于测量记录，见 [固件指南](firmware-guide.md#传感器与统一测量接口)。

刹车启用时，主机在本机及各从机的两路副翼指令上分别叠加 `brake_aileron_offset`。该 SD 参数位于 Control 组，默认 0.1、允许 0～1，修改主机参数后下一周期生效；设为 0 可取消刹车偏置。数值是下偏为正的归一化偏置，不是 PWM 限幅或固定微秒量，实际 PWM 变化由各机的 min/trim/max、rev 和原有指令决定。最终输出仍使用原有 PWM 限幅。从机接收已包含刹车偏置的分配结果，不再叠加本机参数。

## 逻辑坐标、交联与融合

`ActuatorCommand` 不含本机 PWM 参数；当前舵面命令为 -1～1，油门命令为 0～1。五路均直接调用同一个 PWM 映射函数，mode=0 接收 -1～1，将 -1/0/+1 映射到 min/trim/max；mode=1 接收 0～1，将 0/0.5/1 映射到 min/两端中点/max，trim 仅为安全位置。调用方必须匹配模式的输入范围，因此当前舵面使用 mode=0、油门使用 mode=1；映射函数不再单独识别油门或转换范围。融合结果允许超出标准范围，最终在物理 PWM 层限幅。副翼和升降舵均以下偏为正。手动俯仰、PID 和 INDI 统一以抬头控制作用为正，在主/从 mixer 选用结果时取反。INDI 内部仍以下偏正计算舵偏与 PWM，仅在出口逆映射后取负；交联、前馈、通信及融合的方向均不变。模式 1 的 INDI 出口取负后为 -1～0、中点为 -0.5，不具备有符号俯仰控制量语义；当前升降舵控制与混控应保持 mode=0。

主从机各自使用本机 SD 参数 `elevator_aileron_gain`，向两路副翼叠加 `gain * elevator`。该参数位于 Control 组，默认 -0.05、范围 -1～1，默认等效于 `-elevator/20`；0 关闭交联，负值反向、正值同向，均采用下偏正的舵面坐标。主机第一路另加滚转阻尼，第二路减阻尼；从机两路均另加滚转阻尼。两者的阻尼分配不同，不能把 PWM 同号直接解释为舵面同向。

手动子机升降舵为 `本机 elevator - 0.25*(子机原始副翼1 + 子机原始副翼2)`。从机手动副翼补偿使用收到的完整手动升降舵贡献；增稳时使用本机选中的俯仰闭环贡献，不包含主机升降舵前馈。

| 从机路径 | 上级 `masterContribution.elevator` | 本机 elevator 贡献 | 融合结果 |
| --- | --- | --- | --- |
| 手动 | 完整手动指令 | 0 | 上级手动指令 |
| 增稳 PID | 主机前馈 | `-pitchPidControlOutput.pidOutput` | 本机闭环 + 前馈 |
| 增稳 INDI | 主机前馈 | `-pitchIndiControlOutput.indiOutput` | 本机闭环 + 前馈 |

副翼、油门、方向舵同样通过加法融合。融合函数只相加，不再运行控制律、交联或 PWM 映射。三机策略 3 的直接升降舵前馈在下偏正坐标中为 B=`-0.2R-0.01Rdot`、C=`+0.2R+0.01Rdot`，R 为滚转控制输出低通值；策略 1/2 前馈为 0。每周期清空分配，避免跨策略保留旧前馈。

## 输出、通信和限制

各节点融合后调用 `convertControlCommandsToPWM()`；`prepareActuatorCommands()` 从独立的 `slaveCommands[]` 打包机间命令，随后 `applyAndTransmitActuatorCommands()` 执行锁定/限幅、本机输出和发送/转发。物理映射与安全位置见 [固件指南](firmware-guide.md#执行器映射与方向迁移)。

机间传输 PWM 缩放前的 int16 定点逻辑输出，比例为 4096，范围为 `[-8,32767/4096]`；正常未饱和范围的最近整数舍入误差至多为 1/8192 逻辑量。俯仰目标按 0.1°编码，40 字节命令帧须在所有节点保持同一语义，详见 [机间通讯模块](../src/interaircraft_comm/README.md)。

当前保留首帧判断、有效帧缓存、校验和转发逻辑，没有新增命令超时后自动切换模式或覆盖输出的策略；`radioInputValid()` 仍用于原有地面站、解锁与校准入口。状态链新鲜度与本机锁定保护不能代替命令超时处理。INDI 计算和融合后舵面估计范围见 [INDI 说明](indi-variable-naming.md#模型与使用限制)；目标验收见 [软件需求](requirements.md)。
