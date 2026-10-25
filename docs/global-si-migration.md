# 全局变量 SI 单位迁移与验证

本报告记录全局 SI 迁移时的验证快照。后续已按要求将机间命令/状态线上单位从 0.1 度改为 0.001 rad（角速度为 0.001 rad/s），其最新实现和独立验证见 [机间弧度协议验证报告](interaircraft-radian-protocol.md)。下文关于旧机间线上度制编码的内容不再代表当前协议；独立遥测、GCS 和 OLED 仍沿用原显示/协议单位。

日期：2026-10-18。对照基线：`6a5ddeea176a38b4e865b11b9f0f4c60f3fc727b`。

本次修改覆盖 `src` 中全局物理状态，也包括全局结构体的物理量成员。按照后续要求，**时间相关量全部保留原单位和名字**：`current_time`、`dt`、软硬件计时器、延时、PWM 脉宽与 `ActuatorOutput` 均不迁移。

控制参数、持久化传感器标定参数、控制器函数参数名、局部变量名、私有积分/滤波历史和数值常量保持原约定。控制器读入 SI 全局量时转回原单位，发布全局物理量时转为 SI。归一化指令、布尔值、编号、四元数和原始协议计数不需要物理单位换算。

## 1. 全局变量逐项对照

`R = π/180`，`g₀ = 9.80665 m/s²`。角度及其时间导数均乘 `R`；时间本身不变。

| 原变量或字段 | 现变量或字段 | 原单位 → 新单位 | 实施方式与验证 |
| --- | --- | --- | --- |
| `att[3]` | 同名 | deg → rad | 内置 Madgwick、外置四元数解算的三角函数结果直接发布；C、S |
| `att_B[3]`～`att_G[3]` | 同名 | deg → rad | 状态帧解码时乘 R；发送前转回线上单位；C |
| `ang_vel[3]`、`ang_vel_filtered[3]` | 同名 | deg/s → rad/s | 采样源已转换，统一发布和线性滤波沿用新单位；C、S |
| `ang_vel_B[3]`～`ang_vel_G[3]` | 同名 | deg/s → rad/s | 仅协议实际传输的 X 轴转换；Y/Z 仍为 0；C |
| `ang_acc[3]` | 同名 | deg/s² → rad/s² | 原差分、滤波、限幅数值在原单位计算，发布乘 R；S |
| `acc[3]` | 同名 | 内置 g；外置 m/s² → 统一 m/s² | 内置乘 g₀；外置原本已是 m/s²，保持设备原有 9.8 换算尺度，不再乘 g₀；S |
| `AccX/Y/Z_6050` | 同名 | g → m/s² | BMI088、ICM42688、MPU6050 三条读数路径分别在写全局值前转换；S |
| `GyroX/Y/Z_6050` | 同名 | deg/s → rad/s | 三条内置采样路径分别乘 R；S |
| `AccX/Y/Z_prev_6050` | 同名 | g → m/s² | 随 MPU6050 全局当前样本保存同单位历史；S |
| `GyroX/Y/Z_prev_6050` | 同名 | deg/s → rad/s | 随 MPU6050 全局当前样本保存同单位历史；S |
| `Gyro_X/Y/Z_EXT` | 同名 | deg/s → rad/s | 外置 0x52 帧解码后乘 R；S |
| `Acc_X/Y/Z_EXT` | 同名 | m/s² → m/s² | 原先已为 SI，不重复缩放；S |
| `gyroX/Y/Z_filt_prev` | 同名 | deg/s → rad/s | 差分历史存 SI，取出时还原原计算单位；S |
| 文件级 `AccErrorX/Y/Z_6050` | 同名 | g → m/s² | 未参与当前采样的历史零偏初值也换算；静态检查 |
| 文件级 `GyroErrorX/Y/Z_6050` | 同名 | deg/s → rad/s | 未参与当前采样的历史零偏初值也换算；静态检查 |
| `airdata.aoa`、`airdata.aos` | 同名 | deg → rad | 本地候选报文校验完成后，发布全局值时转换；S |
| `relativeAngle_raw`、`relativeAngle_ready` | 同名 | deg → rad | MT6701 读数及归零计算结果发布时转换；S |
| `targets.roll`、`targets.pitch` | 同名 | 增稳 deg → rad | 遥控比例和配平仍按原参数计算，结果转换；主机手动模式仍为无量纲 ±1；C |
| `targets.yawRate` | 同名 | 增稳 deg/s → rad/s | 原最大角速度参数不变，在目标生成处换算；主机手动模式仍为无量纲 ±1；C |
| `targets.configuration[6]` | 同名 | deg → rad | 由新的 SI 构型角赋值；C |
| `slaveTargets[].pitch`、`slaveCommands[].target.pitch`、`slaveCommand.target.pitch` | 同名 | deg → rad | 从机目标基值、所有交联增量、通信接收结果均转换；C |
| `configurationAngleDeg` | `configurationAngle` | deg → rad | 上/下反仍为物理 ±10°，全局值为 ±π/18；C |
| `rollControlOutput.equivalentRollAngleDeg` | `equivalentRollAngle` | deg → rad | `roll_eq` 等私有状态仍为原值，仅全局发布转换；C |
| `configurationControlOutput.angleErrorsDeg[6]` | `angleErrors[6]` | deg → rad | 全局目标减全局测量；控制增益、mixer 阈值使用前转回度；C |
| `configurationControlOutput.measuredAnglesDeg[6]` | `measuredAngles[6]` | deg → rad | 六对相邻机体的 SI 姿态直接相减；C |
| `configurationControlOutput.measuredRatesDegPerSec[6]` | `measuredRates[6]` | deg/s → rad/s | 六对相邻机体的 SI 角速度直接相减；C |
| `pitchIndiControlOutput.q_ref_deg_s` | `q_ref` | deg/s → rad/s | 仅全局赋值乘 R，局部 `q_ref_deg_s` 不改；C |
| `pitchIndiControlOutput.q_f_deg_s` | `q_f` | deg/s → rad/s | 仅全局赋值乘 R，局部 `q_f_deg_s` 不改；C |
| `pitchIndiControlOutput.q_dot_ref_deg_s2` | `q_dot_ref` | deg/s² → rad/s² | 仅全局赋值乘 R，局部 `q_dot_ref_deg_s2` 不改；C |
| `pitchIndiControlOutput.q_dot_meas_deg_s2` | `q_dot_meas` | deg/s² → rad/s² | 直接记录已为 SI 的 `ang_acc[1]`；C |
| `pitchIndiControlOutput.delta_e_cmd_deg` | `delta_e_cmd` | deg → rad | 局部舵角计算、限幅、滤波和 PWM 反算仍按原值，仅发布乘 R；C |
| `pitchIndiControlOutput.delta_e_hat_deg` | `delta_e_hat` | deg → rad | 私有模型状态保持度制，仅发布乘 R；C |
| `filteredPressure` | 同名 | hPa → Pa | 初始海平面气压 1013.25 改为 101325；静态检查；该全局当前未被使用 |
| `temp` | 同名 | °C → K | BMP280 温度读数加 273.15，打印标签同步改 K；S |

已为 SI 的空速 `airspeed`、`airdata.tas`（m/s）、气压 `pressure`（Pa）、高度 `filteredAltitude`（m）、分配模型质量（kg）、惯量（kg·m²）及几何量（m、m²）保持原值。`strain_value` 与遥控原始通道是设备计数，缺少工程量标定关系，不臆造缩放。

## 2. 每个实现位置的修改与验证

验证代号：**C** 为修改前后实际 C++ 源码数值对照；**S** 为从实际传感器源文件原样提取函数后的寄存器/串口输入对照；**A** 为源码与边界契约检查；**B** 为 Teensy 普通、调试固件编译。

| 文件 / 函数 | 具体修改 | 验证 |
| --- | --- | --- |
| `src/math/units.h` | 新增统一角度、角速度、角加速度、重力单位换算函数；不提供时间迁移 | C：180°→π、1g→9.80665 的编译期断言；B |
| `sensors/sensor_processing.cpp::getBMI088data` | 保留旋转轴映射与标定值，减零偏后转换全局加速度/角速度 | S：120 组正负寄存器读数及非零零偏；B |
| `getICM42688data` | 六轴寄存器换算后写 SI 全局值 | S：120 组寄存器读数；B |
| `getIMUdata` | 原 g、deg/s 标定参数读入时转换；当前样本和历史保存 SI；原低通系数不变 | S：120 次连续采样；B |
| `getIMUdata_EXT` | 0x52 角速度转换，0x59 姿态直接发布弧度；0x51 加速度不重复转换 | S：正负三轴与非平凡四元数帧；B |
| `getairdata` | 校验和仍针对原始候选帧，发布 `aoa/aos` 后转换 | S：真实打包字段及校验和；B |
| `getRotateSensor1` | MT6701 绝对角、相对角转换后发布 | S：0、5、180、350、359.9° 与 350° 零点，包含跨零边界；B |
| `ResetRotateSensor` | 新 SI 原始角保存为旧度制标定参数前转回度 | A：明确检查持久化调用处转换；B |
| `getAngularACC` | 差分历史存 SI；原局部差分、滤波及 ±3000 deg/s² 限幅不变；输出转 SI | S：400 周期，内外置两种数据源，包含 20ms 无效间隔重建历史；B |
| `updateSensorMeasurements` | 将已转换的内外置采样值发布为 SI，保留轴符号、优先级 | S：内外置两次独立编译；B |
| `getbarodata` 与气压初值 | 摄氏转开尔文；hPa 初值转 Pa | S：20°C→293.15K，101325Pa、123m 保持；A：初值；B（默认未启用，S 单独启用分支） |
| `sensors/filters.cpp::Madgwick` | SI 全局输入转回原算法局部输入单位，保留算法常数及局部值约定 | S：内置采样路径；A：输入契约；B |
| `Madgwick6DOF` | 四元数解算的姿态直接发布 rad，去掉输出转度系数 | C：200 周期的四元数和三轴姿态对照；B |
| `updateAngularVelocityFilters` | 原实现不需要改动；输入/全局输出一起变为 rad/s | C、S：在连续周期中执行；B |
| `control/control_types.h` | 全局结构体字段去度制后缀，逐项修订单位注释 | A：旧字段名不再存在；C：所有字段读写；B |
| `control/control_modes.h/.cpp` | 构型角改名；遥控生成的物理目标转 SI，手动归一化目标不变 | C：两种模式、CH7 的 -0.5/-0.2/0/0.2/0.5 阈值；B |
| `control/command_generation.h/.cpp` | 保留三种度制兼容 getter 的返回尺度；所有从机目标基值/增量转 rad；构型误差读入转度 | C：五种机数、三个策略、900 周期；B |
| `master_controller.cpp` | SI 物理目标在传入现有控制器前转原单位 | C：实际完整主机入口，PID/INDI 切换；B |
| `slave_controller.cpp` | 从机俯仰目标在 reset/update 输入边界转度 | C：实际完整从机入口；B |
| `manual_controller.cpp` | 手动模式保持归一化直通；增稳时被记录的未启用手动结果也保持原数值尺度 | C：两种模式四个发布字段对照；B |
| `roll_pid_controller.cpp` | 全部本机/编队反馈读入转原单位；等效滚转角仅发布时转 SI | C：五种构型、积分开关、使能与模式切换；B |
| `pitch_pid_controller.cpp` | 姿态、角速度反馈读入转原单位，原私有历史及 PID 计算不变 | C：900 周期对照；B |
| `yaw_rate_pid_controller.cpp` | Z 轴滤波角速度读入转原单位，协调转弯公式不变 | C：900 周期对照；B |
| `roll_damping_controller.cpp` | 滚转角速度读入转原单位，阻尼系数不变 | C：900 周期对照；B |
| `pitch_indi_controller.cpp` | SI 反馈读入转原单位；六个物理诊断量发布为 SI；PWM、模型、限幅和所有局部名/常量不变 | C：900 周期、完整主从入口、使能、模式和 PID/INDI 切换；A；B |
| `configuration_controller.cpp` | 全局测量/误差直接用 SI；进入原控制状态和增益前转原单位 | C：六路测量、误差、角速度与输出逐值对照；B |
| `allocation/master_mixer.cpp` | 全局构型误差读入转度，保留原 20° 阈值与混控局部量 | C：实际完整 mixer、五种机数、所有本机/从机指令；B |
| `control/control_allocation.cpp` | SI 姿态读入转回原值，再执行既有 0.01745 换算；保留局部变量及模型数值 | A：逐表达式检查读取与既有系数；B；该旧动态模型尚未接入当前 mixer，不宣称动态模型闭环实测 |
| `interaircraft_comm.cpp::setCommandLink/getCommandLink` | SI 俯仰目标与原 0.1° 定点字段互转，输入合法范围同步转 rad | C：三个槽位、正负角及实际帧编解码；B |
| `getSensorLink/setSensorLink` | SI 姿态/滚转角速度与原 0.1°、0.1°/s 字段互转 | C：左右链路、B～G 六种身份、全部槽位；B |
| `telemetry/telemetry.cpp` | 全局遥测缓存保留 rad，只在打包线上值时转回度 | C：实际 19 字节帧与八个定点字段；B |
| `ground_station_service.cpp` | DATA/CONFIG 的姿态、角速度和相对角在输出边界转回原显示单位；内置加速度回到旧 GCS 的 g，外置保持旧 m/s² | A：各字段边界契约；B；未连接真实地面站作 UI 实测 |
| `interface/human_interface.cpp` | 三轴姿态和相对角送 OLED 度符号前转度 | A：四个显示入口；B；未上板观察 OLED |
| `diagnostics/debug_print.cpp` | 构型字段改名、打印 rad 与六位小数；等效滚转全局字段改名 | A：格式、字段与单位一致；B |
| `logging/logger.cpp` | 跟随全局字段改名、直接记录 SI；新增 `si_units_version=1`，时间/PWM 列原样保留 | A：字段清单与单一表头/数据写入路径；B；未写真实 SD 卡 |
| 传感器、通信头文件与相关 README、INDI 文档 | 同步单位说明、全局/局部边界和日志列语义 | A：与上表及实际声明交叉核对 |
| `tools/verification/verify_si_units.py` | 可重复的基线对照、硬件桩和范围审计，不烧写设备 | 自身实际执行，结果见下节 |

## 3. 验证结果与重现

运行：

```powershell
& C:/Users/chen/.platformio/penv/Scripts/python.exe tools/verification/verify_si_units.py
& C:/Users/chen/.platformio/penv/Scripts/python.exe -m platformio run -e teensy41 -e teensy41_debug
git diff --check
```

完整主机回归包含五种机数、900 周期控制及混控、模式/积分切换、200 周期姿态估计、真实命令/状态/遥测编码。对照时把 SI 全局值转回基线单位，输出指令直接比较。普通控制量容差为 `0.002 + |基线|×0.0001`，协议容差上限为一个 0.1 单位量化步长；这不是更改协议精度。

传感器测试直接从两个版本的 `sensor_processing.cpp` 提取被测函数，保留预处理分支，使用相同模拟寄存器、校准参数和串口帧。一般量容差为 `0.0003 + |基线|×0.0002`；角加速度容差为 `0.1 + |基线|×0.0002 deg/s²`，单独考虑差分对 float32 换算舍入的放大。

时间、PWM、参数代码、遥控代码以及所有控制器私有状态头文件共 35 个文件逐文件核对保持不变；同时核对各控制器局部声明/形参名字和全部数值字面量保持不变。浮点数值不承诺逐位相同，原闭环公式与参数尺度保持不变。

最终执行结果：

| 验证项目 | 实际结果 |
| --- | --- |
| 1 / 3 / 4 / 5 / 7 机控制、主从入口、mixer、目标、姿态和通信对照 | 每种 79,948 个数值，全部 PASS；每种最大绝对差 0.000184（转回基线各自单位后统计） |
| 内置采样、校准、滤波/差分、温度、转角和姿态包装入口 | 6,139 个数值，PASS；最大绝对差 0.000001 |
| 外置采样及同一组边界测试 | 6,139 个数值，PASS；最大绝对差 0.03290392 deg/s²，发生在角加速度差分，约 0.00057427 rad/s² |
| 数值对照总计 | 412,018 个数值，全部在上述容差内 |
| 保留范围检查 | 35 个源码/头文件保持不变，PASS |
| 控制器声明、形参名和数值常量检查 | PASS |
| 全局字段、CSV、GCS/OLED、标定存储与旧分配模型边界检查 | PASS |
| 普通固件 `teensy41` | PASS |
| 调试固件 `teensy41_debug` | PASS |
| `git diff --check` | PASS |

最终采样补充使用 `verify_si_units.py --sensors-only` 重跑；这包含对真实 `Madgwick()` 包装入口的额外 50 周期测试。上述差异是浮点舍入后的绝对误差，不是控制增益或限幅值的变更。

主机硬件桩与交叉编译不等于真实传感器、板载时序、SD/OLED 或飞行验证。

## 4. 外部兼容与例外

- 度制仍可出现在控制器局部变量、私有状态、参数和协议编码/显示边界，这是本次限定范围所需。已迁移的全局物理状态不再使用度制后缀。
- `relativeAngle_offset`、`calAccGyroData`、`appliedImuCalibration` 属于原有持久化标定参数，存储值、校验和算法、范围和参数键保持不变。
- 命令帧仍为 40 字节，状态帧仍为 34 字节，遥测帧仍为 19 字节；没有把线上度制数值直接解释为弧度。
- 现有地面站和 OLED 的角度显示尺度保持不变。旧 GCS 加速度本身有内置 g / 外置 m/s² 的历史差别，本次只统一固件全局物理状态，不改变旧客户端协议。
- 新日志的同名 `att/ang_vel/ang_acc/acc` 列语义已经变为 SI；使用 `si_units_version=1` 区分历史日志。INDI、构型及等效滚转字段列名随成员一起更新。`current_time` 仍为 μs，`dt` 仍为 s，`ActuatorOutput` 仍为 μs。

## 5. 精确修改行清单

下方清单以最终差异逐块生成。每项列出新文件位置、旧行号和该块变更的实际行数；验证方式以上一节对应文件/函数为准。只有注释、包含头文件或文档的块使用 A/B，不将它们描述为硬件实测。

| 编号 | 文件 | 新起始行 | 旧起始行 | 删除/新增行数 | 所属位置（Git 上下文） |
| --- | --- | --- | --- | --- | --- |
| 1 | [docs/indi-variable-naming.md](../docs/indi-variable-naming.md) | 3 | 3 | -1 / +1 | `文件级声明/注释` |
| 2 | [docs/indi-variable-naming.md](../docs/indi-variable-naming.md) | 19 | 19 | -1 / +1 | `文件级声明/注释` |
| 3 | [docs/indi-variable-naming.md](../docs/indi-variable-naming.md) | 23 | 23 | -6 / +6 | `文件级声明/注释` |
| 4 | [docs/indi-variable-naming.md](../docs/indi-variable-naming.md) | 36 | 35 | -0 / +2 | `文件级声明/注释` |
| 5 | [docs/indi-variable-naming.md](../docs/indi-variable-naming.md) | 59 | 57 | -1 / +1 | `delta_e_cmd      = delta_e_hat + delta_delta_e` |
| 6 | [docs/indi-variable-naming.md](../docs/indi-variable-naming.md) | 84 | 82 | -1 / +1 | `C 对应 0°。上下偏使用同一个 1000*k 系数，不按两侧行程分段` |
| 7 | [src/README.md](../src/README.md) | 38 | 38 | -1 / +1 | `PlatformIO 的 `include_dir = src` 与 `-I src` 提供统一头文件根目录` |
| 8 | [src/README.md](../src/README.md) | 48 | 48 | -1 / +1 | `PlatformIO 的 `include_dir = src` 与 `-I src` 提供统一头文件根目录` |
| 9 | [src/README.md](../src/README.md) | 58 | 58 | -1 / +1 | `PlatformIO 的 `include_dir = src` 与 `-I src` 提供统一头文件根目录` |
| 10 | [src/README.md](../src/README.md) | 69 | 68 | -0 / +2 | `PlatformIO 的 `include_dir = src` 与 `-I src` 提供统一头文件根目录` |
| 11 | [src/allocation/master_mixer.cpp](../src/allocation/master_mixer.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 12 | [src/allocation/master_mixer.cpp](../src/allocation/master_mixer.cpp) | 26 | 25 | -3 / +3 | `static void mixLegacyMaster()` |
| 13 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 14 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 17 | 16 | -1 / +1 | `float targetRollDegrees() {` |
| 15 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 22 | 21 | -1 / +1 | `float targetPitchDegrees() {` |
| 16 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 27 | 26 | -1 / +1 | `float targetYawRateDegreesPerSec() {` |
| 17 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 34 | 33 | -1 / +1 | `void generateSlaveCommand() {` |
| 18 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 47 | 46 | -2 / +2 | `void generateSlaveCommand() {` |
| 19 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 50 | 49 | -2 / +2 | `void generateSlaveCommand() {` |
| 20 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 60 | 59 | -2 / +2 | `void generateSlaveCommand() {` |
| 21 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 65 | 64 | -3 / +3 | `void generateSlaveCommand() {` |
| 22 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 69 | 68 | -3 / +3 | `void generateSlaveCommand() {` |
| 23 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 74 | 73 | -3 / +3 | `void generateSlaveCommand() {` |
| 24 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 79 | 78 | -4 / +4 | `void generateSlaveCommand() {` |
| 25 | [src/control/command_generation.cpp](../src/control/command_generation.cpp) | 84 | 83 | -6 / +6 | `void generateSlaveCommand() {` |
| 26 | [src/control/command_generation.h](../src/control/command_generation.h) | 7 | 7 | -1 / +1 | `extern SlaveCommand slaveCommands[CONTROL_AIRCRAFT_SLOTS];` |
| 27 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 28 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 15 | 14 | -12 / +12 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 29 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 31 | 30 | -1 / +1 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 30 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 33 | 32 | -11 / +11 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 31 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 57 | 56 | -1 / +1 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 32 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 66 | 65 | -1 / +1 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 33 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 75 | 74 | -1 / +1 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 34 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 84 | 83 | -1 / +1 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 35 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 93 | 92 | -1 / +1 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 36 | [src/control/configuration_controller.cpp](../src/control/configuration_controller.cpp) | 102 | 101 | -1 / +1 | `void ConfigurationController::update(const float (&targets)[6]) {` |
| 37 | [src/control/control_allocation.cpp](../src/control/control_allocation.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 38 | [src/control/control_allocation.cpp](../src/control/control_allocation.cpp) | 105 | 104 | -4 / +4 | `void initializeRotationMatrices(Matrix3f& Eab, Matrix3f& Eac, Matrix3f& Ebd, Mat` |
| 39 | [src/control/control_allocation.cpp](../src/control/control_allocation.cpp) | 437 | 436 | -1 / +1 | `void getpinvBplusmini() {` |
| 40 | [src/control/control_modes.cpp](../src/control/control_modes.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 41 | [src/control/control_modes.cpp](../src/control/control_modes.cpp) | 18 | 17 | -1 / +1 | `float configurationControlScale = 1.0f;` |
| 42 | [src/control/control_modes.cpp](../src/control/control_modes.cpp) | 33 | 32 | -1 / +1 | `void initializeInitialControlMode() {` |
| 43 | [src/control/control_modes.cpp](../src/control/control_modes.cpp) | 46 | 45 | -2 / +2 | `void prepareControlMode() {` |
| 44 | [src/control/control_modes.cpp](../src/control/control_modes.cpp) | 50 | 49 | -1 / +1 | `void prepareControlMode() {` |
| 45 | [src/control/control_modes.cpp](../src/control/control_modes.cpp) | 59 | 58 | -3 / +3 | `void prepareControlMode() {` |
| 46 | [src/control/control_modes.h](../src/control/control_modes.h) | 14 | 14 | -1 / +1 | `extern float configurationControlScale; // 构型控制输出系数，固定为` |
| 47 | [src/control/control_types.h](../src/control/control_types.h) | 8 | 8 | -2 / +2 | `enum class PitchController { PID = 0, INDI = 1 };` |
| 48 | [src/control/control_types.h](../src/control/control_types.h) | 12 | 12 | -1 / +1 | `struct ControlTargets {` |
| 49 | [src/control/control_types.h](../src/control/control_types.h) | 18 | 18 | -1 / +1 | `struct SlaveControlTargets {` |
| 50 | [src/control/control_types.h](../src/control/control_types.h) | 26 | 26 | -3 / +3 | `struct IndiPitchControlOutput {` |
| 51 | [src/control/control_types.h](../src/control/control_types.h) | 31 | 31 | -3 / +3 | `struct IndiPitchControlOutput {` |
| 52 | [src/control/control_types.h](../src/control/control_types.h) | 53 | 53 | -1 / +1 | `struct RollControlOutput {` |
| 53 | [src/control/control_types.h](../src/control/control_types.h) | 67 | 67 | -3 / +3 | `struct ConfigurationControlOutput {` |
| 54 | [src/control/manual_controller.cpp](../src/control/manual_controller.cpp) | 3 | 2 | -0 / +2 | `文件级声明/注释` |
| 55 | [src/control/manual_controller.cpp](../src/control/manual_controller.cpp) | 7 | 5 | -2 / +4 | `void ManualController::update(const MasterControlTargets &targets) const {` |
| 56 | [src/control/manual_controller.cpp](../src/control/manual_controller.cpp) | 12 | 8 | -1 / +1 | `void ManualController::update(const MasterControlTargets &targets) const {` |
| 57 | [src/control/master_controller.cpp](../src/control/master_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 58 | [src/control/master_controller.cpp](../src/control/master_controller.cpp) | 22 | 21 | -2 / +2 | `void runMasterController()` |
| 59 | [src/control/master_controller.cpp](../src/control/master_controller.cpp) | 25 | 24 | -2 / +2 | `void runMasterController()` |
| 60 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 61 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 22 | 21 | -2 / +2 | `using namespace ControlParameters;` |
| 62 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 73 | 72 | -1 / +1 | `void PitchIndiController::update(float theta_ref_deg, bool enabled) {` |
| 63 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 85 | 84 | -1 / +1 | `void PitchIndiController::update(float theta_ref_deg, bool enabled) {` |
| 64 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 123 | 122 | -1 / +1 | `void PitchIndiController::update(float theta_ref_deg, bool enabled) {` |
| 65 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 129 | 128 | -2 / +2 | `void PitchIndiController::update(float theta_ref_deg, bool enabled) {` |
| 66 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 137 | 136 | -3 / +3 | `void PitchIndiController::update(float theta_ref_deg, bool enabled) {` |
| 67 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 150 | 149 | -1 / +1 | `void PitchIndiController::update(float theta_ref_deg, bool enabled) {` |
| 68 | [src/control/pitch_indi_controller.cpp](../src/control/pitch_indi_controller.cpp) | 219 | 218 | -3 / +3 | `void PitchIndiController::update(float theta_ref_deg, bool enabled) {` |
| 69 | [src/control/pitch_pid_controller.cpp](../src/control/pitch_pid_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 70 | [src/control/pitch_pid_controller.cpp](../src/control/pitch_pid_controller.cpp) | 21 | 20 | -1 / +1 | `void PitchPidController::update(float pitchTarget, bool enabled) {` |
| 71 | [src/control/pitch_pid_controller.cpp](../src/control/pitch_pid_controller.cpp) | 30 | 29 | -1 / +1 | `void PitchPidController::update(float pitchTarget, bool enabled) {` |
| 72 | [src/control/pitch_pid_controller.cpp](../src/control/pitch_pid_controller.cpp) | 35 | 34 | -2 / +2 | `void PitchPidController::update(float pitchTarget, bool enabled) {` |
| 73 | [src/control/pitch_pid_controller.cpp](../src/control/pitch_pid_controller.cpp) | 39 | 38 | -1 / +1 | `void PitchPidController::update(float pitchTarget, bool enabled) {` |
| 74 | [src/control/pitch_pid_controller.cpp](../src/control/pitch_pid_controller.cpp) | 58 | 57 | -1 / +1 | `void PitchPidController::update(float pitchTarget, bool enabled) {` |
| 75 | [src/control/roll_damping_controller.cpp](../src/control/roll_damping_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 76 | [src/control/roll_damping_controller.cpp](../src/control/roll_damping_controller.cpp) | 10 | 9 | -1 / +1 | `void RollDampingController::update() const {` |
| 77 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 78 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 21 | 20 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 79 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 26 | 25 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 80 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 31 | 30 | -7 / +7 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 81 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 74 | 73 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 82 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 84 | 83 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 83 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 90 | 89 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 84 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 94 | 93 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 85 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 98 | 97 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 86 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 102 | 101 | -2 / +2 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 87 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 159 | 158 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 88 | [src/control/roll_pid_controller.cpp](../src/control/roll_pid_controller.cpp) | 161 | 160 | -1 / +1 | `void RollPidController::update(float rollTarget, bool enabled) {` |
| 89 | [src/control/slave_controller.cpp](../src/control/slave_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 90 | [src/control/slave_controller.cpp](../src/control/slave_controller.cpp) | 13 | 12 | -1 / +1 | `void runSlaveController() {` |
| 91 | [src/control/slave_controller.cpp](../src/control/slave_controller.cpp) | 19 | 18 | -2 / +2 | `void runSlaveController() {` |
| 92 | [src/control/yaw_rate_pid_controller.cpp](../src/control/yaw_rate_pid_controller.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 93 | [src/control/yaw_rate_pid_controller.cpp](../src/control/yaw_rate_pid_controller.cpp) | 26 | 25 | -1 / +1 | `void YawRatePidController::update(float yawRateTarget, float rollTarget, float p` |
| 94 | [src/diagnostics/debug_print.cpp](../src/diagnostics/debug_print.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 95 | [src/diagnostics/debug_print.cpp](../src/diagnostics/debug_print.cpp) | 37 | 36 | -1 / +1 | `void printControlMode(int enabled) {` |
| 96 | [src/diagnostics/debug_print.cpp](../src/diagnostics/debug_print.cpp) | 43 | 42 | -1 / +1 | `void printControlMode(int enabled) {` |
| 97 | [src/diagnostics/debug_print.cpp](../src/diagnostics/debug_print.cpp) | 139 | 138 | -1 / +1 | `void printConfigurationData() {` |
| 98 | [src/ground_station_service/ground_station_service.cpp](../src/ground_station_service/ground_station_service.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 99 | [src/ground_station_service/ground_station_service.cpp](../src/ground_station_service/ground_station_service.cpp) | 21 | 19 | -0 / +8 | `namespace {` |
| 100 | [src/ground_station_service/ground_station_service.cpp](../src/ground_station_service/ground_station_service.cpp) | 44 | 35 | -1 / +1 | `bool readGroundAircraftAttitude(int id, float &roll, float &pitch) {` |
| 101 | [src/ground_station_service/ground_station_service.cpp](../src/ground_station_service/ground_station_service.cpp) | 46 | 37 | -2 / +2 | `bool readGroundAircraftAttitude(int id, float &roll, float &pitch) {` |
| 102 | [src/ground_station_service/ground_station_service.cpp](../src/ground_station_service/ground_station_service.cpp) | 141 | 132 | -3 / +3 | `void handleGroundStationCommand(char *line, char *response, size_t capacity) {` |
| 103 | [src/ground_station_service/ground_station_service.cpp](../src/ground_station_service/ground_station_service.cpp) | 157 | 148 | -3 / +3 | `void handleGroundStationCommand(char *line, char *response, size_t capacity) {` |
| 104 | [src/interaircraft_comm/README.md](../src/interaircraft_comm/README.md) | 34 | 34 | -1 / +1 | `文件级声明/注释` |
| 105 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 106 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 10 | 9 | -1 / +1 | `文件级声明/注释` |
| 107 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 45 | 44 | -2 / +2 | `bool AircraftCommandLink::setCommandLink(unsigned int index, const SlaveCommand` |
| 108 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 57 | 56 | -1 / +1 | `bool AircraftCommandLink::setCommandLink(unsigned int index, const SlaveCommand` |
| 109 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 124 | 123 | -1 / +1 | `const SlaveCommand &AircraftCommandLink::getCommandLink() {` |
| 110 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 206 | 205 | -4 / +4 | `void AircraftSensorLink::getSensorLink() {` |
| 111 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 239 | 238 | -1 / +1 | `void AircraftSensorLink::setSensorLink() {` |
| 112 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 246 | 245 | -2 / +2 | `void AircraftSensorLink::setSensorLink() {` |
| 113 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 255 | 254 | -2 / +2 | `void AircraftSensorLink::setSensorLink() {` |
| 114 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 258 | 257 | -1 / +1 | `void AircraftSensorLink::setSensorLink() {` |
| 115 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 260 | 259 | -2 / +2 | `void AircraftSensorLink::setSensorLink() {` |
| 116 | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 263 | 262 | -2 / +2 | `void AircraftSensorLink::setSensorLink() {` |
| 117 | [src/interaircraft_comm/interaircraft_comm.h](../src/interaircraft_comm/interaircraft_comm.h) | 13 | 13 | -1 / +1 | `public:` |
| 118 | [src/interaircraft_comm/interaircraft_comm.h](../src/interaircraft_comm/interaircraft_comm.h) | 131 | 131 | -1 / +1 | `bool isSensorDataFresh(bool left);` |
| 119 | [src/interaircraft_comm/interaircraft_comm.h](../src/interaircraft_comm/interaircraft_comm.h) | 133 | 133 | -1 / +1 | `extern float att_B[3], att_C[3], att_D[3], att_E[3], att_F[3], att_G[3];` |
| 120 | [src/interface/human_interface.cpp](../src/interface/human_interface.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 121 | [src/interface/human_interface.cpp](../src/interface/human_interface.cpp) | 271 | 270 | -1 / +1 | `void displayAttitude() {` |
| 122 | [src/interface/human_interface.cpp](../src/interface/human_interface.cpp) | 283 | 282 | -1 / +1 | `void displayAttitude() {` |
| 123 | [src/interface/human_interface.cpp](../src/interface/human_interface.cpp) | 292 | 291 | -1 / +1 | `void displayAttitude() {` |
| 124 | [src/interface/human_interface.cpp](../src/interface/human_interface.cpp) | 301 | 300 | -1 / +1 | `void displayAttitude() {` |
| 125 | [src/logging/logger.cpp](../src/logging/logger.cpp) | 167 | 166 | -0 / +3 | `namespace` |
| 126 | [src/logging/logger.cpp](../src/logging/logger.cpp) | 209 | 206 | -1 / +1 | `namespace` |
| 127 | [src/logging/logger.cpp](../src/logging/logger.cpp) | 229 | 226 | -1 / +1 | `namespace` |
| 128 | [src/logging/logger.cpp](../src/logging/logger.cpp) | 231 | 228 | -3 / +3 | `namespace` |
| 129 | [src/logging/logger.cpp](../src/logging/logger.cpp) | 236 | 233 | -3 / +3 | `namespace` |
| 130 | [src/logging/logger.cpp](../src/logging/logger.cpp) | 243 | 240 | -3 / +3 | `namespace` |
| 131 | [src/parameters/README.md](../src/parameters/README.md) | 38 | 38 | -1 / +1 | `文件级声明/注释` |
| 132 | [src/sensors/filters.cpp](../src/sensors/filters.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 133 | [src/sensors/filters.cpp](../src/sensors/filters.cpp) | 86 | 85 | -2 / +3 | `void Madgwick() {` |
| 134 | [src/sensors/filters.cpp](../src/sensors/filters.cpp) | 187 | 185 | -2 / +1 | `void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,` |
| 135 | [src/sensors/filters.cpp](../src/sensors/filters.cpp) | 189 | 188 | -4 / +2 | `void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,` |
| 136 | [src/sensors/filters.h](../src/sensors/filters.h) | 20 | 20 | -2 / +2 | `extern PX4LowPassFilter2p gyroFiltYIndi, gyroFiltXIndi, gyroFiltZIndi;` |
| 137 | [src/sensors/imu_internal.h](../src/sensors/imu_internal.h) | 3 | 2 | -0 / +1 | `文件级声明/注释` |
| 138 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 139 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 23 | 22 | -1 / +1 | `Adafruit_BMP280 bmp(&BMP280_I2C);` |
| 140 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 128 | 127 | -1 / +1 | `const float inv_gyr_lsb = 1.0f / 16.384f;` |
| 141 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 165 | 164 | -2 / +1 | `void getIMUdata_EXT() {` |
| 142 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 167 | 167 | -4 / +2 | `void getIMUdata_EXT() {` |
| 143 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 173 | 175 | -3 / +3 | `void getIMUdata_EXT() {` |
| 144 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 297 | 299 | -1 / +1 | `void Strain_read_all() {` |
| 145 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 321 | 323 | -3 / +3 | `void getICM42688data() {` |
| 146 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 325 | 327 | -3 / +3 | `void getICM42688data() {` |
| 147 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 347 | 349 | -1 / +1 | `void getICM42688data() {` |
| 148 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 386 | 388 | -3 / +3 | `void getBMI088data() {` |
| 149 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 390 | 392 | -3 / +3 | `void getBMI088data() {` |
| 150 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 464 | 465 | -0 / +2 | `void getairdata() {` |
| 151 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 518 | 518 | -6 / +6 | `float B_gyro_6050 = 0.47;  // MPU6050 陀螺仪低通系数。` |
| 152 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 601 | 601 | -1 / +1 | `void getIMUdata() {` |
| 153 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 609 | 609 | -1 / +1 | `void getIMUdata() {` |
| 154 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 611 | 611 | -3 / +3 | `void getIMUdata() {` |
| 155 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 615 | 615 | -3 / +3 | `void getIMUdata() {` |
| 156 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 627 | 627 | -1 / +1 | `void getIMUdata() {` |
| 157 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 629 | 629 | -3 / +3 | `void getIMUdata() {` |
| 158 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 633 | 633 | -3 / +3 | `void getIMUdata() {` |
| 159 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 767 | 767 | -1 / +1 | `void getbarodata() {` |
| 160 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 772 | 772 | -1 / +1 | `void getbarodata() {` |
| 161 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 824 | 824 | -1 / +1 | `void getRotateSensor1() {` |
| 162 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 826 | 826 | -1 / +1 | `void getRotateSensor1() {` |
| 163 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 832 | 832 | -1 / +1 | `void ResetRotateSensor() {` |
| 164 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 907 | 907 | -1 / +1 | `void initializeAngularAccelerationFilters() {` |
| 165 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 917 | 917 | -3 / +3 | `void getAngularACC() {` |
| 166 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 926 | 926 | -3 / +3 | `void getAngularACC() {` |
| 167 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 930 | 930 | -3 / +3 | `void getAngularACC() {` |
| 168 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 934 | 934 | -3 / +3 | `void getAngularACC() {` |
| 169 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 938 | 938 | -3 / +3 | `void getAngularACC() {` |
| 170 | [src/sensors/sensor_processing.cpp](../src/sensors/sensor_processing.cpp) | 943 | 943 | -3 / +3 | `void getAngularACC() {` |
| 171 | [src/sensors/sensor_processing.h](../src/sensors/sensor_processing.h) | 12 | 12 | -3 / +3 | `struct FC_Binary_Packet {` |
| 172 | [src/sensors/sensor_processing.h](../src/sensors/sensor_processing.h) | 36 | 35 | -0 / +1 | `extern int32_t imuCalibrationChecksumParameter;` |
| 173 | [src/sensors/sensor_processing.h](../src/sensors/sensor_processing.h) | 57 | 55 | -0 / +1 | `void writeRegSPI(int cs, uint8_t reg, uint8_t val);` |
| 174 | [src/sensors/sensor_processing.h](../src/sensors/sensor_processing.h) | 60 | 58 | -1 / +1 | `extern float airspeed;` |
| 175 | [src/sensors/sensor_processing.h](../src/sensors/sensor_processing.h) | 63 | 61 | -1 / +1 | `extern float ang_vel[3];` |
| 176 | [src/telemetry/telemetry.cpp](../src/telemetry/telemetry.cpp) | 1 | 0 | -0 / +1 | `文件级声明/注释` |
| 177 | [src/telemetry/telemetry.cpp](../src/telemetry/telemetry.cpp) | 42 | 41 | -1 / +1 | `void telemetry() // 主机数传` |
| 178 | [src/telemetry/telemetry.cpp](../src/telemetry/telemetry.cpp) | 56 | 55 | -1 / +1 | `void telemetry() // 主机数传` |
| 179 | [src/telemetry/telemetry.cpp](../src/telemetry/telemetry.cpp) | 65 | 64 | -1 / +1 | `void telemetry() // 主机数传` |
| 180 | [tools/flight_data_browser/README.md](../tools/flight_data_browser/README.md) | 81 | 80 | -0 / +2 | `MATLAB 参考文件作为实现参考读取，原脚本与原始飞行日志不` |
| 181 | [src/math/units.h](../src/math/units.h) | 1 | 新增 | +13 | 完整新文件，验证见第 2 节 |
| 182 | [tools/verification/verify_si_units.py](../tools/verification/verify_si_units.py) | 1 | 新增 | +527 | 完整新文件，验证见第 2 节 |

## 控制器内部弧度化补充

2026-10-25 起，滚转、俯仰、构型和 INDI 内部计算、积分、滤波历史及相关控制参数也采用弧度。本文此前关于这些控制器私有状态继续使用度的说明属于迁移前状态，现以 [控制输出等效核对](control-output-equivalence.md) 为准。输出字段和参数数量不增加，参数直接使用当前弧度值，不兼容或自动换算旧度制参数。
